// INTENT: end-to-end Layer-3 placement + LNS (step 5 Phase G + step 6
// Phase 5). Runs the full pipeline:
//
//   1. enumerate(workflow, strategies)          (steps 1-3, T.7)
//   2. allocate(enumeration)                    (step 4 + T.5)
//   3. positional_prior over palletize tasks    (Phase C)
//   4. build a LayoutProblem from the allocation, with Transports
//      threaded in (step 6 Phase 1)
//   5. solve(layout_problem)                    -> cold solve (Phase E)
//   6. lns_solve(layout_problem)                -> SA-annealed best
//      (step 6 Phases 3-4)
//   7. emit before.svg, after.svg, cost_trace.csv
//
// Success: cost_trace.csv lists every LNS iteration; after.svg shows
// a layout with total cost <= before.svg's. Exits 0 iff all stations
// allocated and hard_constraints_satisfied on the LNS best.
//
// What this shows beyond the cold solve alone:
//   - The transport-distance term moves stations measurably toward
//     what they exchange items with (vs. just sitting at the prior
//     nominal).
//   - The LNS outer loop is the seam where richer destroy operators,
//     non-smooth objectives, and the StationFootprint cache will
//     plug in next; for the current single-station + smooth NLP setup
//     the loop confirms the cold solve is at a local optimum.

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <map>
#include <mp-units/systems/si.h>
#include <span>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

#include <tinycell/adapters/boost_conv.hpp>
#include <tinycell/io/catalog_loader.hpp>
#include <tinycell/model/arm.hpp>
#include <tinycell/model/port.hpp>
#include <tinycell/model/pusher.hpp>
#include <tinycell/model/task.hpp>
#include <tinycell/solver/allocator.hpp>
#include <tinycell/solver/anchor_strategy.hpp>
#include <tinycell/solver/arm_strategy.hpp>
#include <tinycell/solver/enumerator.hpp>
#include <tinycell/solver/layout_problem.hpp>
#include <tinycell/solver/lns.hpp>
#include <tinycell/solver/positional_prior.hpp>
#include <tinycell/solver/pusher_strategy.hpp>
#include <tinycell/solver/transfer_strategy.hpp>
#include <tinycell/svg.hpp>

namespace {

using namespace mp_units;
using mp_units::si::metre;
using mp_units::si::radian;
using mp_units::si::second;
using mp_units::si::kilogram;
namespace tc = tinycell::core;
namespace ts = tinycell::solver;
namespace io = tinycell::io;
namespace svg = tinycell::svg;
namespace adapters = tinycell::adapters;

// ---- workflow construction ----------------------------------------------

tc::Task make_palletize(const std::string& id,
                        double item_mass_kg,
                        double pallet_w_m, double pallet_l_m,
                        int box_count, double target_s_per_item) {
    return tc::Task{
        .id = id,
        .params = tc::PalletizeParams{
            .item_id = "box",
            .item = tc::BoxSpec{
                .physical = tc::ItemPhysical{
                    .width = 0.3 * metre, .length = 0.4 * metre,
                    .height = 0.2 * metre, .mass = item_mass_kg * kilogram,
                    .symmetry = tc::symmetry::discrete(180),
                },
            },
            .pallet = tc::PalletSpec{
                .physical = tc::ItemPhysical{
                    .width = pallet_w_m * metre, .length = pallet_l_m * metre,
                    .height = 0.15 * metre, .mass = 25.0 * kilogram,
                    .symmetry = tc::symmetry::discrete(180),
                },
            },
            .box_count = box_count,
        },
        .target_ct_per_item = tc::CycleTimePerItem{target_s_per_item * second},
    };
}

tc::Task make_feeder(const std::string& id, double world_x, double world_y) {
    return tc::Task{
        .id = id,
        .params = tc::AnchorParams{
            .name = id,
            .role = tc::PortDirection::Output,  // emits items into the cell
            .world_x = world_x * metre,
            .world_y = world_y * metre,
            .world_theta = 0.0 * radian,
        },
        .target_ct_per_item = tc::CycleTimePerItem{1.0 * second},
    };
}

tc::Task make_dispatch(const std::string& id, double world_x, double world_y) {
    return tc::Task{
        .id = id,
        .params = tc::AnchorParams{
            .name = id,
            .role = tc::PortDirection::Input,  // receives full pallets leaving the cell
            .world_x = world_x * metre,
            .world_y = world_y * metre,
            .world_theta = 0.0 * radian,
        },
        .target_ct_per_item = tc::CycleTimePerItem{1.0 * second},
    };
}

tc::Task make_transport(const std::string& id,
                        const std::string& src_task, const std::string& src_port,
                        const std::string& dst_task, const std::string& dst_port,
                        double target_s_per_item) {
    return tc::Task{
        .id = id,
        .params = tc::TransportParams{
            .source_task_id = src_task,
            .source_port_name = src_port,
            .sink_task_id = dst_task,
            .sink_port_name = dst_port,
        },
        .target_ct_per_item = tc::CycleTimePerItem{target_s_per_item * second},
    };
}

// ---- catalog lookup -----------------------------------------------------

const tc::ArmSpec* find_arm(const std::vector<tc::ArmSpec>& arms, const std::string& id) {
    auto it = std::find_if(arms.begin(), arms.end(),
                           [&](const tc::ArmSpec& a) { return a.id == id; });
    return it == arms.end() ? nullptr : &*it;
}
const tc::PusherSpec* find_pusher(const std::vector<tc::PusherSpec>& pushers, const std::string& id) {
    auto it = std::find_if(pushers.begin(), pushers.end(),
                           [&](const tc::PusherSpec& p) { return p.id == id; });
    return it == pushers.end() ? nullptr : &*it;
}

// ---- geometry helpers ---------------------------------------------------

// Bounding radius = max distance from station origin to any vertex of
// the (buffered) polygon. Used by the broad-phase non-overlap encoder.
double bounding_radius_m(const tc::Polygon& p) {
    double r2_max = 0.0;
    for (const auto& v : p) {
        const double vx = v.x.numerical_value_in(metre);
        const double vy = v.y.numerical_value_in(metre);
        r2_max = std::max(r2_max, vx * vx + vy * vy);
    }
    return std::sqrt(r2_max);
}

// Look up the catalog footprint for a bound instance. Branches on
// strategy_name to pick the right catalog.
tc::Polygon lookup_footprint(const ts::BoundInstance& bi,
                             const std::vector<tc::ArmSpec>& arms,
                             const std::vector<tc::PusherSpec>& pushers) {
    if (bi.strategy_name == "ArmStrategy") {
        const auto* spec = find_arm(arms, bi.catalog_id);
        if (!spec) throw std::runtime_error("unknown arm: " + bi.catalog_id);
        return spec->footprint;
    }
    if (bi.strategy_name == "PusherStrategy") {
        const auto* spec = find_pusher(pushers, bi.catalog_id);
        if (!spec) throw std::runtime_error("unknown pusher: " + bi.catalog_id);
        return spec->footprint;
    }
    throw std::runtime_error("unknown strategy: " + bi.strategy_name);
}

// Centroid of nominal positions for the served palletize tasks. A
// shared instance lands "between" the tasks it serves; an instance
// with one served task lands at that task's nominal. Residual sub-
// tasks inherit their primary's nominal (positional_prior didn't see
// them) - sibling instances seed coincident, the overlap penalty
// pushes them apart (see "what happens if a robot arm returns
// partial" design note).
tc::Vec2 instance_nominal(const ts::BoundInstance& bi,
                          const std::map<std::string, tc::Vec2>& nominal_for_task) {
    double sx = 0.0, sy = 0.0;
    int count = 0;
    for (const auto& s : bi.served) {
        // Drop "_residual" suffix to inherit the primary's nominal.
        std::string lookup_id = s.task_id;
        const auto suffix = std::string("_residual");
        if (lookup_id.size() > suffix.size() &&
            lookup_id.compare(lookup_id.size() - suffix.size(), suffix.size(), suffix) == 0) {
            lookup_id.erase(lookup_id.size() - suffix.size());
        }
        auto it = nominal_for_task.find(lookup_id);
        if (it == nominal_for_task.end()) continue;
        sx += it->second.x.numerical_value_in(metre);
        sy += it->second.y.numerical_value_in(metre);
        ++count;
    }
    if (count == 0) return tc::Vec2{0.0 * metre, 0.0 * metre};
    return tc::Vec2{(sx / count) * metre, (sy / count) * metre};
}

// ---- LayoutProblem construction -----------------------------------------

// Source of each StationProblem in the LayoutProblem - lets the SVG
// drawing code know whether a given solved pose came from a bound
// instance (draw footprint + reach) or a pinned anchor (draw marker).
enum class StationSourceKind { Instance, Anchor };
struct StationSource {
    StationSourceKind kind;
    std::size_t source_index;   // index into allocation.instances or .anchors
};

struct LayoutBuildResult {
    ts::LayoutProblem problem;
    std::vector<StationSource> sources;
    std::map<std::string, std::size_t> task_to_station;  // task_id -> station index
};

// Look up the winning StrategyResult's PortConstraint for (task_id,
// port_name). Returns nullptr if either the task wasn't enumerated, has
// no winner, or the winner didn't emit that port. Walked once per
// TransportEdge endpoint at build time; not on the LNS inner-loop path.
const tc::PortConstraint* find_port_constraint(
    std::span<const ts::TaskEnumeration> enumeration,
    const std::string& task_id,
    const std::string& port_name)
{
    for (const auto& te : enumeration) {
        if (te.task.id != task_id) continue;
        if (!te.winner_index.has_value()) return nullptr;
        const auto& winner = te.proposals[*te.winner_index];
        for (const auto& pc : winner.port_constraints) {
            if (pc.port_name == port_name) return &pc;
        }
        return nullptr;
    }
    return nullptr;
}

LayoutBuildResult build_layout_problem(
    const ts::AllocationResult& alloc,
    std::span<const ts::TaskEnumeration> enumeration,
    const std::vector<tc::ArmSpec>& arms,
    const std::vector<tc::PusherSpec>& pushers,
    const std::map<std::string, tc::Vec2>& nominal_for_task,
    const ts::Floor& floor,
    const ts::ObjectiveWeights& weights,
    double clearance_m)
{
    LayoutBuildResult result;
    result.problem.floor = floor;
    result.problem.weights = weights;

    // Instances first, then anchors.
    for (std::size_t i = 0; i < alloc.instances.size(); ++i) {
        const auto& bi = alloc.instances[i];
        const auto fp = lookup_footprint(bi, arms, pushers);
        const auto buffered = adapters::buffer_outward(fp, clearance_m * metre);
        const double r = bounding_radius_m(buffered);
        const auto nom = instance_nominal(bi, nominal_for_task);

        result.problem.stations.push_back(ts::StationProblem{
            .id = "instance_" + std::to_string(bi.id),
            .buffered_hull = buffered,
            .bounding_radius = r * metre,
            .nominal = nom,
            .initial_pose = tc::Pose2D{
                .x = nom.x, .y = nom.y,
                .theta = 0.0 * radian, .frame = tc::kWorldFrame,
            },
            .frozen = false,
        });
        result.sources.push_back({StationSourceKind::Instance, i});

        const std::size_t station_idx = result.problem.stations.size() - 1;
        for (const auto& s : bi.served) {
            result.task_to_station[s.task_id] = station_idx;
        }
    }

    for (std::size_t i = 0; i < alloc.anchors.size(); ++i) {
        const auto& a = alloc.anchors[i];
        result.problem.stations.push_back(ts::StationProblem{
            .id = "anchor_" + a.task_id,
            .buffered_hull = {},
            .bounding_radius = 0.0 * metre,
            .nominal = tc::Vec2{a.world_x, a.world_y},
            .initial_pose = tc::Pose2D{
                .x = a.world_x, .y = a.world_y,
                .theta = a.world_theta, .frame = tc::kWorldFrame,
            },
            .frozen = true,
        });
        result.sources.push_back({StationSourceKind::Anchor, i});
        result.task_to_station[a.task_id] = result.problem.stations.size() - 1;
    }

    // Transport constraints: one per TransportEdge in the allocation.
    // Each endpoint's port-local position comes from the corresponding
    // winning strategy's PortConstraint, looked up in the enumeration.
    // Both endpoints MUST resolve — a Transport task whose source or
    // sink wasn't enumerated, has no winner, or whose winner doesn't
    // declare the named port, indicates a workflow / strategy mismatch
    // that the LayoutProblem cannot meaningfully score. Skip these
    // (rather than fabricate a (0, 0) port) and warn; workflow
    // validation will become the proper enforcement point when it lands.
    for (const auto& edge : alloc.transports) {
        const auto src_st = result.task_to_station.find(edge.source_task_id);
        const auto dst_st = result.task_to_station.find(edge.sink_task_id);
        if (src_st == result.task_to_station.end() ||
            dst_st == result.task_to_station.end()) {
            std::cerr << "warning: transport '" << edge.task_id
                      << "' references unknown task; skipped\n";
            continue;
        }
        const auto* src_pc = find_port_constraint(
            enumeration, edge.source_task_id, edge.source_port_name);
        const auto* dst_pc = find_port_constraint(
            enumeration, edge.sink_task_id, edge.sink_port_name);
        if (src_pc == nullptr || dst_pc == nullptr) {
            std::cerr << "warning: transport '" << edge.task_id
                      << "' has no PortConstraint on source or sink; skipped\n";
            continue;
        }
        result.problem.transports.push_back(ts::TransportConstraint{
            .source_station = src_st->second,
            .source_port_local = tc::Vec2{src_pc->x, src_pc->y},
            .sink_station = dst_st->second,
            .sink_port_local = tc::Vec2{dst_pc->x, dst_pc->y},
        });
    }
    return result;
}

// ---- SVG drawing --------------------------------------------------------

void draw_layout_svg(const ts::LayoutSolution& solution,
                     const ts::LayoutProblem& problem,
                     const std::vector<StationSource>& sources,
                     const ts::AllocationResult& alloc,
                     const std::vector<tc::ArmSpec>& arms,
                     const std::filesystem::path& out_path) {
    // viewBox: floor extended by 10 % margin.
    const double xmin = problem.floor.x_min.numerical_value_in(metre);
    const double xmax = problem.floor.x_max.numerical_value_in(metre);
    const double ymin = problem.floor.y_min.numerical_value_in(metre);
    const double ymax = problem.floor.y_max.numerical_value_in(metre);
    const double margin = 0.1 * std::max(xmax - xmin, ymax - ymin);

    const double view_x = xmin - margin;
    const double view_y = ymin - margin;
    const double view_w = (xmax - xmin) + 2 * margin;
    const double view_h = (ymax - ymin) + 2 * margin;
    const double page_w_mm = 600.0;
    const double page_h_mm = page_w_mm * (view_h / view_w);

    svg::Writer w(page_w_mm, page_h_mm, view_x, view_y, view_w, view_h);

    // Floor outline.
    w.polygon(tc::Polygon{
                  tc::Vec2{xmin * metre, ymin * metre},
                  tc::Vec2{xmax * metre, ymin * metre},
                  tc::Vec2{xmax * metre, ymax * metre},
                  tc::Vec2{xmin * metre, ymax * metre}},
              "rgba(0,0,0,0.03)", "rgba(0,0,0,0.40)", 0.05);

    // Transport lines underneath stations so they don't obscure
    // footprints. Endpoints are PORT world poses (station pose ∘ port
    // local), not station centres — this is the geometry the Phase-1
    // transport-distance objective scores against.
    for (const auto& tr : problem.transports) {
        const auto& src_pose  = solution.station_poses[tr.source_station];
        const auto& dst_pose  = solution.station_poses[tr.sink_station];
        const tc::Transform2D t_src{src_pose.x, src_pose.y, src_pose.theta};
        const tc::Transform2D t_dst{dst_pose.x, dst_pose.y, dst_pose.theta};
        const auto src_world = tc::apply(t_src, tr.source_port_local);
        const auto dst_world = tc::apply(t_dst, tr.sink_port_local);
        w.polygon(tc::Polygon{src_world, dst_world},
                  "none", "rgba(0,80,200,0.55)", 0.04);
        // Length label at the midpoint.
        const double dx = (dst_world.x - src_world.x).numerical_value_in(metre);
        const double dy = (dst_world.y - src_world.y).numerical_value_in(metre);
        const double len_m = std::sqrt(dx * dx + dy * dy);
        char buf[32];
        std::snprintf(buf, sizeof(buf), "%.2f m", len_m);
        w.text(tc::Vec2{(src_world.x + dst_world.x) * 0.5,
                        (src_world.y + dst_world.y) * 0.5},
               buf, 0.18, "rgba(0,80,200,0.85)");
    }

    // Stations: per source kind.
    for (std::size_t i = 0; i < problem.stations.size(); ++i) {
        const auto& station = problem.stations[i];
        const auto& pose = solution.station_poses[i];
        const auto& source = sources[i];

        if (source.kind == StationSourceKind::Anchor) {
            // Anchor marker: filled green circle.
            w.circle(tc::Vec2{pose.x, pose.y}, 0.25 * metre,
                     "rgba(0,160,0,0.80)", "rgba(0,80,0,1.0)", 0.04);
            w.text(tc::Vec2{pose.x + 0.30 * metre, pose.y + 0.30 * metre},
                   station.id, 0.30);
            continue;
        }

        // Instance: transform the buffered hull to world coords + draw.
        const tc::Transform2D t_world{pose.x, pose.y, pose.theta};
        const auto world_fp = tc::apply(t_world, station.buffered_hull);
        const auto& bi = alloc.instances[source.source_index];

        // For arms, draw reach envelope underneath the footprint.
        if (bi.strategy_name == "ArmStrategy") {
            const auto* spec = find_arm(arms, bi.catalog_id);
            if (spec != nullptr) {
                w.ring(tc::Vec2{pose.x, pose.y},
                       spec->reach.min_radius,
                       spec->reach.max_radius.value());
            }
            w.polygon(world_fp, "rgba(64,64,64,0.25)", "black", 0.04);
        } else {
            // Pusher: green-tinted fill.
            w.polygon(world_fp, "rgba(0,128,0,0.20)", "rgba(0,128,0,0.80)", 0.04);
        }

        w.text(tc::Vec2{pose.x + 0.25 * metre, pose.y + 0.35 * metre},
               station.id + ":" + bi.catalog_id, 0.25);
    }

    // World-origin crosshair for orientation.
    w.polygon(tc::Polygon{
                  tc::Vec2{-0.5 * metre, 0.0 * metre},
                  tc::Vec2{0.5 * metre, 0.0 * metre}},
              "none", "rgba(0,0,0,0.30)", 0.02);
    w.polygon(tc::Polygon{
                  tc::Vec2{0.0 * metre, -0.5 * metre},
                  tc::Vec2{0.0 * metre, 0.5 * metre}},
              "none", "rgba(0,0,0,0.30)", 0.02);

    w.write_to_file(out_path);
}

} // namespace

int main() {
    const auto repo = std::filesystem::path(TINYCELL_REPO_ROOT);
    const auto out_dir = std::filesystem::path(TINYCELL_DEMO_OUTPUT_DIR);
    std::error_code ec;
    std::filesystem::create_directories(out_dir, ec);
    if (ec) {
        std::cerr << "Failed to create output dir " << out_dir
                  << ": " << ec.message() << "\n";
        return 1;
    }

    const auto arms = io::load_arm_catalog(
        repo / "assets" / "arm" / "kuka" / "catalog.json");
    const auto pushers = io::load_pusher_catalog(
        repo / "assets" / "pusher" / "generic" / "catalog.json");

    ts::ArmStrategy arm_strategy(arms);
    ts::PusherStrategy pusher_strategy(pushers);
    ts::TransferStrategy transfer_strategy;
    ts::AnchorStrategy anchor_strategy;
    const std::vector<const ts::Strategy*> strategies{
        &arm_strategy, &pusher_strategy, &transfer_strategy, &anchor_strategy};

    const std::vector<tc::Task> workflow{
        // Anchors: feeder pinned at world origin, dispatch pinned at
        // (20, 0). The prior pinning matches the workflow geometry.
        make_feeder("t_feeder",     0.0,  0.0),
        make_dispatch("t_dispatch", 20.0, 0.0),
        // Palletize tasks.
        make_palletize("t_pallet_a",     5.0, 1.2, 0.8, 24, 5.0),
        make_palletize("t_pallet_b",     5.0, 1.2, 0.8, 24, 5.0),
        make_palletize("t_pallet_tight", 8.0, 2.0, 2.0, 12, 2.0),
        // Item flow: feeder -> each palletize's item_in.
        make_transport("t_xport_in_a",     "t_feeder", "port", "t_pallet_a",     "item_in", 5.0),
        make_transport("t_xport_in_b",     "t_feeder", "port", "t_pallet_b",     "item_in", 5.0),
        make_transport("t_xport_in_tight", "t_feeder", "port", "t_pallet_tight", "item_in", 2.0),
        // Pallet flow: each palletize's pallet_out -> dispatch.
        // target_ct_per_item is per pallet here (= cycle_time of the
        // palletize task); magical TransferStrategy beats any positive
        // target, so the value just needs to be > 0.
        make_transport("t_xport_out_a",     "t_pallet_a",     "pallet_out", "t_dispatch", "port", 120.0),
        make_transport("t_xport_out_b",     "t_pallet_b",     "pallet_out", "t_dispatch", "port", 120.0),
        make_transport("t_xport_out_tight", "t_pallet_tight", "pallet_out", "t_dispatch", "port",  24.0),
    };

    const auto enumeration = ts::enumerate(workflow, strategies);
    const auto allocation = ts::allocate(enumeration);

    if (!allocation.unallocated.empty()) {
        std::cerr << "Unallocated tasks:";
        for (const auto& u : allocation.unallocated) std::cerr << ' ' << u;
        std::cerr << "\n";
        return 2;
    }

    // Positional prior over the PALLETIZE tasks only (Anchors are
    // pinned; Transports are edges). Input anchor = feeder; output
    // anchor at (20, 0) - notional dispatch point used only as the
    // prior's pinning, not a workflow task at MVP.
    std::vector<tc::Task> palletize_only;
    for (const auto& t : workflow) {
        if (t.kind() == tc::TaskKind::Palletize) palletize_only.push_back(t);
    }
    const auto prior = ts::positional_prior(palletize_only,
        tc::Vec2{0.0 * metre, 0.0 * metre},
        tc::Vec2{20.0 * metre, 0.0 * metre});
    std::map<std::string, tc::Vec2> nominal_for_task;
    for (const auto& e : prior.entries) nominal_for_task[e.task_id] = e.nominal;

    // Floor: enough room for the palletizer arms (3.2 m reach) at the
    // far end without colliding with the feeder at world origin.
    const ts::Floor floor{
        .x_min = -5.0 * metre, .x_max = 25.0 * metre,
        .y_min = -10.0 * metre, .y_max = 10.0 * metre};
    const ts::ObjectiveWeights weights{
        .overlap = 1000.0, .floor = 100.0, .positional_prior = 1.0};
    constexpr double kClearance_m = 0.2;   // placeholder, standards.md is a stub

    auto layout = build_layout_problem(
        allocation, enumeration, arms, pushers, nominal_for_task,
        floor, weights, kClearance_m);

    // CLI header.
    std::cout << "Workflow: " << workflow.size() << " task(s) -> "
              << enumeration.size() << " enumerated\n";
    std::cout << "Allocation: " << allocation.instances.size()
              << " bound instance(s), " << allocation.anchors.size()
              << " anchor(s), " << allocation.transports.size()
              << " transport edge(s)\n\n";

    // Cold solve -> "before". This is what every previous run of this
    // demo produced; serves as the LNS baseline.
    const auto cold = ts::solve(layout.problem);
    const auto print_cost = [](const char* label, const ts::ObjectiveBreakdown& b,
                               bool feasible) {
        std::cout << label
                  << "  total=" << b.total
                  << "  overlap=" << b.overlap
                  << "  floor=" << b.floor
                  << "  prior=" << b.prior
                  << "  transport=" << b.transport
                  << "  feasible=" << (feasible ? "yes" : "no") << "\n";
    };
    print_cost("[cold] ", cold.cost, cold.hard_constraints_satisfied);

    // LNS: SA-annealed destroy-and-repair. Auto temperature picks
    // T0 = cold_cost / 10; alpha = 0.9 cools to ~12% of T0 over 20
    // iters. Single-station deterministic re-solves on this smooth
    // 2D problem won't usually fire Metropolis (see lns.hpp), so
    // expect the LNS best to equal the cold solve here — the value
    // lands when destroy operators broaden or the objective gets
    // non-smooth.
    const ts::LnsParams params{
        .max_iterations = 20,
        .seed = 1,
        .temperature_initial = 0.0,
        .auto_temperature = true,
        .temperature_decay = 0.9,
    };
    const auto lns = ts::lns_solve(layout.problem, params);
    print_cost("[lns]  ", lns.best.cost, lns.best.hard_constraints_satisfied);
    std::cout << "  iterations=" << lns.trace.size()
              << "  accepted="
              << std::count_if(lns.trace.begin(), lns.trace.end(),
                               [](const ts::LnsIteration& it) { return it.accepted; })
              << "\n\n";

    for (std::size_t i = 0; i < layout.problem.stations.size(); ++i) {
        const auto& s = layout.problem.stations[i];
        const auto& p_cold = cold.station_poses[i];
        const auto& p_lns  = lns.best.station_poses[i];
        std::cout << "  " << s.id
                  << "  cold=(" << p_cold.x.numerical_value_in(metre)
                  << ", " << p_cold.y.numerical_value_in(metre) << ")"
                  << "  lns=(" << p_lns.x.numerical_value_in(metre)
                  << ", " << p_lns.y.numerical_value_in(metre) << ")"
                  << (s.frozen ? "  [frozen]" : "") << "\n";
    }

    // Artefacts: before.svg (cold), after.svg (LNS best), cost_trace.csv.
    // before/after pairs are diff-friendly — when LNS actually moves
    // a station, the diff is visible byte-for-byte at the changed
    // <polygon points="..."/>.
    const auto before_path = out_dir / "before.svg";
    const auto after_path  = out_dir / "after.svg";
    draw_layout_svg(cold, layout.problem, layout.sources,
                    allocation, arms, before_path);
    draw_layout_svg(lns.best, layout.problem, layout.sources,
                    allocation, arms, after_path);

    const auto csv_path = out_dir / "cost_trace.csv";
    std::ofstream csv(csv_path);
    csv << "iter,destroyed_station,temperature,current_total,proposed_total,"
        << "best_total,accepted\n";
    for (const auto& it : lns.trace) {
        csv << it.iter << ','
            << it.destroyed_station << ','
            << it.temperature << ','
            << it.current_total << ','
            << it.proposed_total << ','
            << it.best_total << ','
            << (it.accepted ? 1 : 0) << '\n';
    }

    std::cout << "\nWrote " << before_path.string() << "\n"
              << "Wrote " << after_path.string() << "\n"
              << "Wrote " << csv_path.string() << "\n";

    return lns.best.hard_constraints_satisfied ? 0 : 3;
}
