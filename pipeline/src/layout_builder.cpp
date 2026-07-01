// pipeline/build_layout_problem — extracted verbatim from
// demos/solve_workflow (post-MVP demo Phase 2) so the SVG demo and the threepp
// render app share one implementation. See layout_builder.hpp for the why.

#include <tinycell/pipeline/layout_builder.hpp>

#include <tinycell/adapters/boost_conv.hpp>   // convex_hull, buffer_outward
#include <tinycell/solver/station_template.hpp> // layout_palletizer_cell

#include <cmath>
#include <iostream>
#include <mp-units/systems/si.h>

namespace tinycell::pipeline {

namespace {

using namespace mp_units;
using mp_units::si::metre;
using mp_units::si::radian;
namespace tc       = tinycell::core;
namespace ts       = tinycell::solver;
namespace adapters = tinycell::adapters;

// Bounding radius = max distance from station origin to any vertex of the
// (buffered) polygon. Used by the broad-phase non-overlap encoder.
double bounding_radius_m(const tc::Polygon& p) {
    double r2_max = 0.0;
    for (const auto& v : p) {
        const double vx = v.x.numerical_value_in(metre);
        const double vy = v.y.numerical_value_in(metre);
        r2_max = std::max(r2_max, vx * vx + vy * vy);
    }
    return std::sqrt(r2_max);
}

// Domain-credible (NOT certified) safety clearance per equipment category, in
// metres. ~500 mm around an industrial robot reflects common ISO 10218-2 /
// ISO 13857 cell-integration practice, but these are ENGINEERING PLACEHOLDERS
// pending a safety engineer (standards.md is a stub; CLAUDE.md §5 forbids
// inventing certified numbers). They feed buffer_outward, so real per-rule
// values swap in later without structural change.
double clearance_for(const std::string& strategy_name) {
    if (strategy_name == "ArmStrategy") return 0.50;    // robot safeguarded space
    if (strategy_name == "PusherStrategy") return 0.30; // guarded linear actuator
    return 0.30;                                        // conservative default
}

// Look up the catalog footprint for a bound instance (branches on strategy).
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

// Centroid of nominal positions for the served palletize tasks. A shared
// instance lands "between" the tasks it serves; residual sub-tasks inherit
// their primary's nominal (drop the "_residual" suffix).
tc::Vec2 instance_nominal(const ts::BoundInstance& bi,
                          const std::map<std::string, tc::Vec2>& nominal_for_task) {
    double sx = 0.0, sy = 0.0;
    int    count = 0;
    for (const auto& s : bi.served) {
        std::string lookup_id = s.task_id;
        const auto  suffix    = std::string("_residual");
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

// Axis-aligned rectangle centred at (cx, cy), half-extents (hw, hl), CCW.
tc::Polygon rect(double cx, double cy, double hw, double hl) {
    return {
            tc::Vec2{(cx - hw) * metre, (cy - hl) * metre},
            tc::Vec2{(cx + hw) * metre, (cy - hl) * metre},
            tc::Vec2{(cx + hw) * metre, (cy + hl) * metre},
            tc::Vec2{(cx - hw) * metre, (cy + hl) * metre},
    };
}

// Accumulated convex-hull footprint of several station-frame polygons: collect
// every vertex, convex-hull them (feeds both the broad-phase bounding circle
// and the M4.2 SAT narrow phase). The tighter non-convex union is deferred.
tc::Polygon accumulate_hull(std::span<const tc::Polygon> parts) {
    tc::Polygon all;
    for (const auto& p : parts) all.insert(all.end(), p.begin(), p.end());
    return adapters::convex_hull(all);
}

// A pallet build-zone footprint (pallet + working margin) centred at a slot the
// intra-station template chose. Station-frame, axis-aligned.
constexpr double kPalletWorkMargin_m = 0.25;
tc::Polygon pallet_zone_at(const tc::Vec2& centre, const tc::PalletizeParams& p) {
    const double cx = centre.x.numerical_value_in(metre);
    const double cy = centre.y.numerical_value_in(metre);
    const double hw = 0.5 * p.pallet.physical.width.numerical_value_in(metre) + kPalletWorkMargin_m;
    const double hl = 0.5 * p.pallet.physical.length.numerical_value_in(metre) + kPalletWorkMargin_m;
    return rect(cx, cy, hw, hl);
}

// Find the PalletizeParams of a task by id (nullptr if not a Palletize).
const tc::PalletizeParams* find_palletize_params(
        std::span<const ts::TaskEnumeration> enumeration, const std::string& task_id) {
    for (const auto& te : enumeration) {
        if (te.task.id != task_id) continue;
        if (te.task.kind() != tc::TaskKind::Palletize) return nullptr;
        return &std::get<tc::PalletizeParams>(te.task.params);
    }
    return nullptr;
}

// Look up the winning StrategyResult's PortConstraint for (task_id, port_name).
const tc::PortConstraint* find_port_constraint(
        std::span<const ts::TaskEnumeration> enumeration,
        const std::string& task_id,
        const std::string& port_name) {
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

} // namespace

LayoutBuildResult build_layout_problem(
        const ts::AllocationResult&              alloc,
        std::span<const ts::TaskEnumeration>     enumeration,
        const std::vector<tc::ArmSpec>&          arms,
        const std::vector<tc::PusherSpec>&       pushers,
        const std::map<std::string, tc::Vec2>&   nominal_for_task,
        const ts::Floor&                         floor,
        const ts::ObjectiveWeights&              weights) {
    LayoutBuildResult result;
    result.problem.floor   = floor;
    result.problem.weights = weights;

    // Instances first, then anchors.
    for (std::size_t i = 0; i < alloc.instances.size(); ++i) {
        const auto& bi = alloc.instances[i];

        // M4 capstone: assemble the station's equipment in its station frame. A
        // palletizer (ArmStrategy) is the arm at the origin PLUS one pallet
        // build zone per served Palletize task (a dual-pallet cell serves two).
        std::vector<tc::Polygon> parts{lookup_footprint(bi, arms, pushers)};
        std::vector<tc::Polygon> zones; // this cell's pallet zones (for consumers)
        if (bi.strategy_name == "ArmStrategy") {
            const auto*              arm_spec = find_arm(arms, bi.catalog_id);
            std::vector<std::string> pallet_tasks;
            const tc::PalletizeParams* pp = nullptr;
            for (const auto& s : bi.served) {
                if (const auto* q = find_palletize_params(enumeration, s.task_id)) {
                    pallet_tasks.push_back(s.task_id);
                    if (pp == nullptr) pp = q; // shared arm => identical pallets
                }
            }
            if (arm_spec != nullptr && pp != nullptr && !pallet_tasks.empty()) {
                const auto cell = ts::layout_palletizer_cell(
                        arm_spec->reach.min_radius, arm_spec->reach.max_radius.value(),
                        pp->pallet.physical.width, pp->pallet.physical.length,
                        kPalletWorkMargin_m * metre, static_cast<int>(pallet_tasks.size()));
                if (!cell.feasible) {
                    result.cell_diagnostics.push_back(
                            "instance_" + std::to_string(bi.id) + " (" + bi.catalog_id + ", " +
                            std::to_string(pallet_tasks.size()) + " pallet(s)): " + cell.diagnostic);
                }
                for (std::size_t k = 0; k < cell.slots.size() && k < pallet_tasks.size(); ++k) {
                    auto zone = pallet_zone_at(cell.slots[k].centre, *pp);
                    parts.push_back(zone);
                    zones.push_back(std::move(zone));
                    result.task_pallet_slot[pallet_tasks[k]] = cell.slots[k].centre;
                }
            }
        }

        // Accumulated cell footprint (hull of all equipment), buffered by the
        // governing clearance — the most dangerous equipment (the robot) sets
        // the whole cell's keep-out.
        const auto   buffered = adapters::buffer_outward(
                accumulate_hull(parts), clearance_for(bi.strategy_name) * metre);
        const double r   = bounding_radius_m(buffered);
        const auto   nom = instance_nominal(bi, nominal_for_task);

        result.problem.stations.push_back(ts::StationProblem{
                .id              = "instance_" + std::to_string(bi.id),
                .buffered_hull   = buffered,
                .bounding_radius = r * metre,
                .nominal         = nom,
                .initial_pose    = tc::Pose2D{
                        .x = nom.x, .y = nom.y, .theta = 0.0 * radian, .frame = tc::kWorldFrame},
                .frozen = false,
        });
        result.sources.push_back({StationSourceKind::Instance, i});
        result.pallet_zones_local.push_back(std::move(zones));

        const std::size_t station_idx = result.problem.stations.size() - 1;
        for (const auto& s : bi.served) result.task_to_station[s.task_id] = station_idx;
    }

    for (std::size_t i = 0; i < alloc.anchors.size(); ++i) {
        const auto& a = alloc.anchors[i];
        result.problem.stations.push_back(ts::StationProblem{
                .id              = "anchor_" + a.task_id,
                .buffered_hull   = {},
                .bounding_radius = 0.0 * metre,
                .nominal         = tc::Vec2{a.world_x, a.world_y},
                .initial_pose    = tc::Pose2D{
                        .x = a.world_x, .y = a.world_y, .theta = a.world_theta, .frame = tc::kWorldFrame},
                .frozen = true,
        });
        result.sources.push_back({StationSourceKind::Anchor, i});
        result.pallet_zones_local.push_back({}); // anchors have no pallet zones
        result.task_to_station[a.task_id] = result.problem.stations.size() - 1;
    }

    // Transport constraints: one per TransportEdge. Both endpoints MUST resolve
    // (task known + winner emitted the named port); skip + warn otherwise
    // rather than fabricate a (0,0) port.
    for (const auto& edge : alloc.transports) {
        const auto src_st = result.task_to_station.find(edge.source_task_id);
        const auto dst_st = result.task_to_station.find(edge.sink_task_id);
        if (src_st == result.task_to_station.end() || dst_st == result.task_to_station.end()) {
            std::cerr << "warning: transport '" << edge.task_id
                      << "' references unknown task; skipped\n";
            continue;
        }
        const auto* src_pc =
                find_port_constraint(enumeration, edge.source_task_id, edge.source_port_name);
        const auto* dst_pc =
                find_port_constraint(enumeration, edge.sink_task_id, edge.sink_port_name);
        if (src_pc == nullptr || dst_pc == nullptr) {
            std::cerr << "warning: transport '" << edge.task_id
                      << "' has no PortConstraint on source or sink; skipped\n";
            continue;
        }
        // Effective port: a pallet_in / pallet_out endpoint lands on the actual
        // slot the intra-station layout assigned that task (distinct per pallet
        // in a dual-pallet cell); other ports keep the emitted position.
        auto effective_port = [&](const std::string& task_id, const std::string& port_name,
                                  const tc::PortConstraint* pc) -> tc::Vec2 {
            if (port_name == "pallet_in" || port_name == "pallet_out") {
                const auto it = result.task_pallet_slot.find(task_id);
                if (it != result.task_pallet_slot.end()) return it->second;
            }
            return tc::Vec2{pc->x, pc->y};
        };

        result.problem.transports.push_back(ts::TransportConstraint{
                .source_station    = src_st->second,
                .source_port_local = effective_port(edge.source_task_id, edge.source_port_name, src_pc),
                .sink_station      = dst_st->second,
                .sink_port_local   = effective_port(edge.sink_task_id, edge.sink_port_name, dst_pc),
                .source_reach_min  = src_pc->reach_min,
                .source_reach_max  = src_pc->reach_max,
                .sink_reach_min    = dst_pc->reach_min,
                .sink_reach_max    = dst_pc->reach_max,
        });
    }
    return result;
}

} // namespace tinycell::pipeline
