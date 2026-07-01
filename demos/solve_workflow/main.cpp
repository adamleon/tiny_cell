// INTENT: step-6 M4 capstone — a DUAL-PALLET robotic palletizing cell, the
// realistic configuration that makes the intra-station layout a genuine
// sub-problem (roadmap.md step-6 M4). ONE big arm (KUKA KR 120 PA, 3.2 m
// reach) serves TWO pallet build positions, alternating between them to hide
// pallet-change dead time. The two Palletize tasks are identical, so the
// allocator SHARES them onto a single arm — a station that physically holds
// the arm + two pallet zones. The intra-station template
// (solver/station_template) lays the two pallet positions out within the
// arm's reach and CHECKS they fit; the inter-station placer then sites the
// whole cell amid the material-flow anchors.
//
// (Earlier milestones this demo carried, now covered by tests + git history:
// M3 the two-line realistic plant; M4.1 the accumulated multi-equipment
// footprint; M4.2 the convex-polygon narrow-phase overlap. The strategy set
// is arm + transfer + anchor — a robot cell; arm-vs-pusher competition lives
// in the brute_force / layer_2 demos.)
//
// Pipeline:
//   1. validate_workflow(workflow)              (step-6 M3, net-new)
//      — port-mismatch / unconnected-input topology check; refuse to
//        solve on any Error (e.g. a pallet_in with no empty-pallet feed).
//   2. enumerate(workflow, strategies)          (steps 1-3, T.7)
//   3. allocate(enumeration)                    (step 4 + T.5)
//   4. build a LayoutProblem from the allocation
//      - one StationProblem per BoundInstance, seeded on its line's ROW
//        (heavy north / light south) so the two cells spread into two
//        rows rather than crowding the feeder→dispatch centreline
//      - each palletizer station is a MULTI-EQUIPMENT cell (step-6 M4):
//        arm + a real pallet-zone footprint, laid out at templated
//        station-frame offsets; the station's collision footprint is the
//        ACCUMULATED convex hull of both (the intra-station accumulated
//        footprint, data-model.md §3.1), buffered by clearance
//      - one StationProblem per PinnedAnchor, frozen at its world pose
//      - per-equipment-category DOMAIN-CREDIBLE clearance buffer
//        (~500 mm around robots — NOT certified; standards.md is a stub)
//   5. solve(layout_problem)                    (Phase D + E + F + M1)
//   6. route_belts(...)                         (step-6 M2)
//   7. write a world-coords SVG of the solved layout + belts
//
// Success / go-no-go: writes <build>/demos/solve_workflow/output/layout.svg
// that a person recognises as a two-line palletizing plant — two robot
// cells (footprint + reach ring), box/empty-pallet/full-pallet belts
// routing CLEAR of stations (the 2D-spacing test the old collinear
// scenario failed). Exits 0 iff the workflow validates, nothing is
// unallocated, and hard_constraints_satisfied is true; non-zero with a
// diagnostic otherwise. Belt collisions / unfitted spans are REPORTED,
// not failures (sequential placement; joint placement deferred).

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <iostream>
#include <map>
#include <mp-units/systems/si.h>
#include <span>
#include <stdexcept>
#include <string>
#include <variant>
#include <vector>

#include <tinycell/io/catalog_loader.hpp>
#include <tinycell/pipeline/layout_builder.hpp>
#include <tinycell/model/arm.hpp>
#include <tinycell/model/port.hpp>
#include <tinycell/model/pusher.hpp>
#include <tinycell/model/task.hpp>
#include <tinycell/solver/allocator.hpp>
#include <tinycell/solver/anchor_strategy.hpp>
#include <tinycell/solver/arm_strategy.hpp>
#include <tinycell/solver/belt_routing.hpp>
#include <tinycell/solver/enumerator.hpp>
#include <tinycell/solver/layout_objective.hpp>
#include <tinycell/solver/layout_problem.hpp>
#include <tinycell/solver/transfer_strategy.hpp>
#include <tinycell/solver/workflow_validation.hpp>
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
namespace pipeline = tinycell::pipeline;

// build_layout_problem + its result/source types moved to the pipeline library
// (shared with the render app, post-MVP Phase 2); bridge their names so the SVG
// + main code below reads unchanged.
using pipeline::build_layout_problem;
using pipeline::find_arm;
using pipeline::LayoutBuildResult;
using pipeline::StationSource;
using pipeline::StationSourceKind;

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

// ---- SVG drawing --------------------------------------------------------

void draw_layout_svg(const ts::LayoutSolution& solution,
                     const ts::LayoutProblem& problem,
                     const std::vector<StationSource>& sources,
                     const ts::AllocationResult& alloc,
                     const std::vector<tc::ArmSpec>& arms,
                     std::span<const ts::PlacedBelt> belts,
                     const std::vector<std::vector<tc::Polygon>>& pallet_zones,
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

    // Belts (step-6 M2): each transport routed as a real placed conveyor,
    // drawn underneath stations so a belt-through-station collision shows the
    // belt footprint poking into the station. Colour encodes status: blue =
    // fitted & clear, red = collides with a station. An unfitted transport
    // (no catalog belt covers the span) has no footprint — just its orange
    // port-to-port centreline. The belt endpoints ARE the SOLVED port world
    // positions (route_belts composes station pose ∘ port local).
    for (const auto& b : belts) {
        if (b.fitted) {
            const bool hit = b.collides_with_station;
            w.polygon(ts::belt_footprint(b.start, b.end, b.width),
                      hit ? "rgba(220,40,40,0.20)" : "rgba(0,120,200,0.16)",
                      hit ? "rgba(200,0,0,0.75)" : "rgba(0,120,200,0.55)",
                      0.03);
        }
        // Centreline + length label (the abstract port-to-port edge).
        w.polygon(tc::Polygon{b.start, b.end}, "none",
                  b.fitted ? "rgba(0,80,200,0.55)" : "rgba(230,140,0,0.90)", 0.03);
        char buf[40];
        std::snprintf(buf, sizeof(buf), b.fitted ? "%.2f m" : "%.2f m (no belt)",
                      b.length.numerical_value_in(metre));
        w.text(tc::Vec2{(b.start.x + b.end.x) * 0.5, (b.start.y + b.end.y) * 0.5},
               buf, 0.18,
               b.fitted ? "rgba(0,80,200,0.85)" : "rgba(230,140,0,0.95)");
    }

    // Stations: per source kind.
    for (std::size_t i = 0; i < problem.stations.size(); ++i) {
        const auto& station = problem.stations[i];
        const auto& pose = solution.station_poses[i];
        const auto& source = sources[i];

        if (source.kind == StationSourceKind::Anchor) {
            // Anchor marker, coloured by material-flow role so the cell
            // reads at a glance: SOURCE anchors (Output — box feeder,
            // empty-pallet supply) green; SINK anchors (Input — dispatch)
            // amber. Both filled circles.
            const bool is_sink =
                alloc.anchors[source.source_index].role == tc::PortDirection::Input;
            w.circle(tc::Vec2{pose.x, pose.y}, 0.25 * metre,
                     is_sink ? "rgba(230,140,0,0.85)" : "rgba(0,160,0,0.80)",
                     is_sink ? "rgba(150,90,0,1.0)" : "rgba(0,80,0,1.0)", 0.04);
            w.text(tc::Vec2{pose.x + 0.30 * metre, pose.y + 0.30 * metre},
                   station.id, 0.30);
            continue;
        }

        // Instance: a multi-equipment palletizing CELL (M4). Draw bottom-up
        // so the parts read as one cell: reach ring, then the accumulated
        // buffered keep-out (the whole cell's footprint), then the reserved
        // pallet zone, then the equipment's own base footprint on top.
        const tc::Transform2D t_world{pose.x, pose.y, pose.theta};
        const auto& bi = alloc.instances[source.source_index];
        const auto* spec = (bi.strategy_name == "ArmStrategy")
                               ? find_arm(arms, bi.catalog_id)
                               : nullptr;

        // Arm reach envelope (underneath everything).
        if (spec != nullptr) {
            w.ring(tc::Vec2{pose.x, pose.y}, spec->reach.min_radius,
                   spec->reach.max_radius.value());
        }
        // Accumulated cell keep-out (buffered hull of all equipment).
        w.polygon(tc::apply(t_world, station.buffered_hull),
                  "rgba(0,0,0,0.04)", "rgba(0,0,0,0.35)", 0.03);
        // Reserved pallet build zones (M4 — the pallet positions as floor;
        // a dual-pallet cell has two). Each is labelled "pallet N".
        for (std::size_t z = 0; z < pallet_zones[i].size(); ++z) {
            const auto& zone = pallet_zones[i][z];
            if (zone.empty()) continue;
            w.polygon(tc::apply(t_world, zone),
                      "rgba(160,110,40,0.22)", "rgba(120,80,20,0.85)", 0.03);
            char lbl[16];
            std::snprintf(lbl, sizeof(lbl), "pallet %zu", z + 1);
            w.text(tc::apply(t_world, tc::Vec2{zone.front().x, zone.front().y}),
                   lbl, 0.18, "rgba(90,60,15,0.95)");
        }
        // The arm's own base plate, drawn on top so the robot base reads
        // distinctly from the accumulated cell keep-out. (A pusher cell —
        // none in this scenario — has no pallet zone, so its keep-out hull
        // above already IS its footprint; it gets no separate base layer,
        // rather than double-drawing the same buffered polygon.)
        if (spec != nullptr) {
            w.polygon(tc::apply(t_world, spec->footprint),
                      "rgba(64,64,64,0.55)", "black", 0.04);
        }

        w.text(tc::Vec2{pose.x + 0.25 * metre, pose.y + 0.35 * metre},
               station.id + ":" + bi.catalog_id, 0.25);
    }

    // Annulus port markers (step-6 M1): a filled dot at each variable
    // reach-annulus port's SOLVED world position, drawn on TOP of the
    // reach rings so you can see item_in landed OFF-origin inside its
    // arm's reach band — the visible payoff of making the port a placer
    // variable. Point ports (no reach band) get no marker.
    for (const auto& tr : solution.transports) {
        if (tr.source_reach_max.has_value()) {
            const auto& sp = solution.station_poses[tr.source_station];
            const tc::Transform2D t{sp.x, sp.y, sp.theta};
            w.circle(tc::apply(t, tr.source_port_local), 0.10 * metre,
                     "rgba(220,40,40,0.90)", "rgba(120,0,0,1.0)", 0.02);
        }
        if (tr.sink_reach_max.has_value()) {
            const auto& sp = solution.station_poses[tr.sink_station];
            const tc::Transform2D t{sp.x, sp.y, sp.theta};
            w.circle(tc::apply(t, tr.sink_port_local), 0.10 * metre,
                     "rgba(220,40,40,0.90)", "rgba(120,0,0,1.0)", 0.02);
        }
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
    const auto belt_catalog = io::load_belt_catalog(
        repo / "assets" / "belt" / "generic" / "catalog.json");

    // Robotic palletizing cell: arm + transfer + anchor only (pusher omitted
    // — this is a robot cell; arm-vs-pusher competition lives in the
    // brute_force / layer_2 demos). The pusher CATALOG is still loaded so
    // lookup_footprint stays general; it just has no instances here.
    ts::ArmStrategy arm_strategy(arms);
    ts::TransferStrategy transfer_strategy;
    ts::AnchorStrategy anchor_strategy;
    const std::vector<const ts::Strategy*> strategies{
        &arm_strategy, &transfer_strategy, &anchor_strategy};

    // ---- the scenario: a DUAL-PALLET robotic palletizing cell ----------
    // (step-6 M4 capstone) The textbook robotic palletizer: ONE big arm
    // (50 kg cases → KUKA KR 120 PA, 3.2 m reach) serving TWO pallet build
    // positions, alternating between them to hide pallet-change dead time.
    // The two Palletize tasks are IDENTICAL (same product), so the allocator
    // SHARES them onto one arm — a single cell that physically holds the arm
    // + two pallet zones. That makes the intra-station layout a real problem:
    // the two pallet positions must both sit within the arm's reach without
    // colliding (layout_palletizer_cell, the M4 capstone), not just a single
    // fixed offset. A box infeed feeds both, each pallet has its own
    // empty-pallet supply, and both full pallets ship to one dispatch.
    const std::vector<tc::Task> workflow{
        // Anchors (frozen world poses), laid out around the cell.
        make_feeder("t_feeder",    0.0,  0.0),   // box infeed
        make_feeder("t_empty_a",   9.0,  4.0),   // empty-pallet supply, pallet 1
        make_feeder("t_empty_b",   9.0, -4.0),   // empty-pallet supply, pallet 2
        make_dispatch("t_dispatch", 13.0, 0.0),  // full-pallet outfeed (shared)
        // Two IDENTICAL Palletize tasks → one shared KR 120 PA arm serving
        // two pallet positions. 50 kg forces the PA-class arm (only it has a
        // long enough reach to actually cover a full 1.2 × 0.8 m pallet — the
        // intra-station feasibility check enforces that); 12 s/box keeps a
        // clean single-arm FULL with capacity to share both.
        make_palletize("t_pal_1", 50.0, 1.2, 0.8, 24, 12.0),
        make_palletize("t_pal_2", 50.0, 1.2, 0.8, 24, 12.0),
        // Box flow: feeder -> each pallet task's item_in (the shared arm pick
        // annulus).
        make_transport("t_box_1", "t_feeder", "port", "t_pal_1", "item_in", 12.0),
        make_transport("t_box_2", "t_feeder", "port", "t_pal_2", "item_in", 12.0),
        // Empty-pallet flow: each pallet position fed from its own supply.
        make_transport("t_empty_in_1", "t_empty_a", "port", "t_pal_1", "pallet_in", 120.0),
        make_transport("t_empty_in_2", "t_empty_b", "port", "t_pal_2", "pallet_in", 120.0),
        // Full-pallet flow: both pallet_out -> shared dispatch (target is per
        // pallet; magical TransferStrategy beats any positive target).
        make_transport("t_full_1", "t_pal_1", "pallet_out", "t_dispatch", "port", 240.0),
        make_transport("t_full_2", "t_pal_2", "pallet_out", "t_dispatch", "port", 240.0),
    };

    // Validate workflow topology BEFORE solving (step-6 M3): catch
    // port-mismatch / unconnected-input gaps early with an actionable
    // diagnostic instead of silently producing a layout with missing
    // material edges. Errors block the solve; warnings are advisory.
    const auto validation = ts::validate_workflow(workflow);
    for (const auto& issue : validation.issues) {
        std::cerr << (issue.severity == ts::WorkflowIssueSeverity::Error
                          ? "  workflow ERROR: "
                          : "  workflow warning: ")
                  << issue.message << "\n";
    }
    if (!validation.ok()) {
        std::cerr << "Workflow validation failed ("
                  << validation.error_count() << " error(s)); refusing to solve.\n";
        return 4;
    }

    const auto enumeration = ts::enumerate(workflow, strategies);
    const auto allocation = ts::allocate(enumeration);

    if (!allocation.unallocated.empty()) {
        std::cerr << "Unallocated tasks:";
        for (const auto& u : allocation.unallocated) std::cerr << ' ' << u;
        std::cerr << "\n";
        return 2;
    }

    // Both Palletize tasks share one arm, so both nominals seed the SAME
    // cell; instance_nominal averages them. Seed the arm mid-corridor between
    // feeder and dispatch — the transport pulls (item_in toward the feeder,
    // the two pallet ports toward their supplies + the dispatch) refine it.
    const std::map<std::string, tc::Vec2> nominal_for_task{
        {"t_pal_1", tc::Vec2{6.0 * metre, 0.0 * metre}},
        {"t_pal_2", tc::Vec2{6.0 * metre, 0.0 * metre}},
    };

    // Floor: room for the PA arm's 3.2 m reach envelope + the material-flow
    // anchors around it.
    const ts::Floor floor{
        .x_min = -3.0 * metre, .x_max = 16.0 * metre,
        .y_min = -7.0 * metre, .y_max = 7.0 * metre};
    // Overlap weight stays stiff (M4.2): the narrow-phase polygon overlap is
    // a one-sided soft penalty, and a high weight keeps any inter-cell
    // contact just-clear rather than a few mm into penetration. (This
    // single-cell scenario has no inter-station overlap to resolve; the term
    // is exercised by the unit tests and the prior two-cell scenario.)
    const ts::ObjectiveWeights weights{
        .overlap = 20000.0, .floor = 100.0, .positional_prior = 1.0};

    auto layout = build_layout_problem(
        allocation, enumeration, arms, pushers, nominal_for_task,
        floor, weights);

    // Intra-station feasibility (M4 capstone): a cell whose pallets can't fit
    // the arm's reach without colliding is not physically realizable — refuse
    // rather than draw a fantasy layout (CLAUDE.md §1 — never fabricate past a
    // constraint).
    if (!layout.cell_diagnostics.empty()) {
        std::cerr << "Intra-station layout infeasible:\n";
        for (const auto& d : layout.cell_diagnostics) std::cerr << "  " << d << "\n";
        return 5;
    }

    auto solution = ts::solve(layout.problem);

    // CLI summary.
    std::cout << "Workflow: " << workflow.size() << " task(s) -> "
              << enumeration.size() << " enumerated\n";
    std::cout << "Allocation: " << allocation.instances.size()
              << " bound instance(s), " << allocation.anchors.size()
              << " anchor(s), " << allocation.transports.size()
              << " transport edge(s)\n\n";

    const auto& c = solution.cost;
    std::cout << "Solver result (hard_constraints_satisfied = "
              << (solution.hard_constraints_satisfied ? "yes" : "no") << ")\n"
              << "  total=" << c.total
              << "  overlap=" << c.overlap
              << "  floor=" << c.floor
              << "  prior=" << c.prior
              << "  transport=" << c.transport << "\n";
    for (std::size_t i = 0; i < layout.problem.stations.size(); ++i) {
        const auto& s = layout.problem.stations[i];
        const auto& p = solution.station_poses[i];
        std::cout << "  " << s.id
                  << "  pose=(" << p.x.numerical_value_in(metre)
                  << ", " << p.y.numerical_value_in(metre) << ")"
                  << "  nominal=(" << s.nominal.x.numerical_value_in(metre)
                  << ", " << s.nominal.y.numerical_value_in(metre) << ")"
                  << (s.frozen ? "  [frozen]" : "") << "\n";
    }

    // Intra-station layout diagnostic (M4 capstone): how many pallet build
    // positions each cell holds and where they landed in the station frame.
    // A cell with >1 pallet is the dual-pallet palletizer the intra-station
    // template laid out within the arm's reach.
    std::cout << "\nIntra-station layout (pallets per cell):\n";
    for (std::size_t i = 0; i < layout.problem.stations.size(); ++i) {
        if (layout.sources[i].kind != StationSourceKind::Instance) continue;
        const auto& zones = layout.pallet_zones_local[i];
        std::cout << "  " << layout.problem.stations[i].id << "  "
                  << zones.size() << " pallet zone(s)";
        for (const auto& zone : zones) {
            if (zone.empty()) continue;
            // Zone centre = mean of its 4 corners (station frame).
            double cx = 0.0, cy = 0.0;
            for (const auto& v : zone) {
                cx += v.x.numerical_value_in(metre);
                cy += v.y.numerical_value_in(metre);
            }
            cx /= static_cast<double>(zone.size());
            cy /= static_cast<double>(zone.size());
            std::cout << "  [r=" << std::sqrt(cx * cx + cy * cy) << "m]";
        }
        std::cout << "\n";
    }

    // Narrow-phase payoff (M4.2): when there is more than one equipment cell,
    // contrast what a bounding-CIRCLE overlap test would charge at the solved
    // poses against the actual polygon overlap — the proof the narrow phase
    // lets elongated cells pack tighter than a circle test would allow.
    bool any_pair = false;
    for (std::size_t i = 0; i < layout.problem.stations.size(); ++i) {
        if (layout.sources[i].kind != StationSourceKind::Instance) continue;
        for (std::size_t j = i + 1; j < layout.problem.stations.size(); ++j) {
            if (layout.sources[j].kind != StationSourceKind::Instance) continue;
            if (!any_pair) std::cout << "\nNarrow-phase payoff (cell pairs):\n";
            any_pair = true;
            const auto& pi = solution.station_poses[i];
            const auto& pj = solution.station_poses[j];
            const double ri = layout.problem.stations[i].bounding_radius.numerical_value_in(metre);
            const double rj = layout.problem.stations[j].bounding_radius.numerical_value_in(metre);
            const double dx = (pi.x - pj.x).numerical_value_in(metre);
            const double dy = (pi.y - pj.y).numerical_value_in(metre);
            const double dist = std::sqrt(dx * dx + dy * dy);
            const double circle = ts::overlap_penalty(
                pi, layout.problem.stations[i].bounding_radius,
                pj, layout.problem.stations[j].bounding_radius);
            std::cout << "  " << layout.problem.stations[i].id << " <-> "
                      << layout.problem.stations[j].id
                      << "  centre-dist=" << dist << "m"
                      << "  circle-overlap-penalty=" << circle
                      << "  (circles want >= " << (ri + rj)
                      << "m apart; polygons fit at " << dist << "m)\n";
        }
    }
    if (!any_pair) {
        std::cout << "\nNarrow-phase payoff: single equipment cell — no "
                     "inter-station overlap to resolve here (the narrow phase "
                     "is exercised by the unit tests).\n";
    }

    // Per-transport port diagnostic: which endpoints are variable
    // reach-annulus ports (step-6 M1) and where each landed relative to
    // its station origin (radius), so the band placement is legible in
    // the terminal, not just the SVG.
    auto port_radius = [](const tc::Vec2& v) {
        const double x = v.x.numerical_value_in(metre);
        const double y = v.y.numerical_value_in(metre);
        return std::sqrt(x * x + y * y);
    };
    std::cout << "\nTransport ports (solved):\n";
    for (std::size_t t = 0; t < solution.transports.size(); ++t) {
        const auto& tr = solution.transports[t];
        std::cout << "  xport[" << t << "]  src st" << tr.source_station
                  << (tr.source_reach_max.has_value()
                          ? "  ANNULUS r=" + std::to_string(port_radius(tr.source_port_local))
                          : "  point")
                  << "  ->  sink st" << tr.sink_station
                  << (tr.sink_reach_max.has_value()
                          ? "  ANNULUS r=" + std::to_string(port_radius(tr.sink_port_local))
                          : "  point")
                  << "\n";
    }

    // Belt routing (step-6 M2): turn each resolved transport into a real
    // placed conveyor — cheapest catalog belt covering the routed distance,
    // with belt-vs-station collision flagged (not repaired: sequential
    // placement, joint placement deferred — decisions.md).
    const auto belts = ts::route_belts(layout.problem, solution, belt_catalog);
    std::size_t unfitted = 0, colliding = 0;
    std::cout << "\nBelts (routed):\n";
    for (const auto& b : belts) {
        std::cout << "  belt[" << b.transport_index << "]  ";
        if (b.fitted) {
            std::cout << b.belt_catalog_id
                      << "  len=" << b.length.numerical_value_in(metre) << "m"
                      << "  width=" << b.width.numerical_value_in(metre) << "m";
            if (b.collides_with_station) {
                std::cout << "  *** COLLIDES with a station ***";
                ++colliding;
            }
        } else {
            std::cout << "NO CATALOG BELT FITS  (len="
                      << b.length.numerical_value_in(metre) << "m)";
            ++unfitted;
        }
        std::cout << "\n";
    }
    std::cout << "  -> " << belts.size() << " belt(s), " << colliding
              << " colliding, " << unfitted << " unfitted\n";

    const auto svg_path = out_dir / "layout.svg";
    draw_layout_svg(solution, layout.problem, layout.sources,
                    allocation, arms, belts, layout.pallet_zones_local, svg_path);
    std::cout << "\nWrote " << svg_path.string() << "\n";

    // Exit status: any unalloc was already an early-exit, here we
    // signal solver-side infeasibility too.
    return solution.hard_constraints_satisfied ? 0 : 3;
}
