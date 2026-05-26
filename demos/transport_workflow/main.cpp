// INTENT: Phase T cleanup demo — exercise the transport-and-ports
// pipeline end-to-end on a workflow that wires together Anchor +
// Palletize + WORKFLOW-DECLARED Transport tasks.
//
// What it demonstrates:
//   * An Anchor task pinned to a world pose acts as a feeder for
//     the cell's items.
//   * Three Palletize tasks (one tight target that gets parallel
//     arms; two looser ones that share a pusher).
//   * Explicit Transport tasks in the workflow connect the feeder
//     to each palletize task's item_in port. The pusher's strict
//     PortConstraint (direction = stroke axis) restricts what a
//     placer can do with those Transports, but the topology itself
//     lives in the workflow, not in strategy preconditions
//     (cleanup-reverting-T.6 commit).
//   * The allocator emits BoundInstances (palletize stations),
//     TransportEdges (the workflow-declared Transports), and
//     PinnedAnchors (the feeder).
//
// Success: prints
//   * Per-task winners with strategy names + feasibilities
//   * Bound instances with served tasks
//   * Pinned anchors with world poses
//   * Transport edges with source/sink (task, port)
// Exit 0 means the pipeline ran cleanly with no unallocated tasks.

#include <algorithm>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <mp-units/systems/si.h>
#include <string>
#include <variant>
#include <vector>

#include <tinycell/io/catalog_loader.hpp>
#include <tinycell/model/port.hpp>
#include <tinycell/model/task.hpp>
#include <tinycell/solver/allocator.hpp>
#include <tinycell/solver/anchor_strategy.hpp>
#include <tinycell/solver/arm_strategy.hpp>
#include <tinycell/solver/enumerator.hpp>
#include <tinycell/solver/pusher_strategy.hpp>
#include <tinycell/solver/transfer_strategy.hpp>

namespace {

using namespace mp_units;
using mp_units::si::metre;
using mp_units::si::radian;
using mp_units::si::second;
using mp_units::si::kilogram;
namespace tc = tinycell::core;
namespace ts = tinycell::solver;
namespace io = tinycell::io;

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

tc::Task make_feeder(const std::string& id, const std::string& name,
                     double world_x, double world_y, double world_theta_rad) {
    return tc::Task{
        .id = id,
        .params = tc::AnchorParams{
            .name = name,
            .role = tc::PortDirection::Output,  // emits items into the cell
            .world_x = world_x * metre,
            .world_y = world_y * metre,
            .world_theta = world_theta_rad * radian,
        },
        // Anchors don't consume cycle time; CycleTimePerItem still
        // validates > 0, so any positive value works.
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

const char* feasibility_str(ts::Feasibility f) {
    switch (f) {
    case ts::Feasibility::FULL:       return "FULL";
    case ts::Feasibility::PARTIAL:    return "PARTIAL";
    case ts::Feasibility::INFEASIBLE: return "INFEASIBLE";
    }
    return "?";
}

} // namespace

int main() {
    const auto repo = std::filesystem::path(TINYCELL_REPO_ROOT);

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
        // Feeder pinned at world origin, emitting items along +X.
        make_feeder("t_feeder", "item_feeder", 0.0, 0.0, 0.0),
        // Same three palletize tasks the step-4 demo uses.
        make_palletize("t_pallet_a",     5.0, 1.2, 0.8, 24, 5.0),
        make_palletize("t_pallet_b",     5.0, 1.2, 0.8, 24, 5.0),
        make_palletize("t_pallet_tight", 8.0, 2.0, 2.0, 12, 2.0),
        // Workflow-declared Transports: feeder.port -> each palletize.item_in.
        // The transports' target_ct_per_item matches the sink palletize's
        // rate so the transfer is sized for the actual throughput.
        make_transport("t_xport_a",     "t_feeder", "port", "t_pallet_a",     "item_in", 5.0),
        make_transport("t_xport_b",     "t_feeder", "port", "t_pallet_b",     "item_in", 5.0),
        make_transport("t_xport_tight", "t_feeder", "port", "t_pallet_tight", "item_in", 2.0),
    };

    auto enumeration = ts::enumerate(workflow, strategies);

    std::cout << "Workflow tasks: " << workflow.size()
              << "  (enumerated incl. spawned residuals/transports: "
              << enumeration.size() << ")\n";
    std::cout << "Strategies: arm + pusher + transfer + anchor\n\n";

    std::cout << "Per-task winners:\n";
    std::cout << "  " << std::left << std::setw(28) << "task"
              << std::setw(16) << "kind"
              << std::setw(18) << "winner"
              << std::setw(11) << "feasibility" << "\n";
    for (const auto& te : enumeration) {
        std::string kind_str;
        switch (te.task.kind()) {
        case tc::TaskKind::Palletize: kind_str = "Palletize"; break;
        case tc::TaskKind::Transport: kind_str = "Transport"; break;
        case tc::TaskKind::Anchor:    kind_str = "Anchor"; break;
        }
        if (!te.winner_index) {
            std::cout << "  " << std::left << std::setw(28) << te.task.id
                      << std::setw(16) << kind_str
                      << std::setw(18) << "(no winner)"
                      << "\n";
            continue;
        }
        const auto& w = te.proposals[*te.winner_index];
        std::cout << "  " << std::left << std::setw(28) << te.task.id
                  << std::setw(16) << kind_str
                  << std::setw(18) << w.strategy_name
                  << std::setw(11) << feasibility_str(w.feasibility) << "\n";
    }

    const auto alloc = ts::allocate(enumeration);

    std::cout << "\nAllocation result:\n";
    std::cout << "  " << alloc.instances.size() << " bound instance(s)\n";
    for (const auto& inst : alloc.instances) {
        std::cout << "    instance " << inst.id << ": "
                  << std::left << std::setw(16) << inst.catalog_id
                  << " (" << inst.strategy_name << ") serves:";
        for (const auto& s : inst.served) std::cout << ' ' << s.task_id;
        std::cout << "\n";
    }

    std::cout << "  " << alloc.anchors.size() << " pinned anchor(s)\n";
    for (const auto& a : alloc.anchors) {
        std::cout << "    " << a.task_id << " (\"" << a.name << "\") at world ("
                  << a.world_x.numerical_value_in(metre) << ", "
                  << a.world_y.numerical_value_in(metre) << ", "
                  << a.world_theta.numerical_value_in(radian) << " rad)"
                  << "  role=" << (a.role == tc::PortDirection::Output ? "Output" : "Input")
                  << "\n";
    }

    std::cout << "  " << alloc.transports.size() << " transport edge(s)\n";
    for (const auto& e : alloc.transports) {
        std::cout << "    " << e.task_id << ": "
                  << e.source_task_id << "." << e.source_port_name
                  << " -> "
                  << e.sink_task_id << "." << e.sink_port_name
                  << "  (" << e.strategy_name << ")\n";
    }

    if (!alloc.unallocated.empty()) {
        std::cout << "  WARNING " << alloc.unallocated.size() << " unallocated task(s):";
        for (const auto& u : alloc.unallocated) std::cout << ' ' << u;
        std::cout << "\n";
        return 1;
    }

    return 0;
}
