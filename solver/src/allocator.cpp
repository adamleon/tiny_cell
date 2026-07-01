#include "tinycell/solver/allocator.hpp"

#include <mp-units/systems/si.h>

namespace tinycell::solver {

namespace {

using namespace mp_units;
namespace tc = tinycell::core;

// Item-rate (items/sec) demanded by one served task, or delivered by
// one instance. Computed as 1 / time_per_item; using doubles here for
// the comparison since rate is dimensionally items/sec and mp-units
// doesn't carry the "items" dimension.
double rate_from_time_per_item(tc::Duration time_per_item) {
    return 1.0 / time_per_item.numerical_value_in(si::second);
}

// Does adding a new task with `new_target` per-item demand fit on this
// instance's spare capacity?  spare = 1/achievable − Σ(1/target_i for
// already-served tasks). Small numeric tolerance to avoid
// false-negatives from the inverse + sum arithmetic.
bool task_fits(const BoundInstance& inst, tc::Duration new_target) {
    constexpr double tolerance = 1e-9;
    double committed = 0.0;
    for (const auto& s : inst.served) {
        committed += rate_from_time_per_item(s.target_ct_per_item);
    }
    const double capacity = rate_from_time_per_item(inst.achievable_ct_per_item);
    const double needed = rate_from_time_per_item(new_target);
    return needed <= (capacity - committed) + tolerance;
}

} // namespace

AllocationResult allocate(std::span<const TaskEnumeration> per_task) {
    AllocationResult result;
    std::size_t next_id = 0;

    for (const auto& te : per_task) {
        if (!te.winner_index) {
            result.unallocated.push_back(te.task.id);
            continue;
        }
        const auto& winner = te.proposals[*te.winner_index];

        if (winner.feasibility == Feasibility::INFEASIBLE) {
            // pick_winner shouldn't select INFEASIBLE, but defensive:
            // route to unallocated rather than fabricate a binding.
            result.unallocated.push_back(te.task.id);
            continue;
        }

        // Transport tasks (T.2/T.4): the winner is TransferStrategy
        // (magical) or a future BeltStrategy; either way the task is
        // an EDGE, not a bound instance. Record the endpoints and move
        // on — no sharing logic at MVP because magical transfers don't
        // share equipment with each other (a BeltStrategy variant
        // would, but that lives in Layer-2 belt sharing later).
        if (te.task.kind() == tc::TaskKind::Transport) {
            const auto& p = std::get<tc::TransportParams>(te.task.params);
            result.transports.push_back(TransportEdge{
                .task_id = te.task.id,
                .source_task_id = p.source_task_id,
                .source_port_name = p.source_port_name,
                .sink_task_id = p.sink_task_id,
                .sink_port_name = p.sink_port_name,
                .strategy_name = winner.strategy_name,
            });
            continue;
        }

        // Anchor tasks (T.7): pinned world poses, no equipment to
        // bind. The placer reads these to fix the anchor "station's"
        // world coords (and refuses to optimise over them).
        if (te.task.kind() == tc::TaskKind::Anchor) {
            const auto& a = std::get<tc::AnchorParams>(te.task.params);
            result.anchors.push_back(PinnedAnchor{
                .task_id = te.task.id,
                .name = a.name,
                .world_x = a.world_x,
                .world_y = a.world_y,
                .world_theta = a.world_theta,
                .role = a.role,
            });
            continue;
        }

        // FULL and PARTIAL are both bindable. The PARTIAL primary's
        // residual is already in the enumerator output as a separate
        // TaskEnumeration with is_residual=true; iterating in DFS
        // order, it lands on its own instance because task_fits
        // rejects sharing with a PARTIAL primary (its committed rate
        // already exceeds the instance's capacity, so spare is
        // negative). The residual binds as a new instance — the
        // physical interpretation: two arms in parallel.
        if (!winner.equipment) {
            // A FULL result with no equipment AND not a Transport task
            // is a strategy bug, not the allocator's concern — route
            // to `unallocated` rather than fabricate a binding
            // (engineering.md §3, "never clamp / never invent").
            result.unallocated.push_back(te.task.id);
            continue;
        }

        const auto target = te.task.target_ct_per_item.value();
        BoundInstance::ServedTask st{
            .task_id = te.task.id,
            .target_ct_per_item = target,
            .achievable_ct_per_item = winner.achievable_ct_per_item,
            .cycle_time = winner.cycle_time,
            .energy_per_cycle = winner.energy_per_cycle,
        };

        // Try to share an existing instance with the same (catalog_id,
        // strategy_name) and spare capacity. First fit; the catalog
        // and result lists are both small at MVP scale.
        BoundInstance* shared = nullptr;
        for (auto& inst : result.instances) {
            if (inst.catalog_id != winner.equipment->catalog_id) continue;
            if (inst.strategy_name != winner.strategy_name) continue;
            if (task_fits(inst, target)) {
                shared = &inst;
                break;
            }
        }

        if (shared != nullptr) {
            shared->served.push_back(std::move(st));
        } else {
            result.instances.push_back(BoundInstance{
                .id = next_id++,
                .catalog_id = winner.equipment->catalog_id,
                .strategy_name = winner.strategy_name,
                .achievable_ct_per_item = winner.achievable_ct_per_item,
                .served = {std::move(st)},
            });
        }
    }

    return result;
}

} // namespace tinycell::solver
