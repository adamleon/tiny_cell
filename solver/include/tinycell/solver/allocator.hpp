#pragma once

// Layer-2 hand-rolled allocator (decisions.md "Hand-rolled allocator at
// Layer 2"). Behind an abstract solve() seam — no library types in the
// inputs or outputs. Takes the brute-force enumerator's per-task
// proposals and decides which catalog ids share physical instances,
// with cross-task sharing surfacing where capacity allows.
//
// Sharing is bin-packing on item-rate. Each task demands
//   r = 1 / target_ct_per_item items/sec.
// Each instance delivers
//   1 / achievable_ct_per_item items/sec.
// An instance can serve multiple tasks as long as Σ r_i ≤ 1 / achievable.
// Reach feasibility is a Layer-3 concern (decisions.md "2D geometry for
// MVP"); capacity is the only sharing constraint at Layer 2.
//
// PARTIAL chain walking is staged: today pick_winner inside the
// enumerator only selects FULL, so the allocator only sees FULL
// winners. Tasks whose only proposals are PARTIAL or INFEASIBLE end
// up in `unallocated`. Commit 9 extends both the enumerator (so it
// can pick PARTIAL when no FULL exists and walk residuals) and the
// allocator's handling of the PARTIAL branch.
//
// Cost reporting against the bound instance set (one BoundInstance =
// one physical equipment = one capex line) lands in commit 8.

#include <cstddef>
#include <span>
#include <string>
#include <tinycell/solver/enumerator.hpp>
#include <tinycell/units.hpp>
#include <vector>

namespace tinycell::solver {

// One physical equipment instance bound to one or more tasks. Two
// BoundInstances with the same catalog_id are TWO distinct physical
// units; sharing collapses what would have been multiple instances
// into one.
struct BoundInstance {
    // Per-task contribution of this instance.
    struct ServedTask {
        std::string task_id;
        core::Duration target_ct_per_item;
        core::Duration cycle_time;
        core::Energy energy_per_cycle;
    };

    std::size_t id;
    std::string catalog_id;
    std::string strategy_name;
    core::Duration achievable_ct_per_item;
    std::vector<ServedTask> served;
};

// AllocationResult — the allocator's output. `instances` holds bound
// physical units; `unallocated` holds task ids the allocator could not
// place (no FULL chain found).
struct AllocationResult {
    std::vector<BoundInstance> instances;
    std::vector<std::string> unallocated;
};

// allocate(): walk the per-task enumerations greedily, sharing where
// capacity allows.
//
//   - No winner → task goes to `unallocated`.
//   - FULL winner → try to fit on an existing instance with matching
//     (catalog_id, strategy_name) and spare capacity for this task's
//     rate; otherwise mint a new instance.
//   - PARTIAL winner → today routed to `unallocated` (placeholder);
//     commit 9 wires up the chain walk through `preconditions`.
AllocationResult allocate(std::span<const TaskEnumeration> per_task);

} // namespace tinycell::solver
