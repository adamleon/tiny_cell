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
// PARTIAL chain integration: the enumerator walks PARTIAL chains
// before the allocator runs, surfacing residual TaskEnumerations
// (with is_residual=true) in the input span. The allocator binds
// PARTIAL winners the same way as FULL — the difference is that a
// PARTIAL primary's per-task demand exceeds the instance's capacity,
// so task_fits rejects any subsequent sharing on that instance and
// the residual (which appears next in DFS order) binds to its own
// instance. The physical interpretation is "two units in parallel."
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
    // Per-task contribution of this instance. `achievable_ct_per_item`
    // is THIS task's achievable on this instance (the arm's per-item
    // time when serving this task), which can differ from the
    // instance's stored `achievable_ct_per_item` when tasks have
    // different pick distances on the same arm. Cost calc uses the
    // per-task value to compute utilization correctly.
    struct ServedTask {
        std::string task_id;
        core::Duration target_ct_per_item;
        core::Duration achievable_ct_per_item;
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
//   - No winner / INFEASIBLE → `unallocated`.
//   - FULL or PARTIAL winner → try to fit on an existing instance
//     with matching (catalog_id, strategy_name) and spare capacity
//     for this task's rate; otherwise mint a new instance. PARTIAL
//     primaries naturally fail the share check (their committed rate
//     exceeds capacity) so they always mint a fresh instance, and
//     their residuals (which follow in DFS order) likewise mint
//     their own.
AllocationResult allocate(std::span<const TaskEnumeration> per_task);

} // namespace tinycell::solver
