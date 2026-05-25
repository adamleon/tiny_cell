#pragma once

// Strategy interface and the data types it produces. A strategy encodes
// engineering knowledge for solving one *kind of work*; the solver iterates
// the registered strategies for each task, asking each "do you apply?" then
// "what would you cost?", and keeps the cheapest feasible result.
//
// Strategies are named for the EQUIPMENT TYPE only — ArmStrategy,
// PusherStrategy, ConveyorStrategy — never for a (type × task) combination
// (decisions.md). One strategy may cover multiple TaskKinds via applies_to.

#include <functional>
#include <optional>
#include <string>
#include <string_view>
#include <tinycell/model/item.hpp>
#include <tinycell/model/port.hpp>
#include <tinycell/model/task.hpp>
#include <tinycell/units.hpp>
#include <vector>

namespace tinycell::solver {

// Feasibility of one strategy proposal:
//   FULL       — the strategy fully serves the task at or below the task's
//                `target_ct_per_item`.
//   PARTIAL    — the strategy serves the task but slower than `target_ct_per_item`;
//                a residual sub-task with a tightened target is emitted as
//                a precondition for the leftover throughput. Produced today
//                by ArmStrategy only — PusherStrategy never emits PARTIAL
//                (decisions.md "Pusher strategies emit only FULL or INFEASIBLE").
//   INFEASIBLE — the strategy cannot serve the task at all.
enum class Feasibility { FULL, PARTIAL, INFEASIBLE };

// EquipmentRef names a specific catalog entry by id. A strategy proposes
// this as a CANDIDATE binding, not a commitment — the Layer-2 allocator
// (`solver/allocator.hpp`) decides whether one physical instance is reused
// across tasks. Committing inside the strategy would foreclose cross-task
// instance sharing (data-model.md §2, "candidate binding, not commitment").
struct EquipmentRef {
    std::string catalog_id;
};

// State-flow primitive: `requires_state` + `effect` are the load-bearing
// pair every state-aware solver consumer reads (data-model.md §4). A
// live solver threads state INLINE through its per-task loop — at each
// task, filter applicable strategies by `requires_state(s)` against the
// running state, pick a winner, then `s = winner.effect(s)` to produce
// the input for the next task. State is *input to selection*, not
// something validated after a whole candidate is built. The batch
// walker `propagate_state` (state_propagation.hpp) is a separate,
// validator-only consumer of these same two fields — used for finished
// candidates that didn't thread state, plus demo and tests. See
// decisions.md "State-flow primitive … is load-bearing; the batch
// walker … is a validator-only role."
//
// **Which axis each strategy reads** — the Knowledge/Control split on
// orientation (item.hpp) lets different strategies depend on different
// guarantees:
//   * Arm-style strategies plan their motion from the planner's
//     BELIEF and have end-effector compliance to absorb minor drift —
//     they read orientation.knowledge.
//   * Pusher-style strategies' strokes are OPEN-LOOP; they need the
//     item to be PHYSICALLY held in the right alignment — they read
//     orientation.control.
// `requires_state` is the seam; each strategy picks the axis its
// failure mode demands.

// RequiresStateFn — predicate over the inbound ItemState at the node
// where this strategy sits. Pure; no captured mutable state. Returning
// false means the strategy cannot run with the item in that state.
using RequiresStateFn = std::function<bool(const core::ItemState&)>;

// EffectFn — ItemState → ItemState applied AFTER the task to produce
// the state the next task observes. Pure; no captured mutable state.
using EffectFn = std::function<core::ItemState(const core::ItemState&)>;

// PartialInfo — the throughput gap a PARTIAL result reports. `achievable`
// is what this strategy actually produces on this task; `target` is what
// the task asked for. Both are per-item times so the comparison is
// direct (decisions.md "Throughput target stored as time_per_item …").
struct PartialInfo {
    core::Duration achievable_ct_per_item;
    core::Duration target_ct_per_item;
};

// StrategyResult — the output of one Strategy::evaluate() call.
//
// Strategies SHOULD always populate `requires_state` and `effect`,
// even on INFEASIBLE returns — they describe the strategy's state-flow
// contract, which is conceptually independent of whether a particular
// catalog selection succeeded. Empty std::function on a consumed result
// is a defect in the caller's selection logic, not a state-flow
// failure, and consumers (e.g. propagate_state) reject it rather than
// fabricate identity behaviour (engineering.md §3).
//
// `achievable_ct_per_item` is the per-item cycle time this strategy
// claims to deliver on this task. The Layer-2 allocator reads it to
// compute each instance's contribution to the task's required rate
// (decisions.md "Per-task throughput target"). On INFEASIBLE the field
// is 0 (matching the cycle_time / energy_per_cycle convention) — it is
// not meaningful and should not be read.
//
// `partial_info` is set iff feasibility == PARTIAL — it carries the
// achievable/target gap for diagnostics, redundant with the value on
// `Task` but co-located here so consumers don't have to look back at
// the task.
//
// `preconditions` carries residual sub-tasks the strategy emits as
// PARTIAL chaining. Empty unless feasibility == PARTIAL. Each residual
// has `is_residual = true` and a tightened target.
struct StrategyResult {
    Feasibility feasibility;
    std::string strategy_name;
    std::optional<EquipmentRef> equipment;
    core::Energy energy_per_cycle;
    core::Duration cycle_time;
    core::Duration achievable_ct_per_item;
    std::optional<PartialInfo> partial_info;
    std::vector<core::Task> preconditions;
    RequiresStateFn requires_state;
    EffectFn effect;

    // port_constraints — one PortConstraint per LogicalPort the task
    // declared (task_ports(task)), each naming where this strategy's
    // chosen equipment places that port + how strict the direction is.
    // Empty on INFEASIBLE — there's no binding to constrain. The
    // Layer-3 placer reads these to score / validate Transport edges
    // (Phase D). Arms emit wide-tolerance ports (any direction inside
    // their reach works); pushers emit strict-direction ports (stroke
    // axis is fixed by the equipment).
    std::vector<core::PortConstraint> port_constraints;
};

// Strategy — abstract base for all engineering-knowledge encoders. Concrete
// strategies inherit directly (no intermediate base classes).
//
// Contract:
//   * name() is a short identifier used in logging and StrategyResult.
//   * applies_to(task) is a pure, side-effect-free capability check. It is
//     called by the solver before evaluate() to filter strategies; callers
//     of evaluate() may rely on it having returned true.
//   * evaluate(task) computes the proposal for ONE task. Must not mutate
//     state shared between strategies or tasks (the solver may call it on
//     the same strategy across many tasks in any order).
class Strategy {
public:
    virtual ~Strategy() = default;

    virtual std::string_view name() const = 0;
    virtual bool applies_to(const core::Task& task) const = 0;
    virtual StrategyResult evaluate(const core::Task& task) const = 0;
};

} // namespace tinycell::solver
