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
#include <tinycell/model/task.hpp>
#include <tinycell/units.hpp>

namespace tinycell::solver {

// Feasibility of one strategy proposal:
//   FULL       — the strategy fully serves the task
//   PARTIAL    — the strategy serves part of it; a residual sub-task is
//                emitted as a precondition for the leftover work
//                (PLACEHOLDER (step 4): not produced by any strategy today;
//                requires the analytic throughput model — decisions.md)
//   INFEASIBLE — the strategy cannot serve the task at all
enum class Feasibility { FULL, PARTIAL, INFEASIBLE };

// EquipmentRef names a specific catalog entry by id. A strategy proposes
// this as a CANDIDATE binding, not a commitment — Layer 2 (allocation,
// step 4) decides whether one physical instance is reused across tasks.
// Committing inside the strategy would foreclose cross-task instance
// sharing (data-model.md §2, "candidate binding, not commitment").
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
// PLACEHOLDER (step 4): partial_info + preconditions are absent — they
// turn on with the analytic throughput model (decisions.md).
struct StrategyResult {
    Feasibility feasibility;
    std::string strategy_name;
    std::optional<EquipmentRef> equipment;
    core::Energy energy_per_cycle;
    core::Duration cycle_time;
    RequiresStateFn requires_state;
    EffectFn effect;
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
