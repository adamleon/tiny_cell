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

// RequiresStateFn — predicate over the inbound ItemState at the node where
// this strategy sits. Pure; called once per task by the propagation pass
// (state_propagation.hpp). Returning false FAILs the pass at this task.
using RequiresStateFn = std::function<bool(const core::ItemState&)>;

// EffectFn — ItemState → ItemState applied AFTER the task to produce the
// state the next task observes. Pure; no captured mutable state.
using EffectFn = std::function<core::ItemState(const core::ItemState&)>;

// StrategyResult — the output of one Strategy::evaluate() call.
//
// `requires_state` and `effect` are the state-propagation contract
// (data-model.md §4): the propagator checks the predicate against the
// inbound state, and applies the effect to produce the next state.
// Strategies SHOULD set both. INFEASIBLE results may leave them empty —
// the propagator only walks results the caller passes in (typically the
// per-task winners), so an INFEASIBLE result is not normally walked. If
// the propagator does encounter an empty std::function, that is a defect
// in the caller's selection logic, not a state-flow failure, and the
// propagator asserts (engineering.md §3).
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
