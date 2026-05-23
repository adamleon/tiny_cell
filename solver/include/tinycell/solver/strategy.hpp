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

// Knowledge-flow primitive: `requires_knowledge` + `effect` are the
// load-bearing pair every knowledge-aware solver consumer reads
// (data-model.md §4). A live solver threads knowledge INLINE through its
// per-task loop — at each task, filter applicable strategies by
// `requires_knowledge(k)` against the running knowledge, pick a winner,
// then `k = winner.effect(k)` to produce the input for the next task.
// Knowledge is *input to selection*, not something validated after a
// whole candidate is built. The batch walker `propagate_knowledge`
// (knowledge_propagation.hpp) is a separate, validator-only consumer of
// these same two fields — used for finished candidates that didn't
// thread knowledge, plus demo and tests. See decisions.md "State-flow
// primitive … is load-bearing; the batch walker … is a validator-only
// role." (The decisions.md entry still uses the old "state-flow" name;
// it's the same primitive, renamed for honesty about what it tracks.)

// RequiresKnowledgeFn — predicate over the inbound ItemKnowledge at the
// node where this strategy sits. Pure; no captured mutable state.
// Returning false means the strategy cannot run with the item in that
// knowledge state.
using RequiresKnowledgeFn = std::function<bool(const core::ItemKnowledge&)>;

// EffectFn — ItemKnowledge → ItemKnowledge applied AFTER the task to
// produce the knowledge state the next task observes. Pure; no captured
// mutable state.
using EffectFn = std::function<core::ItemKnowledge(const core::ItemKnowledge&)>;

// StrategyResult — the output of one Strategy::evaluate() call.
//
// Strategies SHOULD always populate `requires_knowledge` and `effect`,
// even on INFEASIBLE returns — they describe the strategy's knowledge-
// flow contract, which is conceptually independent of whether a
// particular catalog selection succeeded. Empty std::function on a
// consumed result is a defect in the caller's selection logic, not a
// knowledge-flow failure, and consumers (e.g. propagate_knowledge)
// reject it rather than fabricate identity behaviour (engineering.md §3).
//
// PLACEHOLDER (step 4): partial_info + preconditions are absent — they
// turn on with the analytic throughput model (decisions.md).
struct StrategyResult {
    Feasibility feasibility;
    std::string strategy_name;
    std::optional<EquipmentRef> equipment;
    core::Energy energy_per_cycle;
    core::Duration cycle_time;
    RequiresKnowledgeFn requires_knowledge;
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
