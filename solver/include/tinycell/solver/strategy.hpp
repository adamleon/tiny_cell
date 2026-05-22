#pragma once

// Strategy interface and the data types it produces. A strategy encodes
// engineering knowledge for solving one *kind of work*; the solver iterates
// the registered strategies for each task, asking each "do you apply?" then
// "what would you cost?", and keeps the cheapest feasible result.
//
// Strategies are named for the EQUIPMENT TYPE only — ArmStrategy,
// PusherStrategy, ConveyorStrategy — never for a (type × task) combination
// (decisions.md). One strategy may cover multiple TaskKinds via applies_to.

#include <optional>
#include <string>
#include <string_view>
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

// StrategyResult — the output of one Strategy::evaluate() call. The minimum
// set of fields needed to compare proposals on cost + feasibility. The full
// spec in data-model.md §2 has more fields (poses, requires_state, effect,
// preconditions, partial_info); they're added here as solver consumers
// emerge.
//
// PLACEHOLDER (step 4): partial_info + preconditions are absent. They're
// turned on with the analytic throughput model — PARTIAL can't be computed
// honestly without it (decisions.md).
struct StrategyResult {
    Feasibility feasibility;
    std::string strategy_name;
    std::optional<EquipmentRef> equipment;
    core::Energy energy_per_cycle;
    core::Duration cycle_time;
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
