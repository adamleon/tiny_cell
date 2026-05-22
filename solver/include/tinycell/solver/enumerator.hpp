#pragma once

// Brute-force enumerator — the validator of the strategy library (roadmap.md
// step 2). Given a workflow (a list of Tasks) and a set of Strategy
// implementations, asks every applicable strategy to evaluate every task and
// records the proposals plus the chosen winner per task.
//
// The point of step 2 is NOT to compute realistic plans (the throughput /
// cost model arrives at step 4, decisions.md). It's to (a) exercise every
// strategy on a non-trivial set of tasks and (b) expose what real consumers
// will need from StrategyResult. The output shape — all proposals + a
// winner index — surfaces both:
//   * "no winner" reveals tasks no strategy can serve (strategy library hole)
//   * the proposals list lets the demo print a side-by-side comparison that
//     would be hidden if only the winner were returned
//
// This module is catalog-agnostic; catalogs live behind the strategies that
// hold them. Strategies are passed as non-owning const pointers — the caller
// owns the strategy objects and the catalogs they reference.

#include <cstddef>
#include <optional>
#include <span>
#include <tinycell/model/task.hpp>
#include <tinycell/solver/strategy.hpp>
#include <vector>

namespace tinycell::solver {

// TaskEnumeration — what the enumerator produces for ONE task.
//   * task is a non-owning pointer back into the workflow span the caller
//     passed in. The caller's workflow must outlive the returned vector.
//   * proposals holds one StrategyResult per applicable strategy, in the
//     order strategies were registered. Inapplicable strategies do not
//     appear here (filtered by applies_to()).
//   * winner_index points into `proposals` at the chosen winner, or is
//     nullopt when no proposal is FULL.
struct TaskEnumeration {
    const core::Task* task;
    std::vector<StrategyResult> proposals;
    std::optional<std::size_t> winner_index;
};

// enumerate(): for every task, call evaluate on every strategy that applies,
// then pick the FULL proposal with the lowest energy_per_cycle as winner.
// Returns one TaskEnumeration per task, in workflow order.
//
// PLACEHOLDER (step 4): ranking by energy_per_cycle is provisional. Both
// the energy and cycle-time values are themselves placeholders inside the
// strategies (see ArmStrategy / PusherStrategy), so the winner is a
// plausibility check, not an engineering decision. Step 4 introduces the
// real cost function (capex + opex from data-model.md §6) and PARTIAL
// feasibility chaining; this comparator becomes a multi-criterion ranker.
std::vector<TaskEnumeration> enumerate(std::span<const core::Task> workflow,
                                       std::span<const Strategy* const> strategies);

} // namespace tinycell::solver
