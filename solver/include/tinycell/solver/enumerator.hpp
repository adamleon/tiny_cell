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

// TaskEnumeration — what the enumerator produces for ONE task (original
// workflow task or a PARTIAL residual).
//   * task is owned by value so residual tasks generated mid-enumeration
//     have stable lifetime alongside their workflow-original siblings.
//     Residual tasks carry `task.is_residual = true`.
//   * proposals holds one StrategyResult per applicable strategy, in the
//     order strategies were registered. Inapplicable strategies do not
//     appear here (filtered by applies_to()).
//   * winner_index points into `proposals` at the chosen winner, or is
//     nullopt when no proposal is feasible at all.
struct TaskEnumeration {
    core::Task task;
    std::vector<StrategyResult> proposals;
    std::optional<std::size_t> winner_index;
};

// Maximum recursion depth for PARTIAL chain walking. Each PARTIAL
// residual that produces another PARTIAL costs one depth. The math:
// ArmStrategy's residual_target = (a × t) / (a − t) converges toward
// `achievable` from below, so a chain is bounded in length by roughly
// log(achievable / target). For pathological cases where the cheapest
// feasible arm is many times slower than the original target, the
// chain converges slowly; this cap protects against unbounded
// recursion and is high enough that realistic chains terminate at
// FULL well before hitting it.
constexpr std::size_t MAX_PARTIAL_CHAIN_DEPTH = 10;

// enumerate(): for every task in the workflow, evaluate every applicable
// strategy and pick a winner. When the winner is PARTIAL, recurse on
// each residual task in `winner.preconditions`, appending each recursive
// TaskEnumeration to the output. The output is a flat list in DFS order:
// original workflow task first, then its residuals (each followed by
// any residuals of its own), then the next workflow task.
//
// Winner selection: prefer FULL (lowest energy_per_cycle); if no FULL
// proposal exists, fall back to PARTIAL (lowest energy_per_cycle).
// INFEASIBLE-only tasks have no winner.
//
// PLACEHOLDER (step 4-5): ranking by energy_per_cycle alone is
// provisional. Step 5+ introduces the full cost function (capex +
// lifetime energy + maintenance — see cost.hpp) and a multi-criterion
// ranker; the comparator inside pick_winner is the seam.
std::vector<TaskEnumeration> enumerate(std::span<const core::Task> workflow,
                                       std::span<const Strategy* const> strategies);

} // namespace tinycell::solver
