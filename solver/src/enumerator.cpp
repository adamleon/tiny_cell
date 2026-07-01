#include "tinycell/solver/enumerator.hpp"

#include <mp-units/systems/si.h>

namespace tinycell::solver {

namespace {

// pick_winner(): prefer FULL (lowest energy_per_cycle); if no FULL,
// fall back to PARTIAL (lowest energy_per_cycle). INFEASIBLE proposals
// are ignored. Returns nullopt when every proposal is INFEASIBLE.
std::optional<std::size_t> pick_winner(const std::vector<StrategyResult>& proposals) {
    std::optional<std::size_t> best_full;
    std::optional<std::size_t> best_partial;
    for (std::size_t i = 0; i < proposals.size(); ++i) {
        const auto& p = proposals[i];
        if (p.feasibility == Feasibility::FULL) {
            if (!best_full ||
                p.energy_per_cycle < proposals[*best_full].energy_per_cycle) {
                best_full = i;
            }
        } else if (p.feasibility == Feasibility::PARTIAL) {
            if (!best_partial ||
                p.energy_per_cycle < proposals[*best_partial].energy_per_cycle) {
                best_partial = i;
            }
        }
    }
    return best_full ? best_full : best_partial;
}

// Recurse on one task: evaluate strategies, pick winner, append a
// TaskEnumeration to `out`, then recurse on every precondition the
// winner emitted (depth-bounded).
//
// Preconditions take two shapes:
//   * PARTIAL residuals  — ArmStrategy emits a smaller copy of the
//                          same task with a tightened target when no
//                          single arm can meet the rate alone. Same
//                          task kind, different params.
//   * Structural preconds — strategies emit Transport tasks (T.6) or
//                          other prerequisite tasks the binding can't
//                          run without. Different task kind.
// Both flow through StrategyResult.preconditions; both must be
// enumerated. The earlier "only on PARTIAL" guard was a leftover
// from when residuals were the only precondition shape and dropped
// FULL-spawned structural preconds (T.7-pre-bugfix).
void enumerate_recursive(const core::Task& task,
                         std::span<const Strategy* const> strategies,
                         std::size_t depth_remaining,
                         std::vector<TaskEnumeration>& out) {
    TaskEnumeration te{.task = task};
    for (const auto* s : strategies) {
        if (s->applies_to(task)) {
            te.proposals.push_back(s->evaluate(task));
        }
    }
    te.winner_index = pick_winner(te.proposals);

    // Capture preconditions before moving `te` into `out`.
    std::vector<core::Task> preconds;
    if (te.winner_index && depth_remaining > 0) {
        preconds = te.proposals[*te.winner_index].preconditions;
    }

    out.push_back(std::move(te));

    for (const auto& p : preconds) {
        enumerate_recursive(p, strategies, depth_remaining - 1, out);
    }
}

} // namespace

std::vector<TaskEnumeration> enumerate(std::span<const core::Task> workflow,
                                       std::span<const Strategy* const> strategies) {
    std::vector<TaskEnumeration> out;
    out.reserve(workflow.size());
    for (const auto& task : workflow) {
        enumerate_recursive(task, strategies, MAX_PARTIAL_CHAIN_DEPTH, out);
    }
    return out;
}

} // namespace tinycell::solver
