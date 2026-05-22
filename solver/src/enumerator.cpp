#include "tinycell/solver/enumerator.hpp"

#include <mp-units/systems/si.h>

namespace tinycell::solver {

namespace {

using namespace mp_units;

// pick_winner(): among the proposals, the lowest-energy FULL result wins.
// PARTIAL / INFEASIBLE proposals are ignored — PARTIAL chaining is a step-4
// feature (decisions.md), and an INFEASIBLE proposal by definition cannot
// serve the task. Returns nullopt when no proposal is FULL.
std::optional<std::size_t> pick_winner(const std::vector<StrategyResult>& proposals) {
    std::optional<std::size_t> best;
    for (std::size_t i = 0; i < proposals.size(); ++i) {
        if (proposals[i].feasibility != Feasibility::FULL) {
            continue;
        }
        if (!best || proposals[i].energy_per_cycle < proposals[*best].energy_per_cycle) {
            best = i;
        }
    }
    return best;
}

} // namespace

std::vector<TaskEnumeration> enumerate(std::span<const core::Task> workflow,
                                       std::span<const Strategy* const> strategies) {
    std::vector<TaskEnumeration> out;
    out.reserve(workflow.size());
    for (const auto& task : workflow) {
        TaskEnumeration te;
        te.task = &task;
        for (const auto* s : strategies) {
            if (!s->applies_to(task)) {
                continue;
            }
            te.proposals.push_back(s->evaluate(task));
        }
        te.winner_index = pick_winner(te.proposals);
        out.push_back(std::move(te));
    }
    return out;
}

} // namespace tinycell::solver
