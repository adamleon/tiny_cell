#include "tinycell/solver/knowledge_propagation.hpp"

#include <stdexcept>
#include <utility>

namespace tinycell::solver {

// propagate_knowledge(): the implementation.
//   1. validate workflow and chosen sizes match
//   2. seed the trajectory with the initial user-declared ItemKnowledge
//   3. for each task index i:
//        a. defect-check chosen[i] (non-null, non-empty
//           requires_knowledge / effect)
//        b. evaluate requires_knowledge against the running knowledge —
//           return PropagationFailure on first false
//        c. apply effect and append the result to the trajectory
//   4. return PropagationSuccess with the complete trajectory
PropagationResult propagate_knowledge(std::span<const core::Task> workflow,
                                      std::span<const StrategyResult* const> chosen,
                                      const core::ItemKnowledge& initial) {
    if (workflow.size() != chosen.size()) {
        throw std::invalid_argument(
            "propagate_knowledge: workflow.size() must equal chosen.size()");
    }

    std::vector<core::ItemKnowledge> trajectory;
    trajectory.reserve(workflow.size() + 1);
    trajectory.push_back(initial);

    for (std::size_t i = 0; i < workflow.size(); ++i) {
        const StrategyResult* result = chosen[i];
        if (result == nullptr) {
            throw std::logic_error(
                "propagate_knowledge: chosen[i] is null — selection logic defect");
        }
        if (!result->requires_knowledge || !result->effect) {
            throw std::logic_error(
                "propagate_knowledge: StrategyResult missing "
                "requires_knowledge/effect — strategy did not populate the "
                "knowledge-flow contract");
        }

        const core::ItemKnowledge& inbound = trajectory.back();
        if (!result->requires_knowledge(inbound)) {
            return PropagationFailure{
                .task_index = i,
                .strategy_name = result->strategy_name,
                .inbound_knowledge = inbound,
            };
        }
        trajectory.push_back(result->effect(inbound));
    }

    return PropagationSuccess{.trajectory = std::move(trajectory)};
}

} // namespace tinycell::solver
