#include "tinycell/solver/state_propagation.hpp"

#include <stdexcept>
#include <utility>

namespace tinycell::solver {

// propagate_state(): the implementation.
//   1. validate workflow and chosen sizes match
//   2. seed the trajectory with the initial user-declared ItemState
//   3. for each task index i:
//        a. defect-check chosen[i] (non-null, non-empty requires_state
//           / effect)
//        b. evaluate requires_state against the running state — return
//           PropagationFailure on first false
//        c. apply effect and append the result to the trajectory
//   4. return PropagationSuccess with the complete trajectory
PropagationResult propagate_state(std::span<const core::Task> workflow,
                                  std::span<const StrategyResult* const> chosen,
                                  const core::ItemState& initial) {
    if (workflow.size() != chosen.size()) {
        throw std::invalid_argument(
            "propagate_state: workflow.size() must equal chosen.size()");
    }

    std::vector<core::ItemState> trajectory;
    trajectory.reserve(workflow.size() + 1);
    trajectory.push_back(initial);

    for (std::size_t i = 0; i < workflow.size(); ++i) {
        const StrategyResult* result = chosen[i];
        if (result == nullptr) {
            throw std::logic_error(
                "propagate_state: chosen[i] is null — selection logic defect");
        }
        if (!result->requires_state || !result->effect) {
            throw std::logic_error(
                "propagate_state: StrategyResult missing "
                "requires_state/effect — strategy did not populate the "
                "state-flow contract");
        }

        const core::ItemState& inbound = trajectory.back();
        if (!result->requires_state(inbound)) {
            return PropagationFailure{
                .task_index = i,
                .strategy_name = result->strategy_name,
                .inbound_state = inbound,
            };
        }
        trajectory.push_back(result->effect(inbound));
    }

    return PropagationSuccess{.trajectory = std::move(trajectory)};
}

} // namespace tinycell::solver
