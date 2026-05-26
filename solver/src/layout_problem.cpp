#include "tinycell/solver/layout_problem.hpp"

#include "tinycell/solver/layout_objective.hpp"

namespace tinycell::solver {

// Phase D stub: pass the initial poses through, evaluate the
// objective there, report feasibility. Phase E swaps this body for
// a real NLopt/SLSQP solve.
LayoutSolution solve(const LayoutProblem& problem) {
    LayoutSolution out;
    out.station_poses.reserve(problem.stations.size());
    for (const auto& s : problem.stations) {
        out.station_poses.push_back(s.initial_pose);
    }
    out.final_objective = evaluate_objective(problem, out.station_poses);
    out.hard_constraints_satisfied =
        ::tinycell::solver::hard_constraints_satisfied(problem, out.station_poses);
    return out;
}

} // namespace tinycell::solver
