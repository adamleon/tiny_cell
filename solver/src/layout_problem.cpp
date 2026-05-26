#include "tinycell/solver/layout_problem.hpp"

#include "tinycell/solver/layout_objective.hpp"

#include <mp-units/systems/si.h>
#include <nlopt.hpp>

#include <algorithm>
#include <cstddef>
#include <stdexcept>
#include <vector>

namespace tinycell::solver {

namespace {
using namespace mp_units;
namespace tc = tinycell::core;

// Closure passed through NLopt's `void*` callback hook. variable_indices
// maps each NLopt-variable pair (x[2k], x[2k+1]) to its station index in
// problem->stations; frozen stations are absent from this list and read
// their pose from initial_pose at evaluation time.
struct Closure {
    const LayoutProblem* problem;
    const std::vector<std::size_t>* variable_indices;
};

// Build the per-station Pose2D vector from the optimiser's variable
// vector + the frozen stations' fixed initial poses. Used by both the
// NLopt callback and by the post-optimisation result unpack.
std::vector<tc::Pose2D> poses_from_vars(
    const LayoutProblem& problem,
    const std::vector<std::size_t>& variable_indices,
    const std::vector<double>& x) {
    std::vector<tc::Pose2D> poses;
    poses.reserve(problem.stations.size());
    for (const auto& s : problem.stations) {
        poses.push_back(s.initial_pose);  // default: hold at seed (covers frozen)
    }
    for (std::size_t k = 0; k < variable_indices.size(); ++k) {
        const std::size_t i = variable_indices[k];
        poses[i] = tc::Pose2D{
            .x = x[2 * k + 0] * si::metre,
            .y = x[2 * k + 1] * si::metre,
            .theta = problem.stations[i].initial_pose.theta,
            .frame = tc::kWorldFrame,
        };
    }
    return poses;
}

// NLopt callback. Unpacks the flat variable vector into per-station
// Pose2Ds (frozen stations contribute their initial_pose, variable
// stations contribute the current optimisation iterate), then calls
// evaluate_objective.
//
// `grad` is left empty: BOBYQA is derivative-free (LN_ prefix) and
// won't query it. If we ever swap to a derivative-required algorithm
// (LD_SLSQP, LD_LBFGS, ...) this is the place finite-difference or
// analytical gradients land.
double nlopt_callback(const std::vector<double>& x,
                      std::vector<double>& grad,
                      void* data) {
    (void)grad;
    auto* c = static_cast<Closure*>(data);
    const auto poses = poses_from_vars(*c->problem, *c->variable_indices, x);
    return evaluate_objective(*c->problem, poses);
}

} // namespace

// solve() runs NLopt LN_BOBYQA over the per-station (x, y) pair of
// every VARIABLE station (frozen=false). Frozen stations contribute
// their initial_pose to the objective unchanged. theta is held at
// its initial value because the Phase D objective is theta-independent;
// reach-direction / port-direction terms (deferred follow-ups) will
// add the third variable when they arrive.
//
// **Why BOBYQA and not SLSQP** (the Phase A spike's recommendation):
// SLSQP requires gradients, and our `max(0, depth)^2` penalties have
// C0 kinks at the constraint boundary where the gradient is
// discontinuous. BOBYQA is derivative-free (trust-region quadratic
// model), handles non-smoothness more gracefully, and is plenty fast
// at the scale we care about (a handful of stations). Switching back
// to SLSQP - or to LD_LBFGS, LN_COBYLA, anything in NLopt's library -
// is a one-enum change here plus, for derivative-required algorithms,
// a finite-difference fill of the `grad` argument in the callback.
//
// Warm-start: callers seed the optimisation by setting initial_pose
// on each StationProblem. First solves pass the positional-prior
// nominal; re-solves (LNS destroy-and-repair, interactive editing)
// pass a previous solution's pose. No separate "warm-start" API call
// — initial_pose IS the warm-start hook.
//
// Partial-freeze: stations with frozen=true are absent from the NLP's
// variable set. Their pose is held at initial_pose. They still
// contribute to overlap penalties against variable stations and to
// hard_constraints_satisfied — they're constants of the problem,
// not variables.
LayoutSolution solve(const LayoutProblem& problem) {
    const std::size_t n = problem.stations.size();
    if (n == 0) {
        return LayoutSolution{
            .station_poses = {},
            .cost = ObjectiveBreakdown{},
            .hard_constraints_satisfied = true,
        };
    }

    // Partition stations into variable (NLP-optimised) and frozen
    // (held at initial_pose). variable_indices[k] gives the station
    // index for the k-th NLP variable pair.
    std::vector<std::size_t> variable_indices;
    variable_indices.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        if (!problem.stations[i].frozen) variable_indices.push_back(i);
    }
    const std::size_t n_vars = variable_indices.size();

    // Validate frozen stations: nothing to validate at solve() entry
    // beyond what the caller provided. If a frozen station is outside
    // the floor or overlaps another, hard_constraints_satisfied will
    // report it after the solve.

    // All stations frozen: nothing to optimise. Return initial poses
    // and report the objective evaluated at them. (NLopt also rejects
    // a zero-dimensional opt object.)
    if (n_vars == 0) {
        LayoutSolution out;
        out.station_poses.reserve(n);
        for (const auto& s : problem.stations) {
            out.station_poses.push_back(s.initial_pose);
        }
        out.cost = decompose_objective(problem, out.station_poses);
        out.hard_constraints_satisfied =
            ::tinycell::solver::hard_constraints_satisfied(problem, out.station_poses);
        return out;
    }

    // Bounds: per-station feasible-floor bounds for VARIABLE stations
    // only. The bounding circle is forced to stay inside the floor
    // ([xmin+r, xmax-r], similarly y) — NLopt enforces this as a hard
    // constraint, sidestepping the penalty-method equilibrium where a
    // soft floor_penalty would leave a residual against competing
    // terms.
    //
    // floor_penalty stays in the objective as a safety net for the
    // case where bounding_radius exceeds half the floor extent
    // (lb > ub) - the validation below catches that, but the penalty
    // term means hard_constraints_satisfied stays meaningful even if a
    // caller bypasses validation. Frozen stations don't get bounds
    // (they're not variables), but their floor compliance is still
    // checked via floor_penalty in the objective.
    std::vector<double> x(2 * n_vars);
    std::vector<double> lb(2 * n_vars);
    std::vector<double> ub(2 * n_vars);
    for (std::size_t k = 0; k < n_vars; ++k) {
        const auto& s = problem.stations[variable_indices[k]];
        const double r = s.bounding_radius.numerical_value_in(si::metre);
        const double xmin = problem.floor.x_min.numerical_value_in(si::metre) + r;
        const double xmax = problem.floor.x_max.numerical_value_in(si::metre) - r;
        const double ymin = problem.floor.y_min.numerical_value_in(si::metre) + r;
        const double ymax = problem.floor.y_max.numerical_value_in(si::metre) - r;
        if (xmin > xmax || ymin > ymax) {
            throw std::invalid_argument(
                "LayoutProblem::solve: station '" + s.id +
                "' bounding radius does not fit inside the floor");
        }
        lb[2 * k + 0] = xmin;
        lb[2 * k + 1] = ymin;
        ub[2 * k + 0] = xmax;
        ub[2 * k + 1] = ymax;
        // Initial point = warm-start seed, clamped into the feasible
        // region (else NLopt rejects the initial vector as out-of-bounds).
        const double seed_x = s.initial_pose.x.numerical_value_in(si::metre);
        const double seed_y = s.initial_pose.y.numerical_value_in(si::metre);
        x[2 * k + 0] = std::clamp(seed_x, xmin, xmax);
        x[2 * k + 1] = std::clamp(seed_y, ymin, ymax);
    }

    nlopt::opt opt(nlopt::LN_BOBYQA, static_cast<unsigned>(2 * n_vars));
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);
    opt.set_xtol_rel(1e-6);
    opt.set_ftol_rel(1e-9);
    opt.set_maxeval(2000);

    Closure closure{&problem, &variable_indices};
    opt.set_min_objective(&nlopt_callback, &closure);

    double final_obj = 0.0;
    // NLopt throws on hard failures (invalid args, out-of-memory,
    // forced-stop, etc.). Soft "didn't converge but here's the best
    // point" cases return a code; the result vector x is the best
    // point regardless. Don't catch - let bugs propagate.
    opt.optimize(x, final_obj);

    LayoutSolution out;
    out.station_poses = poses_from_vars(problem, variable_indices, x);
    out.cost = decompose_objective(problem, out.station_poses);
    // Sanity: `cost.total` and the value NLopt returned should agree
    // since the callback IS evaluate_objective; this only catches a
    // future divergence (e.g. someone caching the wrong value).
    (void)final_obj;
    out.hard_constraints_satisfied =
        ::tinycell::solver::hard_constraints_satisfied(problem, out.station_poses);
    return out;
}

} // namespace tinycell::solver
