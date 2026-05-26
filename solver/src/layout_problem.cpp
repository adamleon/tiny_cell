#include "tinycell/solver/layout_problem.hpp"

#include "tinycell/solver/layout_objective.hpp"

#include <mp-units/systems/si.h>
#include <nlopt.hpp>

#include <algorithm>
#include <stdexcept>
#include <vector>

namespace tinycell::solver {

namespace {
using namespace mp_units;
namespace tc = tinycell::core;

// Closure passed through NLopt's `void*` callback hook. Pointers
// outlive the callback's invocation because optimize() runs
// synchronously within solve().
struct Closure {
    const LayoutProblem* problem;
};

// NLopt callback. Unpacks the flat optimizer variable vector into
// per-station Pose2Ds (theta fixed at initial - the Phase D
// objective doesn't depend on it), then calls evaluate_objective.
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
    const std::size_t n = c->problem->stations.size();
    std::vector<tc::Pose2D> poses;
    poses.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        poses.push_back(tc::Pose2D{
            .x = x[2 * i + 0] * si::metre,
            .y = x[2 * i + 1] * si::metre,
            .theta = c->problem->stations[i].initial_pose.theta,
            .frame = tc::kWorldFrame,
        });
    }
    return evaluate_objective(*c->problem, poses);
}

} // namespace

// solve() runs NLopt LN_BOBYQA over the per-station (x, y) pair. theta
// is held at its initial value because the Phase D objective is
// theta-independent; reach-direction / port-direction terms (deferred
// follow-ups) will add the third variable when they arrive.
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
LayoutSolution solve(const LayoutProblem& problem) {
    const std::size_t n = problem.stations.size();
    if (n == 0) {
        return LayoutSolution{
            .station_poses = {},
            .final_objective = 0.0,
            .hard_constraints_satisfied = true,
        };
    }

    // Bounds: per-station feasible-floor bounds. Setting lb/ub so the
    // station's bounding circle is forced to stay inside the floor
    // (x in [xmin+r, xmax-r], similarly y) makes the floor a HARD
    // constraint NLopt enforces directly, sidestepping the penalty-
    // method equilibrium where a soft floor_penalty leaves a small
    // residual at the balance with competing terms (e.g. positional
    // prior pulling outside).
    //
    // floor_penalty stays in the objective as a safety net for the
    // degenerate case where bounding_radius exceeds half the floor
    // extent (lb > ub) - the validation below catches that, but the
    // penalty term means hard_constraints_satisfied stays meaningful
    // even if a caller bypasses validation.
    std::vector<double> x(2 * n);
    std::vector<double> lb(2 * n);
    std::vector<double> ub(2 * n);
    for (std::size_t i = 0; i < n; ++i) {
        const auto& s = problem.stations[i];
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
        lb[2 * i + 0] = xmin;
        lb[2 * i + 1] = ymin;
        ub[2 * i + 0] = xmax;
        ub[2 * i + 1] = ymax;
        // Initial point: clamp the seed into the feasible region (else
        // NLopt rejects the initial vector as out-of-bounds).
        const double seed_x = s.initial_pose.x.numerical_value_in(si::metre);
        const double seed_y = s.initial_pose.y.numerical_value_in(si::metre);
        x[2 * i + 0] = std::clamp(seed_x, xmin, xmax);
        x[2 * i + 1] = std::clamp(seed_y, ymin, ymax);
    }

    nlopt::opt opt(nlopt::LN_BOBYQA, static_cast<unsigned>(2 * n));
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);
    opt.set_xtol_rel(1e-6);
    opt.set_ftol_rel(1e-9);
    opt.set_maxeval(2000);

    Closure closure{&problem};
    opt.set_min_objective(&nlopt_callback, &closure);

    double final_obj = 0.0;
    // NLopt throws on hard failures (invalid args, out-of-memory,
    // forced-stop, etc.). Soft "didn't converge but here's the best
    // point" cases return a code; the result vector x is the best
    // point regardless. Don't catch - let bugs propagate.
    opt.optimize(x, final_obj);

    LayoutSolution out;
    out.station_poses.reserve(n);
    for (std::size_t i = 0; i < n; ++i) {
        out.station_poses.push_back(tc::Pose2D{
            .x = x[2 * i + 0] * si::metre,
            .y = x[2 * i + 1] * si::metre,
            .theta = problem.stations[i].initial_pose.theta,
            .frame = tc::kWorldFrame,
        });
    }
    out.final_objective = final_obj;
    out.hard_constraints_satisfied =
        ::tinycell::solver::hard_constraints_satisfied(problem, out.station_poses);
    return out;
}

} // namespace tinycell::solver
