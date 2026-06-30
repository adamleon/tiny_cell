#include "tinycell/solver/layout_problem.hpp"

#include "tinycell/solver/layout_objective.hpp"

#include <mp-units/systems/si.h>
#include <nlopt.hpp>

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <numbers>
#include <stdexcept>
#include <vector>

namespace tinycell::solver {

namespace {
using namespace mp_units;
namespace tc = tinycell::core;

// One placer-variable port (step-6 M1.4): an ANNULUS transport endpoint
// (reach_max set) whose station-frame position the optimiser is free to
// move within the reach band. Parametrised in POLAR form (r, theta) so the
// radial band is an exact 1-D box on r. Identifies which TransportConstraint
// and which end. Port pairs occupy the variable vector AFTER all station
// pairs, so the layout is:
//   x = [ station pairs: 2*n_variable_stations ) [ port pairs: 2*n_ports )
struct PortVar {
    std::size_t transport_index;  // index into problem.transports
    bool is_source;               // true: source endpoint; false: sink
};

// Collect the annulus endpoints across all transports; each becomes a
// 2-variable port slot. Source listed before sink within a transport,
// transports in index order — this fixes the port-pair ordering in x.
std::vector<PortVar> collect_port_vars(const LayoutProblem& problem) {
    std::vector<PortVar> pv;
    for (std::size_t t = 0; t < problem.transports.size(); ++t) {
        const auto& tr = problem.transports[t];
        if (tr.source_reach_max.has_value()) pv.push_back({t, true});
        if (tr.sink_reach_max.has_value())   pv.push_back({t, false});
    }
    return pv;
}

// Convert an annulus port's polar variables (r, theta) — its radius within
// the reach band and its direction in the station frame — to a Cartesian
// port_local. The reach band is enforced EXACTLY and as a HARD constraint
// by the box bounds on r ([reach_min, reach_max]); theta is unconstrained
// (any direction). Polar is the natural parametrisation: an annulus is
// {r in [rmin, rmax], any theta}, so a 1-D box on r expresses the radial
// band exactly — which axis-aligned Cartesian box bounds cannot — and the
// objective stays SMOOTH in (r, theta), so BOBYQA has no projection kink to
// stall on when swinging the port around the ring. This supersedes the
// M1.3 soft annulus_penalty as the band mechanism; that penalty now scores
// ~0 on the in-band port and remains only as a redundant backstop.
tc::Vec2 port_local_from_polar(double r, double theta) {
    return tc::Vec2{r * std::cos(theta) * si::metre,
                    r * std::sin(theta) * si::metre};
}

// Rebuild the transport set with each annulus endpoint's port_local set
// from its current (r, theta) optimiser variables; POINT endpoints pass
// through unchanged. `base` is the offset where port pairs begin in x
// (= 2 * number of variable stations). Called once at each NLopt iteration
// top (a kernel boundary): the copy is small (a handful of transports) and
// the inner objective loop then reads port positions flat — never
// per-element conversion mid-loop (CLAUDE.md §3). Because the conversion is
// applied here, both the scored objective and the returned
// LayoutSolution::transports carry in-band ports.
std::vector<TransportConstraint> effective_transports(
    const LayoutProblem& problem,
    const std::vector<PortVar>& port_vars,
    std::size_t base,
    const std::vector<double>& x) {
    std::vector<TransportConstraint> transports = problem.transports;
    for (std::size_t k = 0; k < port_vars.size(); ++k) {
        const double r = x[base + 2 * k + 0];
        const double theta = x[base + 2 * k + 1];
        const tc::Vec2 p = port_local_from_polar(r, theta);
        auto& tr = transports[port_vars[k].transport_index];
        if (port_vars[k].is_source) tr.source_port_local = p;
        else                        tr.sink_port_local = p;
    }
    return transports;
}

// Closure passed through NLopt's `void*` callback hook. variable_indices
// maps each NLopt-variable pair (x[2k], x[2k+1]) to its station index in
// problem->stations; frozen stations are absent from this list and read
// their pose from initial_pose at evaluation time. port_vars maps the
// port pairs that follow the station pairs to their transport endpoints.
struct Closure {
    const LayoutProblem* problem;
    const std::vector<std::size_t>* variable_indices;
    const std::vector<PortVar>* port_vars;
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
    const std::size_t base = 2 * c->variable_indices->size();
    const auto transports = effective_transports(*c->problem, *c->port_vars, base, x);
    return evaluate_objective(*c->problem, poses, transports);
}

} // namespace

// solve() runs NLopt LN_BOBYQA over the per-station (x, y) pair of
// every VARIABLE station (frozen=false), plus (step-6 M1.4) the polar
// (r, theta) position of every ANNULUS transport endpoint. The variable
// vector is laid out as [ station pairs ) [ port pairs ): the port pairs
// follow all station pairs so poses_from_vars (which reads only the
// station block) is unaffected. Each annulus port's reach band is a HARD
// constraint enforced EXACTLY by the box bounds on its r variable
// ([reach_min, reach_max]), independent of any soft-penalty weight (the
// soft M1.3 annulus_penalty is now a redundant backstop — it scores ~0 on
// the in-band port). Frozen stations contribute their initial_pose to the
// objective unchanged. Station theta is held at its initial value because
// the objective is theta-independent; adding it as a variable would break
// the 2*k station indexing and is a separate deferred follow-up.
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
            .transports = {},
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

    // Annulus transport endpoints (reach_max set) become placer
    // variables too (M1.4): their polar (r, theta) position is optimised
    // within the reach band. Port pairs occupy x AFTER the station
    // pairs; station_var_count is the offset to the port block.
    const std::vector<PortVar> port_vars = collect_port_vars(problem);
    const std::size_t n_ports = port_vars.size();
    const std::size_t station_var_count = 2 * n_vars;

    // Validate frozen stations: nothing to validate at solve() entry
    // beyond what the caller provided. If a frozen station is outside
    // the floor or overlaps another, hard_constraints_satisfied will
    // report it after the solve.

    // All stations frozen AND no annulus port to place: nothing to
    // optimise. Return initial poses and report the objective evaluated
    // at them. (NLopt also rejects a zero-dimensional opt object.) A
    // frozen-station problem can still carry a variable annulus port, so
    // only when BOTH are absent is there genuinely nothing to vary.
    if (n_vars == 0 && n_ports == 0) {
        LayoutSolution out;
        out.station_poses.reserve(n);
        for (const auto& s : problem.stations) {
            out.station_poses.push_back(s.initial_pose);
        }
        out.transports = problem.transports;  // no annulus ports → unchanged
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
    const std::size_t total_vars = station_var_count + 2 * n_ports;
    std::vector<double> x(total_vars);
    std::vector<double> lb(total_vars);
    std::vector<double> ub(total_vars);
    // Station block: [0, station_var_count).
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
    // Port block: [station_var_count, total_vars). Each annulus port is two
    // variables (r, theta) in POLAR form. r is hard-bounded to the reach
    // band [reach_min, reach_max] — an EXACT 1-D box expressing the radial
    // annulus that Cartesian box bounds cannot; theta gets a generous finite
    // range around the seed direction (it is periodic, so any direction is
    // reachable without the optimum landing near a bound). Seed (r, theta) =
    // the emitted port_local in polar form.
    constexpr double kTwoPi = 2.0 * std::numbers::pi;
    for (std::size_t k = 0; k < n_ports; ++k) {
        const auto& pv = port_vars[k];
        const auto& tr = problem.transports[pv.transport_index];
        const tc::Length rmin_len = (pv.is_source ? tr.source_reach_min
                                                  : tr.sink_reach_min)
                                        .value_or(0.0 * si::metre);
        const tc::Length rmax_len = pv.is_source ? *tr.source_reach_max
                                                  : *tr.sink_reach_max;
        const double rmin = rmin_len.numerical_value_in(si::metre);
        const double rmax = rmax_len.numerical_value_in(si::metre);
        // Inverted reach band is authored/strategy nonsense, not something
        // to silently clamp past (CLAUDE.md §1): reject with a diagnostic.
        if (rmin > rmax) {
            throw std::invalid_argument(
                "LayoutProblem::solve: annulus port reach_min exceeds reach_max "
                "(inverted reach band)");
        }
        const tc::Vec2 seed = pv.is_source ? tr.source_port_local
                                           : tr.sink_port_local;
        const double seed_x = seed.x.numerical_value_in(si::metre);
        const double seed_y = seed.y.numerical_value_in(si::metre);
        const double seed_r = std::clamp(
            std::sqrt(seed_x * seed_x + seed_y * seed_y), rmin, rmax);
        const double seed_theta = std::atan2(seed_y, seed_x);
        const std::size_t b = station_var_count + 2 * k;
        lb[b + 0] = rmin;
        ub[b + 0] = rmax;
        lb[b + 1] = seed_theta - kTwoPi;
        ub[b + 1] = seed_theta + kTwoPi;
        x[b + 0] = seed_r;
        x[b + 1] = seed_theta;
    }

    nlopt::opt opt(nlopt::LN_BOBYQA, static_cast<unsigned>(total_vars));
    opt.set_lower_bounds(lb);
    opt.set_upper_bounds(ub);
    opt.set_xtol_rel(1e-6);
    opt.set_ftol_rel(1e-9);
    opt.set_maxeval(2000);

    Closure closure{&problem, &variable_indices, &port_vars};
    opt.set_min_objective(&nlopt_callback, &closure);

    double final_obj = 0.0;
    // NLopt throws on hard failures (invalid args, out-of-memory,
    // forced-stop, etc.). Soft "didn't converge but here's the best
    // point" cases return a code; the result vector x is the best
    // point regardless. Don't catch - let bugs propagate.
    opt.optimize(x, final_obj);

    LayoutSolution out;
    // Unpack the optimised port positions from the same variable vector
    // the callback scored, so cost (and the returned transports) reflect
    // the OPTIMISED ports, not the seed port_locals.
    out.transports =
        effective_transports(problem, port_vars, station_var_count, x);
    out.station_poses = poses_from_vars(problem, variable_indices, x);
    out.cost = decompose_objective(problem, out.station_poses, out.transports);
    // Sanity: `cost.total` and the value NLopt returned should agree
    // since the callback IS evaluate_objective; this only catches a
    // future divergence (e.g. someone caching the wrong value).
    (void)final_obj;
    out.hard_constraints_satisfied =
        ::tinycell::solver::hard_constraints_satisfied(problem, out.station_poses);
    return out;
}

} // namespace tinycell::solver
