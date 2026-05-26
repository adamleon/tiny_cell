#pragma once

// Large Neighborhood Search outer loop over LayoutProblem (step 6 of
// roadmap.md; solver.md "Outer loop").
//
// Mechanics. Each iteration:
//   1. DESTROY — pick one station, free it. Other variable stations
//      get frozen at their current poses; stations the caller already
//      marked frozen (anchors etc.) stay frozen for all iterations.
//   2. REPAIR — call solve() on the destroyed problem. The freed
//      station re-optimises against the now-frozen-rest, warm-started
//      from its current pose.
//   3. ACCEPT — Metropolis criterion at Phase 4:
//        * if proposed_total < current_total: accept (improvement)
//        * else accept with probability exp(-(Δ) / T) (escape)
//      Temperature T decays geometrically: T(k) = T0 · α^k.
//      `best` tracks the best total seen across all iterations;
//      `current` is the SA walker that may briefly worsen. With
//      T0 = 0 the loop is pure greedy (the Phase 3 behaviour).
//
// **SA is forward-compatible scaffolding.** With the current
// single-station destroy operator and the smooth 2D NLP inner solver,
// freed stations re-converge deterministically against the frozen
// rest — proposed_total never strictly exceeds current_total, so the
// Metropolis branch never fires in practice on smooth/convex
// sub-problems. The branch earns its keep when (a) destroy operators
// cover >1 station so the joint sub-problem has multiple local mins,
// (b) the objective gets non-smooth (port-direction tolerances),
// or (c) the inner solver switches to COBYLA. The seam is wired
// now so those changes don't have to re-touch the outer loop.
//
// The destroy operator is single-station-round-robin. Concrete-over-
// abstract: this is the smallest move we can validate the loop with;
// other operators (pair, neighborhood, random-k) wait until a second
// concrete operator is needed.
//
// Phase F's partial-freeze + warm-start on StationProblem are what
// makes this cheap — destroying-and-repairing is just flipping the
// `frozen` flag on N-1 stations and calling solve() again with
// initial_pose set to the current solution. No new optimizer scaffold.

#include <cstddef>
#include <cstdint>
#include <tinycell/solver/layout_problem.hpp>
#include <vector>

namespace tinycell::solver {

struct LnsParams {
    // Number of destroy-and-repair iterations to run after the initial
    // cold solve. The cold solve itself is iteration -1 (it establishes
    // the baseline cost the loop tries to improve on); the trace starts
    // at iter=0.
    std::size_t max_iterations = 100;

    // RNG seed for the Metropolis acceptance test. Pinned for
    // reproducibility — same seed + same problem + same temperature
    // schedule = same trace.
    std::uint64_t seed = 0;

    // Initial temperature for the SA schedule (units = same as
    // cost.total). 0 means "auto": calibrate to cold_solve.cost.total
    // / 10, so a move that worsens cost by 10% has acceptance
    // probability exp(-1) ≈ 0.37 at iter=0. Set explicitly to 0.0 with
    // `auto_temperature = false` for pure greedy (the Phase 3
    // behaviour).
    double temperature_initial = 0.0;
    bool auto_temperature = true;

    // Geometric cooling factor: T(k+1) = T(k) · temperature_decay.
    // 0.95 over 100 iterations takes T to ~0.6% of initial — the loop
    // ends effectively greedy.
    double temperature_decay = 0.95;
};

// One trace entry per destroy-and-repair attempt. Populated for every
// iteration, accepted or not — lets the caller plot acceptance
// behaviour, attribute cost changes to which station moved, and
// (Phase 5) feed a CSV cost-trace.
struct LnsIteration {
    std::size_t iter;                 // 0-based iteration index
    std::size_t destroyed_station;    // index into LayoutProblem.stations
    double temperature;               // T at this iteration (geometric)
    double proposed_total;            // candidate's cost.total
    double current_total;             // SA walker's cost.total before this iteration
    double best_total;                // best.cost.total before this iteration
    bool accepted;                    // proposed accepted (greedy OR Metropolis)
};

struct LnsResult {
    // Best layout found across the cold solve + all iterations. If no
    // iteration improved on the cold solve, this is the cold solve.
    LayoutSolution best;

    // Cost-trace, one entry per iteration. Length equals
    // params.max_iterations (every iteration produces a trace entry,
    // including rejected proposals). Empty iff max_iterations == 0 or
    // there were no variable stations to destroy.
    std::vector<LnsIteration> trace;
};

// lns_solve(): cold-solve `initial`, then run max_iterations of
// destroy-and-repair, keeping the best total cost. The input's
// `initial_pose` on each StationProblem is the cold-solve warm-start
// seed (typically the positional-prior nominal). Stations with
// `frozen=true` in the input stay frozen for every iteration.
LnsResult lns_solve(const LayoutProblem& initial, const LnsParams& params);

} // namespace tinycell::solver
