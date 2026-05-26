#pragma once

// Large Neighborhood Search outer loop over LayoutProblem (step 6 of
// roadmap.md; solver.md "Outer loop"). Greedy at Phase 3; Phase 4 adds
// simulated-annealing acceptance behind the same API.
//
// Mechanics. Each iteration:
//   1. DESTROY — pick one station, free it. Other variable stations
//      get frozen at their current poses; stations the caller already
//      marked frozen (anchors etc.) stay frozen for all iterations.
//   2. REPAIR — call solve() on the destroyed problem. The freed
//      station re-optimises against the now-frozen-rest, warm-started
//      from its current pose.
//   3. ACCEPT — greedy at Phase 3: keep the candidate iff its total
//      cost is strictly lower than the current best. (Phase 4: SA.)
//
// The destroy operator is single-station-round-robin at Phase 3.
// Concrete-over-abstract: this is the smallest move we can validate
// the loop with; other operators (pair, neighborhood, random-k) wait
// until a second concrete operator is needed.
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

    // RNG seed for any randomised behaviour. Round-robin station
    // selection at Phase 3 doesn't use it; Phase 4's SA acceptance and
    // any future randomised destroy operator do. Pinned for
    // reproducibility — same seed + same problem = same trace.
    std::uint64_t seed = 0;
};

// One trace entry per destroy-and-repair attempt. Populated for every
// iteration, accepted or not — lets the caller plot acceptance
// behaviour, attribute cost changes to which station moved, and
// (Phase 5) feed a CSV cost-trace.
struct LnsIteration {
    std::size_t iter;                 // 0-based iteration index
    std::size_t destroyed_station;    // index into LayoutProblem.stations
    double proposed_total;            // candidate's cost.total
    double current_total;             // current best's cost.total at decision time
    bool accepted;                    // greedy: proposed_total < current_total
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
