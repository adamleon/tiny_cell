#include "tinycell/solver/lns.hpp"

#include <cstddef>

namespace tinycell::solver {

namespace {

// Strict-better acceptance with a small epsilon to ignore numerical
// noise: a re-solve that converges to the same minimum can return a
// total that differs by ~1e-12 even though it represents the same
// layout. Without an epsilon the loop would "accept" these no-ops and
// the trace would lie about which moves made progress.
constexpr double kImprovementEps = 1e-9;

} // namespace

LnsResult lns_solve(const LayoutProblem& initial, const LnsParams& params) {
    LnsResult out;

    // Cold solve establishes the baseline. Frozen stations in `initial`
    // stay where they are; variable stations land at the layout the
    // single solve() call finds from their initial_pose seeds.
    LayoutSolution current = solve(initial);
    out.best = current;

    // Variable stations are the destroy candidates. Stations the caller
    // froze (anchors, user-locked, etc.) are never picked.
    std::vector<std::size_t> variables;
    for (std::size_t i = 0; i < initial.stations.size(); ++i) {
        if (!initial.stations[i].frozen) variables.push_back(i);
    }
    if (variables.empty()) return out;

    for (std::size_t it = 0; it < params.max_iterations; ++it) {
        const std::size_t pick = variables[it % variables.size()];

        // Build the destroyed problem: every variable station except
        // `pick` is frozen at its CURRENT pose. `pick` keeps its current
        // pose as warm-start but stays variable, so it re-optimises
        // against the now-static rest.
        LayoutProblem destroyed = initial;
        for (std::size_t i = 0; i < destroyed.stations.size(); ++i) {
            destroyed.stations[i].initial_pose = current.station_poses[i];
            if (!initial.stations[i].frozen && i != pick) {
                destroyed.stations[i].frozen = true;
            }
        }

        const LayoutSolution candidate = solve(destroyed);
        LnsIteration entry{
            .iter = it,
            .destroyed_station = pick,
            .proposed_total = candidate.cost.total,
            .current_total = current.cost.total,
            .accepted = false,
        };
        if (candidate.cost.total < current.cost.total - kImprovementEps) {
            current = candidate;
            out.best = current;
            entry.accepted = true;
        }
        out.trace.push_back(entry);
    }
    return out;
}

} // namespace tinycell::solver
