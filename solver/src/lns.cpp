#include "tinycell/solver/lns.hpp"

#include <cmath>
#include <cstddef>
#include <random>

namespace tinycell::solver {

namespace {

// Strict-better acceptance with a small epsilon to ignore numerical
// noise: a re-solve that converges to the same minimum can return a
// total that differs by ~1e-12 even though it represents the same
// layout. Without an epsilon the trace would lie about which moves
// made progress.
constexpr double kImprovementEps = 1e-9;

// Metropolis acceptance for a non-improving move (Δ > 0). Returns true
// with probability exp(-Δ / T). T <= 0 collapses to pure greedy
// (always reject worsening moves) — covers both the explicit
// temperature=0 case and any pathological inputs where the auto T
// calibration produces a non-positive value.
bool metropolis_accept(double delta, double temperature, std::mt19937_64& rng) {
    if (temperature <= 0.0) return false;
    const double p = std::exp(-delta / temperature);
    std::uniform_real_distribution<double> uniform(0.0, 1.0);
    return uniform(rng) < p;
}

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

    // Temperature calibration. Auto picks T0 = cold_cost / 10, so a
    // proposal worsening cost by 10 % at iter 0 has acceptance
    // probability exp(-1) ≈ 0.37. Explicit 0 (with auto_temperature
    // false) is the pure-greedy path: metropolis_accept short-circuits
    // on T <= 0.
    double T0 = params.temperature_initial;
    if (params.auto_temperature && T0 == 0.0) {
        T0 = std::max(current.cost.total / 10.0, 0.0);
    }

    std::mt19937_64 rng(params.seed);

    for (std::size_t it = 0; it < params.max_iterations; ++it) {
        const std::size_t pick = variables[it % variables.size()];
        const double T = T0 * std::pow(params.temperature_decay,
                                       static_cast<double>(it));

        // Build the destroyed problem: every variable station except
        // `pick` is frozen at its CURRENT pose (SA walker, not best).
        // `pick` keeps its current pose as warm-start but stays
        // variable, so it re-optimises against the now-static rest.
        LayoutProblem destroyed = initial;
        for (std::size_t i = 0; i < destroyed.stations.size(); ++i) {
            destroyed.stations[i].initial_pose = current.station_poses[i];
            if (!initial.stations[i].frozen && i != pick) {
                destroyed.stations[i].frozen = true;
            }
        }

        const LayoutSolution candidate = solve(destroyed);
        const double delta = candidate.cost.total - current.cost.total;

        LnsIteration entry{
            .iter = it,
            .destroyed_station = pick,
            .temperature = T,
            .proposed_total = candidate.cost.total,
            .current_total = current.cost.total,
            .best_total = out.best.cost.total,
            .accepted = false,
        };

        // Accept iff strictly better OR Metropolis fires. Equal-cost
        // moves are not accepted (no information gained, would inflate
        // the "accepted" rate spuriously).
        const bool greedy_accept = delta < -kImprovementEps;
        const bool sa_accept = delta > kImprovementEps &&
                               metropolis_accept(delta, T, rng);
        if (greedy_accept || sa_accept) {
            current = candidate;
            entry.accepted = true;
            // Best tracks the best-ever total seen; SA may accept worse
            // moves that don't beat it.
            if (current.cost.total < out.best.cost.total - kImprovementEps) {
                out.best = current;
            }
        }
        out.trace.push_back(entry);
    }
    return out;
}

} // namespace tinycell::solver
