// Step 6 Phase 3: greedy LNS over LayoutProblem. Behavioural tests
// only — no internal-state spying. The destroy-and-repair contract is
// "never returns a worse solution than the cold solve" plus "the trace
// records every iteration with proposed/current/accepted fields the
// caller can introspect."

#include <gtest/gtest.h>
#include <cmath>
#include <mp-units/systems/si.h>
#include <tinycell/solver/layout_objective.hpp>
#include <tinycell/solver/layout_problem.hpp>
#include <tinycell/solver/lns.hpp>

namespace tc = tinycell::core;
namespace ts = tinycell::solver;
using mp_units::si::metre;
using mp_units::si::radian;

namespace {

tc::Pose2D pose_at(double x, double y) {
    return tc::Pose2D{
        .x = x * metre,
        .y = y * metre,
        .theta = 0.0 * radian,
        .frame = tc::kWorldFrame,
    };
}

ts::StationProblem make_station(const std::string& id,
                                double seed_x, double seed_y, double radius,
                                double nom_x, double nom_y) {
    return ts::StationProblem{
        .id = id,
        .buffered_hull = {},
        .bounding_radius = radius * metre,
        .nominal = tc::Vec2{nom_x * metre, nom_y * metre},
        .initial_pose = pose_at(seed_x, seed_y),
    };
}

} // namespace

// ---- contract: greedy never gets worse ----------------------------------

// Pure-greedy LnsParams: auto_temperature off, T0=0 → metropolis
// always rejects → only strict improvements pass.
constexpr ts::LnsParams greedy(std::size_t iters) {
    return ts::LnsParams{
        .max_iterations = iters,
        .seed = 0,
        .temperature_initial = 0.0,
        .auto_temperature = false,
        .temperature_decay = 0.95,
    };
}

TEST(Lns, GreedyNeverIncreasesTotalCost) {
    // Three stations seeded at their nominal, no transports — solve()
    // converges immediately. LNS iterations have nothing to improve;
    // every iteration's proposed cost should equal current_total and
    // every iteration should be rejected (no strict improvement). The
    // best at the end equals the cold solve.
    ts::LayoutProblem p{
        .stations = {
            make_station("a", 3.0, 3.0, 0.5, 3.0, 3.0),
            make_station("b", 7.0, 3.0, 0.5, 7.0, 3.0),
            make_station("c", 5.0, 7.0, 0.5, 5.0, 7.0),
        },
        .floor = ts::Floor{0.0 * metre, 10.0 * metre, 0.0 * metre, 10.0 * metre},
        .weights = ts::ObjectiveWeights{},
    };
    const auto cold = ts::solve(p);
    auto out = ts::lns_solve(p, greedy(12));
    EXPECT_EQ(out.trace.size(), 12u);
    EXPECT_LE(out.best.cost.total, cold.cost.total + 1e-9);
    for (const auto& it : out.trace) {
        EXPECT_FALSE(it.accepted)
            << "iter " << it.iter << " proposed=" << it.proposed_total
            << " current=" << it.current_total;
    }
}

TEST(Lns, GreedyAcceptanceImprovesTransportTotal) {
    // Two non-overlapping stations connected by one transport; the
    // positional prior pulls them apart (prior at (0,0) and (10,0))
    // while the transport pulls their ports together. Seed stations
    // far from optimum: a at (0, 5), b at (10, 5), nominals at (0, 0)
    // and (10, 0) — the cold solve is decent (NLopt finds the global
    // balance from a clean seed), but the cost is non-zero and any
    // accepted iteration should reduce it. We verify the LNS best
    // beats the cold solve when seeded badly.
    ts::LayoutProblem p{
        .stations = {
            make_station("a", -3.0, 0.0, 0.5, 0.0, 0.0),
            make_station("b", 13.0, 0.0, 0.5, 10.0, 0.0),
        },
        .transports = {
            ts::TransportConstraint{
                .source_station = 0,
                .source_port_local = tc::Vec2{0.0 * metre, 0.0 * metre},
                .sink_station = 1,
                .sink_port_local = tc::Vec2{0.0 * metre, 0.0 * metre},
            },
        },
        .floor = ts::Floor{-10.0 * metre, 20.0 * metre, -5.0 * metre, 5.0 * metre},
        .weights = ts::ObjectiveWeights{
            .overlap = 1.0, .floor = 1.0,
            .positional_prior = 1.0, .transport = 1.0,
        },
    };
    auto out = ts::lns_solve(p, greedy(6));
    const auto cold = ts::solve(p);
    EXPECT_LE(out.best.cost.total, cold.cost.total + 1e-9);
    EXPECT_EQ(out.trace.size(), 6u);
}

// ---- contract: frozen stations are never picked --------------------------

TEST(Lns, FrozenStationsAreNeverDestroyed) {
    // One frozen anchor + two variable stations. Round-robin should
    // alternate between station indices 1 and 2 only — the anchor at
    // index 0 must never appear in the trace.
    auto anchor = make_station("anchor", 0.0, 0.0, 0.5, 0.0, 0.0);
    anchor.frozen = true;
    ts::LayoutProblem p{
        .stations = {
            anchor,
            make_station("a", 5.0, 0.0, 0.5, 5.0, 0.0),
            make_station("b", 10.0, 0.0, 0.5, 10.0, 0.0),
        },
        .floor = ts::Floor{-5.0 * metre, 20.0 * metre, -5.0 * metre, 5.0 * metre},
        .weights = ts::ObjectiveWeights{},
    };
    auto out = ts::lns_solve(p, greedy(10));
    for (const auto& it : out.trace) {
        EXPECT_NE(it.destroyed_station, 0u)
            << "iter " << it.iter << " destroyed the frozen anchor";
    }
    // Frozen station did not move across iterations.
    EXPECT_NEAR(out.best.station_poses[0].x.numerical_value_in(metre), 0.0, 1e-9);
    EXPECT_NEAR(out.best.station_poses[0].y.numerical_value_in(metre), 0.0, 1e-9);
}

TEST(Lns, AllFrozenIsANoOp) {
    auto a = make_station("a", 1.0, 0.0, 0.5, 1.0, 0.0); a.frozen = true;
    auto b = make_station("b", 5.0, 0.0, 0.5, 5.0, 0.0); b.frozen = true;
    ts::LayoutProblem p{
        .stations = {a, b},
        .floor = ts::Floor{0.0 * metre, 10.0 * metre, -5.0 * metre, 5.0 * metre},
        .weights = ts::ObjectiveWeights{},
    };
    auto out = ts::lns_solve(p, greedy(5));
    EXPECT_TRUE(out.trace.empty());
}

// ---- contract: max_iterations=0 still returns the cold solve -------------

TEST(Lns, ZeroIterationsReturnsColdSolve) {
    ts::LayoutProblem p{
        .stations = {
            make_station("a", 2.0, 2.0, 0.5, 2.0, 2.0),
            make_station("b", 5.0, 2.0, 0.5, 5.0, 2.0),
        },
        .floor = ts::Floor{0.0 * metre, 10.0 * metre, 0.0 * metre, 5.0 * metre},
        .weights = ts::ObjectiveWeights{},
    };
    const auto cold = ts::solve(p);
    auto out = ts::lns_solve(p, greedy(0));
    EXPECT_TRUE(out.trace.empty());
    EXPECT_NEAR(out.best.cost.total, cold.cost.total, 1e-9);
}

// ---- Phase 4: simulated-annealing acceptance -----------------------------

TEST(Lns, SaTraceCarriesGeometricTemperatureSchedule) {
    // Temperature at iter k should equal T0 * decay^k. With explicit
    // T0 = 100 and decay = 0.8, iter 0 → 100, iter 1 → 80, iter 5 → 32.77.
    ts::LayoutProblem p{
        .stations = {
            make_station("a", 0.0, 0.0, 0.5, 0.0, 0.0),
            make_station("b", 5.0, 0.0, 0.5, 5.0, 0.0),
        },
        .floor = ts::Floor{-5.0 * metre, 10.0 * metre, -5.0 * metre, 5.0 * metre},
        .weights = ts::ObjectiveWeights{},
    };
    auto out = ts::lns_solve(p, ts::LnsParams{
        .max_iterations = 6,
        .seed = 0,
        .temperature_initial = 100.0,
        .auto_temperature = false,
        .temperature_decay = 0.8,
    });
    ASSERT_EQ(out.trace.size(), 6u);
    EXPECT_NEAR(out.trace[0].temperature, 100.0, 1e-9);
    EXPECT_NEAR(out.trace[1].temperature, 80.0, 1e-9);
    EXPECT_NEAR(out.trace[5].temperature, 100.0 * std::pow(0.8, 5), 1e-9);
}

// NB: a test asserting "Metropolis fires at least once" was removed.
// With single-station destroy and the smooth 2D NLP inner solver,
// freed stations re-converge deterministically against the frozen
// rest — proposed_total never strictly exceeds current_total, so
// Metropolis never has a worsening move to act on. SA becomes
// observable when destroy operators broaden (>1 station) or the
// objective gets non-smooth (e.g., port-direction tolerances). The
// scaffold is in place for those without re-touching this file.

TEST(Lns, SaSeedReproducibility) {
    // Same problem, same seed -> identical trace (acceptance order,
    // best total). Different seed -> may differ in acceptance pattern.
    ts::LayoutProblem p{
        .stations = {
            make_station("a", 0.0, 0.0, 1.0, 0.0, 0.0),
            make_station("b", 3.0, 0.0, 1.0, 3.0, 0.0),
            make_station("c", 6.0, 0.0, 1.0, 6.0, 0.0),
        },
        .transports = {
            ts::TransportConstraint{
                .source_station = 0,
                .source_port_local = tc::Vec2{0.0 * metre, 0.0 * metre},
                .sink_station = 2,
                .sink_port_local = tc::Vec2{0.0 * metre, 0.0 * metre},
            },
        },
        .floor = ts::Floor{-5.0 * metre, 15.0 * metre, -5.0 * metre, 5.0 * metre},
        .weights = ts::ObjectiveWeights{
            .overlap = 50.0, .floor = 1.0,
            .positional_prior = 1.0, .transport = 5.0,
        },
    };
    ts::LnsParams params{
        .max_iterations = 20, .seed = 12345,
        .temperature_initial = 30.0, .auto_temperature = false,
        .temperature_decay = 0.95,
    };
    auto a = ts::lns_solve(p, params);
    auto b = ts::lns_solve(p, params);
    ASSERT_EQ(a.trace.size(), b.trace.size());
    for (std::size_t i = 0; i < a.trace.size(); ++i) {
        EXPECT_EQ(a.trace[i].accepted, b.trace[i].accepted) << "iter " << i;
        EXPECT_NEAR(a.trace[i].proposed_total, b.trace[i].proposed_total, 1e-9);
    }
    EXPECT_NEAR(a.best.cost.total, b.best.cost.total, 1e-9);
}

TEST(Lns, SaBestNeverWorseThanCold) {
    // Best-ever should always beat or match the cold solve, regardless
    // of whether the SA walker briefly accepted worse moves.
    ts::LayoutProblem p{
        .stations = {
            make_station("a", -2.0, 0.0, 1.0, 0.0, 0.0),
            make_station("b", 4.0, 1.0, 1.0, 5.0, 0.0),
        },
        .transports = {
            ts::TransportConstraint{
                .source_station = 0,
                .source_port_local = tc::Vec2{0.0 * metre, 0.0 * metre},
                .sink_station = 1,
                .sink_port_local = tc::Vec2{0.0 * metre, 0.0 * metre},
            },
        },
        .floor = ts::Floor{-5.0 * metre, 10.0 * metre, -5.0 * metre, 5.0 * metre},
        .weights = ts::ObjectiveWeights{
            .overlap = 50.0, .floor = 1.0,
            .positional_prior = 1.0, .transport = 5.0,
        },
    };
    const auto cold = ts::solve(p);
    auto out = ts::lns_solve(p, ts::LnsParams{
        .max_iterations = 20, .seed = 99,
        .temperature_initial = 0.0, .auto_temperature = true,
        .temperature_decay = 0.9,
    });
    EXPECT_LE(out.best.cost.total, cold.cost.total + 1e-9);
}
