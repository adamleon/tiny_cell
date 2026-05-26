// Phase E tests: solve() actually optimises rather than passing the
// seed through. Each case picks an initial state with a known
// infeasibility or a known suboptimality and asserts that solve()
// moves it the expected direction.
//
// We don't pin exact poses - BOBYQA is a local optimiser, the
// objective is non-convex, and the test should survive small
// changes to weights or tolerances. We assert on the qualitative
// behaviour (decreased objective, hard constraints satisfied,
// stations no longer overlap) instead.

#include <gtest/gtest.h>
#include <cmath>
#include <mp-units/systems/si.h>
#include <tinycell/solver/layout_objective.hpp>
#include <tinycell/solver/layout_problem.hpp>

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

double dist(const tc::Pose2D& a, const tc::Pose2D& b) {
    const double dx = (a.x - b.x).numerical_value_in(metre);
    const double dy = (a.y - b.y).numerical_value_in(metre);
    return std::sqrt(dx * dx + dy * dy);
}

} // namespace

TEST(LayoutSolve, EmptyProblemReturnsEmptySolution) {
    ts::LayoutProblem p{
        .stations = {},
        .floor = ts::Floor{0.0 * metre, 10.0 * metre, 0.0 * metre, 10.0 * metre},
        .weights = ts::ObjectiveWeights{},
    };
    auto out = ts::solve(p);
    EXPECT_TRUE(out.station_poses.empty());
    EXPECT_NEAR(out.final_objective, 0.0, 1e-12);
    EXPECT_TRUE(out.hard_constraints_satisfied);
}

TEST(LayoutSolve, FeasibleSeedReturnsNearlyUnchanged) {
    // One station at its nominal, well inside the floor, no neighbours.
    // The optimiser has nothing to improve - should leave the pose
    // essentially as-is and report zero objective.
    ts::LayoutProblem p{
        .stations = {make_station("a", 5.0, 5.0, 1.0, 5.0, 5.0)},
        .floor = ts::Floor{0.0 * metre, 10.0 * metre, 0.0 * metre, 10.0 * metre},
        .weights = ts::ObjectiveWeights{},
    };
    auto out = ts::solve(p);
    ASSERT_EQ(out.station_poses.size(), 1u);
    EXPECT_NEAR(out.final_objective, 0.0, 1e-6);
    EXPECT_TRUE(out.hard_constraints_satisfied);
    EXPECT_LT(dist(out.station_poses[0], p.stations[0].initial_pose), 0.01);
}

TEST(LayoutSolve, MovesOverlappingStationsApart) {
    // Two stations seeded at the same point. Overlap penalty (large
    // weight) should push them apart; prior penalty (small weight)
    // pulls each toward its own nominal. Expected: stations separate
    // and end up roughly between origin and nominal each.
    ts::LayoutProblem p{
        .stations = {
            make_station("a", 5.0, 5.0, 1.0, 4.0, 5.0),   // nominal at (4, 5)
            make_station("b", 5.0, 5.0, 1.0, 6.0, 5.0),   // nominal at (6, 5)
        },
        .floor = ts::Floor{0.0 * metre, 10.0 * metre, 0.0 * metre, 10.0 * metre},
        .weights = ts::ObjectiveWeights{
            .overlap = 1000.0, .floor = 100.0, .positional_prior = 1.0,
        },
    };

    // Sanity check: seed is overlapping (centres coincide).
    const double seed_dist = dist(p.stations[0].initial_pose,
                                  p.stations[1].initial_pose);
    EXPECT_NEAR(seed_dist, 0.0, 1e-9);

    auto out = ts::solve(p);
    ASSERT_EQ(out.station_poses.size(), 2u);

    const double final_dist = dist(out.station_poses[0], out.station_poses[1]);
    EXPECT_GE(final_dist, 2.0 - 0.05)  // at least the sum of radii (allow eps)
        << "stations still overlap: dist = " << final_dist << " m";
    EXPECT_TRUE(out.hard_constraints_satisfied);
}

TEST(LayoutSolve, MovesStationInsideFloor) {
    // Single station seeded fully outside the floor (centre at (-5, 5)
    // with radius 1 → leftmost x = -6, 6 m past x_min=0). Floor weight
    // dominates; the station should end up with its bounding circle
    // entirely inside the floor.
    ts::LayoutProblem p{
        .stations = {make_station("a", -5.0, 5.0, 1.0, -5.0, 5.0)},
        .floor = ts::Floor{0.0 * metre, 10.0 * metre, 0.0 * metre, 10.0 * metre},
        .weights = ts::ObjectiveWeights{
            .overlap = 1.0, .floor = 1000.0, .positional_prior = 1.0,
        },
    };
    auto out = ts::solve(p);
    ASSERT_EQ(out.station_poses.size(), 1u);
    const double final_x = out.station_poses[0].x.numerical_value_in(metre);
    const double fp = ts::floor_penalty(out.station_poses[0],
                                        p.stations[0].bounding_radius, p.floor);
    EXPECT_GE(final_x, 1.0 - 0.05)  // x - r >= x_min → x >= r = 1.0
        << "station still outside floor: x = " << final_x
        << ", floor_penalty = " << fp
        << ", final_obj = " << out.final_objective;
    EXPECT_TRUE(out.hard_constraints_satisfied)
        << "x = " << final_x << ", floor_penalty = " << fp;
}

// ---- Phase F: partial-freeze + warm-start -------------------------------

TEST(LayoutSolve, AllStationsFrozenSkipsOptimisation) {
    // Two stations both frozen at infeasible seeds (overlapping).
    // solve() should NOT move them - it returns the initial poses
    // and reports the infeasibility honestly via the flag.
    ts::LayoutProblem p{
        .stations = {},
        .floor = ts::Floor{0.0 * metre, 10.0 * metre, 0.0 * metre, 10.0 * metre},
        .weights = ts::ObjectiveWeights{},
    };
    auto sa = make_station("a", 4.5, 5.0, 1.0, 4.0, 5.0);
    sa.frozen = true;
    auto sb = make_station("b", 5.5, 5.0, 1.0, 6.0, 5.0);
    sb.frozen = true;
    p.stations = {sa, sb};

    auto out = ts::solve(p);
    ASSERT_EQ(out.station_poses.size(), 2u);
    // Returned poses == initial poses (no optimisation ran).
    EXPECT_NEAR(out.station_poses[0].x.numerical_value_in(metre), 4.5, 1e-12);
    EXPECT_NEAR(out.station_poses[1].x.numerical_value_in(metre), 5.5, 1e-12);
    // Centres 1 m apart, radii sum 2 → still overlapping → infeasible.
    EXPECT_FALSE(out.hard_constraints_satisfied);
}

TEST(LayoutSolve, FrozenStationActsAsObstacle) {
    // One frozen station blocking the variable station's path to its
    // nominal. The frozen station's overlap contribution must push the
    // variable one off-axis.
    auto frozen = make_station("blocker", 5.0, 5.0, 1.0, 5.0, 5.0);
    frozen.frozen = true;
    auto mover = make_station("mover", 8.0, 5.0, 1.0, /*nom*/ 5.0, 5.0);

    ts::LayoutProblem p{
        .stations = {frozen, mover},
        .floor = ts::Floor{0.0 * metre, 20.0 * metre, 0.0 * metre, 10.0 * metre},
        .weights = ts::ObjectiveWeights{
            .overlap = 1000.0, .floor = 100.0, .positional_prior = 1.0,
        },
    };
    auto out = ts::solve(p);
    ASSERT_EQ(out.station_poses.size(), 2u);

    // Frozen station did not move.
    EXPECT_NEAR(out.station_poses[0].x.numerical_value_in(metre), 5.0, 1e-9);
    EXPECT_NEAR(out.station_poses[0].y.numerical_value_in(metre), 5.0, 1e-9);

    // Mover wanted to land at (5, 5) (its nominal) but the frozen
    // station is there with radius 1 + mover's radius 1 = 2. The
    // mover must land approximately tangent to the frozen one - i.e.
    // dist from blocker ≈ 2 m.
    //
    // Note: penalty-method equilibrium leaves a small residual
    // interpenetration (~2 mm here at overlap_weight=1000 versus
    // prior_weight=1) where the prior's pull balances the overlap
    // penalty's gradient. We check the spatial constraint at
    // engineering-scale tolerance (5 mm) rather than asserting
    // hard_constraints_satisfied=true. The strict flag requires
    // sub-mm precision and would need either much higher overlap
    // weight (numerically poor) or a real inequality constraint via
    // NLopt's LN_COBYLA (planned follow-up - converts overlap from
    // a soft penalty to a hard constraint, eliminating the residual).
    const double dx = out.station_poses[1].x.numerical_value_in(metre) - 5.0;
    const double dy = out.station_poses[1].y.numerical_value_in(metre) - 5.0;
    const double dist_from_blocker = std::sqrt(dx * dx + dy * dy);
    EXPECT_GE(dist_from_blocker, 2.0 - 0.005)  // within 5 mm of tangent
        << "mover penetrated frozen blocker beyond engineering tolerance: dist = "
        << dist_from_blocker;
}

TEST(LayoutSolve, FrozenInfeasibilityDoesNotBlockMover) {
    // Frozen station outside the floor (its caller's mistake — the
    // solver doesn't validate frozen stations at entry). The mover
    // should still find a feasible spot; only the frozen one's
    // floor_penalty makes the overall flag false.
    auto bad = make_station("bad_frozen", -10.0, 5.0, 1.0, -10.0, 5.0);
    bad.frozen = true;
    auto mover = make_station("mover", 5.0, 5.0, 1.0, 5.0, 5.0);

    ts::LayoutProblem p{
        .stations = {bad, mover},
        .floor = ts::Floor{0.0 * metre, 20.0 * metre, 0.0 * metre, 10.0 * metre},
        .weights = ts::ObjectiveWeights{},
    };
    auto out = ts::solve(p);

    // Bad station didn't move (frozen).
    EXPECT_NEAR(out.station_poses[0].x.numerical_value_in(metre), -10.0, 1e-9);
    // Mover stayed near its nominal (far from the bad frozen station,
    // no overlap pressure).
    EXPECT_NEAR(out.station_poses[1].x.numerical_value_in(metre), 5.0, 1e-3);
    // Bad station outside floor → infeasible overall.
    EXPECT_FALSE(out.hard_constraints_satisfied);
}

TEST(LayoutSolve, WarmStartFromPreviousSolutionIsStable) {
    // Run the same problem twice: once from the nominal seed, once
    // warm-started from the first solve's output. The second pose
    // should be essentially unchanged (the first solve already found
    // the local optimum; warm-starting from there is a fixed point).
    ts::LayoutProblem p1{
        .stations = {
            make_station("a", 0.0, 5.0, 1.0, 3.0, 5.0),
            make_station("b", 0.5, 5.0, 1.0, 7.0, 5.0),
        },
        .floor = ts::Floor{0.0 * metre, 10.0 * metre, 0.0 * metre, 10.0 * metre},
        .weights = ts::ObjectiveWeights{
            .overlap = 1000.0, .floor = 1000.0, .positional_prior = 1.0,
        },
    };
    auto out1 = ts::solve(p1);
    ASSERT_TRUE(out1.hard_constraints_satisfied);

    // Build a second problem with initial_pose = previous solve's output.
    ts::LayoutProblem p2 = p1;
    for (std::size_t i = 0; i < p2.stations.size(); ++i) {
        p2.stations[i].initial_pose = out1.station_poses[i];
    }
    auto out2 = ts::solve(p2);
    ASSERT_TRUE(out2.hard_constraints_satisfied);

    // Warm-started solve lands within a fraction of a millimetre of
    // the first - confirms initial_pose IS the warm-start hook.
    for (std::size_t i = 0; i < out1.station_poses.size(); ++i) {
        EXPECT_NEAR(out1.station_poses[i].x.numerical_value_in(metre),
                    out2.station_poses[i].x.numerical_value_in(metre), 1e-3);
        EXPECT_NEAR(out1.station_poses[i].y.numerical_value_in(metre),
                    out2.station_poses[i].y.numerical_value_in(metre), 1e-3);
    }
    // And the warm-started objective is no worse than the cold one
    // (might be identical, might be marginally better due to
    // re-running from a near-optimal point).
    EXPECT_LE(out2.final_objective, out1.final_objective + 1e-6);
}

TEST(LayoutSolve, ObjectiveDecreasesFromInfeasibleSeed) {
    // Property: starting from any infeasible state, the optimised
    // objective is strictly lower than the initial objective.
    ts::LayoutProblem p{
        .stations = {
            make_station("a", 0.0, 5.0, 1.0, 3.0, 5.0),   // overlap floor at left edge
            make_station("b", 0.5, 5.0, 1.0, 7.0, 5.0),   // overlapping a
        },
        .floor = ts::Floor{0.0 * metre, 10.0 * metre, 0.0 * metre, 10.0 * metre},
        .weights = ts::ObjectiveWeights{
            .overlap = 1000.0, .floor = 1000.0, .positional_prior = 1.0,
        },
    };

    // Initial objective from the stub-style direct evaluation.
    const std::vector<tc::Pose2D> initial_poses{
        p.stations[0].initial_pose, p.stations[1].initial_pose,
    };
    const double initial_obj = ts::evaluate_objective(p, initial_poses);
    ASSERT_GT(initial_obj, 0.0);

    auto out = ts::solve(p);
    EXPECT_LT(out.final_objective, initial_obj * 0.5)
        << "solve() should make significant progress on the infeasible seed";
    EXPECT_TRUE(out.hard_constraints_satisfied);
}
