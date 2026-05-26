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
