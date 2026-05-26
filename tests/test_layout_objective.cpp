// Tests for the Phase D objective encoders. Each test pins one of
// the three penalty terms in isolation: overlap (pair-wise),
// floor (per-station), prior (per-station). evaluate_objective
// stitches them together with weights — its tests verify the sum.

#include <gtest/gtest.h>
#include <cmath>
#include <mp-units/systems/si.h>
#include <stdexcept>
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
                                double x, double y, double radius,
                                double nom_x = 0.0, double nom_y = 0.0) {
    return ts::StationProblem{
        .id = id,
        .buffered_hull = {},  // not consumed at MVP
        .bounding_radius = radius * metre,
        .nominal = tc::Vec2{nom_x * metre, nom_y * metre},
        .initial_pose = pose_at(x, y),
    };
}

} // namespace

// ---- overlap_penalty ----------------------------------------------------

TEST(OverlapPenalty, ZeroWhenFarApart) {
    auto a = pose_at(0, 0);
    auto b = pose_at(10, 0);
    EXPECT_NEAR(ts::overlap_penalty(a, 1.0 * metre, b, 1.0 * metre), 0.0, 1e-12);
}

TEST(OverlapPenalty, ZeroAtExactTouch) {
    // Circles of radius 1 at (0,0) and (2,0) — centres exactly r_a+r_b apart.
    auto a = pose_at(0, 0);
    auto b = pose_at(2, 0);
    EXPECT_NEAR(ts::overlap_penalty(a, 1.0 * metre, b, 1.0 * metre), 0.0, 1e-12);
}

TEST(OverlapPenalty, QuadraticInInterpenetrationDepth) {
    // Centres 1 m apart, radii summing to 2 m → depth = 1 m → penalty = 1.
    auto a = pose_at(0, 0);
    auto b = pose_at(1, 0);
    EXPECT_NEAR(ts::overlap_penalty(a, 1.0 * metre, b, 1.0 * metre), 1.0, 1e-12);
    // Centres 0.5 m apart → depth = 1.5 m → penalty = 2.25.
    auto c = pose_at(0.5, 0);
    EXPECT_NEAR(ts::overlap_penalty(a, 1.0 * metre, c, 1.0 * metre), 2.25, 1e-12);
}

TEST(OverlapPenalty, IndependentOfDirection) {
    // Same depth on +X vs. +Y vs. diagonal — penalty identical.
    auto a = pose_at(0, 0);
    auto bx = pose_at(1, 0);
    auto by = pose_at(0, 1);
    auto bd = pose_at(1.0 / std::sqrt(2.0), 1.0 / std::sqrt(2.0));
    const double px = ts::overlap_penalty(a, 1.0 * metre, bx, 1.0 * metre);
    const double py = ts::overlap_penalty(a, 1.0 * metre, by, 1.0 * metre);
    const double pd = ts::overlap_penalty(a, 1.0 * metre, bd, 1.0 * metre);
    EXPECT_NEAR(px, py, 1e-12);
    EXPECT_NEAR(px, pd, 1e-12);
}

// ---- floor_penalty -------------------------------------------------------

TEST(FloorPenalty, ZeroWhenWhollyInside) {
    ts::Floor floor{
        .x_min = -10.0 * metre, .x_max = 10.0 * metre,
        .y_min = -10.0 * metre, .y_max = 10.0 * metre};
    auto p = pose_at(0, 0);
    EXPECT_NEAR(ts::floor_penalty(p, 1.0 * metre, floor), 0.0, 1e-12);
}

TEST(FloorPenalty, ZeroWhenTouchingEdges) {
    // Bounding circle of radius 1 with centre exactly 1 m from each edge.
    ts::Floor floor{
        .x_min = 0.0 * metre, .x_max = 10.0 * metre,
        .y_min = 0.0 * metre, .y_max = 10.0 * metre};
    auto p = pose_at(1.0, 5.0);  // touches left edge
    EXPECT_NEAR(ts::floor_penalty(p, 1.0 * metre, floor), 0.0, 1e-12);
}

TEST(FloorPenalty, QuadraticInOutOfBoundsDepth) {
    ts::Floor floor{
        .x_min = 0.0 * metre, .x_max = 10.0 * metre,
        .y_min = 0.0 * metre, .y_max = 10.0 * metre};
    // Centre at (0, 5), radius 1 → leftmost point at x = -1 → 1 m past x_min.
    auto p = pose_at(0.0, 5.0);
    EXPECT_NEAR(ts::floor_penalty(p, 1.0 * metre, floor), 1.0, 1e-12);
    // Centre at (-1, 5) → leftmost point at x = -2 → 2 m past x_min.
    auto q = pose_at(-1.0, 5.0);
    EXPECT_NEAR(ts::floor_penalty(q, 1.0 * metre, floor), 4.0, 1e-12);
}

TEST(FloorPenalty, SumsAcrossMultipleEdges) {
    // Bounding circle at (0, 0) with radius 1: leftmost x = -1 (1 m past
    // x_min=0) AND bottommost y = -1 (1 m past y_min=0). Penalty = 1² + 1² = 2.
    ts::Floor floor{
        .x_min = 0.0 * metre, .x_max = 10.0 * metre,
        .y_min = 0.0 * metre, .y_max = 10.0 * metre};
    auto p = pose_at(0.0, 0.0);
    EXPECT_NEAR(ts::floor_penalty(p, 1.0 * metre, floor), 2.0, 1e-12);
}

// ---- prior_penalty -------------------------------------------------------

TEST(PriorPenalty, ZeroAtNominal) {
    auto pose = pose_at(3.0, 4.0);
    tc::Vec2 nominal{3.0 * metre, 4.0 * metre};
    EXPECT_NEAR(ts::prior_penalty(pose, nominal), 0.0, 1e-12);
}

TEST(PriorPenalty, SquaredDistance) {
    auto pose = pose_at(3.0, 4.0);
    tc::Vec2 nominal{0.0 * metre, 0.0 * metre};
    // dx=3, dy=4 → 9 + 16 = 25.
    EXPECT_NEAR(ts::prior_penalty(pose, nominal), 25.0, 1e-12);
}

// ---- evaluate_objective + hard_constraints_satisfied ---------------------

TEST(EvaluateObjective, RejectsPoseCountMismatch) {
    ts::LayoutProblem p{
        .stations = {make_station("a", 0, 0, 1)},
        .floor = ts::Floor{0.0 * metre, 10.0 * metre, 0.0 * metre, 10.0 * metre},
        .weights = ts::ObjectiveWeights{},
    };
    EXPECT_THROW(ts::evaluate_objective(p, {}), std::invalid_argument);
}

TEST(EvaluateObjective, SumsAllTermsWithWeights) {
    // Two stations overlapping (centres 1 m apart, radii sum to 2 →
    // interpenetration depth 1 → overlap penalty 1); station a's
    // leftmost point hangs 1 m past x_min (floor penalty 1, only a
    // violates); each station 1 m off its nominal (prior 1 each).
    // Weights chosen to multiply each term distinctly so the
    // contributions are individually visible.
    ts::LayoutProblem p{
        .stations = {
            // Station a: centre (0, 5), radius 1 → leftmost x = -1, 1 m past
            // x_min=0 → floor 1². Nominal at (1, 5) → prior 1.
            ts::StationProblem{
                .id = "a", .buffered_hull = {},
                .bounding_radius = 1.0 * metre,
                .nominal = tc::Vec2{1.0 * metre, 5.0 * metre},
                .initial_pose = pose_at(0.0, 5.0),
            },
            // Station b: centre (1, 5), radius 1 → wholly in floor.
            // Nominal at (2, 5) → prior 1. Centre-to-centre 1, radii sum 2 →
            // overlap depth 1 → penalty 1.
            ts::StationProblem{
                .id = "b", .buffered_hull = {},
                .bounding_radius = 1.0 * metre,
                .nominal = tc::Vec2{2.0 * metre, 5.0 * metre},
                .initial_pose = pose_at(1.0, 5.0),
            },
        },
        .floor = ts::Floor{0.0 * metre, 10.0 * metre, 0.0 * metre, 10.0 * metre},
        .weights = ts::ObjectiveWeights{
            .overlap = 100.0,
            .floor = 10.0,
            .positional_prior = 1.0,
        },
    };
    const auto poses = std::vector<tc::Pose2D>{
        p.stations[0].initial_pose,
        p.stations[1].initial_pose,
    };
    const double obj = ts::evaluate_objective(p, poses);
    // 100*1 (overlap) + 10*1 (floor on a only) + 1*1 (prior a) + 1*1 (prior b) = 112.
    EXPECT_NEAR(obj, 112.0, 1e-9);
}

TEST(HardConstraintsSatisfied, TrueWhenAllPenaltiesZero) {
    ts::LayoutProblem p{
        .stations = {
            make_station("a", 2, 5, 1, 2, 5),
            make_station("b", 5, 5, 1, 5, 5),  // 3 m apart, sum radii = 2 → clear
        },
        .floor = ts::Floor{0.0 * metre, 10.0 * metre, 0.0 * metre, 10.0 * metre},
        .weights = ts::ObjectiveWeights{},
    };
    const std::vector<tc::Pose2D> poses{
        p.stations[0].initial_pose,
        p.stations[1].initial_pose,
    };
    EXPECT_TRUE(ts::hard_constraints_satisfied(p, poses));
}

TEST(HardConstraintsSatisfied, FalseOnOverlap) {
    ts::LayoutProblem p{
        .stations = {
            make_station("a", 0, 5, 1),
            make_station("b", 1, 5, 1),  // overlapping
        },
        .floor = ts::Floor{-5.0 * metre, 10.0 * metre, 0.0 * metre, 10.0 * metre},
        .weights = ts::ObjectiveWeights{},
    };
    const std::vector<tc::Pose2D> poses{
        p.stations[0].initial_pose,
        p.stations[1].initial_pose,
    };
    EXPECT_FALSE(ts::hard_constraints_satisfied(p, poses));
}

TEST(HardConstraintsSatisfied, FalseOnFloorViolation) {
    ts::LayoutProblem p{
        .stations = {make_station("a", -1, 5, 1)},
        .floor = ts::Floor{0.0 * metre, 10.0 * metre, 0.0 * metre, 10.0 * metre},
        .weights = ts::ObjectiveWeights{},
    };
    EXPECT_FALSE(ts::hard_constraints_satisfied(p, {p.stations[0].initial_pose}));
}

// ---- solve() stub --------------------------------------------------------

TEST(SolveStub, ReturnsInitialPosesAndEvaluatesObjective) {
    ts::LayoutProblem p{
        .stations = {
            make_station("a", 2, 5, 1, 2, 5),  // at nominal, in floor
            make_station("b", 5, 5, 1, 5, 5),  // at nominal, in floor, clear of a
        },
        .floor = ts::Floor{0.0 * metre, 10.0 * metre, 0.0 * metre, 10.0 * metre},
        .weights = ts::ObjectiveWeights{},
    };
    auto out = ts::solve(p);
    ASSERT_EQ(out.station_poses.size(), 2u);
    EXPECT_NEAR(out.station_poses[0].x.numerical_value_in(metre), 2.0, 1e-12);
    EXPECT_NEAR(out.station_poses[1].x.numerical_value_in(metre), 5.0, 1e-12);
    EXPECT_NEAR(out.final_objective, 0.0, 1e-12);
    EXPECT_TRUE(out.hard_constraints_satisfied);
}

TEST(SolveStub, FlagsInfeasibleInitialState) {
    // Two stations overlapping at their seeds. The stub doesn't fix that —
    // it just reports the seeds + their objective + the infeasibility flag.
    ts::LayoutProblem p{
        .stations = {
            make_station("a", 0, 5, 1),
            make_station("b", 1, 5, 1),
        },
        .floor = ts::Floor{-5.0 * metre, 10.0 * metre, 0.0 * metre, 10.0 * metre},
        .weights = ts::ObjectiveWeights{},
    };
    auto out = ts::solve(p);
    EXPECT_GT(out.final_objective, 0.0);
    EXPECT_FALSE(out.hard_constraints_satisfied);
}
