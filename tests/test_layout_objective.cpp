// Tests for the Phase D objective encoders. Each test pins one of
// the three penalty terms in isolation: overlap (pair-wise),
// floor (per-station), prior (per-station). evaluate_objective
// stitches them together with weights — its tests verify the sum.

#include <gtest/gtest.h>
#include <cmath>
#include <mp-units/systems/si.h>
#include <numbers>
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

// ---- transport_penalty ---------------------------------------------------

TEST(TransportPenalty, ZeroWhenPortsCoincide) {
    // Two stations sitting on top of each other with ports at their
    // origins → port_world coincides for both → zero penalty.
    auto a = pose_at(2.0, 3.0);
    auto b = pose_at(2.0, 3.0);
    tc::Vec2 zero{0.0 * metre, 0.0 * metre};
    EXPECT_NEAR(ts::transport_penalty(a, zero, b, zero), 0.0, 1e-12);
}

TEST(TransportPenalty, SquaredDistanceBetweenPortWorldPositions) {
    // Station A at (0, 0) with port at (1, 0) station-frame → port_world = (1, 0).
    // Station B at (5, 0) with port at (-1, 0) station-frame → port_world = (4, 0).
    // Distance = 3, penalty = 9.
    auto a = pose_at(0.0, 0.0);
    auto b = pose_at(5.0, 0.0);
    tc::Vec2 port_a{1.0 * metre, 0.0 * metre};
    tc::Vec2 port_b{-1.0 * metre, 0.0 * metre};
    EXPECT_NEAR(ts::transport_penalty(a, port_a, b, port_b), 9.0, 1e-12);
}

TEST(TransportPenalty, RespectsStationRotation) {
    // Station A at origin rotated 90° CCW with port at local (1, 0).
    // After rotation, the port points along +y; port_world = (0, 1).
    // Station B at (0, 3) with port at origin; port_world = (0, 3).
    // Distance = 2, penalty = 4.
    tc::Pose2D a{
        .x = 0.0 * metre, .y = 0.0 * metre,
        .theta = (std::numbers::pi / 2.0) * radian, .frame = tc::kWorldFrame,
    };
    auto b = pose_at(0.0, 3.0);
    tc::Vec2 port_a{1.0 * metre, 0.0 * metre};
    tc::Vec2 port_b{0.0 * metre, 0.0 * metre};
    EXPECT_NEAR(ts::transport_penalty(a, port_a, b, port_b), 4.0, 1e-9);
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

TEST(EvaluateObjective, IncludesTransportTerm) {
    // Two stations far enough apart not to overlap; one transport
    // edge connects A's port at (1, 0) station-frame to B's port at
    // (-1, 0) station-frame. With A at (0, 0) and B at (5, 0), port
    // distance = (4 - 1) = 3 m → transport penalty = 9. No overlap, no
    // floor violation, prior at exact nominal: only the transport term
    // contributes. Weight = 1 → total = 9.
    ts::LayoutProblem p{
        .stations = {
            ts::StationProblem{
                .id = "a", .buffered_hull = {},
                .bounding_radius = 0.5 * metre,
                .nominal = tc::Vec2{0.0 * metre, 0.0 * metre},
                .initial_pose = pose_at(0.0, 0.0),
            },
            ts::StationProblem{
                .id = "b", .buffered_hull = {},
                .bounding_radius = 0.5 * metre,
                .nominal = tc::Vec2{5.0 * metre, 0.0 * metre},
                .initial_pose = pose_at(5.0, 0.0),
            },
        },
        .transports = {
            ts::TransportConstraint{
                .source_station = 0,
                .source_port_local = tc::Vec2{1.0 * metre, 0.0 * metre},
                .sink_station = 1,
                .sink_port_local = tc::Vec2{-1.0 * metre, 0.0 * metre},
            },
        },
        .floor = ts::Floor{-10.0 * metre, 10.0 * metre, -10.0 * metre, 10.0 * metre},
        .weights = ts::ObjectiveWeights{
            .overlap = 1.0, .floor = 1.0, .positional_prior = 0.0, .transport = 1.0,
        },
    };
    const std::vector<tc::Pose2D> poses{
        p.stations[0].initial_pose, p.stations[1].initial_pose,
    };
    EXPECT_NEAR(ts::evaluate_objective(p, poses), 9.0, 1e-9);
}

TEST(EvaluateObjective, TransportTermDecreasesAsStationsApproach) {
    // Single transport edge, no other penalties active. Moving the
    // sink station from x=10 to x=8 to x=6 must monotonically reduce
    // the transport penalty.
    ts::LayoutProblem p{
        .stations = {
            ts::StationProblem{
                .id = "src", .buffered_hull = {},
                .bounding_radius = 0.1 * metre,
                .nominal = tc::Vec2{0.0 * metre, 0.0 * metre},
                .initial_pose = pose_at(0.0, 0.0),
            },
            ts::StationProblem{
                .id = "dst", .buffered_hull = {},
                .bounding_radius = 0.1 * metre,
                .nominal = tc::Vec2{10.0 * metre, 0.0 * metre},
                .initial_pose = pose_at(10.0, 0.0),
            },
        },
        .transports = {
            ts::TransportConstraint{
                .source_station = 0,
                .source_port_local = tc::Vec2{0.0 * metre, 0.0 * metre},
                .sink_station = 1,
                .sink_port_local = tc::Vec2{0.0 * metre, 0.0 * metre},
            },
        },
        .floor = ts::Floor{-20.0 * metre, 20.0 * metre, -10.0 * metre, 10.0 * metre},
        .weights = ts::ObjectiveWeights{
            .overlap = 0.0, .floor = 0.0, .positional_prior = 0.0, .transport = 1.0,
        },
    };
    const auto src = p.stations[0].initial_pose;
    const double obj_at_10 = ts::evaluate_objective(p, {src, pose_at(10.0, 0.0)});
    const double obj_at_8  = ts::evaluate_objective(p, {src, pose_at( 8.0, 0.0)});
    const double obj_at_6  = ts::evaluate_objective(p, {src, pose_at( 6.0, 0.0)});
    EXPECT_GT(obj_at_10, obj_at_8);
    EXPECT_GT(obj_at_8,  obj_at_6);
    // Concrete check: at 10 the distance is 10 → penalty 100; at 6 → 36.
    EXPECT_NEAR(obj_at_10, 100.0, 1e-9);
    EXPECT_NEAR(obj_at_6,   36.0, 1e-9);
}

TEST(DecomposeObjective, FieldsSumToTotalAndMatchEvaluate) {
    // Construct a problem where each term contributes something distinct
    // and check that decompose returns per-term values that sum to total
    // and match what evaluate_objective alone reports.
    ts::LayoutProblem p{
        .stations = {
            ts::StationProblem{
                .id = "a", .buffered_hull = {},
                .bounding_radius = 1.0 * metre,
                .nominal = tc::Vec2{2.0 * metre, 5.0 * metre},  // 1 m off pose
                .initial_pose = pose_at(1.0, 5.0),
            },
            ts::StationProblem{
                .id = "b", .buffered_hull = {},
                .bounding_radius = 1.0 * metre,
                .nominal = tc::Vec2{3.0 * metre, 5.0 * metre},  // 1 m off pose
                .initial_pose = pose_at(2.0, 5.0),  // 1 m from a, depth 1 → overlap 1
            },
        },
        .transports = {
            ts::TransportConstraint{
                .source_station = 0,
                .source_port_local = tc::Vec2{0.0 * metre, 0.0 * metre},
                .sink_station = 1,
                .sink_port_local = tc::Vec2{0.0 * metre, 0.0 * metre},
            },  // ports at station centres → distance 1, penalty 1
        },
        .floor = ts::Floor{0.0 * metre, 10.0 * metre, 0.0 * metre, 10.0 * metre},
        .weights = ts::ObjectiveWeights{
            .overlap = 10.0, .floor = 100.0,
            .positional_prior = 2.0, .transport = 5.0,
        },
    };
    const std::vector<tc::Pose2D> poses{
        p.stations[0].initial_pose, p.stations[1].initial_pose,
    };
    const auto b = ts::decompose_objective(p, poses);
    EXPECT_NEAR(b.overlap, 10.0 * 1.0, 1e-9);              // depth 1² × weight 10
    EXPECT_NEAR(b.floor, 0.0, 1e-9);                       // both inside floor
    EXPECT_NEAR(b.prior, 2.0 * (1.0 + 1.0), 1e-9);         // 1 m off each, sum × weight 2
    EXPECT_NEAR(b.transport, 5.0 * 1.0, 1e-9);             // dist² 1 × weight 5
    EXPECT_NEAR(b.total, b.overlap + b.floor + b.prior + b.transport, 1e-9);
    EXPECT_NEAR(b.total, ts::evaluate_objective(p, poses), 1e-9);
}

TEST(EvaluateObjective, RejectsTransportWithOutOfRangeStation) {
    ts::LayoutProblem p{
        .stations = {
            ts::StationProblem{
                .id = "a", .buffered_hull = {},
                .bounding_radius = 0.1 * metre,
                .nominal = tc::Vec2{0.0 * metre, 0.0 * metre},
                .initial_pose = pose_at(0.0, 0.0),
            },
        },
        .transports = {
            ts::TransportConstraint{
                .source_station = 0,
                .source_port_local = tc::Vec2{0.0 * metre, 0.0 * metre},
                .sink_station = 5,  // out of range
                .sink_port_local = tc::Vec2{0.0 * metre, 0.0 * metre},
            },
        },
        .floor = ts::Floor{-10.0 * metre, 10.0 * metre, -10.0 * metre, 10.0 * metre},
        .weights = ts::ObjectiveWeights{},
    };
    EXPECT_THROW(ts::evaluate_objective(p, {p.stations[0].initial_pose}),
                 std::invalid_argument);
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
    EXPECT_NEAR(out.cost.total, 0.0, 1e-12);
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
    EXPECT_GT(out.cost.total, 0.0);
    EXPECT_FALSE(out.hard_constraints_satisfied);
}
