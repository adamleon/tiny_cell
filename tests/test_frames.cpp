// Tests for the core/ frame system (architecture.md §4). Centerpiece is
// the §4.3 regression: a sensor at (belt_length/2, 0, 0) in a belt frame
// rotated 90° in its station must land along the belt's ROTATED axis
// in world coords. Triplet addition fails this; transform composition
// passes. This is the #1 frame bug the rule exists to prevent.

#include <gtest/gtest.h>
#include <cmath>
#include <mp-units/systems/si.h>
#include <numbers>
#include <stdexcept>
#include <tinycell/geometry.hpp>
#include <tinycell/units.hpp>

namespace core = tinycell::core;
using namespace mp_units;
using mp_units::si::metre;
using mp_units::si::radian;

namespace {

constexpr double kTol = 1e-9;

bool near(core::Length a, double b_m, double tol = kTol) {
    return std::abs(a.numerical_value_in(metre) - b_m) < tol;
}
bool near(core::Angle a, double b_rad, double tol = kTol) {
    return std::abs(a.numerical_value_in(radian) - b_rad) < tol;
}

} // namespace

TEST(Frames, IdentityIsNoop) {
    auto id = core::Transform2D::identity();
    auto p = core::Vec2{3.0 * metre, -4.0 * metre};
    auto q = core::apply(id, p);
    EXPECT_TRUE(near(q.x, 3.0));
    EXPECT_TRUE(near(q.y, -4.0));
}

TEST(Frames, InverseRoundTrip) {
    // Apply a non-trivial transform then its inverse — should land back
    // at the original point.
    core::Transform2D t{
        2.0 * metre,
        -1.5 * metre,
        (std::numbers::pi / 3.0) * radian,
    };
    auto inv = core::inverse(t);
    auto p = core::Vec2{0.7 * metre, 1.3 * metre};
    auto round = core::apply(inv, core::apply(t, p));
    EXPECT_TRUE(near(round.x, 0.7, 1e-12));
    EXPECT_TRUE(near(round.y, 1.3, 1e-12));
}

TEST(Frames, ComposeMatchesSequentialApply) {
    // apply(compose(A, B), p) must equal apply(A, apply(B, p)).
    core::Transform2D a{1.0 * metre, 0.0 * metre, (std::numbers::pi / 2.0) * radian};
    core::Transform2D b{0.5 * metre, -0.25 * metre, (std::numbers::pi / 6.0) * radian};
    auto p = core::Vec2{2.0 * metre, 3.0 * metre};
    auto via_compose = core::apply(core::compose(a, b), p);
    auto via_sequential = core::apply(a, core::apply(b, p));
    EXPECT_TRUE(near(via_compose.x, via_sequential.x.numerical_value_in(metre), 1e-12));
    EXPECT_TRUE(near(via_compose.y, via_sequential.y.numerical_value_in(metre), 1e-12));
}

TEST(Frames, WorldResolvesToIdentity) {
    core::FrameTable table;
    auto t = table.resolve_to_world(core::kWorldFrame);
    EXPECT_TRUE(near(t.tx, 0.0));
    EXPECT_TRUE(near(t.ty, 0.0));
    EXPECT_TRUE(near(t.theta, 0.0));
}

TEST(Frames, ExpressInOwnFrameIsIdentity) {
    core::FrameTable table;
    core::Pose2D p{1.0 * metre, 2.0 * metre, 0.3 * radian, core::kWorldFrame};
    auto out = core::express(p, core::kWorldFrame, table);
    EXPECT_TRUE(near(out.x, 1.0));
    EXPECT_TRUE(near(out.y, 2.0));
    EXPECT_TRUE(near(out.theta, 0.3));
    EXPECT_EQ(out.frame, core::kWorldFrame);
}

// ---- The #1 frame bug regression (architecture.md §4.3, CLAUDE.md §2). ----
TEST(Frames, RotatedBeltSensorLandsAlongRotatedAxis) {
    // Setup: world ← station (identity, for simplicity) ← belt (rotated 90°).
    // A sensor at (belt_length/2, 0, 0) in the belt frame must land at
    // (0, belt_length/2, 90°) in world coords. Triplet addition would
    // give (belt_length/2, 0, 90°) — failing this test is the bug.
    core::FrameTable table;
    const auto half_pi = std::numbers::pi / 2.0;
    const core::Transform2D belt_in_world{
        0.0 * metre,
        0.0 * metre,
        half_pi * radian,
    };
    auto belt_frame = table.add(core::kWorldFrame, belt_in_world);

    const double belt_length_m = 2.0;
    core::Pose2D sensor_in_belt{
        (belt_length_m / 2.0) * metre,
        0.0 * metre,
        0.0 * radian,
        belt_frame,
    };

    auto sensor_in_world = core::express(sensor_in_belt, core::kWorldFrame, table);

    EXPECT_TRUE(near(sensor_in_world.x, 0.0));
    EXPECT_TRUE(near(sensor_in_world.y, belt_length_m / 2.0))
        << "sensor landed at y="
        << sensor_in_world.y.numerical_value_in(metre)
        << " instead of " << (belt_length_m / 2.0)
        << " — frame resolution used triplet addition, not transform composition";
    EXPECT_TRUE(near(sensor_in_world.theta, half_pi));
    EXPECT_EQ(sensor_in_world.frame, core::kWorldFrame);
}

TEST(Frames, NestedFrameComposition) {
    // World ← station (translated by (10, 5), no rotation)
    //       ← belt (rotated 90° in station, origin at station origin)
    //       ← sensor (at (1, 0) in belt frame).
    // Expected world: rotate (1,0) by 90° → (0,1), then translate by (10,5) → (10,6).
    core::FrameTable table;
    auto station = table.add(core::kWorldFrame,
        core::Transform2D{10.0 * metre, 5.0 * metre, 0.0 * radian});
    auto belt = table.add(station,
        core::Transform2D{0.0 * metre, 0.0 * metre, (std::numbers::pi / 2.0) * radian});
    core::Pose2D sensor{1.0 * metre, 0.0 * metre, 0.0 * radian, belt};

    auto sensor_world = core::express(sensor, core::kWorldFrame, table);
    EXPECT_TRUE(near(sensor_world.x, 10.0));
    EXPECT_TRUE(near(sensor_world.y, 6.0));
}

TEST(Frames, ExpressBetweenSiblingFrames) {
    // Two stations side-by-side in world; a pose in station A re-expressed
    // in station B should land at the difference.
    core::FrameTable table;
    auto stn_a = table.add(core::kWorldFrame,
        core::Transform2D{2.0 * metre, 0.0 * metre, 0.0 * radian});
    auto stn_b = table.add(core::kWorldFrame,
        core::Transform2D{5.0 * metre, 0.0 * metre, 0.0 * radian});
    core::Pose2D p_in_a{0.0 * metre, 0.0 * metre, 0.0 * radian, stn_a};

    auto p_in_b = core::express(p_in_a, stn_b, table);
    // A is at world x=2; B is at world x=5. A's origin in B's coords: x = 2-5 = -3.
    EXPECT_TRUE(near(p_in_b.x, -3.0));
    EXPECT_TRUE(near(p_in_b.y, 0.0));
}

TEST(Frames, AddRejectsNullParent) {
    core::FrameTable table;
    EXPECT_THROW(
        table.add(core::kNullFrame,
                  core::Transform2D{0.0 * metre, 0.0 * metre, 0.0 * radian}),
        std::invalid_argument);
}

TEST(Frames, AddRejectsNonExistentParent) {
    core::FrameTable table;
    EXPECT_THROW(
        table.add(core::FrameId{999},
                  core::Transform2D{0.0 * metre, 0.0 * metre, 0.0 * radian}),
        std::invalid_argument);
}

TEST(Frames, ResolveRejectsNullFrame) {
    core::FrameTable table;
    EXPECT_THROW(table.resolve_to_world(core::kNullFrame), std::invalid_argument);
}

TEST(Frames, ResolveRejectsOutOfRangeFrame) {
    core::FrameTable table;
    EXPECT_THROW(table.resolve_to_world(core::FrameId{999}), std::invalid_argument);
}
