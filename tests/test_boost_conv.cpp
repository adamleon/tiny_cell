// Tests for adapters/boost_conv: hull and outward-buffer behave as
// expected on a few hand-crafted inputs. The point is to pin the
// strip-and-reattach pattern, not to test Boost.Geometry — Boost has
// its own test suite.

#include <gtest/gtest.h>
#include <mp-units/systems/si.h>
#include <stdexcept>
#include <tinycell/adapters/boost_conv.hpp>
#include <tinycell/geometry.hpp>

namespace core = tinycell::core;
namespace adapters = tinycell::adapters;
using mp_units::si::metre;

namespace {

core::Vec2 v(double x, double y) {
    return core::Vec2{x * metre, y * metre};
}

double x_of(const core::Vec2& p) { return p.x.numerical_value_in(metre); }
double y_of(const core::Vec2& p) { return p.y.numerical_value_in(metre); }

bool contains_point(const core::Polygon& p, double tx, double ty, double tol = 1e-9) {
    for (const auto& v : p) {
        if (std::abs(x_of(v) - tx) < tol && std::abs(y_of(v) - ty) < tol) {
            return true;
        }
    }
    return false;
}

} // namespace

TEST(BoostConv, HullOfTriangleKeepsAllThreeVertices) {
    core::Polygon triangle{v(0, 0), v(1, 0), v(0.5, 1)};
    auto hull = adapters::convex_hull(triangle);
    EXPECT_EQ(hull.size(), 3u);
    EXPECT_TRUE(contains_point(hull, 0, 0));
    EXPECT_TRUE(contains_point(hull, 1, 0));
    EXPECT_TRUE(contains_point(hull, 0.5, 1));
}

TEST(BoostConv, HullDropsInteriorPoint) {
    // Square with an interior fifth point; hull is the square, not the pentagon.
    core::Polygon shape{v(0, 0), v(2, 0), v(2, 2), v(0, 2), v(1, 1)};
    auto hull = adapters::convex_hull(shape);
    EXPECT_EQ(hull.size(), 4u);
    EXPECT_FALSE(contains_point(hull, 1, 1));
}

TEST(BoostConv, HullDegenerateInputUnchanged) {
    // Two-point degenerate input — the function returns it unmodified
    // (per the contract: degenerate inputs pass through, the enclose-all
    // invariant still holds).
    core::Polygon two_pts{v(0, 0), v(1, 1)};
    auto hull = adapters::convex_hull(two_pts);
    EXPECT_EQ(hull.size(), 2u);
}

TEST(BoostConv, BufferOutwardEnlargesPolygon) {
    // 1x1 square buffered outward by 0.1 m — every output vertex should
    // sit at distance ~0.1 from the nearest input edge, and the bounding
    // box should grow from [0,1]² to roughly [-0.1, 1.1]².
    core::Polygon square{v(0, 0), v(1, 0), v(1, 1), v(0, 1)};
    auto buffered = adapters::buffer_outward(square, 0.1 * metre);

    double min_x = 1e9, min_y = 1e9, max_x = -1e9, max_y = -1e9;
    for (const auto& p : buffered) {
        min_x = std::min(min_x, x_of(p));
        min_y = std::min(min_y, y_of(p));
        max_x = std::max(max_x, x_of(p));
        max_y = std::max(max_y, y_of(p));
    }
    EXPECT_NEAR(min_x, -0.1, 1e-6);
    EXPECT_NEAR(min_y, -0.1, 1e-6);
    EXPECT_NEAR(max_x, 1.1, 1e-6);
    EXPECT_NEAR(max_y, 1.1, 1e-6);
}

TEST(BoostConv, BufferRejectsNonPositiveAmount) {
    core::Polygon square{v(0, 0), v(1, 0), v(1, 1), v(0, 1)};
    EXPECT_THROW(adapters::buffer_outward(square, 0.0 * metre), std::invalid_argument);
    EXPECT_THROW(adapters::buffer_outward(square, -0.1 * metre), std::invalid_argument);
}
