// Tests for step-6 M2 belt routing: belt_footprint() geometry and
// route_belts() (length-based catalog selection + belt-vs-station collision).
// Problems/solutions are built by hand so the geometry is exact; route_belts
// only reads solution.station_poses + solution.transports + problem.stations.

#include <gtest/gtest.h>
#include <cmath>
#include <vector>
#include <mp-units/systems/si.h>
#include <tinycell/model/belt.hpp>
#include <tinycell/solver/belt_routing.hpp>
#include <tinycell/solver/layout_problem.hpp>

namespace tc = tinycell::core;
namespace ts = tinycell::solver;
using mp_units::si::metre;
using mp_units::si::radian;
using mp_units::si::second;

namespace {

tc::Vec2 v(double x, double y) { return tc::Vec2{x * metre, y * metre}; }

tc::Pose2D pose(double x, double y) {
    return tc::Pose2D{x * metre, y * metre, 0.0 * radian, tc::kWorldFrame};
}

tc::BeltSpec belt(const std::string& id, double width, double lo, double hi,
                  double price) {
    return tc::BeltSpec{
        .id = id, .model_name = id, .family = "flat",
        .belt_width = width * metre,
        .min_length = lo * metre, .max_length = hi * metre,
        .max_speed = 0.5 * (metre / second),
        .list_price_eur = price,
    };
}

ts::StationProblem station(const std::string& id, tc::Polygon hull) {
    return ts::StationProblem{
        .id = id, .buffered_hull = std::move(hull),
        .bounding_radius = 0.0 * metre,
        .nominal = v(0, 0),
        .initial_pose = pose(0, 0),
    };
}

ts::TransportConstraint xport(std::size_t src, tc::Vec2 sp,
                              std::size_t snk, tc::Vec2 kp) {
    return ts::TransportConstraint{
        .source_station = src, .source_port_local = sp,
        .sink_station = snk, .sink_port_local = kp,
    };
}

// A unit square (side `s`) centred on the station origin, for collision hulls.
tc::Polygon box(double s) {
    const double h = 0.5 * s;
    return tc::Polygon{v(-h, -h), v(h, -h), v(h, h), v(-h, h)};
}

ts::LayoutProblem make_problem(std::vector<ts::StationProblem> stations) {
    return ts::LayoutProblem{
        .stations = std::move(stations),
        .transports = {},
        .floor = ts::Floor{-50.0 * metre, 50.0 * metre, -50.0 * metre, 50.0 * metre},
        .weights = ts::ObjectiveWeights{},
    };
}

} // namespace

// ---- belt_footprint ------------------------------------------------------

TEST(BeltFootprint, HorizontalBeltRectangle) {
    // Belt (0,0)->(4,0), width 2 → rectangle spanning y in [-1, 1].
    const auto fp = ts::belt_footprint(v(0, 0), v(4, 0), 2.0 * metre);
    ASSERT_EQ(fp.size(), 4U);
    EXPECT_NEAR(fp[0].x.numerical_value_in(metre), 0.0, 1e-9);
    EXPECT_NEAR(fp[0].y.numerical_value_in(metre), 1.0, 1e-9);
    EXPECT_NEAR(fp[1].x.numerical_value_in(metre), 4.0, 1e-9);
    EXPECT_NEAR(fp[1].y.numerical_value_in(metre), 1.0, 1e-9);
    EXPECT_NEAR(fp[2].x.numerical_value_in(metre), 4.0, 1e-9);
    EXPECT_NEAR(fp[2].y.numerical_value_in(metre), -1.0, 1e-9);
    EXPECT_NEAR(fp[3].x.numerical_value_in(metre), 0.0, 1e-9);
    EXPECT_NEAR(fp[3].y.numerical_value_in(metre), -1.0, 1e-9);
}

TEST(BeltFootprint, DegenerateZeroLengthIsEmpty) {
    EXPECT_TRUE(ts::belt_footprint(v(2, 2), v(2, 2), 1.0 * metre).empty());
}

// ---- route_belts ---------------------------------------------------------

TEST(RouteBelts, RoutesStraightBeltBetweenResolvedPorts) {
    auto prob = make_problem({station("src", {}), station("dst", {})});
    ts::LayoutSolution sol;
    sol.station_poses = {pose(0, 0), pose(5, 0)};
    sol.transports = {xport(0, v(0, 0), 1, v(0, 0))};

    const std::vector<tc::BeltSpec> cat{belt("b", 0.6, 1.0, 8.0, 1000.0)};
    const auto belts = ts::route_belts(prob, sol, cat);

    ASSERT_EQ(belts.size(), 1U);
    EXPECT_TRUE(belts[0].fitted);
    EXPECT_EQ(belts[0].belt_catalog_id, "b");
    EXPECT_NEAR(belts[0].length.numerical_value_in(metre), 5.0, 1e-9);
    EXPECT_NEAR(belts[0].width.numerical_value_in(metre), 0.6, 1e-9);
    EXPECT_NEAR(belts[0].start.x.numerical_value_in(metre), 0.0, 1e-9);
    EXPECT_NEAR(belts[0].end.x.numerical_value_in(metre), 5.0, 1e-9);
    EXPECT_FALSE(belts[0].collides_with_station);
}

TEST(RouteBelts, PicksCheapestBeltCoveringDistance) {
    auto prob = make_problem({station("src", {}), station("dst", {})});
    ts::LayoutSolution sol;
    sol.station_poses = {pose(0, 0), pose(5, 0)};  // distance 5
    sol.transports = {xport(0, v(0, 0), 1, v(0, 0))};

    // Three belts: two cover 5 m (pick the cheaper), one only covers long runs.
    const std::vector<tc::BeltSpec> cat{
        belt("pricey", 0.6, 1.0, 8.0, 5000.0),
        belt("cheap", 0.6, 3.0, 9.0, 4000.0),
        belt("long_only", 0.8, 6.0, 14.0, 2000.0),  // 5 m outside [6,14]
    };
    const auto belts = ts::route_belts(prob, sol, cat);
    ASSERT_EQ(belts.size(), 1U);
    EXPECT_TRUE(belts[0].fitted);
    EXPECT_EQ(belts[0].belt_catalog_id, "cheap");
}

TEST(RouteBelts, FlagsNoBeltWhenDistanceExceedsCatalog) {
    auto prob = make_problem({station("src", {}), station("dst", {})});
    ts::LayoutSolution sol;
    sol.station_poses = {pose(0, 0), pose(20, 0)};  // 20 m, beyond any range
    sol.transports = {xport(0, v(0, 0), 1, v(0, 0))};

    const std::vector<tc::BeltSpec> cat{belt("b", 0.6, 1.0, 14.0, 1000.0)};
    const auto belts = ts::route_belts(prob, sol, cat);
    ASSERT_EQ(belts.size(), 1U);
    EXPECT_FALSE(belts[0].fitted);
    EXPECT_TRUE(belts[0].belt_catalog_id.empty());
    EXPECT_NEAR(belts[0].width.numerical_value_in(metre), 0.0, 1e-9);
    EXPECT_FALSE(belts[0].collides_with_station);  // no belt → no collision check
}

TEST(RouteBelts, DetectsBeltThroughStationCollision) {
    // Belt runs station0 (0,0) -> station2 (10,0); station1 sits on the path
    // at (5,0) with a 1x1 footprint. The belt (width 1) must flag it.
    auto prob = make_problem({
        station("src", {}),
        station("blocker", box(1.0)),
        station("dst", {}),
    });
    ts::LayoutSolution sol;
    sol.station_poses = {pose(0, 0), pose(5, 0), pose(10, 0)};
    sol.transports = {xport(0, v(0, 0), 2, v(0, 0))};

    const std::vector<tc::BeltSpec> cat{belt("b", 1.0, 6.0, 14.0, 1000.0)};
    const auto belts = ts::route_belts(prob, sol, cat);
    ASSERT_EQ(belts.size(), 1U);
    EXPECT_TRUE(belts[0].fitted);
    EXPECT_TRUE(belts[0].collides_with_station)
        << "belt routed straight through the blocker station was not flagged";
}

TEST(RouteBelts, NoCollisionWhenStationOffThePath) {
    // Same as above but the blocker is far off-axis (5, 10) — clear of the belt.
    auto prob = make_problem({
        station("src", {}),
        station("blocker", box(1.0)),
        station("dst", {}),
    });
    ts::LayoutSolution sol;
    sol.station_poses = {pose(0, 0), pose(5, 10), pose(10, 0)};
    sol.transports = {xport(0, v(0, 0), 2, v(0, 0))};

    const std::vector<tc::BeltSpec> cat{belt("b", 1.0, 6.0, 14.0, 1000.0)};
    const auto belts = ts::route_belts(prob, sol, cat);
    ASSERT_EQ(belts.size(), 1U);
    EXPECT_TRUE(belts[0].fitted);
    EXPECT_FALSE(belts[0].collides_with_station);
}

TEST(RouteBelts, EndpointStationsAreNotSelfCollisions) {
    // The belt's own endpoint stations carry footprints overlapping the ports;
    // those must NOT count as collisions (a belt meets the stations it joins).
    auto prob = make_problem({station("src", box(1.0)), station("dst", box(1.0))});
    ts::LayoutSolution sol;
    sol.station_poses = {pose(0, 0), pose(8, 0)};
    sol.transports = {xport(0, v(0, 0), 1, v(0, 0))};

    const std::vector<tc::BeltSpec> cat{belt("b", 1.0, 6.0, 14.0, 1000.0)};
    const auto belts = ts::route_belts(prob, sol, cat);
    ASSERT_EQ(belts.size(), 1U);
    EXPECT_FALSE(belts[0].collides_with_station);
}
