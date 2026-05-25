// Boost.Geometry conversion implementation. See header for contract.
//
// Strip-and-reattach pattern: every public entry point converts
// core::Polygon (mp-units Length) → boost::geometry::model::polygon<double>
// in plain SI meters at function entry, performs the operation, and
// converts back at the boundary. No boost type ever escapes this file.

#include <tinycell/adapters/boost_conv.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/multi_polygon.hpp>

#include <cassert>
#include <mp-units/systems/si.h>
#include <stdexcept>

namespace tinycell::adapters {

namespace {

namespace bg = boost::geometry;
using BgPoint = bg::model::d2::point_xy<double>;
using BgPolygon = bg::model::polygon<BgPoint>;
using BgMultiPolygon = bg::model::multi_polygon<BgPolygon>;

// core::Polygon → boost polygon in SI meters (units stripped).
BgPolygon to_boost(const core::Polygon& in) {
    BgPolygon out;
    auto& ring = out.outer();
    ring.reserve(in.size());
    for (const auto& v : in) {
        ring.emplace_back(
            v.x.numerical_value_in(mp_units::si::metre),
            v.y.numerical_value_in(mp_units::si::metre)
        );
    }
    bg::correct(out);
    return out;
}

// boost polygon → core::Polygon, reattaching the metre unit.
core::Polygon to_core(const BgPolygon& in) {
    using mp_units::si::metre;
    core::Polygon out;
    const auto& ring = in.outer();
    out.reserve(ring.size());
    for (const auto& p : ring) {
        out.push_back(core::Vec2{
            .x = p.x() * metre,
            .y = p.y() * metre,
        });
    }
    // Boost stores rings closed (last == first). Drop the duplicate so
    // core::Polygon stays open per its convention (geometry.hpp).
    if (out.size() >= 2 && out.front().x == out.back().x && out.front().y == out.back().y) {
        out.pop_back();
    }
    return out;
}

} // namespace

core::Polygon convex_hull(const core::Polygon& in) {
    if (in.size() <= 2) {
        return in;
    }
    BgPolygon bg_in = to_boost(in);
    BgPolygon bg_hull;
    bg::convex_hull(bg_in, bg_hull);
    return to_core(bg_hull);
}

core::Polygon buffer_outward(const core::Polygon& in, core::Length amount) {
    const double amount_m = amount.numerical_value_in(mp_units::si::metre);
    if (!(amount_m > 0.0)) {
        throw std::invalid_argument(
            "buffer_outward: amount must be strictly positive (defect — solver should not call buffer with non-positive clearance)"
        );
    }

    BgPolygon bg_in = to_boost(in);

    // Buffer strategies. Round joins + circle endpoints give a smooth
    // inflated boundary. 16 points per quarter-turn keeps the polygon
    // vertex count modest while staying visually smooth.
    constexpr int kPointsPerCircle = 16;
    bg::strategy::buffer::distance_symmetric<double> distance(amount_m);
    bg::strategy::buffer::join_round join(kPointsPerCircle);
    bg::strategy::buffer::end_round end(kPointsPerCircle);
    bg::strategy::buffer::point_circle circle(kPointsPerCircle);
    bg::strategy::buffer::side_straight side;

    BgMultiPolygon bg_out;
    bg::buffer(bg_in, bg_out, distance, side, join, end, circle);

    // A convex outward buffer must produce exactly one component.
    // Non-convex inputs that produce multi-component results are a
    // future case (narrow-phase unions); assert loudly until that
    // arrives so the failure is caught the moment the assumption
    // breaks.
    assert(bg_out.size() == 1 && "buffer_outward produced a multi-component result; non-convex buffering requires a richer return type");
    if (bg_out.size() != 1) {
        throw std::runtime_error("buffer_outward: multi-component buffer not yet supported");
    }
    return to_core(bg_out[0]);
}

} // namespace tinycell::adapters
