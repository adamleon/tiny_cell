#include "tinycell/solver/belt_routing.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <utility>

#include <mp-units/systems/si.h>

namespace tinycell::solver {

namespace {
using namespace mp_units;
namespace tc = tinycell::core;

// Strip a polygon to plain (x, y) metres. Belt routing is a solver geometry
// kernel, so it works in doubles like layout_objective's penalty math; units
// are reattached at the boundary (belt_footprint output / Length results).
std::vector<std::pair<double, double>> strip(const tc::Polygon& p) {
    std::vector<std::pair<double, double>> out;
    out.reserve(p.size());
    for (const auto& v : p) {
        out.push_back({v.x.numerical_value_in(si::metre),
                       v.y.numerical_value_in(si::metre)});
    }
    return out;
}

// True iff some edge of `edges` yields an axis that separates point sets A and
// B (their projections don't overlap). One half of the separating-axis test.
bool separated_by(const std::vector<std::pair<double, double>>& edges,
                  const std::vector<std::pair<double, double>>& a,
                  const std::vector<std::pair<double, double>>& b) {
    const std::size_t n = edges.size();
    for (std::size_t i = 0; i < n; ++i) {
        const auto& p0 = edges[i];
        const auto& p1 = edges[(i + 1) % n];
        // Axis = outward normal of edge (p1 - p0): (-dy, dx).
        const double ax = -(p1.second - p0.second);
        const double ay = (p1.first - p0.first);
        double min_a = std::numeric_limits<double>::max(), max_a = -min_a;
        for (const auto& v : a) {
            const double d = v.first * ax + v.second * ay;
            min_a = std::min(min_a, d);
            max_a = std::max(max_a, d);
        }
        double min_b = std::numeric_limits<double>::max(), max_b = -min_b;
        for (const auto& v : b) {
            const double d = v.first * ax + v.second * ay;
            min_b = std::min(min_b, d);
            max_b = std::max(max_b, d);
        }
        if (max_a < min_b || max_b < min_a) return true;  // separating axis
    }
    return false;
}

// Convex-convex overlap via the separating-axis theorem. BOTH inputs must be
// convex (a belt rectangle and a buffered convex station hull both are). Returns
// true if they overlap (including containment); false if a separating axis
// exists. Degenerate input (< 3 vertices) returns false — no footprint, no
// collision.
bool convex_overlap(const tc::Polygon& a, const tc::Polygon& b) {
    if (a.size() < 3 || b.size() < 3) return false;
    const auto pa = strip(a);
    const auto pb = strip(b);
    if (separated_by(pa, pa, pb)) return false;
    if (separated_by(pb, pa, pb)) return false;
    return true;
}

} // namespace

core::Polygon belt_footprint(core::Vec2 start, core::Vec2 end, core::Length width) {
    const double sx = start.x.numerical_value_in(si::metre);
    const double sy = start.y.numerical_value_in(si::metre);
    const double ex = end.x.numerical_value_in(si::metre);
    const double ey = end.y.numerical_value_in(si::metre);
    const double dx = ex - sx;
    const double dy = ey - sy;
    const double len = std::sqrt(dx * dx + dy * dy);
    if (len < 1e-12) return {};  // degenerate: zero-length belt has no footprint
    const double nx = -dy / len;  // unit normal to the belt direction
    const double ny = dx / len;
    const double hw = 0.5 * width.numerical_value_in(si::metre);
    auto pt = [](double x, double y) {
        return tc::Vec2{x * si::metre, y * si::metre};
    };
    return tc::Polygon{
        pt(sx + hw * nx, sy + hw * ny),
        pt(ex + hw * nx, ey + hw * ny),
        pt(ex - hw * nx, ey - hw * ny),
        pt(sx - hw * nx, sy - hw * ny),
    };
}

std::vector<PlacedBelt> route_belts(const LayoutProblem& problem,
                                    const LayoutSolution& solution,
                                    std::span<const core::BeltSpec> belt_catalog) {
    std::vector<PlacedBelt> belts;
    belts.reserve(solution.transports.size());

    for (std::size_t t = 0; t < solution.transports.size(); ++t) {
        const auto& tr = solution.transports[t];
        const auto& src_pose = solution.station_poses[tr.source_station];
        const auto& dst_pose = solution.station_poses[tr.sink_station];
        const tc::Transform2D t_src{src_pose.x, src_pose.y, src_pose.theta};
        const tc::Transform2D t_dst{dst_pose.x, dst_pose.y, dst_pose.theta};
        const tc::Vec2 start = tc::apply(t_src, tr.source_port_local);
        const tc::Vec2 end = tc::apply(t_dst, tr.sink_port_local);

        const double dx = (end.x - start.x).numerical_value_in(si::metre);
        const double dy = (end.y - start.y).numerical_value_in(si::metre);
        const double dist = std::sqrt(dx * dx + dy * dy);

        // Cheapest catalog belt whose length range covers the routed distance.
        const core::BeltSpec* chosen = nullptr;
        for (const auto& spec : belt_catalog) {
            const double lo = spec.min_length.numerical_value_in(si::metre);
            const double hi = spec.max_length.numerical_value_in(si::metre);
            if (dist >= lo && dist <= hi &&
                (chosen == nullptr || spec.list_price_eur < chosen->list_price_eur)) {
                chosen = &spec;
            }
        }

        core::Length width = 0.0 * si::metre;
        if (chosen != nullptr) width = chosen->belt_width;

        PlacedBelt belt{
            .transport_index = t,
            .belt_catalog_id = chosen ? chosen->id : std::string{},
            .start = start,
            .end = end,
            .width = width,
            .length = dist * si::metre,
            .fitted = chosen != nullptr,
            .collides_with_station = false,
        };

        // belt-vs-station collision: footprint vs every station's world hull,
        // skipping the belt's own two endpoints and footprint-less anchors.
        if (chosen != nullptr) {
            const core::Polygon fp = belt_footprint(start, end, width);
            for (std::size_t i = 0; i < problem.stations.size(); ++i) {
                if (i == tr.source_station || i == tr.sink_station) continue;
                const auto& hull = problem.stations[i].buffered_hull;
                if (hull.size() < 3) continue;  // anchors have no footprint
                const auto& pose = solution.station_poses[i];
                const tc::Transform2D t_st{pose.x, pose.y, pose.theta};
                if (convex_overlap(fp, tc::apply(t_st, hull))) {
                    belt.collides_with_station = true;
                    break;
                }
            }
        }
        belts.push_back(std::move(belt));
    }
    return belts;
}

} // namespace tinycell::solver
