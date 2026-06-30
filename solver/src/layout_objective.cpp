#include "tinycell/solver/layout_objective.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <mp-units/systems/si.h>
#include <stdexcept>
#include <utility>
#include <vector>

namespace tinycell::solver {

namespace {
using namespace mp_units;
namespace tc = tinycell::core;

double sq(double x) { return x * x; }

// Strip a polygon to plain (x, y) metres for the SAT math (the overlap
// kernel works in doubles like the other penalties; units reattach at the
// boundary).
std::vector<std::pair<double, double>> strip(const tc::Polygon& p) {
    std::vector<std::pair<double, double>> out;
    out.reserve(p.size());
    for (const auto& v : p) {
        out.push_back({v.x.numerical_value_in(si::metre),
                       v.y.numerical_value_in(si::metre)});
    }
    return out;
}

// Minimum translation distance (penetration depth) between two convex
// polygons via SAT. Tests every edge normal of both polygons (NORMALISED,
// so the projection overlap is a real distance); the depth is the smallest
// such overlap, and any axis with non-positive overlap is a separating
// axis → return 0 (no overlap). BOTH inputs must be convex (an accumulated
// station hull is a convex hull). Degenerate input (< 3 vertices) → 0.
double convex_penetration(const tc::Polygon& a, const tc::Polygon& b) {
    if (a.size() < 3 || b.size() < 3) return 0.0;
    const auto pa = strip(a);
    const auto pb = strip(b);
    double min_overlap = std::numeric_limits<double>::max();
    for (const auto* edges : {&pa, &pb}) {
        const auto& e = *edges;
        const std::size_t n = e.size();
        for (std::size_t i = 0; i < n; ++i) {
            const auto& p0 = e[i];
            const auto& p1 = e[(i + 1) % n];
            double ax = -(p1.second - p0.second);  // outward normal of edge
            double ay = (p1.first - p0.first);
            const double len = std::sqrt(ax * ax + ay * ay);
            if (len < 1e-12) continue;  // degenerate edge
            ax /= len;
            ay /= len;
            double min_a = std::numeric_limits<double>::max(), max_a = -min_a;
            for (const auto& v : pa) {
                const double d = v.first * ax + v.second * ay;
                min_a = std::min(min_a, d);
                max_a = std::max(max_a, d);
            }
            double min_b = std::numeric_limits<double>::max(), max_b = -min_b;
            for (const auto& v : pb) {
                const double d = v.first * ax + v.second * ay;
                min_b = std::min(min_b, d);
                max_b = std::max(max_b, d);
            }
            const double overlap = std::min(max_a, max_b) - std::max(min_a, min_b);
            if (overlap <= 0.0) return 0.0;  // separating axis → no overlap
            min_overlap = std::min(min_overlap, overlap);
        }
    }
    return min_overlap;
}

// World-frame accumulated hull of a station at `pose` (empty if the
// station has no footprint — anchors, radius-only test stations). One
// transform on the cached accumulated polygon: the allowed flatten
// granularity (CLAUDE.md §3), NOT a per-equipment express().
tc::Polygon world_hull_of(const StationProblem& s, const tc::Pose2D& pose) {
    if (s.buffered_hull.size() < 3) return {};
    return tc::apply(tc::Transform2D{pose.x, pose.y, pose.theta}, s.buffered_hull);
}

// Overlap penalty between two stations: bounding-circle broad phase, then
// convex-polygon narrow phase when BOTH carry a footprint. `wa`/`wb` are
// the pre-transformed world hulls (empty when the station has none, in
// which case the circle term is the answer — the fallback that keeps
// radius-only callers, anchors, and the existing circle tests intact).
double pair_overlap_penalty(const tc::Pose2D& pa, const StationProblem& sa,
                            const tc::Polygon& wa, const tc::Pose2D& pb,
                            const StationProblem& sb, const tc::Polygon& wb) {
    const double circle =
        overlap_penalty(pa, sa.bounding_radius, pb, sb.bounding_radius);
    if (wa.empty() || wb.empty()) return circle;  // radius-only fallback
    if (circle == 0.0) return 0.0;                 // broad-phase reject
    return sq(convex_penetration(wa, wb));         // narrow-phase depth
}

} // namespace

double overlap_penalty(const tc::Pose2D& pose_a, tc::Length radius_a,
                       const tc::Pose2D& pose_b, tc::Length radius_b) {
    const double dx = (pose_a.x - pose_b.x).numerical_value_in(si::metre);
    const double dy = (pose_a.y - pose_b.y).numerical_value_in(si::metre);
    const double dist = std::sqrt(dx * dx + dy * dy);
    const double min_required =
        radius_a.numerical_value_in(si::metre) + radius_b.numerical_value_in(si::metre);
    if (dist >= min_required) return 0.0;
    const double depth = min_required - dist;
    return sq(depth);
}

double overlap_penalty_poly(const tc::Polygon& world_a, const tc::Polygon& world_b) {
    return sq(convex_penetration(world_a, world_b));
}

double floor_penalty(const tc::Pose2D& pose, tc::Length radius,
                     const Floor& floor) {
    const double x = pose.x.numerical_value_in(si::metre);
    const double y = pose.y.numerical_value_in(si::metre);
    const double r = radius.numerical_value_in(si::metre);
    const double xmin = floor.x_min.numerical_value_in(si::metre);
    const double xmax = floor.x_max.numerical_value_in(si::metre);
    const double ymin = floor.y_min.numerical_value_in(si::metre);
    const double ymax = floor.y_max.numerical_value_in(si::metre);

    double pen = 0.0;
    // Left edge: out-of-bounds depth = xmin - (x - r) when positive.
    const double left_depth = xmin - (x - r);
    if (left_depth > 0.0) pen += sq(left_depth);
    // Right edge.
    const double right_depth = (x + r) - xmax;
    if (right_depth > 0.0) pen += sq(right_depth);
    // Bottom edge.
    const double bottom_depth = ymin - (y - r);
    if (bottom_depth > 0.0) pen += sq(bottom_depth);
    // Top edge.
    const double top_depth = (y + r) - ymax;
    if (top_depth > 0.0) pen += sq(top_depth);
    return pen;
}

double prior_penalty(const tc::Pose2D& pose, const tc::Vec2& nominal) {
    const double dx = (pose.x - nominal.x).numerical_value_in(si::metre);
    const double dy = (pose.y - nominal.y).numerical_value_in(si::metre);
    return dx * dx + dy * dy;
}

double transport_penalty(const tc::Pose2D& pose_src,
                         const tc::Vec2& port_src_local,
                         const tc::Pose2D& pose_sink,
                         const tc::Vec2& port_sink_local) {
    // Compose each station pose with its port-local position to get the
    // port's world coordinates. Pose2D's theta is the station's rotation;
    // a Transform2D with the station's (x, y, theta) is what core::apply
    // expects.
    const tc::Transform2D t_src{pose_src.x, pose_src.y, pose_src.theta};
    const tc::Transform2D t_sink{pose_sink.x, pose_sink.y, pose_sink.theta};
    const tc::Vec2 port_src_world  = tc::apply(t_src,  port_src_local);
    const tc::Vec2 port_sink_world = tc::apply(t_sink, port_sink_local);
    const double dx = (port_src_world.x - port_sink_world.x).numerical_value_in(si::metre);
    const double dy = (port_src_world.y - port_sink_world.y).numerical_value_in(si::metre);
    return dx * dx + dy * dy;
}

double annulus_penalty(const tc::Vec2& port_local,
                       tc::Length reach_min, tc::Length reach_max) {
    // Radial distance of the port from its station origin, in the station
    // frame (invariant to the station's world pose).
    const double x = port_local.x.numerical_value_in(si::metre);
    const double y = port_local.y.numerical_value_in(si::metre);
    const double r = std::sqrt(x * x + y * y);
    const double rmin = reach_min.numerical_value_in(si::metre);
    const double rmax = reach_max.numerical_value_in(si::metre);
    if (r < rmin) return sq(rmin - r);
    if (r > rmax) return sq(r - rmax);
    return 0.0;
}

ObjectiveBreakdown decompose_objective(const LayoutProblem& problem,
                                       const std::vector<tc::Pose2D>& poses,
                                       const std::vector<TransportConstraint>& transports) {
    if (poses.size() != problem.stations.size()) {
        throw std::invalid_argument(
            "decompose_objective: pose count does not match problem.stations.size()");
    }
    const auto& w = problem.weights;
    ObjectiveBreakdown b;

    // Flatten each station's accumulated hull to world ONCE (one transform
    // per station — the kernel-boundary flatten, CLAUDE.md §3); the
    // pair-wise overlap loop then reads these flat instead of re-transforming
    // per pair.
    std::vector<tc::Polygon> world_hull(problem.stations.size());
    for (std::size_t i = 0; i < problem.stations.size(); ++i) {
        world_hull[i] = world_hull_of(problem.stations[i], poses[i]);
    }

    for (std::size_t i = 0; i < problem.stations.size(); ++i) {
        const auto& s = problem.stations[i];
        b.floor += w.floor * floor_penalty(poses[i], s.bounding_radius, problem.floor);
        b.prior += w.positional_prior * prior_penalty(poses[i], s.nominal);
        for (std::size_t j = i + 1; j < problem.stations.size(); ++j) {
            const auto& s2 = problem.stations[j];
            b.overlap += w.overlap * pair_overlap_penalty(
                poses[i], s, world_hull[i], poses[j], s2, world_hull[j]);
        }
    }
    for (const auto& tr : transports) {
        if (tr.source_station >= problem.stations.size() ||
            tr.sink_station >= problem.stations.size()) {
            throw std::invalid_argument(
                "decompose_objective: transport references a station index out of range");
        }
        b.transport += w.transport * transport_penalty(
            poses[tr.source_station], tr.source_port_local,
            poses[tr.sink_station],   tr.sink_port_local);
        // Annulus term for any endpoint that is a reach-annulus port
        // (reach_max set). reach_min defaults to 0 (a disc) when only
        // reach_max is given. The port_local read here is whatever the
        // caller supplied in `transports` — for the placer's callback
        // that is the OPTIMISED port position (M1.4).
        if (tr.source_reach_max.has_value()) {
            const tc::Length rmin = tr.source_reach_min.value_or(0.0 * si::metre);
            b.annulus += w.annulus *
                         annulus_penalty(tr.source_port_local, rmin, *tr.source_reach_max);
        }
        if (tr.sink_reach_max.has_value()) {
            const tc::Length rmin = tr.sink_reach_min.value_or(0.0 * si::metre);
            b.annulus += w.annulus *
                         annulus_penalty(tr.sink_port_local, rmin, *tr.sink_reach_max);
        }
    }
    b.total = b.overlap + b.floor + b.prior + b.transport + b.annulus;
    return b;
}

ObjectiveBreakdown decompose_objective(const LayoutProblem& problem,
                                       const std::vector<tc::Pose2D>& poses) {
    return decompose_objective(problem, poses, problem.transports);
}

double evaluate_objective(const LayoutProblem& problem,
                          const std::vector<tc::Pose2D>& poses,
                          const std::vector<TransportConstraint>& transports) {
    return decompose_objective(problem, poses, transports).total;
}

double evaluate_objective(const LayoutProblem& problem,
                          const std::vector<tc::Pose2D>& poses) {
    return evaluate_objective(problem, poses, problem.transports);
}

bool hard_constraints_satisfied(const LayoutProblem& problem,
                                const std::vector<tc::Pose2D>& poses) {
    if (poses.size() != problem.stations.size()) {
        throw std::invalid_argument(
            "hard_constraints_satisfied: pose count does not match problem.stations.size()");
    }
    // Penalty-based feasibility is approximate by construction: a
    // smooth optimiser will converge to within its tolerance of the
    // constraint boundary, leaving a residual penalty no larger than
    // the square of that tolerance. Treat anything below the threshold
    // below as effectively zero. (1 micrometre² of overlap, 1 micrometre²
    // of out-of-floor extent - far below any real engineering tolerance.)
    constexpr double kFeasibilityEps = 1e-6;
    std::vector<tc::Polygon> world_hull(problem.stations.size());
    for (std::size_t i = 0; i < problem.stations.size(); ++i) {
        world_hull[i] = world_hull_of(problem.stations[i], poses[i]);
    }
    for (std::size_t i = 0; i < problem.stations.size(); ++i) {
        const auto& s = problem.stations[i];
        if (floor_penalty(poses[i], s.bounding_radius, problem.floor) > kFeasibilityEps) {
            return false;
        }
        for (std::size_t j = i + 1; j < problem.stations.size(); ++j) {
            const auto& s2 = problem.stations[j];
            // Same broad-then-narrow phase the objective uses, so the
            // feasibility flag matches the term it scored: a footprint
            // pair must be polygon-clear, not merely circle-clear.
            if (pair_overlap_penalty(poses[i], s, world_hull[i],
                                     poses[j], s2, world_hull[j]) > kFeasibilityEps) {
                return false;
            }
        }
    }
    return true;
}

} // namespace tinycell::solver
