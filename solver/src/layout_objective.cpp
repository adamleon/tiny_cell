#include "tinycell/solver/layout_objective.hpp"

#include <cmath>
#include <mp-units/systems/si.h>
#include <stdexcept>

namespace tinycell::solver {

namespace {
using namespace mp_units;
namespace tc = tinycell::core;

double sq(double x) { return x * x; }
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

double evaluate_objective(const LayoutProblem& problem,
                          const std::vector<tc::Pose2D>& poses) {
    if (poses.size() != problem.stations.size()) {
        throw std::invalid_argument(
            "evaluate_objective: pose count does not match problem.stations.size()");
    }
    const auto& w = problem.weights;
    double total = 0.0;

    for (std::size_t i = 0; i < problem.stations.size(); ++i) {
        const auto& s = problem.stations[i];
        total += w.floor * floor_penalty(poses[i], s.bounding_radius, problem.floor);
        total += w.positional_prior * prior_penalty(poses[i], s.nominal);
        for (std::size_t j = i + 1; j < problem.stations.size(); ++j) {
            const auto& s2 = problem.stations[j];
            total += w.overlap * overlap_penalty(
                poses[i], s.bounding_radius,
                poses[j], s2.bounding_radius);
        }
    }
    for (const auto& tr : problem.transports) {
        if (tr.source_station >= problem.stations.size() ||
            tr.sink_station >= problem.stations.size()) {
            throw std::invalid_argument(
                "evaluate_objective: transport references a station index out of range");
        }
        total += w.transport * transport_penalty(
            poses[tr.source_station], tr.source_port_local,
            poses[tr.sink_station],   tr.sink_port_local);
    }
    return total;
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
    for (std::size_t i = 0; i < problem.stations.size(); ++i) {
        const auto& s = problem.stations[i];
        if (floor_penalty(poses[i], s.bounding_radius, problem.floor) > kFeasibilityEps) {
            return false;
        }
        for (std::size_t j = i + 1; j < problem.stations.size(); ++j) {
            const auto& s2 = problem.stations[j];
            if (overlap_penalty(poses[i], s.bounding_radius,
                                poses[j], s2.bounding_radius) > kFeasibilityEps) {
                return false;
            }
        }
    }
    return true;
}

} // namespace tinycell::solver
