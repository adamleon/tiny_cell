#pragma once

// Phase D objective + constraint encoders for the LayoutProblem
// (solver/layout_problem.hpp). Free functions in solver:: namespace,
// callable directly by the eventual NLopt callback (Phase E) and by
// tests for unit-level verification.
//
// All penalties are NON-NEGATIVE and equal ZERO at their target
// minimum. "Hard constraints satisfied" iff overlap_penalty +
// floor_penalty over the whole problem evaluates to zero.

#include <tinycell/solver/layout_problem.hpp>

namespace tinycell::solver {

// Soft non-overlap penalty between two stations' bounding circles.
// The two stations interpenetrate iff
//     ‖pose_a.translation - pose_b.translation‖ < (radius_a + radius_b).
// Penalty grows quadratically with interpenetration depth, smooth
// everywhere except at the touching boundary (where it's
// continuously zero on the non-overlapping side and quadratic-from-
// zero on the overlapping side — fine for SLSQP). Returns 0 if the
// circles don't intersect.
double overlap_penalty(const core::Pose2D& pose_a, core::Length radius_a,
                       const core::Pose2D& pose_b, core::Length radius_b);

// Soft floor-bounds penalty: a station's bounding circle should fit
// entirely inside the floor rectangle. Penalty grows quadratically
// with the out-of-bounds depth on each side (left/right/top/bottom);
// returns 0 if the bounding circle is wholly inside the floor.
double floor_penalty(const core::Pose2D& pose, core::Length radius,
                     const Floor& floor);

// Positional-prior soft term: squared distance between a station's
// translation and its nominal seed. Always smooth; zero exactly at
// the nominal.
double prior_penalty(const core::Pose2D& pose, const core::Vec2& nominal);

// Sum the three terms across the whole problem, weighted by
// problem.weights. Pose count must match problem.stations.size();
// throws std::invalid_argument otherwise. The pair-wise overlap
// term sums over each unordered pair once.
double evaluate_objective(const LayoutProblem& problem,
                          const std::vector<core::Pose2D>& poses);

// After-the-fact hard-constraint check: returns true iff
// overlap_penalty + floor_penalty over the whole problem is exactly
// zero at the given poses. Used by solve() to set
// LayoutSolution.hard_constraints_satisfied.
bool hard_constraints_satisfied(const LayoutProblem& problem,
                                const std::vector<core::Pose2D>& poses);

} // namespace tinycell::solver
