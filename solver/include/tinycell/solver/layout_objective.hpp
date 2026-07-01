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

// Convex-polygon NARROW-PHASE overlap penalty (step-6 M4.2): squared
// penetration depth (minimum translation distance) between two convex
// footprint polygons given in WORLD coordinates, via the separating-axis
// theorem. Returns 0 when a separating axis exists (no overlap). The
// accumulated station hull is a convex hull, so SAT applies.
//
// This is the narrow-phase refinement of the bounding-circle
// overlap_penalty above: an elongated cell (e.g. an arm + an offset pallet
// zone) packs to FOOTPRINT contact instead of being held off by its
// conservative bounding circle, which is dominated by the cell's longest
// extent. decompose_objective / hard_constraints_satisfied use the circle
// as a cheap broad-phase reject, then this when the circles overlap AND
// both stations carry a footprint; a station with no footprint (anchors,
// or radius-only callers) falls back to the circle term. Pure core — no
// foreign lib (CLAUDE.md §1) — same SAT family as belt-vs-station collision.
double overlap_penalty_poly(const core::Polygon& world_a,
                            const core::Polygon& world_b);

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

// Transport-edge soft term: squared distance between the source port
// and sink port in world coords. port_*_local is in the source/sink
// station's frame; this function composes each with the corresponding
// station pose to get the port's world position. Quadratic-from-zero,
// smooth everywhere; minimum at coincident endpoints. The Phase-1
// transport term (step 6); deferred follow-ups (port direction
// tolerances, real belt geometry) will land on top of this.
double transport_penalty(const core::Pose2D& pose_src,
                         const core::Vec2& port_src_local,
                         const core::Pose2D& pose_sink,
                         const core::Vec2& port_sink_local);

// Reach-annulus soft term (step-6 M1.3): an annulus port's port-local
// position should lie within [reach_min, reach_max] of its station
// origin. Zero inside the band; grows quadratically with the radial
// violation outside it (r < reach_min or r > reach_max). port_local is
// in the station frame, so this term is invariant to the station's world
// pose - it constrains only where the port sits relative to its own
// equipment. A soft preferred-radius pull toward the band midpoint is a
// deferred refinement; the crude band-only form is enough to make an
// annulus port land inside reach.
double annulus_penalty(const core::Vec2& port_local,
                       core::Length reach_min, core::Length reach_max);

// Sum the four terms across the whole problem, weighted by
// problem.weights. Pose count must match problem.stations.size();
// throws std::invalid_argument otherwise. The pair-wise overlap
// term sums over each unordered pair once; the transport term sums
// over each TransportConstraint once.
double evaluate_objective(const LayoutProblem& problem,
                          const std::vector<core::Pose2D>& poses);

// Same as evaluate_objective but returns the per-term breakdown. The
// `total` field equals what evaluate_objective returns. Used by
// solve() to populate LayoutSolution.cost, and by the LNS outer loop
// to surface which term drove each accepted/rejected move.
ObjectiveBreakdown decompose_objective(const LayoutProblem& problem,
                                       const std::vector<core::Pose2D>& poses);

// Overloads scoring an EXPLICIT transport set instead of
// problem.transports (step-6 M1.4). The placer's NLopt callback uses
// these to evaluate the OPTIMISED annulus-port positions, which live in
// the variable vector rather than in problem.transports. Station and
// pose data still come from `problem`; `transports` supplies the
// (possibly port-substituted) edges. The no-transport-arg overloads
// above delegate here with problem.transports, so both paths run the
// same code.
double evaluate_objective(const LayoutProblem& problem,
                          const std::vector<core::Pose2D>& poses,
                          const std::vector<TransportConstraint>& transports);

ObjectiveBreakdown decompose_objective(const LayoutProblem& problem,
                                       const std::vector<core::Pose2D>& poses,
                                       const std::vector<TransportConstraint>& transports);

// After-the-fact hard-constraint check: returns true iff
// overlap_penalty + floor_penalty over the whole problem is exactly
// zero at the given poses. Used by solve() to set
// LayoutSolution.hard_constraints_satisfied.
bool hard_constraints_satisfied(const LayoutProblem& problem,
                                const std::vector<core::Pose2D>& poses);

} // namespace tinycell::solver
