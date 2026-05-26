#pragma once

// Layer-3 placement problem rep + abstract solve() seam (step 5,
// Phase D). The LayoutProblem describes ONE inter-station placement
// task: a set of equipment-bearing stations with each station's
// buffered footprint hull and nominal seed position, a rectangular
// floor, and weights for the soft objective terms. solve() returns
// the per-station world poses + the final objective value + a flag
// indicating whether the hard constraints (non-overlap, in-floor)
// were satisfied after smoothing.
//
// Library-neutral by design: no NLopt / IPOPT types appear in this
// header (CLAUDE.md §1, §9). Phase D ships a STUB solve() that just
// returns the initial poses unchanged and evaluates the objective;
// Phase E wires NLopt/SLSQP behind the same signature.
//
// MVP scope at D — three objective terms only:
//   * overlap (smooth penalty between station bounding-circles —
//     broad-phase non-overlap; narrow-phase polygon union arrives
//     when the StationFootprint cache lands and a real consumer
//     needs it)
//   * floor bounds (smooth penalty when a bounding-circle extends
//     past a floor edge)
//   * positional prior (squared distance from nominal seed)
// Deferred from D: transfer-length (needs transports in the problem
// + port-world-pose resolution); reach feasibility (needs task
// pickup/dropoff modelling); energy as a function of placed distance
// (needs the motion model coupled to actual poses); blueprint
// obstacles; narrow-phase polygon union.

#include <string>
#include <tinycell/geometry.hpp>
#include <tinycell/units.hpp>
#include <vector>

namespace tinycell::solver {

// One station's contribution to the LayoutProblem. Bounding-circle
// is precomputed from the buffered hull (max distance from station
// origin to any hull vertex); the placer reads it for the
// broad-phase non-overlap check. The full polygon stays available
// for narrow-phase code to consume later when it lands.
struct StationProblem {
    std::string id;
    core::Polygon buffered_hull;        // station-frame, clearance-baked
    core::Length bounding_radius;       // max ‖v - origin‖ over hull vertices
    core::Vec2 nominal;                 // seed from the positional prior
    core::Pose2D initial_pose;          // world-frame seed for the NLP
};

// Rectangular floor. Stations' bounding circles must stay inside
// this box (smooth penalty when they don't).
struct Floor {
    core::Length x_min;
    core::Length x_max;
    core::Length y_min;
    core::Length y_max;
};

// Soft objective weights. Defaults are reasonable MVP starting points;
// callers should expect to tune them once the placer is wired (Phase
// E) and behaviour can be observed on real workloads.
struct ObjectiveWeights {
    double overlap = 100.0;          // penalty per metre² of bounding-circle interpenetration
    double floor = 100.0;            // penalty per metre² of out-of-floor extent
    double positional_prior = 1.0;   // penalty per metre² of deviation from nominal
};

struct LayoutProblem {
    std::vector<StationProblem> stations;
    Floor floor;
    ObjectiveWeights weights;
};

struct LayoutSolution {
    // World-frame pose per station, in the same order as
    // LayoutProblem.stations.
    std::vector<core::Pose2D> station_poses;

    // Sum of weighted penalties at the returned poses. Lower is
    // better; zero means every soft term hit its minimum.
    double final_objective;

    // True iff every HARD constraint (non-overlap, in-floor) is
    // satisfied — i.e. the corresponding penalty terms evaluate to
    // exactly zero. Smooth penalisation during search allows the
    // solver to traverse infeasible regions; this flag is the
    // after-the-fact check whether the final pose set is actually
    // usable.
    bool hard_constraints_satisfied;
};

// solve(problem) returns the placed poses. Phase D stub: returns
// initial_pose for each station and evaluates the objective at that
// initial point — no optimisation yet. Phase E wires NLopt/SLSQP
// behind this signature; callers see the same struct in and out.
LayoutSolution solve(const LayoutProblem& problem);

} // namespace tinycell::solver
