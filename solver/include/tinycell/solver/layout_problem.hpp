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
// Objective terms (after step-6 Phase 1):
//   * overlap (smooth penalty between station bounding-circles —
//     broad-phase non-overlap; narrow-phase polygon union arrives
//     when the StationFootprint cache lands and a real consumer
//     needs it)
//   * floor bounds (smooth penalty when a bounding-circle extends
//     past a floor edge)
//   * positional prior (squared distance from nominal seed)
//   * transport length (squared distance between connected port
//     world-poses; the term that makes LNS care about moving stations
//     closer to what they exchange items with)
// Deferred: port-direction tolerances (step-5 deferral #3); reach
// feasibility (needs task pickup/dropoff modelling); energy as a
// function of placed distance (needs the motion model coupled to
// actual poses); blueprint obstacles; narrow-phase polygon union.

#include <optional>
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
//
// `initial_pose` doubles as the WARM-START seed: solve() begins the
// optimisation from this point. Callers re-running the placer
// (LNS destroy-and-repair, interactive editing, multi-stage solves)
// pass a previous solution's pose here for warm-start; first solves
// pass the positional-prior nominal.
//
// `frozen` is the PARTIAL-FREEZE flag (Phase F): when true, the
// station's pose is held at `initial_pose` and not varied by the
// NLP. Frozen stations STILL participate in the objective — they
// contribute to overlap penalties against variable stations, and to
// hard_constraints_satisfied — but they're constants of the problem,
// not variables. Use cases: pinned Anchors (feeders/dispatches),
// stations the user has dragged-and-locked, LNS subset-solves where
// most of the layout stays put.
struct StationProblem {
    std::string id;
    core::Polygon buffered_hull;        // station-frame, clearance-baked
    core::Length bounding_radius;       // max ‖v - origin‖ over hull vertices
    core::Vec2 nominal;                 // seed from the positional prior
    core::Pose2D initial_pose;          // world-frame seed AND warm-start input
    bool frozen = false;                // if true, NLP holds pose at initial_pose
};

// Rectangular floor. Stations' bounding circles must stay inside
// this box (smooth penalty when they don't).
struct Floor {
    core::Length x_min;
    core::Length x_max;
    core::Length y_min;
    core::Length y_max;
};

// One transport edge as seen by the placer (step-6 Phase 1). Each
// transport connects a source port on one station to a sink port on
// another. The port-local fields are in the corresponding station's
// frame (port.hpp convention); the placer composes them with the
// station's optimised pose to get the port's world position, and
// penalises the squared distance between source-world and sink-world.
//
// Endpoints of transports terminating at an anchor use port_local
// (0, 0): the anchor's station is pinned at its world pose and the
// anchor's port sits at the station origin (anchor_strategy.cpp).
//
// theta + direction tolerances are deferred (step-5 deferral #3);
// only port positions feed the Phase-1 transport-distance term.
struct TransportConstraint {
    std::size_t source_station;       // index into LayoutProblem::stations
    core::Vec2 source_port_local;     // in source station's frame
    std::size_t sink_station;
    core::Vec2 sink_port_local;       // in sink station's frame

    // Reach-annulus band for each endpoint (step-6 M1.3), copied from the
    // emitting PortConstraint at problem-build time so the kernel reads it
    // flat (no PortConstraint lookup mid-solve, CLAUDE.md s3). An endpoint
    // with reach_max set is an ANNULUS port: its port-local position should
    // lie within [reach_min, reach_max] of its station origin, enforced by
    // the soft annulus_penalty. Unset => POINT endpoint, no annulus term.
    std::optional<core::Length> source_reach_min;
    std::optional<core::Length> source_reach_max;
    std::optional<core::Length> sink_reach_min;
    std::optional<core::Length> sink_reach_max;
};

// Soft objective weights. Defaults are reasonable MVP starting points;
// callers should expect to tune them once the placer is wired (Phase
// E) and behaviour can be observed on real workloads.
struct ObjectiveWeights {
    double annulus = 1.0;            // penalty per m^2 of reach-band radial violation (M1.3)
    double overlap = 100.0;          // penalty per metre² of bounding-circle interpenetration
    double floor = 100.0;            // penalty per metre² of out-of-floor extent
    double positional_prior = 1.0;   // penalty per metre² of deviation from nominal
    double transport = 1.0;          // penalty per metre² of transport-edge length²
};

struct LayoutProblem {
    std::vector<StationProblem> stations;
    std::vector<TransportConstraint> transports;
    Floor floor;
    ObjectiveWeights weights;
};

// Per-term breakdown of the weighted objective at a pose set (step-6
// Phase 2). Each field is already weighted; their sum equals `total`.
// Lives here (not in layout_objective.hpp) because LayoutSolution
// embeds it and layout_objective.hpp already includes layout_problem.hpp.
// Surfaces the diagnostic LNS uses to attribute cost changes to a
// specific term ("the move improved transport but cost overlap"); the
// demo's cost-trace line and Phase-4 SA temperature calibration also
// read it.
struct ObjectiveBreakdown {
    double overlap = 0.0;
    double floor = 0.0;
    double prior = 0.0;
    double transport = 0.0;
    double annulus = 0.0;
    double total = 0.0;
};

struct LayoutSolution {
    // World-frame pose per station, in the same order as
    // LayoutProblem.stations.
    std::vector<core::Pose2D> station_poses;

    // Weighted per-term breakdown at the returned poses. `total` is
    // the scalar to compare across solutions; the per-term fields tell
    // you which constraint or soft term drove the move. Lower is
    // better on every field; zero means that term hit its minimum.
    ObjectiveBreakdown cost;

    // True iff every HARD constraint (non-overlap, in-floor) is
    // satisfied — i.e. the corresponding penalty terms evaluate to
    // exactly zero. Smooth penalisation during search allows the
    // solver to traverse infeasible regions; this flag is the
    // after-the-fact check whether the final pose set is actually
    // usable.
    bool hard_constraints_satisfied;
};

// solve(problem) runs the NLP over the variable stations (those with
// frozen=false) starting from their initial_pose seeds and returns
// the optimised world-frame poses for every station. Frozen stations
// pass through unchanged in station_poses but still appear in the
// objective evaluation. Phase E backed this with NLopt/BOBYQA;
// Phase F added partial-freeze. The signature is library-neutral so
// the backend can be swapped without touching callers.
LayoutSolution solve(const LayoutProblem& problem);

} // namespace tinycell::solver
