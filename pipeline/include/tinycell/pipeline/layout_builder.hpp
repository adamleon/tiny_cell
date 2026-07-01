#pragma once

// pipeline/ — application-level orchestration that turns a solved allocation
// into a LayoutProblem ready for solve(), plus the render/debug side tables
// (per-station pallet zones, task->station map, pallet-slot assignments,
// intra-station feasibility diagnostics).
//
// WHY ITS OWN MODULE (not solver/): assembling a multi-equipment station
// footprint uses the boost adapter (convex_hull / buffer_outward), and solver/
// depends on NO foreign lib (CLAUDE.md §1). This glue composes solver +
// adapters + core, so it sits above solver as its own layer. Extracted from
// demos/solve_workflow (post-MVP demo Phase 2) so BOTH the SVG demo and the
// threepp render app build the IDENTICAL LayoutProblem from a workflow — no
// duplication.

#include <algorithm>
#include <map>
#include <span>
#include <string>
#include <vector>

#include <tinycell/geometry.hpp>
#include <tinycell/model/arm.hpp>
#include <tinycell/model/port.hpp>
#include <tinycell/model/pusher.hpp>
#include <tinycell/model/task.hpp>
#include <tinycell/solver/allocator.hpp>
#include <tinycell/solver/enumerator.hpp>
#include <tinycell/solver/layout_problem.hpp>

namespace tinycell::pipeline {

// Catalog lookup by id (linear scan; catalogs are small). Public because the
// render / SVG consumers also resolve a bound instance's ArmSpec to draw its
// reach ring. Returns nullptr if not found.
inline const core::ArmSpec* find_arm(const std::vector<core::ArmSpec>& arms,
                                      const std::string& id) {
    auto it = std::find_if(arms.begin(), arms.end(),
                           [&](const core::ArmSpec& a) { return a.id == id; });
    return it == arms.end() ? nullptr : &*it;
}
inline const core::PusherSpec* find_pusher(const std::vector<core::PusherSpec>& pushers,
                                           const std::string& id) {
    auto it = std::find_if(pushers.begin(), pushers.end(),
                           [&](const core::PusherSpec& p) { return p.id == id; });
    return it == pushers.end() ? nullptr : &*it;
}

// Whether a StationProblem came from a bound instance (draw footprint + reach)
// or a pinned anchor (draw a marker) — lets consumers interpret solved poses.
enum class StationSourceKind { Instance, Anchor };
struct StationSource {
    StationSourceKind kind;
    std::size_t       source_index; // index into AllocationResult.instances or .anchors
};

// The LayoutProblem plus the side tables render / SVG need. The parallel
// arrays (sources, pallet_zones_local) are indexed by station, in
// problem.stations order (instances first, then anchors).
struct LayoutBuildResult {
    solver::LayoutProblem                   problem;
    std::vector<StationSource>              sources;
    std::map<std::string, std::size_t>      task_to_station;
    // Per-station pallet build zones in STATION frame (a dual-pallet cell holds
    // several; empty list for anchors). Consumers world-resolve via the pose.
    std::vector<std::vector<core::Polygon>> pallet_zones_local;
    // task_id -> its assigned pallet slot centre (station frame), so pallet_in /
    // pallet_out transport endpoints land on the slot the intra-station layout
    // chose, not the strategy's single templated offset.
    std::map<std::string, core::Vec2>       task_pallet_slot;
    // Intra-station feasibility diagnostics (a cell whose pallets don't fit the
    // arm's reach). Non-empty => the layout is not physically realizable.
    std::vector<std::string>                cell_diagnostics;
};

// Build a LayoutProblem (+ side tables) from an allocation. Each BoundInstance
// becomes a StationProblem whose collision footprint is the accumulated,
// clearance-buffered convex hull of its equipment (arm + one pallet zone per
// served Palletize task); each PinnedAnchor becomes a frozen StationProblem at
// its world pose; each TransportEdge becomes a TransportConstraint carrying its
// endpoints' port-local positions and reach bands. Uses the boost adapter for
// the hull/buffer (hence this module lives above solver/, CLAUDE.md §1).
LayoutBuildResult build_layout_problem(
        const solver::AllocationResult&               alloc,
        std::span<const solver::TaskEnumeration>      enumeration,
        const std::vector<core::ArmSpec>&             arms,
        const std::vector<core::PusherSpec>&          pushers,
        const std::map<std::string, core::Vec2>&      nominal_for_task,
        const solver::Floor&                          floor,
        const solver::ObjectiveWeights&               weights);

} // namespace tinycell::pipeline
