#pragma once

// Belt routing (step-6 M2). After the Layer-3 placer resolves station poses and
// port positions, transports become REAL placed objects: route_belts() turns
// each transport edge into a PlacedBelt — a straight conveyor of catalog width
// running between the two resolved port world positions — picking the cheapest
// catalog belt whose length range covers the routed distance, and flagging any
// belt whose footprint collides with a (non-endpoint) station.
//
// SEQUENTIAL placement (decisions.md "Belt geometry: sequential placement at
// MVP"): stations are placed first, belts routed after. Collisions are
// REPORTED, not repaired — feeding them back into placement is joint placement,
// deferred until a real failure case (a belt forced through a station) appears.
// Belt-vs-belt is not checked (belts may cross / run at different heights).

#include <span>
#include <string>
#include <vector>

#include <tinycell/geometry.hpp>
#include <tinycell/model/belt.hpp>
#include <tinycell/solver/layout_problem.hpp>

namespace tinycell::solver {

// One routed belt: a straight conveyor between two resolved port world
// positions. `start`/`end` are world-frame points (the source/sink port
// positions); the belt's direction is end - start, so no separate pose is
// stored. The footprint is a `width`-wide rectangle along start->end
// (belt_footprint()). When no catalog belt fits the routed length, `fitted` is
// false, `belt_catalog_id` is empty and `width` is zero (collision is then not
// evaluated — there is no belt to collide).
struct PlacedBelt {
    std::size_t transport_index;     // index into LayoutSolution::transports
    std::string belt_catalog_id;     // chosen BeltSpec.id; empty if none fit
    core::Vec2 start;                // source port, world frame
    core::Vec2 end;                  // sink port, world frame
    core::Length width;              // chosen belt width (0 if none fit)
    core::Length length;             // routed straight-line distance
    bool fitted;                     // a catalog belt's range covers `length`
    bool collides_with_station;      // footprint overlaps a non-endpoint station
};

// The footprint of a belt: a `width`-wide rectangle centred on the segment
// start->end, as 4 CCW vertices. Degenerate input (start == end) returns an
// empty polygon.
core::Polygon belt_footprint(core::Vec2 start, core::Vec2 end, core::Length width);

// Route one belt per transport in `solution.transports`, in the same order.
// Each belt connects the resolved source/sink port world positions; the
// cheapest BeltSpec whose [min_length, max_length] covers the routed distance
// is chosen (fitted=false if none). belt-vs-station collision is checked
// against every station EXCEPT the belt's own two endpoints (a belt naturally
// meets the stations it connects); stations with no footprint (anchors) are
// skipped.
std::vector<PlacedBelt> route_belts(const LayoutProblem& problem,
                                    const LayoutSolution& solution,
                                    std::span<const core::BeltSpec> belt_catalog);

} // namespace tinycell::solver
