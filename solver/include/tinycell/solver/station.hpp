#pragma once

// Station — a group of equipment instances that share a station frame
// (architecture.md §4, solver.md "Geometry flattening & caching").
// Solver-internal (CLAUDE.md §0): the layer-3 placer reasons about
// stations as the unit of intra-station layout (equipment poses
// relative to the station frame) and inter-station collision (one
// world transform of the station's accumulated footprint, when it
// lands).
//
// Phase B.3 introduces the type and a hand-rolled placement consumer
// (demos/layer_3_placement). The footprint cache that accumulates an
// instance's polygon into a station-frame hull/union arrives in a
// later phase, when a real consumer (Phase D collision encoders)
// requires it — concrete-over-abstract.

#include <cstddef>
#include <string>
#include <tinycell/geometry.hpp>
#include <vector>

namespace tinycell::solver {

// One placed instance inside a station. References the allocator's
// BoundInstance by id (allocator.hpp). catalog_id + strategy_name
// are denormalised here so SVG / placement code can look up the
// footprint and label the instance without walking back to the
// AllocationResult — the BoundInstance set is the source of truth
// for throughput, this struct is the source of truth for placement.
struct InstancePlacement {
    std::size_t bound_instance_id;
    std::string catalog_id;
    std::string strategy_name;
    core::Pose2D pose_in_station;
};

// A station. own_frame is its FrameId in the FrameTable (the table
// is the source of truth for the station's pose-in-world; the Station
// just names which frame is its). instances are the equipment placed
// inside it, in declaration order.
struct Station {
    std::string id;
    core::FrameId own_frame;
    std::vector<InstancePlacement> instances;
};

} // namespace tinycell::solver
