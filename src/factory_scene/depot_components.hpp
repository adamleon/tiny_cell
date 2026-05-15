#pragma once
#include <memory>
#include <entt/entt.hpp>
#include "placement_pattern.hpp"
#include "components.hpp"

namespace factory {

// A depot is a stationary "transport" — it holds items at pattern-determined
// positions but does not move them. Items appear here (spawned by a source)
// and are removed by a picker. Conceptually adjacent to ConveyorBelt in the
// transport-flavour family: same TransportComponent + Item-on-Transport
// pipeline, just with zero motion.
//
// Capacity is *implicit* in the pattern: `pattern->next_pose(...)` returning
// nullopt means the depot is full.
//
// Backpressure: for capacity-1 depots, a laser sensor at the depot's port
// position naturally blocks the source while the item is there. For
// multi-slot depots, a future `depot::step` system will be needed to update
// a virtual "full" sensor when the pattern reports no free slot. Not
// implemented yet — multi-slot depots aren't in scope.
//
// The depot's footprint (length/width/height) defines the PlacementSurface
// the pattern operates on. For a single-slot depot (one box, one pallet),
// set the footprint equal to the item's dimensions — GridPattern then yields
// exactly one slot at the centre.

struct DepotTransportComponent {
    int               length_mm()    const { return length_mm_; }
    int               width_mm()     const { return width_mm_;  }
    int               height_mm()    const { return height_mm_; }
    Vec3              forward_axis() const { return forward_axis_; }
    PlacementPattern* pattern()      const { return pattern_.get(); }

    void set_length_mm(int mm)            { length_mm_ = detail::positive(mm); }
    void set_width_mm(int mm)             { width_mm_  = detail::positive(mm); }
    void set_height_mm(int mm)            { height_mm_ = detail::non_neg(mm); }
    void set_forward_axis(Vec3 v)         { forward_axis_ = v; }
    void set_pattern(std::shared_ptr<PlacementPattern> p) { pattern_ = std::move(p); }

private:
    int  length_mm_       = 1;
    int  width_mm_        = 1;
    int  height_mm_       = 0;
    Vec3 forward_axis_    = {1.f, 0.f, 0.f};
    std::shared_ptr<PlacementPattern> pattern_;
};

}  // namespace factory
