#pragma once
#include <optional>
#include "components.hpp"
#include "pose_component.hpp"

namespace factory {

// A generic surface that holds items. Either a pallet being filled by a
// station, or a depot holding pre-positioned items. The pattern doesn't care
// which — it works from dimensions and a placed-count.
//
//   length_mm / width_mm — surface footprint
//   height_mm            — surface top (z where items rest)
//   placed_count         — number of items already on the surface
struct PlacementSurface {
    int length_mm    = 0;
    int width_mm     = 0;
    int height_mm    = 0;
    int placed_count = 0;
};

class PlacementPattern {
public:
    virtual ~PlacementPattern() = default;
    // Returns the local pose for the next item, or nullopt if the surface is
    // full. Local frame: origin at surface centre, X-Y on the surface plane,
    // Z up. The caller multiplies by the surface's world transform.
    virtual std::optional<PoseComponent> next_pose(
        const PlacementSurface&,
        const ItemPrototypeComponent&) const = 0;
};

class GridPattern : public PlacementPattern {
public:
    std::optional<PoseComponent> next_pose(
        const PlacementSurface&       surface,
        const ItemPrototypeComponent& proto) const override
    {
        // cols along ECS X (across belt / surface width), rows along ECS Y
        // (belt direction / surface length).
        const int cols = surface.width_mm  / proto.width_mm();
        const int rows = surface.length_mm / proto.length_mm();
        if (cols <= 0 || rows <= 0 || surface.placed_count >= cols * rows)
            return std::nullopt;

        const int col = surface.placed_count % cols;
        const int row = surface.placed_count / cols;

        // Centre on the placed footprint so items don't drift to one edge.
        const float x = (col + 0.5f) * proto.width_mm()  - cols * proto.width_mm()  * 0.5f;
        const float y = (row + 0.5f) * proto.length_mm() - rows * proto.length_mm() * 0.5f;
        // z = bottom surface of item (placed on the surface top). Render loop
        // adds half-height to get centre.
        const float z = static_cast<float>(surface.height_mm);

        PoseComponent pose;
        pose.position    = Vec3{x, y, z};
        pose.orientation = Quat{1.f, 0.f, 0.f, 0.f};
        pose.parent      = entt::null;
        return pose;
    }
};

}  // namespace factory
