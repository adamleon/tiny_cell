#pragma once
#include <optional>
#include <entt/entt.hpp>
#include "components.hpp"
#include "pose_component.hpp"

namespace factory {

class PlacementPattern {
public:
    virtual ~PlacementPattern() = default;
    virtual std::optional<PoseComponent> next_pose(
        const PalletComponent&,
        const ItemPrototypeComponent&,
        const entt::registry&) const = 0;
};

class GridPattern : public PlacementPattern {
public:
    std::optional<PoseComponent> next_pose(
        const PalletComponent&        pallet,
        const ItemPrototypeComponent& proto,
        const entt::registry&) const override
    {
        int cols   = pallet.length_mm() / proto.length_mm();
        int rows   = pallet.width_mm()  / proto.width_mm();
        int placed = static_cast<int>(pallet.items().size());
        if (cols <= 0 || rows <= 0 || placed >= cols * rows) return std::nullopt;

        int col = placed % cols;
        int row = placed / cols;

        float x = (col + 0.5f) * proto.length_mm() - pallet.length_mm() * 0.5f;
        float y = (row + 0.5f) * proto.width_mm()  - pallet.width_mm()  * 0.5f;
        float z = static_cast<float>(pallet.height_mm()) + proto.height_mm() * 0.5f;

        PoseComponent pose;
        pose.position    = Vec3{x, y, z};
        pose.orientation = Quat{1.f, 0.f, 0.f, 0.f};
        pose.parent      = entt::null;
        return pose;
    }
};

}  // namespace factory
