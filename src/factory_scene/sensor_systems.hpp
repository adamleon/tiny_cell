#pragma once
#include <cmath>
#include <entt/entt.hpp>
#include <glm/glm.hpp>
#include "components.hpp"
#include "factory_scene.hpp"
#include "pose_component.hpp"

namespace factory::sensor {

// Test whether `item_e`'s world centre lies inside `sensor_e`'s detection
// volume. The volume is axis-aligned in the sensor's local frame; the item
// centre is transformed into that frame by the inverse of the sensor's
// parent-chain world transform. This is a "trigger point" model: the sensor
// fires when the item's centroid passes through the sensor's bounds.
//
// For occupancy/spawn-throttling use cases the sensor must be sized large
// enough to encompass the area of interest (typically `2 * item_length +
// 2 * gap` for a spawn port). For "stop-here" use cases the sensor should be
// small and centred on the desired stop position; the item then comes to
// rest with its centre inside the sensor (offset at most by half of the
// sensor's length on the entry axis, plus one tick of belt motion).
inline bool item_in_volume(const entt::registry& reg,
                           entt::entity         item_e,
                           entt::entity         sensor_e)
{
    const auto* dv = reg.try_get<DetectionVolumeComponent>(sensor_e);
    if (!dv) return false;

    const glm::mat4 sw  = world_transform(sensor_e, reg);
    const glm::mat4 inv = glm::inverse(sw);
    const glm::mat4 iw  = world_transform(item_e, reg);

    const glm::vec3 ilocal = glm::vec3(inv * glm::vec4(glm::vec3(iw[3]), 1.f));

    return std::abs(ilocal.x) <= dv->length_mm() * 0.5f
        && std::abs(ilocal.y) <= dv->width_mm()  * 0.5f
        && std::abs(ilocal.z) <= dv->height_mm() * 0.5f;
}

// Refresh `blocked` on every physical sensor (those carrying a
// DetectionVolumeComponent) from current item poses. Virtual sensors are not
// in this view and are left untouched.
inline void scan(FactoryScene& scene) {
    auto& reg = scene.registry();
    auto items = reg.view<SpawnedItemComponent, PoseComponent>();

    for (auto&& [sensor_e, sc, dv] :
         reg.view<SensorComponent, DetectionVolumeComponent>().each())
    {
        (void)dv;
        bool blocked = false;
        for (auto item_e : items) {
            if (item_in_volume(reg, item_e, sensor_e)) {
                blocked = true;
                break;
            }
        }
        sc.set_blocked(blocked);
    }
}

}  // namespace factory::sensor
