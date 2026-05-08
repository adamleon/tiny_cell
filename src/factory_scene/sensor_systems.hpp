#pragma once
#include <cmath>
#include <entt/entt.hpp>
#include <glm/glm.hpp>
#include "components.hpp"
#include "factory_scene.hpp"
#include "pose_component.hpp"

namespace factory::sensor {

// Test whether `item_e`'s bounding box contains `sensor_e`'s world position.
// The sensor is a real point in 3D — a laser at a specific (x, y, z) — and
// the item's collision box (from ItemPrototypeComponent) decides whether it
// is currently on that point. If the item has no prototype it is treated
// as a point and only triggers when its centre coincides exactly with the
// sensor.
//
// Same check serves both "stop here" sensors (the item halts when its
// leading edge first encloses the laser point) and "occupancy" sensors
// (the laser stays triggered while any part of the body covers the point).
// Sensors at different heights or cross-belt offsets do not interfere with
// each other because the test is point-in-box on all three axes.
inline bool item_at_laser(const entt::registry& reg,
                          entt::entity         item_e,
                          entt::entity         sensor_e)
{
    if (!reg.any_of<LaserSensorComponent>(sensor_e)) return false;

    const glm::mat4 sw   = world_transform(sensor_e, reg);
    const glm::vec3 spos = glm::vec3(sw[3]);

    const glm::mat4 iw   = world_transform(item_e, reg);
    const glm::vec3 ipos = glm::vec3(iw[3]);

    float ihl = 0.f, ihw = 0.f, ihh = 0.f;
    if (const auto* sp = reg.try_get<SpawnedItemComponent>(item_e)) {
        if (const auto* proto = reg.try_get<ItemPrototypeComponent>(sp->prototype())) {
            ihl = proto->length_mm() * 0.5f;
            ihw = proto->width_mm()  * 0.5f;
            ihh = proto->height_mm() * 0.5f;
        }
    }

    // Project (sensor − item.centre) onto each item-local axis. The laser
    // is inside the item's OBB iff every projection magnitude is within
    // that axis's half-extent.
    const glm::vec3 d  = spos - ipos;
    const glm::vec3 ix = glm::vec3(iw[0]);
    const glm::vec3 iy = glm::vec3(iw[1]);
    const glm::vec3 iz = glm::vec3(iw[2]);

    if (std::abs(glm::dot(d, ix)) > ihl) return false;
    if (std::abs(glm::dot(d, iy)) > ihw) return false;
    if (std::abs(glm::dot(d, iz)) > ihh) return false;
    return true;
}

// Refresh `blocked` on every laser sensor from current item poses. Virtual
// sensors (no LaserSensorComponent) are not in this view and are left
// untouched — their `blocked` is written by orchestration code.
inline void scan(FactoryScene& scene) {
    auto& reg = scene.registry();
    auto items = reg.view<SpawnedItemComponent, PoseComponent>();

    // LaserSensorComponent is an empty marker — EnTT's .each() drops empty
    // types from the tuple, so we only bind entity + SensorComponent here.
    for (auto&& [sensor_e, sc] :
         reg.view<SensorComponent, LaserSensorComponent>().each())
    {
        bool blocked = false;
        for (auto item_e : items) {
            if (item_at_laser(reg, item_e, sensor_e)) {
                blocked = true;
                break;
            }
        }
        sc.set_blocked(blocked);
    }
}

}  // namespace factory::sensor
