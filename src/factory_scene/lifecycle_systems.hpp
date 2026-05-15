#pragma once
#include <algorithm>
#include <cmath>
#include <vector>
#include <entt/entt.hpp>
#include <glm/glm.hpp>
#include "components.hpp"
#include "depot_components.hpp"
#include "factory_scene.hpp"
#include "placement_pattern.hpp"
#include "pose_component.hpp"
#include "sensor_systems.hpp"

namespace factory::lifecycle {

struct SpawnedItem {
    entt::entity entity;
    entt::entity prototype;
};

struct LifecycleEvents {
    std::vector<SpawnedItem>  spawned;
    std::vector<entt::entity> despawned;   // caller is responsible for reg.destroy
};

// Source spawn + sink despawn. The only system that creates or destroys
// item entities. Caller destroys despawned entities (so it can traverse
// container children, remove meshes, etc., before destruction).
inline LifecycleEvents step(FactoryScene& scene, float dt) {
    auto& reg = scene.registry();
    LifecycleEvents events;

    // ── Sources ──────────────────────────────────────────────────────────────

    for (auto&& [src_e, src] : reg.view<SourceComponent>().each()) {
        src.add_spawn_debt(src.rate_per_hour() / 3600.f * dt);

        // At most one spawn per source per tick. A `while` loop would read the
        // sensor's cached `blocked` flag from the start of the tick, miss the
        // just-spawned item, and stamp out N items at the same world position
        // when debt > 1 — exactly the pile-up the spawn-area sensor is meant
        // to prevent. Throughput is then capped at one item per tick (≈ 60/s
        // at a 60 Hz tick rate); excess debt accumulates and is consumed on
        // following ticks.
        if (src.spawn_debt() >= 1.f && port_is_clear(reg, src.out_port())) {
            const auto* port = reg.try_get<PortComponent>(src.out_port());
            if (port) {
                const auto* proto =
                    reg.try_get<ItemPrototypeComponent>(src.prototype());
                const auto transport_e = port->transport();
                const auto* depot      =
                    reg.try_get<DepotTransportComponent>(transport_e);

                glm::vec3 spawn_pos{0.f};
                bool      have_pos = false;

                if (depot && depot->pattern() && proto) {
                    // Depot: place at pattern's next free slot in depot-local
                    // frame, then transform to world. The depot's own world
                    // pose carries the placement.
                    int placed = 0;
                    reg.view<ItemOnTransportComponent>().each(
                        [&](const ItemOnTransportComponent& it) {
                            if (it.transport() == transport_e) ++placed;
                        });
                    PlacementSurface surface;
                    surface.length_mm    = depot->length_mm();
                    surface.width_mm     = depot->width_mm();
                    surface.height_mm    = depot->height_mm();
                    surface.placed_count = placed;
                    const auto slot = depot->pattern()->next_pose(surface, *proto);
                    if (slot.has_value()) {
                        const glm::mat4 dw    = world_transform(transport_e, reg);
                        const glm::vec4 local = glm::vec4(slot->position, 1.f);
                        spawn_pos = glm::vec3(dw * local);
                        have_pos  = true;
                    }
                } else {
                    // Belt (or bare transport): spawn with the item's leading
                    // edge at the port — shift the centre back by half the
                    // item's extent along the port's forward direction. This
                    // keeps the freshly spawned item from clipping into the
                    // previous one (whose trailing edge has only just cleared
                    // the source's laser).
                    const glm::mat4 pw   = world_transform(src.out_port(), reg);
                    const glm::vec3 ppos = glm::vec3(pw[3]);
                    const glm::vec3 pdir = glm::vec3(pw[0]);
                    float half_along_belt = 0.f;
                    if (proto) {
                        half_along_belt =
                            std::abs(pdir.x) * proto->length_mm() * 0.5f
                          + std::abs(pdir.y) * proto->width_mm()  * 0.5f
                          + std::abs(pdir.z) * proto->height_mm() * 0.5f;
                    }
                    spawn_pos = ppos - pdir * half_along_belt;
                    have_pos  = true;
                }

                if (have_pos) {
                    src.consume_spawn();

                    auto  item  = reg.create();
                    auto& ipose = reg.emplace<PoseComponent>(item);
                    ipose.position = spawn_pos;
                    ipose.parent   = scene.root_entity();

                    reg.emplace<ItemOnTransportComponent>(item).set_transport(transport_e);
                    reg.emplace<SpawnedItemComponent>(item).set_prototype(src.prototype());

                    events.spawned.push_back({item, src.prototype()});
                }
            }
        }
    }

    // ── Sinks ────────────────────────────────────────────────────────────────
    //
    // A sink despawns any item whose world centre falls inside the volume of
    // any physical sensor on the sink's in_port. (Set up an `add_physical_sensor`
    // on the sink's in_port to enable detection.) Items are NOT destroyed here —
    // the caller is responsible for cleanup (so it can traverse container
    // children, remove meshes, etc.).

    auto items = reg.view<SpawnedItemComponent, PoseComponent>();

    for (auto&& [sink_e, sink] : reg.view<SinkComponent>().each()) {
        const auto* port = reg.try_get<PortComponent>(sink.in_port());
        if (!port) continue;

        std::vector<entt::entity> to_despawn;
        for (auto sensor_e : port->sensors()) {
            for (auto item_e : items) {
                if (sensor::item_at_laser(reg, item_e, sensor_e))
                    to_despawn.push_back(item_e);
            }
        }

        std::sort(to_despawn.begin(), to_despawn.end());
        to_despawn.erase(std::unique(to_despawn.begin(), to_despawn.end()),
                         to_despawn.end());

        for (auto item_e : to_despawn) {
            sink.increment_received();
            events.despawned.push_back(item_e);
        }
    }

    return events;
}

}  // namespace factory::lifecycle
