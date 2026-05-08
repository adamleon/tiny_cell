#pragma once
#include <algorithm>
#include <cmath>
#include <vector>
#include <entt/entt.hpp>
#include <glm/glm.hpp>
#include "components.hpp"
#include "factory_scene.hpp"
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
                src.consume_spawn();

                auto  item  = reg.create();
                auto& ipose = reg.emplace<PoseComponent>(item);
                glm::mat4 pw   = world_transform(src.out_port(), reg);
                ipose.position = Vec3(pw[3]);
                ipose.parent   = scene.root_entity();

                reg.emplace<ItemOnTransportComponent>(item).set_transport(port->transport());
                reg.emplace<SpawnedItemComponent>(item).set_prototype(src.prototype());

                events.spawned.push_back({item, src.prototype()});
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
                if (sensor::item_in_volume(reg, item_e, sensor_e))
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
