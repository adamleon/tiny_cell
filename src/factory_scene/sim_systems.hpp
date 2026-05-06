#pragma once
#include <tuple>
#include <vector>
#include <entt/entt.hpp>
#include "components.hpp"
#include "factory_scene.hpp"

// Pure ECS simulation systems — no threepp, no rendering.
//
// sim::step() advances the simulation by dt seconds and returns StepEvents
// describing what changed. The caller is responsible for:
//   - Creating render objects for spawned entities
//   - Removing render objects and calling registry.destroy() for despawned entities

namespace factory::sim {

struct SpawnedItem {
    entt::entity entity;
    entt::entity prototype;  // ItemPrototypeComponent entity; caller uses for mesh creation
};

struct StepEvents {
    std::vector<SpawnedItem>  spawned;
    std::vector<entt::entity> despawned;  // entities to remove; caller destroys after mesh cleanup
};

inline StepEvents step(FactoryScene& scene, float dt) {
    auto& reg = scene.registry();
    StepEvents events;

    // ── Spawning ──────────────────────────────────────────────────────────────
    for (auto&& [src_ent, src] : reg.view<SourceComponent>().each()) {
        src.add_spawn_debt(src.rate_per_hour() / 3600.f * dt);
        while (src.spawn_debt() >= 1.f) {
            src.consume_spawn();

            auto* port  = reg.try_get<PortComponent>(src.out_port());
            auto* ppose = reg.try_get<PoseComponent>(src.out_port());
            if (!port || port->transport() == entt::null || !ppose) continue;
            if (!scene.transport_has_capacity(port->transport())) break;

            auto item = reg.create();
            reg.emplace<PoseComponent>(item).position = ppose->position;
            reg.emplace<ItemOnTransportComponent>(item).set_transport(port->transport());
            reg.emplace<SpawnedItemComponent>(item).set_prototype(src.prototype());

            events.spawned.push_back({item, src.prototype()});
        }
    }

    // ── Transport ─────────────────────────────────────────────────────────────
    using Transfer = std::tuple<entt::entity, entt::entity, Vec3>;
    std::vector<Transfer> transfers;

    for (auto&& [ent, it, pose] :
         reg.view<ItemOnTransportComponent, PoseComponent>().each())
    {
        auto* bc = reg.try_get<ConveyorBeltComponent>(it.transport());
        if (!bc) continue;

        auto* tc = reg.try_get<TransportComponent>(it.transport());
        if (tc && !tc->running()) continue;

        pose.position += bc->dir() * (bc->belt_speed_mm_s() * dt);

        const auto& ep = reg.get<PoseComponent>(bc->entry_port());
        float t = glm::dot(pose.position - ep.position, bc->dir());

        if (t < static_cast<float>(bc->length_mm())) continue;

        // Reached exit
        auto* exit_pc = reg.try_get<PortComponent>(bc->exit_port());
        auto* exit_pp = reg.try_get<PoseComponent>(bc->exit_port());
        if (!exit_pc || !exit_pp) continue;

        if (exit_pc->transport() != entt::null) {
            if (!scene.transport_has_capacity(exit_pc->transport())) continue;
            transfers.emplace_back(ent, exit_pc->transport(), exit_pp->position);
        } else {
            events.despawned.push_back(ent);
            reg.view<SinkComponent>().each([&](SinkComponent& sk) {
                if (sk.in_port() == bc->exit_port()) sk.increment_received();
            });
        }
    }

    // Apply transfers after the view loop to avoid invalidating iterators
    for (auto& [ent, next_belt, land_pos] : transfers) {
        reg.get<ItemOnTransportComponent>(ent).set_transport(next_belt);
        reg.get<PoseComponent>(ent).position = land_pos;
    }

    return events;
}

}  // namespace factory::sim
