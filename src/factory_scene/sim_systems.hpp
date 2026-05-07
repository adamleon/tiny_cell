#pragma once
#include <tuple>
#include <vector>
#include <entt/entt.hpp>
#include "components.hpp"
#include "factory_scene.hpp"

namespace factory::sim {

struct SpawnedItem {
    entt::entity entity;
    entt::entity prototype;
};

struct ItemArrivedAtPort {
    entt::entity item;
    entt::entity port;
};

struct StepEvents {
    std::vector<SpawnedItem>       spawned;
    std::vector<ItemArrivedAtPort> arrived;
    std::vector<entt::entity>      despawned;
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

            auto item         = reg.create();
            auto& item_pose   = reg.emplace<PoseComponent>(item);
            item_pose.position = ppose->position;
            item_pose.parent   = scene.root_entity();
            reg.emplace<ItemOnTransportComponent>(item).set_transport(port->transport());
            reg.emplace<SpawnedItemComponent>(item).set_prototype(src.prototype());

            events.spawned.push_back({item, src.prototype()});
        }
    }

    // ── Transport ─────────────────────────────────────────────────────────────

    // Auto-resume belts that were blocked but now have space downstream
    for (auto&& [belt_ent, bc, tc] :
         reg.view<ConveyorBeltComponent, TransportComponent>().each())
    {
        if (!bc.capacity_blocked()) continue;
        auto* exit_pc = reg.try_get<PortComponent>(bc.exit_port());
        if (!exit_pc) continue;
        if (exit_pc->transport() == entt::null ||
            scene.transport_has_space(exit_pc->transport()))
        {
            bc.set_capacity_blocked(false);
            tc.set_running(true);
        }
    }

    using Transfer = std::tuple<entt::entity, entt::entity, Vec3>;
    std::vector<Transfer> transfers;

    // Entities to remove ItemOnTransportComponent from (station intercepts)
    std::vector<entt::entity> remove_transport;

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

        auto* exit_pc = reg.try_get<PortComponent>(bc->exit_port());
        auto* exit_pp = reg.try_get<PoseComponent>(bc->exit_port());
        if (!exit_pc || !exit_pp) continue;

        if (exit_pc->transport() != entt::null) {
            if (!scene.transport_has_capacity(exit_pc->transport())) {
                // Block this belt until downstream has space
                auto* this_bc = reg.try_get<ConveyorBeltComponent>(it.transport());
                auto* this_tc = reg.try_get<TransportComponent>(it.transport());
                if (this_bc && this_tc) {
                    this_bc->set_capacity_blocked(true);
                    this_tc->set_running(false);
                }
                continue;
            }
            transfers.emplace_back(ent, exit_pc->transport(), exit_pp->position);
        } else {
            // Terminal exit
            bool sink_found = false;
            reg.view<SinkComponent>().each([&](SinkComponent& sk) {
                if (sk.in_port() == bc->exit_port()) {
                    sk.increment_received();
                    sink_found = true;
                }
            });
            if (sink_found) {
                events.despawned.push_back(ent);
            } else {
                events.arrived.push_back({ent, bc->exit_port()});
                remove_transport.push_back(ent);
            }
        }
    }

    for (auto& [ent, next_belt, land_pos] : transfers) {
        reg.get<ItemOnTransportComponent>(ent).set_transport(next_belt);
        reg.get<PoseComponent>(ent).position = land_pos;
    }

    for (auto e : remove_transport) {
        reg.remove<ItemOnTransportComponent>(e);
    }

    return events;
}

}  // namespace factory::sim
