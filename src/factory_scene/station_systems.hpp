#pragma once
#include <entt/entt.hpp>
#include "station_components.hpp"
#include "factory_scene.hpp"
#include "sim_systems.hpp"

namespace factory::station {

inline void step(FactoryScene& scene,
                 std::vector<sim::ItemArrivedAtPort>& arrived,
                 float dt)
{
    auto& reg = scene.registry();

    auto restart_transport = [&](entt::entity transport) {
        if (transport == entt::null) return;
        auto& tc = reg.get<TransportComponent>(transport);
        tc.set_running(true);
        auto* bc = reg.try_get<ConveyorBeltComponent>(transport);
        if (bc) bc->set_capacity_blocked(false);
    };

    auto stop_transport = [&](entt::entity transport) {
        if (transport == entt::null) return;
        reg.get<TransportComponent>(transport).set_running(false);
    };

    for (auto&& [station_ent, sc] : reg.view<StationComponent>().each()) {
        auto* pc = reg.try_get<PalletizeComponent>(station_ent);

        // Grab incoming pallet
        if (pc && pc->state() == PalletizeState::WaitingForPallet) {
            // Hold box belt until we have a pallet to fill
            stop_transport(sc.controlled_transport());

            for (auto& arr : arrived) {
                if (arr.port != pc->pallet_arrival_port()) continue;
                auto& palletc = reg.emplace_or_replace<PalletComponent>(arr.item);
                palletc.set_length_mm(pc->pallet_length_mm());
                palletc.set_width_mm(pc->pallet_width_mm());
                palletc.set_height_mm(pc->pallet_height_mm());
                palletc.set_max_stack_height_mm(pc->pallet_max_stack_mm());
                pc->set_current_pallet(arr.item);
                pc->set_state(PalletizeState::HasPallet);
                stop_transport(pc->pallet_segment());
                restart_transport(sc.controlled_transport());
                arr.port = entt::null;
                break;
            }
        }

        // Grab incoming item
        if (sc.state() == StationState::Idle &&
            (!pc || pc->state() == PalletizeState::HasPallet))
        {
            for (auto& arr : arrived) {
                if (arr.port != sc.arrival_port()) continue;
                sc.capture(arr.item);
                stop_transport(sc.controlled_transport());
                sc.mechanism()->start(arr.item, reg);
                arr.port = entt::null;
                break;
            }
        }

        // Tick mechanism
        if (sc.state() != StationState::Processing) continue;
        sc.mechanism()->tick(dt, reg);
        if (!sc.mechanism()->done()) continue;

        if (pc && pc->state() == PalletizeState::HasPallet) {
            auto  pallet_e = pc->current_pallet();
            auto& palletc  = reg.get<PalletComponent>(pallet_e);
            const auto& sp = reg.get<SpawnedItemComponent>(sc.held_item());
            const auto* proto = reg.try_get<ItemPrototypeComponent>(sp.prototype());

            if (proto) {
                if (auto placement = pc->pattern()->next_pose(palletc, *proto, reg)) {
                    auto& item_pose       = reg.get<PoseComponent>(sc.held_item());
                    item_pose.position    = placement->position;
                    item_pose.orientation = placement->orientation;
                    item_pose.parent      = pallet_e;
                    palletc.add_item(sc.held_item());
                }
            }

            sc.release();

            bool full = proto && !pc->pattern()->next_pose(palletc, *proto, reg).has_value();

            if (full) {
                auto& out_bc      = reg.get<ConveyorBeltComponent>(pc->pallet_output_transport());
                auto& entry_pose  = reg.get<PoseComponent>(out_bc.entry_port());
                auto& pal_pose    = reg.get<PoseComponent>(pallet_e);
                pal_pose.position = entry_pose.position;
                pal_pose.parent   = scene.root_entity();
                reg.emplace_or_replace<ItemOnTransportComponent>(pallet_e)
                   .set_transport(pc->pallet_output_transport());
                pc->set_current_pallet(entt::null);
                pc->set_state(PalletizeState::WaitingForPallet);
                restart_transport(pc->pallet_segment());
                // box belt stays stopped — WaitingForPallet will hold it
            } else {
                restart_transport(sc.controlled_transport());
            }
        } else {
            sc.release();
            restart_transport(sc.controlled_transport());
        }
    }
}

}  // namespace factory::station
