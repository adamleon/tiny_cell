#pragma once
#include <cmath>
#include <entt/entt.hpp>
#include <glm/glm.hpp>
#include "components.hpp"
#include "factory_scene.hpp"
#include "sensor_systems.hpp"
#include "station_components.hpp"
#include "pose_component.hpp"

namespace factory::station {

namespace detail {

// Return the first SpawnedItem whose bounding box intersects any physical
// sensor on `port_e`'s sensor list. Returns entt::null when no match.
inline entt::entity find_item_at_port(const entt::registry& reg, entt::entity port_e) {
    if (port_e == entt::null) return entt::null;
    const auto* port = reg.try_get<PortComponent>(port_e);
    if (!port) return entt::null;

    auto items = reg.view<SpawnedItemComponent, PoseComponent>();

    for (auto sensor_e : port->sensors()) {
        for (auto item_e : items) {
            if (sensor::item_at_laser(reg, item_e, sensor_e))
                return item_e;
        }
    }
    return entt::null;
}

// ── Picker-OR-magic agent helpers ──────────────────────────────────────────
// Both agent flavours share the same state enum and the same conceptual
// dispatch contract. These helpers let the dispatcher remain ignorant of
// which one it's talking to.

inline PickerState agent_state(const entt::registry& reg, entt::entity e) {
    if (const auto* pt = reg.try_get<PickerTransportComponent>(e)) return pt->state();
    if (const auto* mt = reg.try_get<MagicTransportComponent>(e))  return mt->state();
    return PickerState::Idle;
}

inline entt::entity agent_current_box(const entt::registry& reg, entt::entity e) {
    if (const auto* pt = reg.try_get<PickerTransportComponent>(e)) return pt->current_box();
    if (const auto* mt = reg.try_get<MagicTransportComponent>(e))  return mt->current_box();
    return entt::null;
}

inline void agent_dispatch(entt::registry& reg, entt::entity agent_e,
                           Vec3 pickup_target, Vec3 drop_target,
                           entt::entity drop_container, Quat drop_orientation,
                           entt::entity box)
{
    if (auto* pt = reg.try_get<PickerTransportComponent>(agent_e)) {
        pt->set_pickup_target(pickup_target);
        pt->set_drop_target(drop_target);
        pt->set_drop_container(drop_container);
        pt->set_drop_orientation(drop_orientation);
        pt->set_current_box(box);
        pt->set_state(PickerState::MovingToBox);
        return;
    }
    if (auto* mt = reg.try_get<MagicTransportComponent>(agent_e)) {
        mt->set_pickup_target(pickup_target);
        mt->set_drop_target(drop_target);
        mt->set_drop_container(drop_container);
        mt->set_drop_orientation(drop_orientation);
        mt->set_current_box(box);
        // Snapshot the agent's current world position as the start of this leg.
        const auto& mpose = reg.get<PoseComponent>(agent_e);
        mt->set_leg_origin(mpose.position);
        mt->set_elapsed_s(0.f);
        mt->set_state(PickerState::MovingToBox);
    }
}

}  // namespace detail

// Pure dispatch — runs after sensors and transports have refreshed.
inline void step(FactoryScene& scene, float /*dt*/) {
    auto& reg = scene.registry();

    for (auto&& [station_e, sc] : reg.view<StationComponent>().each()) {
        auto* pc = reg.try_get<PalletizeComponent>(station_e);

        // ── Pallet acquisition ──────────────────────────────────────────────
        // Only claim a pallet that does NOT already carry a PalletComponent —
        // presence of that component means the pallet has been claimed before
        // (and likely just released). Without this guard, the released pallet
        // sits in the detect sensor's volume for many ticks while it advances
        // out, and the station re-claims it every tick, locking the cycle.
        if (pc && pc->current_pallet() == entt::null) {
            auto pallet_item = detail::find_item_at_port(reg, pc->pallet_arrival_port());
            if (pallet_item != entt::null && !reg.any_of<PalletComponent>(pallet_item)) {
                pc->set_current_pallet(pallet_item);
                auto& palletc = reg.emplace<PalletComponent>(pallet_item);
                palletc.set_length_mm(pc->pallet_length_mm());
                palletc.set_width_mm(pc->pallet_width_mm());
                palletc.set_height_mm(pc->pallet_height_mm());
                palletc.set_max_stack_height_mm(pc->pallet_max_stack_mm());
                if (pc->pallet_tap_virtual_sensor() != entt::null)
                    reg.get<SensorComponent>(pc->pallet_tap_virtual_sensor()).set_blocked(true);
            }
        }

        const bool has_pallet = !pc || pc->current_pallet() != entt::null;

        // ── Find an idle agent (picker or magic) ───────────────────────────
        entt::entity idle_agent = entt::null;
        for (auto p : sc.pickers()) {
            if (detail::agent_state(reg, p) == PickerState::Idle) {
                idle_agent = p;
                break;
            }
        }

        // ── Box dispatch ────────────────────────────────────────────────────
        if (has_pallet && idle_agent != entt::null && pc) {
            auto box_item = detail::find_item_at_port(reg, sc.arrival_port());
            if (box_item != entt::null) {
                bool already_claimed = false;
                for (auto p : sc.pickers()) {
                    if (detail::agent_current_box(reg, p) == box_item) {
                        already_claimed = true;
                        break;
                    }
                }
                if (!already_claimed && pc->pattern()) {
                    auto& palletc = reg.get<PalletComponent>(pc->current_pallet());
                    auto* sp = reg.try_get<SpawnedItemComponent>(box_item);
                    if (sp) {
                        auto* proto = reg.try_get<ItemPrototypeComponent>(sp->prototype());
                        if (proto) {
                            PlacementSurface surface;
                            surface.length_mm    = palletc.length_mm();
                            surface.width_mm     = palletc.width_mm();
                            surface.height_mm    = palletc.height_mm();
                            surface.placed_count = static_cast<int>(palletc.items().size());
                            auto slot = pc->pattern()->next_pose(surface, *proto);
                            if (slot.has_value()) {
                                glm::mat4 pallet_world  = world_transform(pc->current_pallet(), reg);
                                glm::vec4 drop_world    = pallet_world * glm::vec4(slot->position, 1.f);
                                glm::mat4 box_w         = world_transform(box_item, reg);

                                detail::agent_dispatch(reg, idle_agent,
                                                       Vec3(box_w[3]),
                                                       Vec3(drop_world),
                                                       pc->current_pallet(),
                                                       slot->orientation,
                                                       box_item);
                            }
                        }
                    }
                }
            }
        }

        // ── Update arrival virtual sensor ──────────────────────────────────
        // Blocked iff station can't accept a new box right now.
        if (sc.arrival_virtual_sensor() != entt::null) {
            bool blocked = !has_pallet;
            if (!blocked) {
                bool any_idle = false;
                for (auto p : sc.pickers()) {
                    if (detail::agent_state(reg, p) == PickerState::Idle) {
                        any_idle = true;
                        break;
                    }
                }
                blocked = !any_idle;
            }
            reg.get<SensorComponent>(sc.arrival_virtual_sensor()).set_blocked(blocked);
        }

        // ── Pallet release (full + agents idle) ────────────────────────────
        if (pc && pc->current_pallet() != entt::null) {
            bool all_idle = true;
            for (auto p : sc.pickers()) {
                if (detail::agent_state(reg, p) != PickerState::Idle) {
                    all_idle = false;
                    break;
                }
            }

            // Pallet "full" iff pattern returns nullopt for the next slot.
            // We need an item prototype to query the pattern; use the prototype
            // of the first placed item.
            bool full = false;
            auto& palletc = reg.get<PalletComponent>(pc->current_pallet());
            if (!palletc.items().empty() && pc->pattern()) {
                auto first  = palletc.items().front();
                auto* sp    = reg.try_get<SpawnedItemComponent>(first);
                if (sp) {
                    auto* proto = reg.try_get<ItemPrototypeComponent>(sp->prototype());
                    if (proto) {
                        PlacementSurface surface;
                        surface.length_mm    = palletc.length_mm();
                        surface.width_mm     = palletc.width_mm();
                        surface.height_mm    = palletc.height_mm();
                        surface.placed_count = static_cast<int>(palletc.items().size());
                        full = !pc->pattern()->next_pose(surface, *proto).has_value();
                    }
                }
            }

            if (full && all_idle) {
                pc->set_current_pallet(entt::null);
                if (pc->pallet_tap_virtual_sensor() != entt::null)
                    reg.get<SensorComponent>(pc->pallet_tap_virtual_sensor()).set_blocked(false);
            }
        }
    }
}

}  // namespace factory::station
