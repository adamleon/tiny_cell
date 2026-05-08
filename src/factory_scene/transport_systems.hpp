#pragma once
#include <cmath>
#include <tuple>
#include <vector>
#include <entt/entt.hpp>
#include <glm/glm.hpp>
#include "components.hpp"
#include "factory_scene.hpp"
#include "pose_component.hpp"

namespace factory::transport {

inline Vec3 world_position(entt::entity e, const entt::registry& reg) {
    glm::mat4 m = world_transform(e, reg);
    return Vec3(m[3]);
}

// A belt is "moving" iff its controller is on AND every gate port is clear.
inline bool belt_is_moving(const entt::registry& reg, entt::entity belt_e) {
    const auto* tc = reg.try_get<TransportComponent>(belt_e);
    if (!tc || !tc->running()) return false;
    const auto* bc = reg.try_get<ConveyorBeltComponent>(belt_e);
    if (!bc) return false;
    for (auto p : bc->gate_ports()) {
        if (!port_is_clear(reg, p)) return false;
    }
    return true;
}

// Step every transport flavour for `dt`:
//   - Belts advance items along `dir` at `belt_speed_mm_s` if moving;
//     items past `length_mm` either hand off (if exit_port.transport != null
//     and the exit port is clear) or clamp to the exit position.
//   - Pickers move toward the target dictated by their state, transition on
//     reach, and perform the matching transport / parent reassignment.
//
// Items in containers (parented to other items) move via parent-chain — they
// are not iterated here.
inline void step(FactoryScene& scene, float dt) {
    auto& reg = scene.registry();

    // ── Belt motion ──────────────────────────────────────────────────────────

    struct Handover {
        entt::entity item;
        entt::entity next_belt;
        Vec3         land_pos;
    };
    std::vector<Handover> handovers;

    for (auto&& [item_e, it, ipose] :
         reg.view<ItemOnTransportComponent, PoseComponent>().each())
    {
        auto belt_e   = it.transport();
        const auto* bc = reg.try_get<ConveyorBeltComponent>(belt_e);
        if (!bc) continue;          // not on a belt (e.g. on a picker)

        if (!belt_is_moving(reg, belt_e)) continue;

        ipose.position += bc->dir() * bc->belt_speed_mm_s() * dt;

        Vec3  entry_world = world_position(bc->entry_port(), reg);
        float t           = glm::dot(ipose.position - entry_world, bc->dir());

        if (t < static_cast<float>(bc->length_mm())) continue;

        const auto* exit_pc = reg.try_get<PortComponent>(bc->exit_port());
        if (!exit_pc) continue;

        if (exit_pc->transport() != entt::null) {
            // Belt-to-belt handover (gated by destination port = exit_port).
            if (!port_is_clear(reg, bc->exit_port())) continue;

            auto next_e = exit_pc->transport();
            const auto* next_bc = reg.try_get<ConveyorBeltComponent>(next_e);
            if (!next_bc) continue;

            Vec3 next_entry_world = world_position(next_bc->entry_port(), reg);
            handovers.push_back({item_e, next_e, next_entry_world});
        } else {
            // Terminal exit — clamp to exit position; sinks (in lifecycle)
            // and stations (via sensor poll) decide what happens next.
            ipose.position = world_position(bc->exit_port(), reg);
        }
    }

    for (auto& h : handovers) {
        reg.get<ItemOnTransportComponent>(h.item).set_transport(h.next_belt);
        reg.get<PoseComponent>(h.item).position = h.land_pos;
    }

    // ── Picker motion ────────────────────────────────────────────────────────

    for (auto&& [picker_e, pt, ppose] :
         reg.view<PickerTransportComponent, PoseComponent>().each())
    {
        if (pt.state() == PickerState::Idle) continue;

        Vec3 target;
        switch (pt.state()) {
            case PickerState::MovingToBox: target = pt.pickup_target(); break;
            case PickerState::Carrying:    target = pt.drop_target();   break;
            case PickerState::Returning:   target = pt.home_pose();     break;
            default: continue;
        }

        Vec3  to_target = target - ppose.position;
        float dist      = glm::length(to_target);
        float step_dist = pt.speed_mm_s() * dt;

        if (step_dist < dist) {
            ppose.position += (to_target / dist) * step_dist;
            continue;
        }

        // Reached / overshot target — clamp and transition.
        ppose.position = target;

        switch (pt.state()) {
            case PickerState::MovingToBox: {
                auto box = pt.current_box();
                if (box != entt::null) {
                    reg.get_or_emplace<ItemOnTransportComponent>(box)
                       .set_transport(picker_e);
                    auto& bpose = reg.get<PoseComponent>(box);
                    bpose.position = Vec3{0.f};   // ride the picker at zero local
                    bpose.parent   = picker_e;
                }
                pt.set_state(PickerState::Carrying);
                break;
            }
            case PickerState::Carrying: {
                auto box       = pt.current_box();
                auto container = pt.drop_container();
                if (box != entt::null) {
                    if (reg.any_of<ItemOnTransportComponent>(box))
                        reg.remove<ItemOnTransportComponent>(box);

                    auto& bpose = reg.get<PoseComponent>(box);
                    if (container != entt::null) {
                        // Convert drop_target (world) into container-local frame
                        // so the box doesn't snap visually on re-parent.
                        glm::mat4 cw    = world_transform(container, reg);
                        glm::mat4 ci    = glm::inverse(cw);
                        Vec3      local = Vec3(ci * glm::vec4(pt.drop_target(), 1.f));
                        bpose.position    = local;
                        bpose.orientation = pt.drop_orientation();
                        bpose.parent      = container;
                        if (auto* palletc = reg.try_get<PalletComponent>(container))
                            palletc->add_item(box);
                    } else {
                        bpose.parent = scene.root_entity();
                    }
                }
                pt.set_state(PickerState::Returning);
                break;
            }
            case PickerState::Returning: {
                pt.set_state(PickerState::Idle);
                pt.set_current_box(entt::null);
                break;
            }
            default: break;
        }
    }
}

}  // namespace factory::transport
