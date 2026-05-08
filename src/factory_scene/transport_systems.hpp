#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <tuple>
#include <unordered_set>
#include <vector>
#include <entt/entt.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include "components.hpp"
#include "factory_scene.hpp"
#include "pose_component.hpp"

namespace factory::transport {

inline Vec3 world_position(entt::entity e, const entt::registry& reg) {
    glm::mat4 m = world_transform(e, reg);
    return Vec3(m[3]);
}

namespace magic {

// Whimsical position interpolation: linear baseline + parabolic vertical
// arc + horizontal swirl whose envelope peaks mid-leg. Designed to look
// non-physical without being chaotic.
inline Vec3 position(Vec3 origin, Vec3 target, float t) {
    constexpr float kPi = 3.14159265358979323846f;
    if (t < 0.f) t = 0.f;
    if (t > 1.f) t = 1.f;

    Vec3        base   = origin + (target - origin) * t;
    const Vec3  d      = target - origin;
    const float length = glm::length(d);
    if (length < 1e-3f) return base;

    const Vec3  dir    = d / length;
    const Vec3  up     = Vec3{0.f, 0.f, 1.f};
    Vec3        right  = glm::cross(dir, up);
    if (glm::length(right) < 1e-3f) right = Vec3{1.f, 0.f, 0.f};
    else                            right = glm::normalize(right);

    // Horizontal swirl: 3 full oscillations across the leg, amplitude
    // peaking at the midpoint and tapering to 0 at both ends so we land
    // cleanly on the target.
    const float swirl_envelope = std::sin(t * kPi);
    const float swirl_phase    = std::sin(t * 6.f * kPi);
    const float swirl_amp      = std::min(length * 0.3f, 800.f);
    base += right * (swirl_phase * swirl_amp * swirl_envelope);

    // Parabolic vertical arc — peaks at t=0.5.
    const float arc_envelope = 4.f * t * (1.f - t);
    const float arc_amp      = std::min(length * 0.25f, 600.f);
    base += up * (arc_envelope * arc_amp);

    return base;
}

// Tumbling rotation: three independent rates around three axes, plus a
// per-entity seed so different magics don't lock-step.
inline Quat rotation(float t, float seed) {
    constexpr float kPi = 3.14159265358979323846f;
    const float yaw   = t * 4.f * kPi + seed;
    const float pitch = t * 6.f * kPi + seed * 1.3f;
    const float roll  = t * 8.f * kPi + seed * 0.7f;
    return glm::angleAxis(yaw,   Vec3{0.f, 0.f, 1.f})
         * glm::angleAxis(pitch, Vec3{0.f, 1.f, 0.f})
         * glm::angleAxis(roll,  Vec3{1.f, 0.f, 0.f});
}

}  // namespace magic

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
    //
    // Two-pass: first collect every item-on-belt with its start-of-tick
    // position-along-belt `t`, sort by belt then by descending `t`. Process
    // in that order so the leading item on each belt claims the exit before
    // any trailing item can leapfrog into it. Once any item on a belt
    // handovers / clamps to its exit this tick, the belt is "frozen" for the
    // rest of the tick and other items behind the leader stay in place; the
    // sensor pass on the next tick will see the leader occupying the exit
    // and freeze the belt cleanly.

    struct PendingItem {
        entt::entity                 item;
        entt::entity                 belt;
        const ConveyorBeltComponent* bc;
        Vec3                         entry_world;
        float                        t_start;
    };
    std::vector<PendingItem> pending;

    for (auto&& [item_e, it, ipose] :
         reg.view<ItemOnTransportComponent, PoseComponent>().each())
    {
        auto belt_e   = it.transport();
        const auto* bc = reg.try_get<ConveyorBeltComponent>(belt_e);
        if (!bc) continue;          // not on a belt (e.g. on a picker)

        if (!belt_is_moving(reg, belt_e)) continue;

        Vec3  entry_world = world_position(bc->entry_port(), reg);
        float t_start     = glm::dot(ipose.position - entry_world, bc->dir());
        pending.push_back({item_e, belt_e, bc, entry_world, t_start});
    }

    std::sort(pending.begin(), pending.end(),
              [](const PendingItem& a, const PendingItem& b) {
                  if (a.belt != b.belt)
                      return static_cast<std::uint32_t>(a.belt)
                           < static_cast<std::uint32_t>(b.belt);
                  return a.t_start > b.t_start;
              });

    struct Handover {
        entt::entity item;
        entt::entity next_belt;
        Vec3         land_pos;
    };
    std::vector<Handover>            handovers;
    std::unordered_set<entt::entity> frozen_belts;

    for (const auto& p : pending) {
        if (frozen_belts.count(p.belt)) continue;

        auto& ipose = reg.get<PoseComponent>(p.item);
        ipose.position += p.bc->dir() * p.bc->belt_speed_mm_s() * dt;

        const float t = glm::dot(ipose.position - p.entry_world, p.bc->dir());
        if (t < static_cast<float>(p.bc->length_mm())) continue;

        const auto* exit_pc = reg.try_get<PortComponent>(p.bc->exit_port());
        if (!exit_pc) {
            frozen_belts.insert(p.belt);
            continue;
        }

        if (exit_pc->transport() != entt::null) {
            // Belt-to-belt handover (gated by destination port = exit_port).
            if (!port_is_clear(reg, p.bc->exit_port())) {
                frozen_belts.insert(p.belt);
                continue;
            }
            auto        next_e  = exit_pc->transport();
            const auto* next_bc = reg.try_get<ConveyorBeltComponent>(next_e);
            if (!next_bc) {
                frozen_belts.insert(p.belt);
                continue;
            }
            Vec3 next_entry_world = world_position(next_bc->entry_port(), reg);
            handovers.push_back({p.item, next_e, next_entry_world});
        } else {
            // Terminal exit — clamp to exit position.
            ipose.position = world_position(p.bc->exit_port(), reg);
        }
        frozen_belts.insert(p.belt);
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

    // ── Magic transport motion (whimsical placeholder) ──────────────────────

    for (auto&& [magic_e, mt, mpose] :
         reg.view<MagicTransportComponent, PoseComponent>().each())
    {
        if (mt.state() == PickerState::Idle) continue;

        Vec3 target;
        switch (mt.state()) {
            case PickerState::MovingToBox: target = mt.pickup_target(); break;
            case PickerState::Carrying:    target = mt.drop_target();   break;
            case PickerState::Returning:   target = mt.home_pose();     break;
            default: continue;
        }

        mt.set_elapsed_s(mt.elapsed_s() + dt);
        const float duration = mt.leg_duration_s() > 0.f ? mt.leg_duration_s() : 1.f;
        const float phase    = mt.elapsed_s() / duration;
        const float seed     = static_cast<float>(static_cast<uint32_t>(magic_e) & 0xffu) * 0.1f;

        if (phase < 1.f) {
            mpose.position    = magic::position(mt.leg_origin(), target, phase);
            mpose.orientation = magic::rotation(phase, seed);
            continue;
        }

        // Reached the end of this leg — clamp to target, settle orientation,
        // run the same transport / parent reassignment side-effects the
        // picker uses.
        mpose.position    = target;
        mpose.orientation = Quat{1.f, 0.f, 0.f, 0.f};

        switch (mt.state()) {
            case PickerState::MovingToBox: {
                auto box = mt.current_box();
                if (box != entt::null) {
                    reg.get_or_emplace<ItemOnTransportComponent>(box)
                       .set_transport(magic_e);
                    auto& bpose = reg.get<PoseComponent>(box);
                    bpose.position = Vec3{0.f};
                    bpose.parent   = magic_e;
                }
                mt.set_state(PickerState::Carrying);
                mt.set_leg_origin(target);
                mt.set_elapsed_s(0.f);
                break;
            }
            case PickerState::Carrying: {
                auto box       = mt.current_box();
                auto container = mt.drop_container();
                if (box != entt::null) {
                    if (reg.any_of<ItemOnTransportComponent>(box))
                        reg.remove<ItemOnTransportComponent>(box);

                    auto& bpose = reg.get<PoseComponent>(box);
                    if (container != entt::null) {
                        glm::mat4 cw    = world_transform(container, reg);
                        glm::mat4 ci    = glm::inverse(cw);
                        Vec3      local = Vec3(ci * glm::vec4(mt.drop_target(), 1.f));
                        bpose.position    = local;
                        bpose.orientation = mt.drop_orientation();
                        bpose.parent      = container;
                        if (auto* palletc = reg.try_get<PalletComponent>(container))
                            palletc->add_item(box);
                    } else {
                        bpose.parent = scene.root_entity();
                    }
                }
                mt.set_state(PickerState::Returning);
                mt.set_leg_origin(target);
                mt.set_elapsed_s(0.f);
                break;
            }
            case PickerState::Returning: {
                mt.set_state(PickerState::Idle);
                mt.set_current_box(entt::null);
                mt.set_leg_origin(mt.home_pose());
                mt.set_elapsed_s(0.f);
                break;
            }
            default: break;
        }
    }
}

}  // namespace factory::transport
