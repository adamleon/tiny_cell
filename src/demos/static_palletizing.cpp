// Static palletizing demo: no belts, no fences. Two depots (one for pallets,
// one for boxes), each spawning one item at a time, and a real picker
// (PickerTransportComponent, straight-line cartesian motion at TCP speed)
// shuttling boxes from box-depot to current pallet. A KUKA KR10 R1100 mesh
// tracks the picker's TCP via inverse kinematics. When the pallet is full,
// the demo despawns it, then holds the pallet source blocked for ~1.2 s for
// a visible gap before the next pallet appears.
//
// Built as a test bed for the depot archetype + the robot/IK rendering
// pipeline. The belt-based palletizing.cpp demo is the realistic-cell
// version; this one is a stripped-down station-and-picker scene.

#include <algorithm>
#include <cmath>
#include <memory>
#include <unordered_map>
#include <vector>

#include <threepp/cameras/OrthographicCamera.hpp>
#include <threepp/core/Clock.hpp>
#include <threepp/geometries/BoxGeometry.hpp>
#include <threepp/geometries/PlaneGeometry.hpp>
#include <threepp/lights/AmbientLight.hpp>
#include <threepp/lights/DirectionalLight.hpp>
#include <threepp/materials/MeshStandardMaterial.hpp>
#include <threepp/objects/Mesh.hpp>

#include <glm/gtc/quaternion.hpp>

#include "common/scene_setup.hpp"
#include "factory_scene/depot_components.hpp"
#include "factory_scene/factory_scene.hpp"
#include "factory_scene/lifecycle_systems.hpp"
#include "factory_scene/placement_pattern.hpp"
#include "factory_scene/pose_component.hpp"
#include "factory_scene/render_system.hpp"
#include "factory_scene/sensor_systems.hpp"
#include "factory_scene/station_components.hpp"
#include "factory_scene/station_systems.hpp"
#include "factory_scene/transport_systems.hpp"

using namespace threepp;

int main() {
    // ── threepp setup ─────────────────────────────────────────────────────────
    SceneSetup ss("Static palletizing");
    ss.scene->background = Color(0xf0ede6);
    ss.camera->position.set(3.0f, 3.0f, 4.5f);
    ss.camera->lookAt({-1.0f, 0.3f, 0.f});
    ss.controls->target        = {-1.0f, 0.3f, 0.f};
    ss.controls->maxPolarAngle = math::PI / 2.0f - 0.04f;
    ss.controls->update();

    ss.renderer.shadowMap().enabled = true;
    ss.renderer.shadowMap().type    = ShadowMap::PFCSoft;

    {
        auto sun = DirectionalLight::create(Color(0xffc87a), 2.0f);
        sun->position.set(-3.f, 6.f, 4.f);
        sun->castShadow = true;
        sun->shadow->mapSize.set(2048, 2048);
        sun->shadow->bias = -0.0005f;
        auto* shadowCam   = sun->shadow->camera->as<OrthographicCamera>();
        shadowCam->left   = -4.f;
        shadowCam->right  =  4.f;
        shadowCam->top    =  4.f;
        shadowCam->bottom = -4.f;
        shadowCam->nearPlane = 0.5f;
        shadowCam->farPlane  = 20.f;
        ss.scene->add(sun);
    }
    ss.scene->add(AmbientLight::create(Color(0xfff0e0), 0.7f));

    {
        auto geo  = PlaneGeometry::create(20.f, 20.f);
        auto mat  = MeshStandardMaterial::create();
        mat->color     = Color(0xe8e4d8);
        mat->roughness = 0.5f;
        auto floor = Mesh::create(geo, mat);
        floor->rotation.x = -math::PI / 2.f;
        floor->position.y = -0.001f;
        floor->receiveShadow = true;
        ss.scene->add(floor);
    }

    // ── Scene wiring ──────────────────────────────────────────────────────────
    factory::FactoryScene scene;
    auto& reg = scene.registry();

    auto pallet_proto = scene.add_prototype(1200, 800, 145, 0xC8A060u);
    auto box_proto    = scene.add_prototype(250,  250, 200, 0x8B4513u);

    // Shared GridPattern is used both as the pallet's stacking pattern and
    // as each depot's slot pattern. With depot footprint = item dims, the
    // pattern yields exactly one slot at the centre.
    auto stack_pattern = std::make_shared<factory::GridPattern>();

    // Pallet depot at ECS origin. Footprint = pallet dims, surface at z=0
    // (pallets sit on the floor).
    auto pallet_depot = scene.add_depot({0.f, 0.f, 0.f},
                                        1200, 800, 0,
                                        std::make_shared<factory::GridPattern>(),
                                        {1.f, 0.f, 0.f});
    auto pallet_port  = scene.add_port("pallet_port", {0.f, 0.f, 0.f},
                                       {1.f, 0.f, 0.f});
    scene.set_port_transport(pallet_port, pallet_depot);
    scene.add_laser_sensor(pallet_port);
    // Demo-controlled virtual sensor: re-blocked briefly after a pallet
    // despawns, to produce a visible pause before the next one appears.
    auto pallet_respawn_gate = scene.add_virtual_sensor(pallet_port);

    // Box depot 2 m to the west, items appear at z ≈ 0 (bottom on floor).
    auto box_depot = scene.add_depot({-2000.f, 0.f, 0.f},
                                     250, 250, 0,
                                     std::make_shared<factory::GridPattern>(),
                                     {1.f, 0.f, 0.f});
    auto box_port  = scene.add_port("box_port", {-2000.f, 0.f, 0.f},
                                    {1.f, 0.f, 0.f});
    scene.set_port_transport(box_port, box_depot);
    scene.add_laser_sensor(box_port);

    // High source rates: the depots are the throughput bottleneck (one item
    // at a time), so the source rate above the cycle rate just ensures debt
    // is ready as soon as the laser clears.
    scene.add_source(3600.f, pallet_proto, pallet_port);  // 1/s
    scene.add_source(3600.f, box_proto,    box_port);     // 1/s

    // Real picker (straight-line cartesian motion at TCP speed). KR10 R1100's
    // published TCP speed is in the 1-2 m/s range; 1000 mm/s gives realistic
    // pick-and-place cycle times without outrunning the IK solver. Home is
    // 1 m above the robot base — within reach for resting between cycles.
    auto picker = scene.add_picker({-1000.f, 0.f, 1000.f}, 1000.f);

    // KUKA KR10 R1100 (1100 mm reach) at the picker's home position. The
    // depots at (0,0,0) and (-2000,0,0) sit 1 m to either side — within reach
    // for the near edges of each. The far edge of the 1200 mm pallet is just
    // out of reach; visually the arm will stretch toward those slots.
    scene.add_robot({-1000.f, 0.f, 0.f},
                    "assets/robots/kuka/agilus/urdf/kr10_r1100_2.urdf",
                    picker);

    // Station: claims pallets from pallet depot, dispatches the picker for
    // each box arriving at the box depot.
    auto station = reg.create();
    auto& sc = reg.emplace<factory::StationComponent>(station);
    sc.set_arrival_port(box_port);
    sc.add_picker(picker);

    auto& pc = reg.emplace<factory::PalletizeComponent>(station);
    pc.set_pattern(stack_pattern);
    pc.set_pallet_arrival_port(pallet_port);
    pc.set_pallet_dimensions(1200, 800, 145, 345);   // single layer, 200 mm boxes
    // No pallet_tap_virtual_sensor — that signal exists to freeze an upstream
    // belt at the claim point; with a depot there's nothing to freeze.

    // ── Render loop ───────────────────────────────────────────────────────────
    Clock clock;
    std::unordered_map<entt::entity, std::shared_ptr<Object3D>> item_meshes;
    render::robot::Registry       robot_registry;
    float pallet_respawn_delay_s = 0.f;

    ss.canvas.animate([&] {
        const float delta = std::min(clock.getDelta(), 0.05f);

        // ── Pallet respawn delay (countdown after a despawn) ──────────────
        if (pallet_respawn_delay_s > 0.f) {
            pallet_respawn_delay_s -= delta;
            if (pallet_respawn_delay_s <= 0.f) {
                reg.get<factory::SensorComponent>(pallet_respawn_gate).set_blocked(false);
            }
        }

        // ── Sim tick ──────────────────────────────────────────────────────
        factory::sensor::scan(scene);
        factory::transport::step(scene, delta);
        factory::station::step(scene, delta);
        auto events = factory::lifecycle::step(scene, delta);

        // ── Meshes for spawned items ──────────────────────────────────────
        for (auto& [item, proto_e] : events.spawned) {
            const auto* proto = reg.try_get<factory::ItemPrototypeComponent>(proto_e);
            if (!proto) continue;
            auto geo = BoxGeometry::create(
                proto->width_mm()  * 0.001f,
                proto->height_mm() * 0.001f,
                proto->length_mm() * 0.001f);
            auto mat = MeshStandardMaterial::create();
            mat->color     = Color(proto->color_hex());
            mat->roughness = 0.6f;
            auto mesh = Mesh::create(geo, mat);
            mesh->castShadow    = true;
            mesh->receiveShadow = true;
            ss.scene->add(mesh);
            item_meshes[item] = mesh;
        }

        // ── Sync mesh positions / orientations ────────────────────────────
        for (auto&& [ent, pose, sp] :
             reg.view<factory::PoseComponent,
                      factory::SpawnedItemComponent>().each())
        {
            auto mit = item_meshes.find(ent);
            if (mit == item_meshes.end()) continue;
            auto* proto = reg.try_get<factory::ItemPrototypeComponent>(sp.prototype());
            float hh_mm = proto ? proto->height_mm() * 0.5f : 0.f;
            auto  wmat  = factory::world_transform(ent, reg);
            glm::vec3 wpos  = glm::vec3(wmat[3]);
            glm::quat wquat = glm::quat_cast(wmat);
            mit->second->position.set(
                wpos.x * 0.001f,
                (wpos.z + hh_mm) * 0.001f,
                wpos.y * 0.001f);
            mit->second->quaternion.set(wquat.x, wquat.z, wquat.y, wquat.w);
        }

        // ── Released pallets: any PalletComponent that no station still
        //    claims (current_pallet != this pallet) is one the station has
        //    released. Despawn it (cascade to its boxes) and start the
        //    respawn-gate timer.
        std::vector<entt::entity> released_pallets;
        reg.view<factory::PalletComponent>().each(
            [&](auto pallet_e, const factory::PalletComponent&) {
                bool claimed = false;
                reg.view<factory::PalletizeComponent>().each(
                    [&](const factory::PalletizeComponent& pcomp) {
                        if (pcomp.current_pallet() == pallet_e) claimed = true;
                    });
                if (!claimed) released_pallets.push_back(pallet_e);
            });

        for (auto e : released_pallets) {
            auto& palletc = reg.get<factory::PalletComponent>(e);
            for (auto child : palletc.items()) {
                auto cmit = item_meshes.find(child);
                if (cmit != item_meshes.end()) {
                    ss.scene->remove(*cmit->second);
                    item_meshes.erase(cmit);
                }
                if (reg.valid(child)) reg.destroy(child);
            }
            auto mit = item_meshes.find(e);
            if (mit != item_meshes.end()) {
                ss.scene->remove(*mit->second);
                item_meshes.erase(mit);
            }
            if (reg.valid(e)) reg.destroy(e);

            // Block the pallet source briefly so the gap is visible.
            reg.get<factory::SensorComponent>(pallet_respawn_gate).set_blocked(true);
            pallet_respawn_delay_s = 1.2f;
        }

        // ── Robot URDF + IK ──────────────────────────────────────────────
        robot_registry.update(scene, *ss.scene);

        ss.renderer.render(*ss.scene, *ss.camera);
    });

    return 0;
}
