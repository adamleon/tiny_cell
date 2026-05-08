#include <cmath>
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

#include "common/scene_setup.hpp"
#include "cell/fence_catalog.hpp"
#include "cell/fence_solver.hpp"
#include "factory_scene/belt_mesh.hpp"
#include "factory_scene/factory_scene.hpp"
#include "factory_scene/pose_component.hpp"
#include "factory_scene/placement_pattern.hpp"
#include "factory_scene/render_system.hpp"
#include "factory_scene/sensor_systems.hpp"
#include "factory_scene/transport_systems.hpp"
#include "factory_scene/station_systems.hpp"
#include "factory_scene/lifecycle_systems.hpp"

using namespace threepp;

int main() {
    const std::string assetDir = "assets/components/fences/axelent_x-guard";

    // ── Fence solve ───────────────────────────────────────────────────────────
    auto catalog = cell::loadCatalog(assetDir + "/catalog.json");
    auto table   = loadTable(assetDir + "/combinations.json");

    factory::FactoryScene scene;
    scene.place_node(-2000.f, -2000.f);
    scene.place_node( 2000.f, -2000.f);
    scene.place_node( 2000.f,  2000.f);
    scene.place_node(-2000.f,  2000.f);

    scene.declare_opening_anchored(0, 2000, 900);
    scene.declare_opening_anchored(2, 2000, 900);
    scene.declare_opening_anchored(3, 2000, 400);

    scene.solve(table, assetDir);

    // ── threepp setup ─────────────────────────────────────────────────────────
    SceneSetup ss("Palletizing");
    ss.scene->background = Color(0xf0ede6);
    ss.camera->position.set(5.0f, 6.0f, 8.0f);
    ss.camera->lookAt({0.f, 0.3f, 0.f});
    ss.controls->target        = {0.f, 0.3f, 0.f};
    ss.controls->maxPolarAngle = math::PI / 2.0f - 0.04f;
    ss.controls->update();

    ss.renderer.shadowMap().enabled = false;

    {
        auto sun = DirectionalLight::create(Color(0xffc87a), 2.0f);
        sun->position.set(-4.f, 8.f, 5.f);
        ss.scene->add(sun);
    }
    ss.scene->add(AmbientLight::create(Color(0xfff0e0), 1.1f));

    {
        auto geo  = PlaneGeometry::create(60.f, 60.f);
        auto mat  = MeshStandardMaterial::create();
        mat->color     = Color(0xe8e4d8);
        mat->roughness = 0.5f;
        mat->metalness = 0.f;
        auto floor = Mesh::create(geo, mat);
        floor->rotation.x = -math::PI / 2.f;
        floor->position.y = -0.001f;
        ss.scene->add(floor);
    }

    // ── Fence ─────────────────────────────────────────────────────────────────
    OBJLoader loader;
    auto protos   = cell::loadCatalogProtos(loader, assetDir, catalog);
    auto fenceGrp = render::buildScene(scene, protos);
    ss.scene->add(fenceGrp);

    // ── Belt meshes (visual only) ─────────────────────────────────────────────
    auto palletTex = belt::makeBeltTexture();
    auto boxTex    = belt::makeBeltTexture();

    // Single 6000 mm pallet belt visual, centred at origin, travelling +y.
    auto [palletObj, palletMat] = belt::buildBeltMesh(800, 6000, 0,
                                                       belt::kGenericCatalog, palletTex);
    {
        auto grp = Group::create();
        grp->add(palletObj);
        grp->position.set(0.f, 0.f, 0.f);
        grp->rotation.y = -math::PI / 2.0f;
        ss.scene->add(grp);
    }

    // Box belt visual, centred at ECS X = -1000 (= threepp X = -1.0), travelling +x.
    auto [boxObj, boxMat] = belt::buildBeltMesh(300, 2800, 800,
                                                 belt::kGenericCatalog, boxTex);
    {
        auto grp = Group::create();
        grp->add(boxObj);
        grp->position.set(-1.0f, 0.f, 0.f);
        ss.scene->add(grp);
    }

    // ── Simulation wiring (single pallet belt with mid-stream tap) ────────────
    auto& reg = scene.registry();

    // Pallet belt: 6000 mm, +y. Entry at south end, exit at north end, tap at
    // mid-belt (= world (0, 0, 0)).
    auto pallet_belt = scene.add_belt(800, 6000, 0, 200.f, {0.f, 1.f, 0.f});
    auto pal_entry   = scene.add_port("pal_entry", {0.f, -3000.f, 0.f}, {0.f, 1.f, 0.f});
    auto pal_exit    = scene.add_port("pal_exit",  {0.f,  3000.f, 0.f}, {0.f, 1.f, 0.f});
    scene.connect_belt(pallet_belt, pal_entry, pal_exit);
    scene.set_port_transport(pal_entry, pallet_belt);

    // Two ports at the tap location: one gates the belt (virtual sensor only),
    // the other detects the pallet (physical sensor only). Splitting them lets
    // the station release the pallet without the physical sensor locking up
    // the belt — once virtual clears, the belt resumes and the pallet moves
    // out of the detection volume on its own.
    auto pal_tap_gate   = scene.add_tap_port(pallet_belt, "pal_tap_gate",   3000);
    scene.make_gate_port(pallet_belt, pal_tap_gate);
    auto pal_tap_virt   = scene.add_virtual_sensor(pal_tap_gate);

    // Stop-here sensor: small + centred on the tap so the pallet's centre
    // comes to rest near the tap location.
    auto pal_tap_detect = scene.add_tap_port(pallet_belt, "pal_tap_detect", 3000);
    scene.add_physical_sensor(pal_tap_detect, 200, 900, 200);

    // Spawn-throttle sensor at the entry: with centre-point detection the
    // sensor must encompass roughly 2 × item_length so the previous pallet's
    // centre clears before the next one is spawned. 2600 mm = 2 × 1200 (pallet
    // length) + 200 mm gap.
    scene.add_physical_sensor(pal_entry, 2600, 900, 200);

    // Sink detection — a small footprint at the exit port suffices.
    scene.add_physical_sensor(pal_exit,  200, 900, 200);

    // Box belt: 2800 mm, +x at height 800.
    auto box_belt  = scene.add_belt(300, 2800, 800, 200.f, {1.f, 0.f, 0.f});
    auto box_entry = scene.add_port("box_entry", {-2400.f, 0.f, 800.f}, {1.f, 0.f, 0.f});
    auto box_exit  = scene.add_port("box_exit",  {  400.f, 0.f, 800.f}, {1.f, 0.f, 0.f});
    scene.connect_belt(box_belt, box_entry, box_exit);
    scene.set_port_transport(box_entry, box_belt);

    // Box spawn throttle: 2 × box_length (250) + 200 mm gap = 700 mm.
    scene.add_physical_sensor(box_entry, 700, 300, 250);
    // Box exit (stop-here / station detection): tight around the port.
    scene.add_physical_sensor(box_exit,  300, 300, 250);
    auto box_exit_virt = scene.add_virtual_sensor(box_exit);   // station-driven

    // Prototypes.
    auto pallet_proto = scene.add_prototype(1200, 800, 145, 0xC8A060u);
    auto box_proto    = scene.add_prototype(250,  250, 200, 0x8B4513u);

    // Sources — rates tuned so the pallet fills in roughly 30 s.
    scene.add_source( 360.f, pallet_proto, pal_entry);
    scene.add_source(1800.f, box_proto,    box_entry);

    // Sink at the north end of the pallet belt.
    scene.add_sink(pal_exit);

    // Picker (no mesh — boxes ride it visibly via parent chain).
    auto picker = scene.add_picker(factory::Vec3{-1000.f, 0.f, 1500.f}, 1200.f);

    // Station entity.
    auto station_e = reg.create();
    auto& sc = reg.emplace<factory::StationComponent>(station_e);
    sc.set_arrival_port(box_exit);
    sc.set_arrival_virtual_sensor(box_exit_virt);
    sc.add_picker(picker);

    auto& palc = reg.emplace<factory::PalletizeComponent>(station_e);
    palc.set_pallet_arrival_port(pal_tap_detect);    // station polls the detection port
    palc.set_pallet_tap_virtual_sensor(pal_tap_virt); // station drives the gate port
    palc.set_pattern(std::make_shared<factory::GridPattern>());
    palc.set_pallet_dimensions(1200, 800, 145, 1500);

    // ── Animate ───────────────────────────────────────────────────────────────
    Clock clock;
    std::unordered_map<entt::entity, std::shared_ptr<threepp::Object3D>> item_meshes;

    ss.canvas.animate([&] {
        float delta = clock.getDelta();

        // ── Simulation tick ───────────────────────────────────────────────
        factory::sensor::scan(scene);
        factory::transport::step(scene, delta);
        factory::station::step(scene, delta);
        auto events = factory::lifecycle::step(scene, delta);

        // ── Belt UV scrolls iff the belt is actually moving ───────────────
        const float tile_pitch_m = belt::kGenericCatalog.tile_pitch_mm * 0.001f;
        if (palletMat->map && factory::transport::belt_is_moving(reg, pallet_belt)) {
            const auto& bc = reg.get<factory::ConveyorBeltComponent>(pallet_belt);
            palletMat->map->offset.x -= bc.belt_speed_mm_s() * 0.001f * delta / tile_pitch_m;
        }
        if (boxMat->map && factory::transport::belt_is_moving(reg, box_belt)) {
            const auto& bc = reg.get<factory::ConveyorBeltComponent>(box_belt);
            boxMat->map->offset.x -= bc.belt_speed_mm_s() * 0.001f * delta / tile_pitch_m;
        }

        // ── Create meshes for newly spawned items ─────────────────────────
        for (auto& [item, proto_e] : events.spawned) {
            auto* proto = reg.try_get<factory::ItemPrototypeComponent>(proto_e);
            if (!proto) continue;
            auto geo = threepp::BoxGeometry::create(
                proto->width_mm()  * 0.001f,
                proto->height_mm() * 0.001f,
                proto->length_mm() * 0.001f);
            auto mat       = threepp::MeshStandardMaterial::create();
            mat->color     = threepp::Color(proto->color_hex());
            mat->roughness = 0.6f;
            auto mesh = threepp::Mesh::create(geo, mat);
            ss.scene->add(mesh);
            item_meshes[item] = mesh;
        }

        // ── Sync mesh positions from world transforms ─────────────────────
        for (auto&& [ent, pose, sp] :
             reg.view<factory::PoseComponent,
                      factory::SpawnedItemComponent>().each())
        {
            auto mit = item_meshes.find(ent);
            if (mit == item_meshes.end()) continue;
            auto* proto = reg.try_get<factory::ItemPrototypeComponent>(sp.prototype());
            float hh_mm = proto ? proto->height_mm() * 0.5f : 0.f;
            auto  wmat  = factory::world_transform(ent, reg);
            glm::vec3 wpos = glm::vec3(wmat[3]);
            // ECS(x, y, z) → threepp(x, z, y) in metres, with half-height offset on Y
            mit->second->position.set(
                wpos.x * 0.001f,
                (wpos.z + hh_mm) * 0.001f,
                wpos.y * 0.001f);
        }

        // ── Remove despawned meshes (cascade for pallet children) ─────────
        for (auto e : events.despawned) {
            auto* palletc = reg.try_get<factory::PalletComponent>(e);
            if (palletc) {
                for (auto child : palletc->items()) {
                    auto cmit = item_meshes.find(child);
                    if (cmit != item_meshes.end()) {
                        ss.scene->remove(*cmit->second);
                        item_meshes.erase(cmit);
                    }
                    if (reg.valid(child)) reg.destroy(child);
                }
            }
            auto mit = item_meshes.find(e);
            if (mit != item_meshes.end()) {
                ss.scene->remove(*mit->second);
                item_meshes.erase(mit);
            }
            if (reg.valid(e)) reg.destroy(e);
        }

        ss.renderer.render(*ss.scene, *ss.camera);
    });

    return 0;
}
