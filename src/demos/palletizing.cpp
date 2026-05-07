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
#include "factory_scene/render_system.hpp"
#include "factory_scene/sim_systems.hpp"
#include "factory_scene/station_systems.hpp"

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
    ss.controls->target       = {0.f, 0.3f, 0.f};
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
        auto geo   = PlaneGeometry::create(60.f, 60.f);
        auto mat   = MeshStandardMaterial::create();
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

    // ── Belt meshes ───────────────────────────────────────────────────────────
    // Separate textures — sharing one DataTexture would let the second buildBeltMesh
    // call overwrite the repeat settings set by the first.
    auto palletTex = belt::makeBeltTexture();
    auto boxTex    = belt::makeBeltTexture();

    // Single visual mesh for combined pallet belt (south+north = 6000mm total).
    // Centre at ECS (0, 0) → threepp (0, 0, 0). Travels N (+y) → rot.y = -π/2.
    auto [palletObj, palletMat] = belt::buildBeltMesh(800, 6000, 0,
                                                       belt::kGenericCatalog, palletTex);
    {
        auto grp = Group::create();
        grp->add(palletObj);
        grp->position.set(0.f, 0.f, 0.f);
        grp->rotation.y = -math::PI / 2.0f;
        ss.scene->add(grp);
    }

    // Box belt: 2800mm, travels +x.
    // Centre ECS X = (-2400 + 400) / 2 = -1000 → threepp X = -1.0.
    auto [boxObj, boxMat] = belt::buildBeltMesh(300, 2800, 800,
                                                 belt::kGenericCatalog, boxTex);
    {
        auto grp = Group::create();
        grp->add(boxObj);
        grp->position.set(-1.0f, 0.f, 0.f);
        grp->rotation.y = 0.f;
        ss.scene->add(grp);
    }

    // ── Simulation ────────────────────────────────────────────────────────────
    auto& reg = scene.registry();
    std::unordered_map<entt::entity, std::shared_ptr<threepp::Object3D>> item_meshes;

    // South pallet segment: travels +y, 3000mm, surface z=0
    auto south_segment_e = scene.add_belt(800, 3000, 0, 200.f, {0.f, 1.f, 0.f});
    auto pal_south_entry_e = scene.add_port("pal_south_entry", {0.f, -3000.f, 0.f});
    auto pal_station_port_e = scene.add_port("pal_station_port", {0.f, 0.f, 0.f});
    scene.connect_belt(south_segment_e, pal_south_entry_e, pal_station_port_e);
    scene.set_port_transport(pal_south_entry_e, south_segment_e);
    // pal_station_port transport stays null — station intercepts

    // North pallet segment: travels +y, 3000mm, surface z=0
    auto north_segment_e = scene.add_belt(800, 3000, 0, 200.f, {0.f, 1.f, 0.f});
    auto pal_north_entry_e = scene.add_port("pal_north_entry", {0.f, 0.f, 0.f});
    auto pal_north_exit_e  = scene.add_port("pal_north_exit",  {0.f, 3000.f, 0.f});
    scene.connect_belt(north_segment_e, pal_north_entry_e, pal_north_exit_e);
    scene.set_port_transport(pal_north_entry_e, north_segment_e);
    // pal_north_exit transport stays null — sink handles it

    // Box belt: travels +x, 2800mm, surface z=800
    auto box_belt_e = scene.add_belt(300, 2800, 800, 200.f, {1.f, 0.f, 0.f});
    auto box_entry_e = scene.add_port("box_entry",       {-2400.f, 0.f, 800.f});
    auto box_station_port_e = scene.add_port("box_station_port", {400.f, 0.f, 800.f});
    scene.connect_belt(box_belt_e, box_entry_e, box_station_port_e);
    scene.set_port_transport(box_entry_e, box_belt_e);
    // box_station_port transport stays null — station intercepts

    // Prototypes
    auto pallet_proto_e = scene.add_prototype(1200, 800, 145, 0xC8A060u);
    auto box_proto_e    = scene.add_prototype(250, 250, 200, 0x8B4513u);

    // Sources — rates tuned for a watchable demo (pallet fills in ~30s)
    scene.add_source( 360.f, pallet_proto_e, pal_south_entry_e);
    scene.add_source(1800.f, box_proto_e,    box_entry_e);

    // Sink at north exit
    scene.add_sink(pal_north_exit_e);

    // Station entity
    auto station_e = reg.create();
    auto& sc = reg.emplace<factory::StationComponent>(station_e);
    sc.set_arrival_port(box_station_port_e);
    sc.set_controlled_transport(box_belt_e);
    sc.set_mechanism(std::make_shared<factory::ProcessMechanism>(1.5f));

    auto& pc = reg.emplace<factory::PalletizeComponent>(station_e);
    pc.set_pallet_arrival_port(pal_station_port_e);
    pc.set_pallet_output_transport(north_segment_e);
    pc.set_pallet_segment(south_segment_e);
    pc.set_pattern(std::make_shared<factory::GridPattern>());
    pc.set_pallet_dimensions(1200, 800, 145, 1500);

    // ── Animate ───────────────────────────────────────────────────────────────
    Clock clock;

    ss.canvas.animate([&] {
        float delta = clock.getDelta();

        // ── Simulation ────────────────────────────────────────────────────────
        auto events = factory::sim::step(scene, delta);
        factory::station::step(scene, events.arrived, delta);

        // ── Render ────────────────────────────────────────────────────────────

        // UV scrolling: Three.js UV = uv*repeat + offset, so d(offset)/dt = -speed/tile_pitch.
        // Pallet: stop when south is stopped (pallet held at station).
        // North segment is never explicitly stopped so OR-ing would always scroll.
        const float tile_pitch_m = belt::kGenericCatalog.tile_pitch_mm * 0.001f;
        {
            auto* tc_s = reg.try_get<factory::TransportComponent>(south_segment_e);
            if (palletMat->map && tc_s && tc_s->running()) {
                auto* bc_s = reg.try_get<factory::ConveyorBeltComponent>(south_segment_e);
                if (bc_s)
                    palletMat->map->offset.x -= bc_s->belt_speed_mm_s() * 0.001f * delta / tile_pitch_m;
            }
        }
        {
            auto* tc = reg.try_get<factory::TransportComponent>(box_belt_e);
            if (boxMat->map && tc && tc->running()) {
                auto* bc = reg.try_get<factory::ConveyorBeltComponent>(box_belt_e);
                if (bc)
                    boxMat->map->offset.x -= bc->belt_speed_mm_s() * 0.001f * delta / tile_pitch_m;
            }
        }

        // Create meshes for newly spawned items
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

        // Sync mesh positions using world_transform for parent-chain support
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

        // Remove meshes for despawned items; cascade children for full pallets
        for (auto e : events.despawned) {
            auto* palletc = reg.try_get<factory::PalletComponent>(e);
            if (palletc) {
                for (auto child : palletc->items()) {
                    auto cmit = item_meshes.find(child);
                    if (cmit != item_meshes.end()) {
                        ss.scene->remove(*cmit->second);
                        item_meshes.erase(cmit);
                    }
                    reg.destroy(child);
                }
            }
            auto mit = item_meshes.find(e);
            if (mit != item_meshes.end()) {
                ss.scene->remove(*mit->second);
                item_meshes.erase(mit);
            }
            reg.destroy(e);
        }

        ss.renderer.render(*ss.scene, *ss.camera);
    });

    return 0;
}
