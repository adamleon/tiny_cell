#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <unordered_map>
#include <vector>

#include <threepp/cameras/OrthographicCamera.hpp>
#include <threepp/core/Clock.hpp>
#include <threepp/geometries/BoxGeometry.hpp>
#include <threepp/geometries/PlaneGeometry.hpp>
#include <threepp/geometries/SphereGeometry.hpp>
#include <threepp/lights/AmbientLight.hpp>
#include <threepp/lights/DirectionalLight.hpp>
#include <threepp/materials/MeshBasicMaterial.hpp>
#include <threepp/materials/MeshStandardMaterial.hpp>
#include <threepp/objects/Mesh.hpp>

#include <glm/gtc/quaternion.hpp>

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

    // GLRenderer is fast enough to afford soft shadow maps; the WGPU path
    // had to skip them.
    ss.renderer.shadowMap().enabled = true;
    ss.renderer.shadowMap().type    = ShadowMap::PFCSoft;

    {
        auto sun = DirectionalLight::create(Color(0xffc87a), 2.0f);
        sun->position.set(-4.f, 8.f, 5.f);
        sun->castShadow = true;
        sun->shadow->mapSize.set(2048, 2048);
        sun->shadow->bias = -0.0005f;
        auto* shadowCam   = sun->shadow->camera->as<OrthographicCamera>();
        shadowCam->left   = -8.f;
        shadowCam->right  =  8.f;
        shadowCam->top    =  8.f;
        shadowCam->bottom = -8.f;
        shadowCam->nearPlane = 0.5f;
        shadowCam->farPlane  = 25.f;
        ss.scene->add(sun);
    }
    ss.scene->add(AmbientLight::create(Color(0xfff0e0), 0.7f));

    {
        auto geo  = PlaneGeometry::create(60.f, 60.f);
        auto mat  = MeshStandardMaterial::create();
        mat->color     = Color(0xe8e4d8);
        mat->roughness = 0.5f;
        mat->metalness = 0.f;
        auto floor = Mesh::create(geo, mat);
        floor->rotation.x = -math::PI / 2.f;
        floor->position.y = -0.001f;
        floor->receiveShadow = true;
        ss.scene->add(floor);
    }

    // ── Fence ─────────────────────────────────────────────────────────────────
    OBJLoader loader;
    auto protos   = cell::loadCatalogProtos(loader, assetDir, catalog);
    auto fenceGrp = render::buildScene(scene, protos);
    fenceGrp->traverseType<Mesh>([](Mesh& m) {
        m.castShadow    = true;
        m.receiveShadow = true;
    });
    ss.scene->add(fenceGrp);

    // ── Belt meshes (visual only) ─────────────────────────────────────────────
    auto palletTex = belt::makeBeltTexture();
    auto boxTex    = belt::makeBeltTexture();

    // Single 6000 mm pallet belt visual, centred at origin, travelling +y.
    auto [palletObj, palletMat] = belt::buildBeltMesh(800, 6000, 0,
                                                       belt::kGenericCatalog, palletTex);
    palletObj->traverseType<Mesh>([](Mesh& m) {
        m.castShadow    = true;
        m.receiveShadow = true;
    });
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
    boxObj->traverseType<Mesh>([](Mesh& m) {
        m.castShadow    = true;
        m.receiveShadow = true;
    });
    {
        auto grp = Group::create();
        grp->add(boxObj);
        grp->position.set(-1.0f, 0.f, 0.f);
        ss.scene->add(grp);
    }

    // ── Workflow declaration ──────────────────────────────────────────────────
    // Define what the cell *does*; let solve_workflow() figure out the layout.
    auto& reg = scene.registry();

    auto pallet_proto = scene.add_prototype(1200, 800, 145, 0xC8A060u);
    auto box_proto    = scene.add_prototype(250,  250, 200, 0x8B4513u);

    auto pallet_source = scene.declare_source( 360.f, pallet_proto);
    auto box_source    = scene.declare_source(1800.f, box_proto);
    auto pallet_sink   = scene.declare_sink();
    auto palletizer    = scene.declare_palletizer_station(pallet_proto, box_proto);

    scene.declare_flow(pallet_source, palletizer);    // pallets in
    scene.declare_flow(box_source,    palletizer);    // boxes in
    scene.declare_flow(palletizer,    pallet_sink);   // full pallets out

    scene.solve_workflow();

    // The render loop scrolls the belt textures based on whether each belt is
    // actually moving. solve_workflow placed the belts; we pick them out
    // here by surface height (pallet belt at z=0, box belt at z=800).
    entt::entity pallet_belt = entt::null;
    entt::entity box_belt    = entt::null;
    reg.view<factory::ConveyorBeltComponent>().each(
        [&](auto e, const factory::ConveyorBeltComponent& bc) {
            if (bc.surface_height_mm() == 0)   pallet_belt = e;
            if (bc.surface_height_mm() == 800) box_belt    = e;
        });

    // ── Animate ───────────────────────────────────────────────────────────────
    Clock clock;
    std::unordered_map<entt::entity, std::shared_ptr<threepp::Object3D>> item_meshes;

    // Magic-particle effects: tiny glowing spheres that fade out, spawned at
    // the magic transport's world position whenever it's actively carrying.
    struct Particle {
        std::shared_ptr<threepp::Mesh>                  mesh;
        std::shared_ptr<threepp::MeshBasicMaterial>     mat;
        threepp::Vector3                                velocity;
        float                                           lifetime_s;
        float                                           total_life_s;
    };
    std::vector<Particle> particles;

    auto spawn_magic_particles = [&](factory::Vec3 ecs_world, int count) {
        static const uint32_t kPalette[] = { 0xff66cc, 0x66ccff, 0xffcc66, 0xcc99ff, 0xffff99 };
        for (int i = 0; i < count; ++i) {
            auto geo = threepp::SphereGeometry::create(0.04f, 6, 4);
            auto mat = threepp::MeshBasicMaterial::create();
            mat->color       = threepp::Color(kPalette[std::rand() % 5]);
            mat->transparent = true;
            mat->opacity     = 1.0f;

            auto mesh = threepp::Mesh::create(geo, mat);
            // ECS(x, y, z) → threepp(x, z, y) in metres.
            const float wx = ecs_world.x * 0.001f;
            const float wy = ecs_world.z * 0.001f;
            const float wz = ecs_world.y * 0.001f;
            const float jx = (std::rand() / float(RAND_MAX) - 0.5f) * 0.18f;
            const float jy = (std::rand() / float(RAND_MAX) - 0.5f) * 0.18f;
            const float jz = (std::rand() / float(RAND_MAX) - 0.5f) * 0.18f;
            mesh->position.set(wx + jx, wy + jy, wz + jz);
            ss.scene->add(mesh);

            const float vx = (std::rand() / float(RAND_MAX) - 0.5f) * 0.6f;
            const float vy =  0.3f + (std::rand() / float(RAND_MAX)) * 0.8f;
            const float vz = (std::rand() / float(RAND_MAX) - 0.5f) * 0.6f;
            particles.push_back({mesh, mat, threepp::Vector3(vx, vy, vz), 0.9f, 0.9f});
        }
    };

    ss.canvas.animate([&] {
        // Clamp dt: a frame drop / window drag / debugger pause produces a
        // huge raw delta. Without the cap, items would advance hundreds of
        // mm in one tick and could tunnel straight through sensor volumes,
        // overshoot exits, or fall behind their parent containers. 50 ms is
        // 3× one tick at 60 Hz — the sim slows down rather than glitching.
        float delta = std::min(clock.getDelta(), 0.05f);

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
            auto mesh           = threepp::Mesh::create(geo, mat);
            mesh->castShadow    = true;
            mesh->receiveShadow = true;
            ss.scene->add(mesh);
            item_meshes[item] = mesh;
        }

        // ── Sync mesh positions and rotations from world transforms ───────
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
            // ECS(x, y, z) → threepp(x, z, y) in metres, with half-height offset on Y.
            mit->second->position.set(
                wpos.x * 0.001f,
                (wpos.z + hh_mm) * 0.001f,
                wpos.y * 0.001f);
            // Same axis swap for the rotation: ECS-z → threepp-y, ECS-y → threepp-z.
            mit->second->quaternion.set(wquat.x, wquat.z, wquat.y, wquat.w);
        }

        // ── Magic particles ───────────────────────────────────────────────
        // Spawn a sparkly trail at every active magic transport.
        for (auto&& [mt_e, mt, mpose] :
             reg.view<factory::MagicTransportComponent,
                      factory::PoseComponent>().each())
        {
            (void)mpose;
            if (mt.state() == factory::PickerState::Idle) continue;
            auto wmat = factory::world_transform(mt_e, reg);
            spawn_magic_particles(glm::vec3(wmat[3]), 3);
        }
        // Update existing particles: drift + fade.
        for (auto& p : particles) {
            p.mesh->position.x += p.velocity.x * delta;
            p.mesh->position.y += p.velocity.y * delta;
            p.mesh->position.z += p.velocity.z * delta;
            p.lifetime_s -= delta;
            const float life_frac = std::max(0.f, p.lifetime_s / p.total_life_s);
            p.mesh->scale.setScalar(life_frac);
            p.mat->opacity = life_frac;
        }
        // Reap expired particles.
        particles.erase(
            std::remove_if(particles.begin(), particles.end(),
                [&](const Particle& p) {
                    if (p.lifetime_s <= 0.f) {
                        ss.scene->remove(*p.mesh);
                        return true;
                    }
                    return false;
                }),
            particles.end());

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
