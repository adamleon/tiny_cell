#include <cmath>

#include <threepp/cameras/OrthographicCamera.hpp>
#include <threepp/core/Clock.hpp>
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
#include "factory_scene/render_system.hpp"

using namespace threepp;

// ── Cell layout ───────────────────────────────────────────────────────────────
//
//  4000 × 4000 mm rectangular cell.  Nodes placed CCW from SW:
//    edge 0: SW → SE  (south wall, Y = -2000)
//    edge 1: SE → NE  (east  wall, X = +2000)
//    edge 2: NE → NW  (north wall, Y = +2000)
//    edge 3: NW → SW  (west  wall, X = -2000)
//
//  Pallet belt: 800 mm wide, N-S through centre (X = 0).
//    Openings on edges 0 and 2 at position 2000 mm from their first node,
//    width = 800 + 2×50 = 900 mm.
//
//  Box belt:    300 mm wide, E-W, ends adjacent to pallet belt west edge.
//    Opening on edge 3 at position 2000 mm from NW node (Y = 0),
//    width = 300 + 2×50 = 400 mm.
//
//  Belt lengths are hardcoded.  The box belt end_b stops at pallet belt
//  west edge (X = -400).  Belt centres:
//    pallet belt centre: ECS ( 0,    0)  → threepp ( 0,  0,  0),  rot.y = -π/2
//    box    belt centre: ECS (-1400, 0)  → threepp (-1.4, 0, 0),  rot.y = 0
//
// Coordinate mapping  ECS(x, y, z)  → threepp(x * 0.001, z * 0.001, y * 0.001)

int main() {
    const std::string assetDir = "assets/components/fences/axelent_x-guard";

    // ── Fence solve ───────────────────────────────────────────────────────────
    auto catalog = cell::loadCatalog(assetDir + "/catalog.json");
    auto table   = loadTable(assetDir + "/combinations.json");

    factory::FactoryScene scene;
    scene.place_node(-2000.f, -2000.f);  // SW  edge 0 starts here
    scene.place_node( 2000.f, -2000.f);  // SE
    scene.place_node( 2000.f,  2000.f);  // NE  edge 2 starts here
    scene.place_node(-2000.f,  2000.f);  // NW  edge 3 starts here

    // Belt pass-through openings
    scene.declare_opening_anchored(0, 2000, 900);  // pallet belt — south fence
    scene.declare_opening_anchored(2, 2000, 900);  // pallet belt — north fence
    scene.declare_opening_anchored(3, 2000, 400);  // box belt    — west  fence

    scene.solve(table, assetDir);

    // ── threepp setup ─────────────────────────────────────────────────────────
    SceneSetup ss("Palletizing");
    ss.scene->background = Color(0xf0ede6);
    ss.camera->position.set(5.0f, 6.0f, 8.0f);
    ss.camera->lookAt({0.f, 0.3f, 0.f});
    ss.controls->target    = {0.f, 0.3f, 0.f};
    ss.controls->maxPolarAngle = math::PI / 2.0f - 0.04f;
    ss.controls->update();

    ss.renderer.usePathTracer = false;
    ss.renderer.shadowMap().enabled = true;

    {
        auto sun = DirectionalLight::create(Color(0xffc87a), 2.0f);
        sun->position.set(-4.f, 8.f, 5.f);
        sun->castShadow          = true;
        sun->shadow->mapSize     = {2048, 2048};
        sun->shadow->bias        = -0.0005f;
        auto* cam = dynamic_cast<OrthographicCamera*>(sun->shadow->camera.get());
        if (cam) {
            cam->left = -7; cam->right  =  7;
            cam->top  =  7; cam->bottom = -7;
            cam->nearPlane = 1.f; cam->farPlane = 20.f;
            cam->updateProjectionMatrix();
        }
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
        floor->receiveShadow = true;
        ss.scene->add(floor);
    }

    // ── Fence ─────────────────────────────────────────────────────────────────
    OBJLoader loader;
    auto protos   = cell::loadCatalogProtos(loader, assetDir, catalog);
    auto fenceGrp = render::buildScene(scene, protos);
    fenceGrp->traverse([](Object3D& o) { o.castShadow = o.receiveShadow = true; });
    ss.scene->add(fenceGrp);

    // ── Belts ─────────────────────────────────────────────────────────────────
    auto beltTex = belt::makeBeltTexture();

    // Pallet belt: 800 mm wide, 6000 mm long, 400 mm surface height.
    // Centre at ECS (0, 0) = threepp (0, 0, 0).  Travels N-S → rotation.y = -π/2.
    auto [palletObj, palletMat] = belt::buildBeltMesh(800, 6000, 400,
                                                       belt::kGenericCatalog, beltTex);
    {
        auto grp = Group::create();
        grp->add(palletObj);
        grp->position.set(0.f, 0.f, 0.f);
        grp->rotation.y = -math::PI / 2.0f;
        grp->traverse([](Object3D& o) { o.castShadow = o.receiveShadow = true; });
        ss.scene->add(grp);
    }

    // Box belt: 300 mm wide, 2000 mm long, 800 mm surface height.
    // end_b (east)  = ECS X = -400 (adjacent to pallet belt west edge)
    // end_a (west)  = ECS X = -2400 (400 mm outside west fence)
    // centre        = ECS X = -1400 → threepp X = -1.4.
    // Travels E → rotation.y = 0.
    auto [boxObj, boxMat] = belt::buildBeltMesh(300, 2000, 800,
                                                 belt::kGenericCatalog, beltTex);
    {
        auto grp = Group::create();
        grp->add(boxObj);
        grp->position.set(-1.4f, 0.f, 0.f);
        grp->rotation.y = 0.f;
        grp->traverse([](Object3D& o) { o.castShadow = o.receiveShadow = true; });
        ss.scene->add(grp);
    }

    // ── Animate ───────────────────────────────────────────────────────────────
    const float tile_pitch_m = belt::kGenericCatalog.tile_pitch_mm * 0.001f;
    Clock       clock;

    ss.canvas.animate([&] {
        float delta = clock.getDelta();

        // UV scrolling — belt surface texture moves in +U (travel direction)
        float scroll = (200.f * 0.001f * delta) / tile_pitch_m;  // 200 mm/s
        if (palletMat->map)
            palletMat->map->offset.x += scroll;
        if (boxMat->map)
            boxMat->map->offset.x += scroll;

        ss.renderer.render(*ss.scene, *ss.camera);
    });

    return 0;
}
