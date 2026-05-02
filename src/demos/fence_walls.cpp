#include <algorithm>

#include <threepp/scenes/Fog.hpp>
#include "common/scene_setup.hpp"
#include "cell/fence_catalog.hpp"
#include "cell/fence_solver.hpp"
#include "factory_scene/factory_scene.hpp"
#include "factory_scene/render_system.hpp"

using namespace threepp;

int main() {
    const std::string assetDir = "assets/components/fences/axelent_x-guard";

    auto catalog = cell::loadCatalog(assetDir + "/catalog.json");
    auto table   = loadTable(assetDir + "/combinations.json");

    // Declare layout intent: 4 corner nodes (mm, Z-up) + one unallocated slab.
    // The solver assigns the slab to edge 0 (south) and chooses panel combinations.
    factory::FactoryScene scene;
    scene.place_node(-2000.f, -1500.f);  // SW
    scene.place_node( 2000.f, -1500.f);  // SE
    scene.place_node( 2000.f,  1500.f);  // NE
    scene.place_node(-2000.f,  1500.f);  // NW
    scene.declare_opening(500);          // slab — solver picks edge + position

    scene.solve(table, assetDir);

    // ── Scene setup ───────────────────────────────────────────────────────────
    SceneSetup ss("FenceWalls");
    ss.scene->background = Color(0x3d2e1e);
    ss.scene->fog         = Fog(Color(0x3d2e1e), 12.0f, 35.0f);
    ss.camera->position.set(7.0f, 5.0f, 9.0f);
    ss.camera->lookAt({0, 0.5f, 0});
    ss.controls->target = {0, 0.5f, 0};
    ss.controls->update();

    // Warm amber key — upper right
    {
        auto key = DirectionalLight::create(0xffe8a0, 1.2f);
        key->position.set(6, 9, 4);
        ss.scene->add(key);
    }
    // Cool blue-purple fill — opposite side
    {
        auto fill = DirectionalLight::create(0x6080b0, 0.4f);
        fill->position.set(-5, 4, -6);
        ss.scene->add(fill);
    }
    // Warm ambient — bright enough to read the whole scene comfortably
    ss.scene->add(AmbientLight::create(0xc87840, 0.5f));

    // Floor — fixed large plane; fog hides the edge so no clipping visible.
    {
        auto geo = PlaneGeometry::create(60.0f, 60.0f);
        auto mat = MeshPhongMaterial::create();
        mat->color     = Color(0x1c1a18);
        mat->specular  = Color(0x886644);
        mat->shininess = 70;
        auto floor = Mesh::create(geo, mat);
        floor->rotation.x = -math::PI / 2.f;
        ss.scene->add(floor);
    }

    OBJLoader loader;
    auto protos = cell::loadCatalogProtos(loader, assetDir, catalog);
    ss.scene->add(render::buildScene(scene, protos));

    ss.canvas.animate([&] {
        ss.renderer.render(*ss.scene, *ss.camera);
    });

    return 0;
}
