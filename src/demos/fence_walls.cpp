#include <algorithm>

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
    ss.scene->background = Color(0x1a1f2e);
    ss.camera->position.set(7.0f, 5.0f, 9.0f);
    ss.camera->lookAt({0, 0.5f, 0});
    ss.controls->target = {0, 0.5f, 0};
    ss.controls->update();

    ss.scene->add(AmbientLight::create(0xffffff, 0.4f));
    {
        auto sun = DirectionalLight::create(0xffffff, 0.8f);
        sun->position.set(5, 10, 7);
        ss.scene->add(sun);
    }

    // Floor — sized from solved node positions in ECS.
    {
        float min_x = 1e9f, max_x = -1e9f, min_z = 1e9f, max_z = -1e9f;
        scene.registry().view<factory::NodeComponent,
                               factory::PoseComponent>().each(
            [&](entt::entity,
                const factory::NodeComponent&,
                const factory::PoseComponent& p)
            {
                min_x = std::min(min_x, p.position.x);
                max_x = std::max(max_x, p.position.x);
                min_z = std::min(min_z, p.position.y);
                max_z = std::max(max_z, p.position.y);
            });
        auto geo = PlaneGeometry::create((max_x - min_x) * 0.001f,
                                         (max_z - min_z) * 0.001f);
        auto mat = MeshPhongMaterial::create();
        mat->color = Color(0x2a2a2a);
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
