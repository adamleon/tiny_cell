#include <algorithm>

#include <threepp/materials/MeshStandardMaterial.hpp>
#include <threepp/renderers/wgpu/WgpuPathTracer.hpp>
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

    factory::FactoryScene scene;
    scene.place_node(-2000.f, -1500.f);
    scene.place_node( 2000.f, -1500.f);
    scene.place_node( 2000.f,  1500.f);
    scene.place_node(-2000.f,  1500.f);
    scene.declare_opening(500);

    scene.solve(table, assetDir);

    // ── Scene setup ───────────────────────────────────────────────────────────
    SceneSetup ss("FenceWalls");
    ss.scene->background = Color(0xfff5e8);
    ss.camera->position.set(7.0f, 5.0f, 9.0f);
    ss.camera->lookAt({0, 0.5f, 0});
    ss.controls->target = {0, 0.5f, 0};
    ss.controls->maxPolarAngle = math::PI / 2.0f - 0.05f;
    ss.controls->update();

    ss.renderer.usePathTracer = true;
    auto& pt = ss.renderer.pathTracer();
    pt.setReSTIREnabled(true);
    pt.setMaxBounces(4);
    pt.setExposure(1.2f);

    // Warm directional key — sunrise angle from camera-right, slightly front.
    {
        auto sun = DirectionalLight::create(Color(0xffc87a), 2.5f);
        sun->position.set(-5, 8, 4);
        sun->castShadow = true;
        ss.scene->add(sun);
    }
    // Soft fill so shadows aren't pitch black.
    ss.scene->add(AmbientLight::create(Color(0xfff0e0), 1.2f));

    // Floor — warm concrete, moderate polish.
    {
        auto geo = PlaneGeometry::create(60.0f, 60.0f);
        auto mat = MeshStandardMaterial::create();
        mat->color     = Color(0xede8d8);
        mat->roughness = 0.4f;
        mat->metalness = 0.0f;
        auto floor = Mesh::create(geo, mat);
        floor->rotation.x    = -math::PI / 2.f;
        floor->receiveShadow = true;
        ss.scene->add(floor);
    }

    // Fence — path tracer handles transmission on the opening indicator.
    OBJLoader loader;
    auto protos   = cell::loadCatalogProtos(loader, assetDir, catalog);
    auto fenceGrp = render::buildScene(scene, protos);
    fenceGrp->traverse([](Object3D& obj) {
        obj.castShadow    = true;
        obj.receiveShadow = true;
    });
    ss.scene->add(fenceGrp);

    ss.canvas.animate([&] {
        ss.scene->fog = FogExp2(Color(0xfff5e8), 0.06f);
        ss.renderer.render(*ss.scene, *ss.camera);
    });

    return 0;
}
