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
    ss.scene->background = Color(0xfff8f0);  // warm cream sky — doubles as path tracer env
    ss.camera->position.set(7.0f, 5.0f, 9.0f);
    ss.camera->lookAt({0, 0.5f, 0});
    ss.controls->target = {0, 0.5f, 0};
    ss.controls->maxPolarAngle = math::PI / 2.0f - 0.05f;
    ss.controls->update();

    // Path tracer — handles transparent shadows physically.
    ss.renderer.usePathTracer = true;
    auto& pt = ss.renderer.pathTracer();
    pt.setReSTIREnabled(true);
    pt.setMaxBounces(4);
    pt.setExposure(1.3f);

    // Warm sun from upper-right.
    {
        auto key = DirectionalLight::create(0xffe8c8, 2.0f);
        key->position.set(6, 9, 4);
        key->castShadow = true;
        ss.scene->add(key);
    }
    ss.scene->add(AmbientLight::create(0xfff8f0, 1.5f));

    // Floor — warm light concrete, slightly polished.
    {
        auto geo = PlaneGeometry::create(60.0f, 60.0f);
        auto mat = MeshStandardMaterial::create();
        mat->color     = Color(0xd0c4b0);
        mat->roughness = 0.25f;
        mat->metalness = 0.0f;
        auto floor = Mesh::create(geo, mat);
        floor->rotation.x    = -math::PI / 2.f;
        floor->receiveShadow = true;
        ss.scene->add(floor);
    }

    // Fence — all objects cast + receive shadows; path tracer handles
    // transmission correctly so the opening no longer needs special casing.
    OBJLoader loader;
    auto protos   = cell::loadCatalogProtos(loader, assetDir, catalog);
    auto fenceGrp = render::buildScene(scene, protos);
    fenceGrp->traverse([](Object3D& obj) {
        obj.castShadow    = true;
        obj.receiveShadow = true;
    });
    ss.scene->add(fenceGrp);

    ss.canvas.animate([&] {
        ss.scene->fog = FogExp2(Color(0xfff5e8), 0.04f);
        pt.setFogAnisotropy(0.2f);
        ss.renderer.render(*ss.scene, *ss.camera);
    });

    return 0;
}
