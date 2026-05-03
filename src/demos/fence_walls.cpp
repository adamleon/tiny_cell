#include <algorithm>

#include <threepp/lights/RectAreaLight.hpp>
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
    // Background = the "factory haze" colour; fog dissolves everything into
    // this warm bright cream so distant geometry softens without going dark.
    SceneSetup ss("FenceWalls");
    ss.scene->background = Color(0xfff6e8);
    ss.camera->position.set(7.0f, 5.0f, 9.0f);
    ss.camera->lookAt({0, 0.5f, 0});
    ss.controls->target = {0, 0.5f, 0};
    ss.controls->maxPolarAngle = math::PI / 2.0f - 0.05f;
    ss.controls->update();

    // Path tracer — handles transparent shadows + RectAreaLight physically.
    ss.renderer.usePathTracer = true;
    auto& pt = ss.renderer.pathTracer();
    pt.setReSTIREnabled(true);
    pt.setMaxBounces(4);
    pt.setExposure(1.1f);

    // Bright warm fill — simulates the haze/ceiling acting as a giant softbox.
    ss.scene->add(AmbientLight::create(Color(0xfff0d8), 2.0f));

    // Overhead fixture strips — warm golden, aimed downward.
    // Reflections in the polished floor appear as three parallel strips.
    {
        const Color  fixtureColor(0xffe0a0);
        const float  intensity  = 20.0f;
        const float  length     = 3.2f;
        const float  width      = 0.12f;
        const float  y          = 3.0f;
        const float  zPos[]     = { -0.9f, 0.0f, 0.9f };

        for (float z : zPos) {
            auto fix = RectAreaLight::create(fixtureColor, intensity, length, width);
            fix->rotation.x = -math::PI / 2.0f;  // face downward
            fix->position.set(0.0f, y, z);
            ss.scene->add(fix);
        }
    }

    // Floor — warm polished concrete: bright base colour, low roughness so
    // fixture strips are visible as clean reflections.
    {
        auto geo = PlaneGeometry::create(60.0f, 60.0f);
        auto mat = MeshStandardMaterial::create();
        mat->color     = Color(0xe8dfd0);
        mat->roughness = 0.15f;
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
        // Fog dissolves into the bright background — warm haze, not dark void.
        ss.scene->fog = FogExp2(Color(0xfff6e8), 0.06f);
        ss.renderer.render(*ss.scene, *ss.camera);
    });

    return 0;
}
