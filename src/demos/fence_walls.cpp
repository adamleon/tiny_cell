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
    SceneSetup ss("FenceWalls");
    ss.scene->background = Color(0x080604);  // near-black warm void
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

    // Overhead fixture strips — warm sunrise orange, aimed downward.
    // Three parallel rows above the cell; RectAreaLight auto-creates its own
    // emissive quad mesh, so the physical tubes are visible in reflections.
    {
        const Color  fixtureColor(0xffa060);
        const float  fixtureIntensity = 18.0f;
        const float  fixtureLength    = 3.2f;  // slightly overhangs the ±1.5m cell depth
        const float  fixtureWidth     = 0.12f;
        const float  fixtureY         = 3.0f;
        const float  zPositions[]     = { -0.9f, 0.0f, 0.9f };

        for (float z : zPositions) {
            auto fix = RectAreaLight::create(fixtureColor, fixtureIntensity,
                                             fixtureLength, fixtureWidth);
            fix->rotation.x = -math::PI / 2.0f;  // face downward
            fix->position.set(0.0f, fixtureY, z);
            ss.scene->add(fix);
        }
    }

    // Dim warm fill so the void isn't absolute black.
    ss.scene->add(AmbientLight::create(Color(0x3a2010), 0.25f));

    // Floor — polished epoxy: low roughness so fixtures show as strips.
    {
        auto geo = PlaneGeometry::create(60.0f, 60.0f);
        auto mat = MeshStandardMaterial::create();
        mat->color     = Color(0xd0c8b8);
        mat->roughness = 0.12f;
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
        // FogExp2 must be set each frame for the path tracer to pick it up.
        ss.scene->fog = FogExp2(Color(0x080604), 0.07f);
        pt.setFogAnisotropy(0.0f);
        ss.renderer.render(*ss.scene, *ss.camera);
    });

    return 0;
}
