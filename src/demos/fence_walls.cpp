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
    // Background = bright warm cream; in path tracing this IS the sky, so it
    // provides ambient fill from all directions and the fog dissolves things
    // into the same warm light rather than into darkness.
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
    pt.setExposure(1.1f);

    // ── Lighting ──────────────────────────────────────────────────────────────
    // Three RectAreaLight strips that simulate a high bank of factory windows
    // on the left side.  They are tilted ~30° from vertical toward the camera
    // so the light comes in at a sunrise angle — directional shadows — while
    // still being overhead enough to reflect as three parallel strips in the
    // polished floor.  mesh()->visible = false so the sources are never seen.
    {
        const Color  fc(0xffc870);
        // rotation.x = -PI/2 is straight down. Adding +PI/6 tilts the emitting
        // face ~30° toward +X (camera side) → light rakes across the cell from
        // upper-right, casting diagonal shadows to the left.
        const float  tilt = -math::PI / 2.0f + math::PI / 6.0f;
        const float  fy   = 3.2f;
        const float  fx   = 1.2f;  // offset toward camera side so strips clear
        for (float z : {-0.9f, 0.0f, 0.9f}) {
            auto fix = RectAreaLight::create(fc, 28.0f, 3.5f, 0.25f);
            fix->rotation.x = tilt;
            fix->position.set(fx, fy, z);
            fix->mesh()->visible = false;
            ss.scene->add(fix);
        }
    }

    // Floor — warm polished concrete.  roughness 0.3: area-light strips show
    // as clear reflections; no tight specular blob from any point source.
    {
        auto geo = PlaneGeometry::create(60.0f, 60.0f);
        auto mat = MeshStandardMaterial::create();
        mat->color     = Color(0xede8d8);
        mat->roughness = 0.3f;
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
        // Fog dissolves into the warm bright background — things soften into
        // light, not darkness.  Set each frame so path tracer picks it up.
        ss.scene->fog = FogExp2(Color(0xfff5e8), 0.06f);
        ss.renderer.render(*ss.scene, *ss.camera);
    });

    return 0;
}
