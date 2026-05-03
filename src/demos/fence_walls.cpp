#include <algorithm>

#include <threepp/cameras/OrthographicCamera.hpp>
#include <threepp/geometries/BoxGeometry.hpp>
#include <threepp/scenes/Fog.hpp>
#include <threepp/textures/DataTexture.hpp>
#include "common/scene_setup.hpp"
#include "cell/fence_catalog.hpp"
#include "cell/fence_solver.hpp"
#include "factory_scene/factory_scene.hpp"
#include "factory_scene/render_system.hpp"

using namespace threepp;

// Build a 1×64 RGBA specularMap with two stripe bands that tile to match
// the fluorescent tube positions (Z=±0.6 m on a 60 m floor tiled 30×).
static std::shared_ptr<DataTexture> makeStripeSpecMap() {
    const int W = 1, H = 64;
    std::vector<unsigned char> px(W * H * 4, 0);
    auto white = [&](int row) {
        px[row * 4 + 0] = px[row * 4 + 1] = px[row * 4 + 2] = px[row * 4 + 3] = 255;
    };
    // Two bands per 2 m tile (tile repeats 30× on 60 m floor):
    //   Z=+0.6 m  →  tile-UV 0.70  →  row 45 (±3 rows = 0.19 m wide)
    //   Z=-0.6 m  →  tile-UV 0.30  →  row 19 (±3 rows)
    for (int r = 16; r <= 22; ++r) white(r);
    for (int r = 42; r <= 48; ++r) white(r);

    auto tex = DataTexture::create(std::vector<unsigned char>(px), W, H);
    tex->wrapS  = TextureWrapping::Repeat;
    tex->wrapT  = TextureWrapping::Repeat;
    tex->repeat.set(1.0f, 30.0f);
    tex->magFilter = Filter::Linear;
    tex->minFilter = Filter::Linear;
    return tex;
}

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
    ss.scene->background = Color(0xede8e2);
    ss.scene->fog         = Fog(Color(0xede8e2), 12.0f, 22.0f);
    ss.camera->position.set(7.0f, 5.0f, 9.0f);
    ss.camera->lookAt({0, 0.5f, 0});
    ss.controls->target = {0, 0.5f, 0};
    ss.controls->maxPolarAngle = math::PI / 2.0f - 0.05f;
    ss.controls->update();

    ss.renderer.shadowMap().enabled = true;

    // Key light: steep angle from upper-right → good for shadows, its floor
    // reflection points away from the default camera so no blob.
    {
        auto key = DirectionalLight::create(0xfff5ec, 0.65f);
        key->position.set(6, 9, 4);
        key->castShadow = true;
        key->shadow->mapSize = {2048, 2048};
        key->shadow->bias    = -0.0005f;
        auto* cam = dynamic_cast<OrthographicCamera*>(key->shadow->camera.get());
        if (cam) {
            cam->left = -8; cam->right  =  8;
            cam->top  =  8; cam->bottom = -8;
            cam->nearPlane = 1.0f; cam->farPlane = 25.0f;
            cam->updateProjectionMatrix();
        }
        ss.scene->add(key);
    }

    // Specular fill: shallow angle from front-left, positioned so its floor
    // reflection aligns with the default camera view → creates the specular
    // the specularMap turns into stripes.
    {
        auto fill = DirectionalLight::create(0xfff0e8, 0.3f);
        fill->position.set(-5, 4, -7);
        ss.scene->add(fill);
    }

    ss.scene->add(AmbientLight::create(0xfff8f0, 0.75f));

    // ── Fluorescent fixture tubes (visual only — no lights) ──────────────────
    {
        auto geo = BoxGeometry::create(3.8f, 0.04f, 0.10f);
        auto mat = MeshPhongMaterial::create();
        mat->color    = Color(0xffffff);
        mat->emissive = Color(0xfff4e8);
        for (float fz : {-0.6f, 0.6f}) {
            auto tube = Mesh::create(geo, mat);
            tube->position.set(0.0f, 3.5f, fz);
            ss.scene->add(tube);
        }
    }

    // ── Floor ─────────────────────────────────────────────────────────────────
    // specularMap stripes align with tube Z positions: specular from the fill
    // light shows only on the stripe bands → looks like fixture reflections.
    {
        auto geo = PlaneGeometry::create(60.0f, 60.0f);
        auto mat = MeshPhongMaterial::create();
        mat->color       = Color(0x8c8278);
        mat->specular    = Color(0xe0e0e0);
        mat->shininess   = 40;
        mat->specularMap = makeStripeSpecMap();
        auto floor = Mesh::create(geo, mat);
        floor->rotation.x    = -math::PI / 2.f;
        floor->receiveShadow = true;
        ss.scene->add(floor);
    }

    // ── Fence ─────────────────────────────────────────────────────────────────
    OBJLoader loader;
    auto protos   = cell::loadCatalogProtos(loader, assetDir, catalog);
    auto fenceGrp = render::buildScene(scene, protos);
    fenceGrp->traverse([](Object3D& obj) {
        obj.castShadow    = true;
        obj.receiveShadow = true;
    });
    ss.scene->add(fenceGrp);

    ss.canvas.animate([&] {
        ss.renderer.render(*ss.scene, *ss.camera);
    });

    return 0;
}
