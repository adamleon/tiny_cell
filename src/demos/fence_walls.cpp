#include <algorithm>

#include <threepp/cameras/OrthographicCamera.hpp>
#include <threepp/geometries/BoxGeometry.hpp>
#include <threepp/lights/PointLight.hpp>
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

    factory::FactoryScene scene;
    scene.place_node(-2000.f, -1500.f);  // SW
    scene.place_node( 2000.f, -1500.f);  // SE
    scene.place_node( 2000.f,  1500.f);  // NE
    scene.place_node(-2000.f,  1500.f);  // NW
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

    // Key light — warm white, casts shadows, no specular blob risk (steep angle)
    {
        auto key = DirectionalLight::create(0xfff5ec, 0.7f);
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

    // Ambient — main brightness, near-white warm
    ss.scene->add(AmbientLight::create(0xfff8f0, 0.8f));

    // ── Fluorescent fixtures ─────────────────────────────────────────────────
    // Two rows of tubes running along X, at ceiling height above the cell.
    // Each row: 5 PointLights spaced 0.9m apart → 5 small specular spots merge
    // into a strip on the polished floor.
    const float fixtureY = 3.5f;
    const std::array<float, 2> fixtureZ = {-0.6f, 0.6f};
    const std::array<float, 5> fixtureX = {-1.8f, -0.9f, 0.0f, 0.9f, 1.8f};

    for (float fz : fixtureZ) {
        // Visible tube mesh
        auto geo = BoxGeometry::create(3.8f, 0.04f, 0.10f);
        auto mat = MeshPhongMaterial::create();
        mat->color     = Color(0xffffff);
        mat->emissive  = Color(0xfff4e8);
        ss.scene->add([&] {
            auto tube = Mesh::create(geo, mat);
            tube->position.set(0.0f, fixtureY, fz);
            return tube;
        }());

        // Point lights along the tube
        for (float fx : fixtureX) {
            auto pt = PointLight::create(Color(0xfff4e8), 0.4f, 7.0f, 1.5f);
            pt->position.set(fx, fixtureY, fz);
            ss.scene->add(pt);
        }
    }

    // ── Floor ────────────────────────────────────────────────────────────────
    {
        auto geo = PlaneGeometry::create(60.0f, 60.0f);
        auto mat = MeshPhongMaterial::create();
        mat->color     = Color(0x8c8278);
        mat->specular  = Color(0xd0d0d0);
        mat->shininess = 120;
        auto floor = Mesh::create(geo, mat);
        floor->rotation.x    = -math::PI / 2.f;
        floor->receiveShadow = true;
        ss.scene->add(floor);
    }

    // ── Fence ────────────────────────────────────────────────────────────────
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
