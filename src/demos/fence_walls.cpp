#include <algorithm>
#include <cmath>
#include <optional>

#include <threepp/cameras/OrthographicCamera.hpp>
#include <threepp/core/Raycaster.hpp>
#include <threepp/geometries/BoxGeometry.hpp>
#include <threepp/input/MouseListener.hpp>
#include <threepp/materials/MeshStandardMaterial.hpp>
#include "common/scene_setup.hpp"
#include "cell/fence_catalog.hpp"
#include "cell/fence_solver.hpp"
#include "factory_scene/factory_scene.hpp"
#include "factory_scene/render_system.hpp"

using namespace threepp;

// ── Interaction system ────────────────────────────────────────────────────────

struct Interactor : MouseListener {

    factory::FactoryScene&      scene;
    const cell::CatalogProtos&  protos;
    const LookupTable&          table;
    const std::string&          assetDir;
    Scene&                      threeScene;
    PerspectiveCamera&          camera;
    Canvas&                     canvas;
    OrbitControls&              controls;
    std::shared_ptr<Object3D>&  fenceGrp;
    std::shared_ptr<Group>      overlayGrp;
    Raycaster                   raycaster;

    struct Drag {
        enum class Type { None, Edge, Opening } type = Type::None;
        entt::entity entity  = entt::null;
        float startX = 0, startZ = 0;   // world floor hit at drag start
        // Edge drag — perpendicular direction in world XZ
        float perpX = 0, perpZ = 0;
        entt::entity nodeA = entt::null, nodeB = entt::null;
        float na_x = 0, na_z = 0, nb_x = 0, nb_z = 0;   // saved node pos (mm)
        // Opening drag — along-edge direction
        float edgeDirX = 0, edgeDirZ = 0;
        float openingLocalX = 0;   // saved local x (mm from edge centre)
    } drag;

    Interactor(factory::FactoryScene& s, const cell::CatalogProtos& p,
               const LookupTable& t, const std::string& a,
               Scene& ts, PerspectiveCamera& cam, Canvas& cv,
               OrbitControls& ctrl, std::shared_ptr<Object3D>& fg)
        : scene(s), protos(p), table(t), assetDir(a),
          threeScene(ts), camera(cam), canvas(cv), controls(ctrl),
          fenceGrp(fg), overlayGrp(Group::create())
    {
        threeScene.add(overlayGrp);
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    std::optional<std::pair<float,float>> rayToFloor(const Vector2& pix) {
        auto sz = canvas.size();
        Vector2 ndc{ (pix.x / sz.width())  * 2.f - 1.f,
                    -(pix.y / sz.height()) * 2.f + 1.f };
        raycaster.setFromCamera(ndc, camera);
        auto& ray = raycaster.ray;
        if (std::abs(ray.direction.y) < 1e-6f) return std::nullopt;
        float t = -ray.origin.y / ray.direction.y;
        if (t < 0.f) return std::nullopt;
        Vector3 hit;
        ray.at(t, hit);
        return std::make_pair(hit.x, hit.z);
    }

    // Distance from world point (wx,wz) to edge line segment. Returns {entity, dist}.
    entt::entity pickEdge(float wx, float wz) {
        auto& reg = scene.registry();
        entt::entity best = entt::null;
        float bestD = 0.35f;
        reg.view<factory::EdgeComponent, factory::PoseComponent>().each(
            [&](entt::entity e, const factory::EdgeComponent& ec, const factory::PoseComponent&) {
                const auto& pa = reg.get<factory::PoseComponent>(ec.node_a);
                const auto& pb = reg.get<factory::PoseComponent>(ec.node_b);
                float ax = pa.position.x * 0.001f, az = pa.position.y * 0.001f;
                float bx = pb.position.x * 0.001f, bz = pb.position.y * 0.001f;
                float dx = bx-ax, dz = bz-az;
                float len2 = dx*dx + dz*dz;
                if (len2 < 1e-6f) return;
                float tc = std::clamp(((wx-ax)*dx + (wz-az)*dz) / len2, 0.f, 1.f);
                float d = std::hypot(wx - (ax + tc*dx), wz - (az + tc*dz));
                if (d < bestD) { bestD = d; best = e; }
            });
        return best;
    }

    entt::entity pickOpening(float wx, float wz) {
        auto& reg = scene.registry();
        entt::entity best = entt::null;
        float bestD = 0.25f;
        reg.view<factory::DeclaredOpeningComponent, factory::PoseComponent>().each(
            [&](entt::entity e, const factory::DeclaredOpeningComponent& oc,
                const factory::PoseComponent& op) {
                if (oc.parent_edge == entt::null) return;
                const auto& ep = reg.get<factory::PoseComponent>(oc.parent_edge);
                const auto& ec = reg.get<factory::EdgeComponent>(oc.parent_edge);
                const auto& pa = reg.get<factory::PoseComponent>(ec.node_a);
                const auto& pb = reg.get<factory::PoseComponent>(ec.node_b);
                float dx = pb.position.x - pa.position.x;
                float dz = pb.position.y - pa.position.y;
                float len = std::hypot(dx, dz);
                if (len < 0.001f) return;
                float ox = ep.position.x*0.001f + op.position.x*0.001f*(dx/len);
                float oz = ep.position.y*0.001f + op.position.x*0.001f*(dz/len);
                float d = std::hypot(wx - ox, wz - oz);
                if (d < bestD) { bestD = d; best = e; }
            });
        return best;
    }

    // Semi-transparent box representing one fence wall segment A→B.
    std::shared_ptr<Mesh> makeWallBox(float ax, float az, float bx, float bz,
                                      const Color& col, float opacity = 0.55f) {
        float dx = bx-ax, dz = bz-az;
        float len = std::max(0.01f, std::hypot(dx, dz));
        float fh = protos.edge_height_mm * 0.001f;
        auto mat = MeshStandardMaterial::create();
        mat->color = col; mat->transparent = true; mat->opacity = opacity;
        auto box = Mesh::create(BoxGeometry::create(len, fh, 0.06f), mat);
        box->position.set((ax+bx)*0.5f, fh*0.5f, (az+bz)*0.5f);
        box->rotation.y = -std::atan2(dz, dx);
        return box;
    }

    void rebuildFence() {
        threeScene.remove(*fenceGrp);
        fenceGrp = render::buildScene(scene, protos);
        fenceGrp->traverse([](Object3D& o) { o.castShadow = o.receiveShadow = true; });
        threeScene.add(fenceGrp);
    }

    // ── Overlay ───────────────────────────────────────────────────────────────

    void updateOverlay(float wx, float wz) {
        overlayGrp->clear();
        auto& reg = scene.registry();

        if (drag.type == Drag::Type::Edge) {
            float dp = (wx - drag.startX)*drag.perpX + (wz - drag.startZ)*drag.perpZ;
            float naX = drag.na_x*0.001f + dp*drag.perpX;
            float naZ = drag.na_z*0.001f + dp*drag.perpZ;
            float nbX = drag.nb_x*0.001f + dp*drag.perpX;
            float nbZ = drag.nb_z*0.001f + dp*drag.perpZ;

            overlayGrp->add(makeWallBox(naX, naZ, nbX, nbZ, Color(0xffe090)));

            reg.view<factory::EdgeComponent>().each(
                [&](entt::entity e, const factory::EdgeComponent& ec) {
                    if (e == drag.entity) return;
                    entt::entity shared = entt::null, fixed = entt::null;
                    if      (ec.node_a == drag.nodeA || ec.node_a == drag.nodeB)
                        { shared = ec.node_a; fixed = ec.node_b; }
                    else if (ec.node_b == drag.nodeA || ec.node_b == drag.nodeB)
                        { shared = ec.node_b; fixed = ec.node_a; }
                    else return;
                    const auto& fp = reg.get<factory::PoseComponent>(fixed);
                    float fx = fp.position.x*0.001f, fz = fp.position.y*0.001f;
                    float sx = (shared == drag.nodeA) ? naX : nbX;
                    float sz = (shared == drag.nodeA) ? naZ : nbZ;
                    overlayGrp->add(makeWallBox(fx, fz, sx, sz, Color(0xffd060), 0.45f));
                });

        } else if (drag.type == Drag::Type::Opening) {
            const auto& oc = reg.get<factory::DeclaredOpeningComponent>(drag.entity);
            const auto& ec = reg.get<factory::EdgeComponent>(oc.parent_edge);
            const auto& ep = reg.get<factory::PoseComponent>(oc.parent_edge);
            const auto& pa = reg.get<factory::PoseComponent>(ec.node_a);
            const auto& pb = reg.get<factory::PoseComponent>(ec.node_b);
            float dx = pb.position.x - pa.position.x;
            float dz = pb.position.y - pa.position.y;
            float len = std::hypot(dx, dz);
            float dirX = dx/len, dirZ = dz/len;
            float da = (wx - drag.startX)*drag.edgeDirX + (wz - drag.startZ)*drag.edgeDirZ;
            float lx = drag.openingLocalX*0.001f + da;
            float ox = ep.position.x*0.001f + lx*dirX;
            float oz = ep.position.y*0.001f + lx*dirZ;
            float ow = oc.width_mm*0.001f, oh = protos.edge_height_mm*0.001f;
            auto mat = MeshStandardMaterial::create();
            mat->color = Color(0xffa050); mat->transparent = true; mat->opacity = 0.7f;
            auto box = Mesh::create(BoxGeometry::create(ow, oh, 0.07f), mat);
            box->position.set(ox, oh*0.5f, oz);
            box->rotation.y = -std::atan2(dz, dx);
            overlayGrp->add(box);
        }
    }

    // ── Mouse events ──────────────────────────────────────────────────────────

    void onMouseDown(int button, const Vector2& pos) override {
        if (button != 0) return;
        auto hit = rayToFloor(pos);
        if (!hit) return;
        auto [wx, wz] = *hit;
        auto& reg = scene.registry();

        // Opening takes priority (smaller target).
        auto oe = pickOpening(wx, wz);
        if (oe != entt::null) {
            const auto& oc = reg.get<factory::DeclaredOpeningComponent>(oe);
            const auto& ec = reg.get<factory::EdgeComponent>(oc.parent_edge);
            const auto& pa = reg.get<factory::PoseComponent>(ec.node_a);
            const auto& pb = reg.get<factory::PoseComponent>(ec.node_b);
            float dx = pb.position.x - pa.position.x;
            float dz = pb.position.y - pa.position.y;
            float len = std::hypot(dx, dz);
            drag.type         = Drag::Type::Opening;
            drag.entity       = oe;
            drag.startX       = wx;  drag.startZ      = wz;
            drag.edgeDirX     = dx/len; drag.edgeDirZ = dz/len;
            drag.openingLocalX = reg.get<factory::PoseComponent>(oe).position.x;
            fenceGrp->visible = false;
            controls.enabled  = false;
            return;
        }

        auto ee = pickEdge(wx, wz);
        if (ee != entt::null) {
            const auto& ec = reg.get<factory::EdgeComponent>(ee);
            const auto& pa = reg.get<factory::PoseComponent>(ec.node_a);
            const auto& pb = reg.get<factory::PoseComponent>(ec.node_b);
            float dx = pb.position.x - pa.position.x;
            float dz = pb.position.y - pa.position.y;
            float len = std::hypot(dx, dz);
            drag.type   = Drag::Type::Edge;
            drag.entity = ee;
            drag.startX = wx;  drag.startZ = wz;
            drag.perpX  = -dz/len; drag.perpZ = dx/len;   // local +Z = inward
            drag.nodeA  = ec.node_a;  drag.nodeB = ec.node_b;
            drag.na_x   = pa.position.x; drag.na_z = pa.position.y;
            drag.nb_x   = pb.position.x; drag.nb_z = pb.position.y;
            fenceGrp->visible = false;
            controls.enabled  = false;
        }
    }

    void onMouseMove(const Vector2& pos) override {
        if (drag.type == Drag::Type::None) return;
        auto hit = rayToFloor(pos);
        if (!hit) return;
        auto [wx, wz] = *hit;
        updateOverlay(wx, wz);
    }

    void onMouseUp(int button, const Vector2& pos) override {
        if (button != 0 || drag.type == Drag::Type::None) return;

        auto hit = rayToFloor(pos);
        if (hit) {
            auto [wx, wz] = *hit;
            auto& reg = scene.registry();

            if (drag.type == Drag::Type::Edge) {
                float dp_mm = ((wx-drag.startX)*drag.perpX + (wz-drag.startZ)*drag.perpZ) * 1000.f;
                auto& pa = reg.get<factory::PoseComponent>(drag.nodeA);
                auto& pb = reg.get<factory::PoseComponent>(drag.nodeB);
                pa.position.x = drag.na_x + dp_mm * drag.perpX;
                pa.position.y = drag.na_z + dp_mm * drag.perpZ;
                pb.position.x = drag.nb_x + dp_mm * drag.perpX;
                pb.position.y = drag.nb_z + dp_mm * drag.perpZ;
                scene.solve(table, assetDir);

            } else if (drag.type == Drag::Type::Opening) {
                // Unallocate so the solver re-places it from scratch.
                auto& oc = reg.get<factory::DeclaredOpeningComponent>(drag.entity);
                oc.parent_edge = entt::null;
                scene.solve(table, assetDir);
            }

            rebuildFence();
        }

        overlayGrp->clear();
        fenceGrp->visible = true;
        drag = Drag{};
        controls.enabled = true;
    }
};

// ── Main ──────────────────────────────────────────────────────────────────────

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

    SceneSetup ss("FenceWalls");
    ss.scene->background = Color(0xfff5e8);
    ss.camera->position.set(7.0f, 5.0f, 9.0f);
    ss.camera->lookAt({0, 0.5f, 0});
    ss.controls->target = {0, 0.5f, 0};
    ss.controls->maxPolarAngle = math::PI / 2.0f - 0.05f;
    ss.controls->update();

    ss.renderer.usePathTracer = false;
    ss.renderer.shadowMap().enabled = true;

    {
        auto sun = DirectionalLight::create(Color(0xffc87a), 2.0f);
        sun->position.set(-5, 8, 4);
        sun->castShadow = true;
        sun->shadow->mapSize = {2048, 2048};
        sun->shadow->bias    = -0.0005f;
        auto* cam = dynamic_cast<OrthographicCamera*>(sun->shadow->camera.get());
        if (cam) {
            cam->left = -6; cam->right  =  6;
            cam->top  =  6; cam->bottom = -6;
            cam->nearPlane = 1.0f; cam->farPlane = 20.0f;
            cam->updateProjectionMatrix();
        }
        ss.scene->add(sun);
    }
    ss.scene->add(AmbientLight::create(Color(0xfff0e0), 1.2f));

    {
        auto geo = PlaneGeometry::create(60.0f, 60.0f);
        auto mat = MeshStandardMaterial::create();
        mat->color = Color(0xede8d8); mat->roughness = 0.4f; mat->metalness = 0.0f;
        auto floor = Mesh::create(geo, mat);
        floor->rotation.x = -math::PI / 2.f;
        floor->receiveShadow = true;
        ss.scene->add(floor);
    }

    OBJLoader loader;
    auto protos   = cell::loadCatalogProtos(loader, assetDir, catalog);
    auto fenceGrp = render::buildScene(scene, protos);
    fenceGrp->traverse([](Object3D& o) { o.castShadow = o.receiveShadow = true; });
    ss.scene->add(fenceGrp);

    Interactor interactor(scene, protos, table, assetDir,
                          *ss.scene, *ss.camera, ss.canvas, *ss.controls, fenceGrp);
    ss.canvas.addMouseListener(interactor);

    ss.canvas.animate([&] {
        ss.scene->fog = FogExp2(Color(0xfff5e8), 0.06f);
        ss.renderer.render(*ss.scene, *ss.camera);
    });

    return 0;
}
