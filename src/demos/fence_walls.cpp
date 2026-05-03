#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <unordered_map>

#include <threepp/cameras/OrthographicCamera.hpp>
#include <threepp/core/Raycaster.hpp>
#include <threepp/geometries/BoxGeometry.hpp>
#include <threepp/input/KeyListener.hpp>
#include <threepp/input/MouseListener.hpp>
#include <threepp/materials/MeshStandardMaterial.hpp>
#include "common/scene_setup.hpp"
#include "cell/fence_catalog.hpp"
#include "cell/fence_solver.hpp"
#include "factory_scene/factory_scene.hpp"
#include "factory_scene/render_system.hpp"

using namespace threepp;

// ── Interaction system ────────────────────────────────────────────────────────

struct Interactor : MouseListener, KeyListener {

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
    std::shared_ptr<Group>      pickGroup;
    Raycaster                   raycaster;
    bool                        shiftDown        = false;
    bool                        pickBoxesVisible = false;

    struct Drag {
        enum class Type { None, Edge, Opening } type = Type::None;
        entt::entity entity   = entt::null;
        float startX = 0, startZ = 0;
        // Edge drag
        float perpX = 0, perpZ = 0;
        float edgeDirX = 0, edgeDirZ = 0;
        entt::entity nodeA = entt::null, nodeB = entt::null;
        float na_x = 0, na_z = 0, nb_x = 0, nb_z = 0;   // saved node pos (mm)
        // Angle-preserving mode: far corners of the two adjacent edges (metres)
        bool  hasCornerA = false, hasCornerB = false;
        float cornerAX = 0, cornerAZ = 0;
        float cornerBX = 0, cornerBZ = 0;
        bool  anglePreserving = false;
        // Opening drag
        float openingLocalX = 0;
    } drag;

    struct PickBox {
        entt::entity          entity;
        int                   priority;
        Drag::Type            dragType;
        std::shared_ptr<Mesh> mesh;
    };
    std::vector<PickBox>                        pickBoxes;
    std::unordered_map<Object3D*, std::size_t>  meshToBox;

    Interactor(factory::FactoryScene& s, const cell::CatalogProtos& p,
               const LookupTable& t, const std::string& a,
               Scene& ts, PerspectiveCamera& cam, Canvas& cv,
               OrbitControls& ctrl, std::shared_ptr<Object3D>& fg)
        : scene(s), protos(p), table(t), assetDir(a),
          threeScene(ts), camera(cam), canvas(cv), controls(ctrl),
          fenceGrp(fg),
          overlayGrp(Group::create()),
          pickGroup(Group::create())
    {
        threeScene.add(overlayGrp);
        threeScene.add(pickGroup);
    }

    // ── Helpers ───────────────────────────────────────────────────────────────

    static Color pickColor(int category) {
        float h = std::fmod(category * 137.508f, 360.0f) / 360.0f;
        const float s = 0.75f, l = 0.55f;
        float c = (1.0f - std::abs(2.0f * l - 1.0f)) * s;
        float x = c * (1.0f - std::abs(std::fmod(h * 6.0f, 2.0f) - 1.0f));
        float m = l - c * 0.5f;
        float r = m, g = m, b = m;
        switch (static_cast<int>(h * 6.0f) % 6) {
            case 0: r += c; g += x;           break;
            case 1: r += x; g += c;           break;
            case 2:         g += c; b += x;   break;
            case 3:         g += x; b += c;   break;
            case 4: r += x;         b += c;   break;
            default: r += c;        b += x;   break;
        }
        return Color(r, g, b);
    }

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

    // Far corner of the unique adjacent edge at `node`, excluding `draggedEdge`.
    // Returns nullopt if zero or multiple adjacent edges found (T-junction fallback).
    std::optional<std::pair<float,float>> findAdjacentCorner(
            entt::entity node, entt::entity draggedEdge) const
    {
        auto& reg = scene.registry();
        std::optional<std::pair<float,float>> result;
        bool multiple = false;
        reg.view<factory::EdgeComponent>().each(
            [&](entt::entity e, const factory::EdgeComponent& ec) {
                if (e == draggedEdge || multiple) return;
                if (ec.node_a == node || ec.node_b == node) {
                    if (result.has_value()) { multiple = true; return; }
                    entt::entity far = (ec.node_a == node) ? ec.node_b : ec.node_a;
                    const auto& fp = reg.get<factory::PoseComponent>(far);
                    result = { fp.position.x * 0.001f, fp.position.y * 0.001f };
                }
            });
        return multiple ? std::nullopt : result;
    }

    // Compute new node positions for an edge drag given the perpendicular displacement dp (metres).
    struct NewPos { float naX, naZ, nbX, nbZ; };
    NewPos computeEdgeDrag(float dp) const {
        float naX0 = drag.na_x * 0.001f, naZ0 = drag.na_z * 0.001f;
        float nbX0 = drag.nb_x * 0.001f, nbZ0 = drag.nb_z * 0.001f;

        if (!drag.anglePreserving) {
            return { naX0 + dp * drag.perpX, naZ0 + dp * drag.perpZ,
                     nbX0 + dp * drag.perpX, nbZ0 + dp * drag.perpZ };
        }

        // Slide each endpoint along the extension of its adjacent edge direction.
        // Finds where the displaced drag line intersects the adjacent edge's ray.
        auto slide = [&](float nx0, float nz0, float cx, float cz, bool has)
                -> std::pair<float,float>
        {
            if (!has) return { nx0 + dp * drag.perpX, nz0 + dp * drag.perpZ };
            float adx = nx0 - cx, adz = nz0 - cz;
            float alen = std::hypot(adx, adz);
            if (alen < 1e-6f) return { nx0 + dp * drag.perpX, nz0 + dp * drag.perpZ };
            float ax = adx / alen, az = adz / alen;
            float ox = nx0 + dp * drag.perpX, oz = nz0 + dp * drag.perpZ;
            // Solve: C + t*adj = O + s*edgeDir  →  det = edgeDirX*az - edgeDirZ*ax
            float det = drag.edgeDirX * az - drag.edgeDirZ * ax;
            if (std::abs(det) < 1e-6f) return { ox, oz };   // parallel: fall back
            float t = (drag.edgeDirX * (oz - cz) - drag.edgeDirZ * (ox - cx)) / det;
            return { cx + t * ax, cz + t * az };
        };

        auto [naX, naZ] = slide(naX0, naZ0, drag.cornerAX, drag.cornerAZ, drag.hasCornerA);
        auto [nbX, nbZ] = slide(nbX0, nbZ0, drag.cornerBX, drag.cornerBZ, drag.hasCornerB);
        return { naX, naZ, nbX, nbZ };
    }

    void buildPickBoxes() {
        while (!pickGroup->children.empty())
            pickGroup->remove(*pickGroup->children.front());
        pickBoxes.clear();
        meshToBox.clear();

        const float fh = protos.edge_height_mm * 0.001f;
        auto& reg = scene.registry();

        reg.view<factory::EdgeComponent, factory::PoseComponent>().each(
            [&](entt::entity e, const factory::EdgeComponent& ec, const factory::PoseComponent&) {
                const auto& pa = reg.get<factory::PoseComponent>(ec.node_a);
                const auto& pb = reg.get<factory::PoseComponent>(ec.node_b);
                float ax = pa.position.x * 0.001f, az = pa.position.y * 0.001f;
                float bx = pb.position.x * 0.001f, bz = pb.position.y * 0.001f;
                float len = std::max(0.01f, std::hypot(bx - ax, bz - az));
                float yaw = std::atan2(bz - az, bx - ax);

                auto mat = MeshStandardMaterial::create();
                mat->color = pickColor(0); mat->transparent = true; mat->opacity = 0.35f;
                auto box = Mesh::create(BoxGeometry::create(len, fh, 0.08f), mat);
                box->position.set((ax + bx) * 0.5f, fh * 0.5f, (az + bz) * 0.5f);
                box->rotation.y    = -yaw;
                box->castShadow    = box->receiveShadow = false;
                box->visible       = pickBoxesVisible;

                std::size_t idx = pickBoxes.size();
                pickBoxes.push_back({ e, 1, Drag::Type::Edge, box });
                meshToBox[box.get()] = idx;
                pickGroup->add(box);
            });

        reg.view<factory::DeclaredOpeningComponent, factory::PoseComponent>().each(
            [&](entt::entity e, const factory::DeclaredOpeningComponent& oc,
                const factory::PoseComponent& op) {
                if (oc.parent_edge == entt::null) return;
                const auto& ep  = reg.get<factory::PoseComponent>(oc.parent_edge);
                const auto& ec  = reg.get<factory::EdgeComponent>(oc.parent_edge);
                const auto& pa  = reg.get<factory::PoseComponent>(ec.node_a);
                const auto& pb  = reg.get<factory::PoseComponent>(ec.node_b);
                float dx = pb.position.x - pa.position.x;
                float dz = pb.position.y - pa.position.y;
                float elen = std::hypot(dx, dz);
                if (elen < 0.001f) return;
                float lx = op.position.x * 0.001f;
                float ox = ep.position.x * 0.001f + lx * (dx / elen);
                float oz = ep.position.y * 0.001f + lx * (dz / elen);
                float yaw = std::atan2(dz, dx);

                auto mat = MeshStandardMaterial::create();
                mat->color = pickColor(1); mat->transparent = true; mat->opacity = 0.4f;
                auto box = Mesh::create(BoxGeometry::create(oc.width_mm * 0.001f, fh, 0.10f), mat);
                box->position.set(ox, fh * 0.5f, oz);
                box->rotation.y    = -yaw;
                box->castShadow    = box->receiveShadow = false;
                box->visible       = pickBoxesVisible;

                std::size_t idx = pickBoxes.size();
                pickBoxes.push_back({ e, 0, Drag::Type::Opening, box });
                meshToBox[box.get()] = idx;
                pickGroup->add(box);
            });
    }

    void togglePickBoxes() {
        pickBoxesVisible = !pickBoxesVisible;
        for (auto& pb : pickBoxes) pb.mesh->visible = pickBoxesVisible;
    }

    std::shared_ptr<Mesh> makeWallBox(float ax, float az, float bx, float bz,
                                      const Color& col, float opacity = 0.55f) {
        float dx = bx - ax, dz = bz - az;
        float fh = protos.edge_height_mm * 0.001f;
        auto mat = MeshStandardMaterial::create();
        mat->color = col; mat->transparent = true; mat->opacity = opacity;
        auto box = Mesh::create(BoxGeometry::create(std::max(0.01f, std::hypot(dx, dz)), fh, 0.06f), mat);
        box->position.set((ax + bx) * 0.5f, fh * 0.5f, (az + bz) * 0.5f);
        box->rotation.y = -std::atan2(dz, dx);
        return box;
    }

    void rebuildFence() {
        threeScene.remove(*fenceGrp);
        fenceGrp = render::buildScene(scene, protos);
        fenceGrp->traverse([](Object3D& o) { o.castShadow = o.receiveShadow = true; });
        threeScene.add(fenceGrp);
        buildPickBoxes();
    }

    // ── Overlay ───────────────────────────────────────────────────────────────

    void updateOverlay(float wx, float wz) {
        overlayGrp->clear();
        auto& reg = scene.registry();

        if (drag.type == Drag::Type::Edge) {
            float dp = (wx - drag.startX) * drag.perpX + (wz - drag.startZ) * drag.perpZ;
            auto [naX, naZ, nbX, nbZ] = computeEdgeDrag(dp);

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
                    float fx = fp.position.x * 0.001f, fz = fp.position.y * 0.001f;
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
            float da  = (wx - drag.startX) * drag.edgeDirX + (wz - drag.startZ) * drag.edgeDirZ;
            float lx  = drag.openingLocalX * 0.001f + da;
            float ox  = ep.position.x * 0.001f + lx * (dx / len);
            float oz  = ep.position.y * 0.001f + lx * (dz / len);
            float oh  = protos.edge_height_mm * 0.001f;
            auto mat  = MeshStandardMaterial::create();
            mat->color = Color(0xffa050); mat->transparent = true; mat->opacity = 0.7f;
            auto box = Mesh::create(BoxGeometry::create(oc.width_mm * 0.001f, oh, 0.07f), mat);
            box->position.set(ox, oh * 0.5f, oz);
            box->rotation.y = -std::atan2(dz, dx);
            overlayGrp->add(box);
        }
    }

    // ── Mouse events ──────────────────────────────────────────────────────────

    void onMouseDown(int button, const Vector2& pos) override {
        if (button != 0) return;

        auto sz = canvas.size();
        Vector2 ndc{ (pos.x / sz.width())  * 2.f - 1.f,
                    -(pos.y / sz.height()) * 2.f + 1.f };
        raycaster.setFromCamera(ndc, camera);

        std::vector<Object3D*> targets;
        targets.reserve(pickBoxes.size());
        for (auto& pb : pickBoxes) targets.push_back(pb.mesh.get());

        auto hits = raycaster.intersectObjects(targets, false);
        if (hits.empty()) return;

        // Lowest priority wins; ties resolved by distance (hits sorted nearest-first).
        int bestPri = std::numeric_limits<int>::max();
        const Intersection* best = nullptr;
        for (const auto& h : hits) {
            auto it = meshToBox.find(h.object);
            if (it == meshToBox.end()) continue;
            int pri = pickBoxes[it->second].priority;
            if (pri < bestPri) { bestPri = pri; best = &h; }
        }
        if (!best) return;

        const PickBox& pb = pickBoxes[meshToBox.at(best->object)];

        auto& ray = raycaster.ray;
        if (std::abs(ray.direction.y) < 1e-6f) return;
        float tFloor = -ray.origin.y / ray.direction.y;
        if (tFloor < 0.f) return;
        Vector3 fp; ray.at(tFloor, fp);
        float wx = fp.x, wz = fp.z;

        auto& reg = scene.registry();

        if (pb.dragType == Drag::Type::Opening) {
            const auto& oc = reg.get<factory::DeclaredOpeningComponent>(pb.entity);
            const auto& ec = reg.get<factory::EdgeComponent>(oc.parent_edge);
            const auto& pa = reg.get<factory::PoseComponent>(ec.node_a);
            const auto& pb2 = reg.get<factory::PoseComponent>(ec.node_b);
            float dx = pb2.position.x - pa.position.x;
            float dz = pb2.position.y - pa.position.y;
            float len = std::hypot(dx, dz);
            drag.type          = Drag::Type::Opening;
            drag.entity        = pb.entity;
            drag.startX        = wx;  drag.startZ       = wz;
            drag.edgeDirX      = dx / len; drag.edgeDirZ  = dz / len;
            drag.openingLocalX = reg.get<factory::PoseComponent>(pb.entity).position.x;
            fenceGrp->visible  = false;
            controls.enabled   = false;
            return;
        }

        if (pb.dragType == Drag::Type::Edge) {
            const auto& ec  = reg.get<factory::EdgeComponent>(pb.entity);
            const auto& pa  = reg.get<factory::PoseComponent>(ec.node_a);
            const auto& pb2 = reg.get<factory::PoseComponent>(ec.node_b);
            float dx = pb2.position.x - pa.position.x;
            float dz = pb2.position.y - pa.position.y;
            float len = std::hypot(dx, dz);
            drag.type    = Drag::Type::Edge;
            drag.entity  = pb.entity;
            drag.startX  = wx;  drag.startZ = wz;
            drag.perpX   = -dz / len; drag.perpZ = dx / len;
            drag.edgeDirX = dx / len; drag.edgeDirZ = dz / len;
            drag.nodeA   = ec.node_a;  drag.nodeB = ec.node_b;
            drag.na_x    = pa.position.x; drag.na_z = pa.position.y;
            drag.nb_x    = pb2.position.x; drag.nb_z = pb2.position.y;

            drag.anglePreserving = shiftDown;
            if (drag.anglePreserving) {
                auto ca = findAdjacentCorner(ec.node_a, pb.entity);
                auto cb = findAdjacentCorner(ec.node_b, pb.entity);
                drag.hasCornerA = ca.has_value();
                drag.hasCornerB = cb.has_value();
                if (ca) { drag.cornerAX = ca->first; drag.cornerAZ = ca->second; }
                if (cb) { drag.cornerBX = cb->first; drag.cornerBZ = cb->second; }
            }

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
                float dp_m = (wx - drag.startX) * drag.perpX + (wz - drag.startZ) * drag.perpZ;
                auto [naX, naZ, nbX, nbZ] = computeEdgeDrag(dp_m);
                auto& pa = reg.get<factory::PoseComponent>(drag.nodeA);
                auto& pb = reg.get<factory::PoseComponent>(drag.nodeB);
                pa.position.x = naX * 1000.f; pa.position.y = naZ * 1000.f;
                pb.position.x = nbX * 1000.f; pb.position.y = nbZ * 1000.f;
                scene.solve(table, assetDir);

            } else if (drag.type == Drag::Type::Opening) {
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

    // ── Key events ────────────────────────────────────────────────────────────

    void onKeyPressed(KeyEvent evt) override {
        if (evt.key == Key::LEFT_SHIFT) shiftDown = true;
        if (evt.key == Key::B) togglePickBoxes();
    }

    void onKeyReleased(KeyEvent evt) override {
        if (evt.key == Key::LEFT_SHIFT) shiftDown = false;
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
    interactor.buildPickBoxes();
    ss.canvas.addMouseListener(interactor);
    ss.canvas.addKeyListener(interactor);

    ss.canvas.animate([&] {
        ss.scene->fog = FogExp2(Color(0xfff5e8), 0.06f);
        ss.renderer.render(*ss.scene, *ss.camera);
    });

    return 0;
}
