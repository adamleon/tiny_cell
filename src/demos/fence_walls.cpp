#include "common/scene_setup.hpp"
#include "cell/cell_layout.hpp"
#include "cell/cell_builder.hpp"
#include "cell/fence_catalog.hpp"
#include "cell/fence_solver.hpp"

#include <iostream>

using namespace threepp;

constexpr int  kNorthSouthMm = 4000;
constexpr int  kEastWestMm   = 3000;
constexpr bool kPreferOver   = true;

int main() {
    const std::string assetDir = "assets/components/fences/axelent_x-guard";

    auto catalog = cell::loadCatalog(assetDir + "/catalog.json");
    auto table   = loadTable(assetDir + "/combinations.json");

    auto ns = solve(table, kNorthSouthMm, kPreferOver);
    auto ew = solve(table, kEastWestMm,   kPreferOver);

    std::cout << "N/S wall: " << ns.actual_mm << " mm (" << ns.panels_mm.size() << " panels)\n";
    std::cout << "E/W wall: " << ew.actual_mm << " mm (" << ew.panels_mm.size() << " panels)\n";

    // Node positions are derived from the solved actual lengths so corners close exactly.
    float hw = ns.actual_mm * 0.0005f;   // half N/S span (metres)
    float hd = ew.actual_mm * 0.0005f;   // half E/W span (metres)

    cell::CellLayout layout;
    int sw = layout.addNode(-hw, -hd);
    int se = layout.addNode(+hw, -hd);
    int ne = layout.addNode(+hw, +hd);
    int nw = layout.addNode(-hw, +hd);

    layout.addEdge(sw, se, ns.panels_mm);   // south
    layout.addEdge(se, ne, ew.panels_mm);   // east
    layout.addEdge(ne, nw, ns.panels_mm);   // north
    layout.addEdge(nw, sw, ew.panels_mm);   // west

    // ── Scene ─────────────────────────────────────────────────────────────────
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

    // Floor
    {
        auto geo = PlaneGeometry::create(ns.actual_mm * 0.001f, ew.actual_mm * 0.001f);
        auto mat = MeshPhongMaterial::create();
        mat->color = Color(0x2a2a2a);
        auto floor = Mesh::create(geo, mat);
        floor->rotation.x = -math::PI / 2.0f;
        ss.scene->add(floor);
    }

    OBJLoader loader;
    auto protos = cell::loadCatalogProtos(loader, assetDir, catalog);
    ss.scene->add(cell::buildCell(layout, protos));

    ss.canvas.animate([&] {
        ss.renderer.render(*ss.scene, *ss.camera);
    });

    return 0;
}
