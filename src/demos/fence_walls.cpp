#include "common/scene_setup.hpp"
#include "cell/cell_layout.hpp"
#include "cell/cell_builder.hpp"
#include "cell/fence_catalog.hpp"
#include "cell/fence_solver.hpp"

#include <cassert>
#include <iostream>

using namespace threepp;

constexpr int  kNorthSouthMm = 4000;
constexpr int  kEastWestMm   = 3000;
constexpr bool kPreferOver   = true;

// Width of the dummy slab opening placed on the south edge.
constexpr int kSlabWidthMm = 500;

int main() {
    const std::string assetDir = "assets/components/fences/axelent_x-guard";

    auto catalog = cell::loadCatalog(assetDir + "/catalog.json");
    auto table   = loadTable(assetDir + "/combinations.json");

    // Standard edges (no opening)
    auto ns = solve(table, kNorthSouthMm, kPreferOver);
    auto ew = solve(table, kEastWestMm,   kPreferOver);

    // South edge: two spans flanking a slab opening.
    // Each span is solved for half of (ns.actual_mm - slab_width).
    // This works exactly when ns.actual_mm is even and the half is in the table.
    // The full solver (Layer 3) will handle general closure.
    int remaining_mm = ns.actual_mm - kSlabWidthMm;
    auto south_a = solve(table, remaining_mm / 2, kPreferOver);
    auto south_b = solve(table, remaining_mm / 2, kPreferOver);

    int south_visual_mm = south_a.actual_mm + kSlabWidthMm + south_b.actual_mm;

    std::cout << "N wall: " << ns.actual_mm     << " mm (" << ns.panels_mm.size()     << " panels)\n";
    std::cout << "S wall: " << south_visual_mm  << " mm (slab + 2 spans)\n";
    std::cout << "E/W:    " << ew.actual_mm     << " mm (" << ew.panels_mm.size()     << " panels)\n";

    // Node positions derived from solved visual lengths so corners close.
    // South and north share the same width (south_visual_mm == ns.actual_mm when
    // the half-span solve is exact; assert guards the demo assumption).
    assert(south_visual_mm == ns.actual_mm &&
           "Demo assumption: south spans + slab must equal north span exactly. "
           "Adjust kNorthSouthMm or kSlabWidthMm if this fires.");

    float hw = ns.actual_mm  * 0.0005f;
    float hd = ew.actual_mm  * 0.0005f;

    cell::CellLayout layout;
    int sw = layout.addNode(-hw, -hd);
    int se = layout.addNode(+hw, -hd);
    int ne = layout.addNode(+hw, +hd);
    int nw = layout.addNode(-hw, +hd);

    // South edge: two spans + one slab opening in the middle.
    // The slab's desired_position is the centre of the edge in mm from the west end.
    int slab_pos_mm = south_a.actual_mm + kSlabWidthMm / 2;
    auto slab = std::make_shared<cell::DeclaredOpening>(slab_pos_mm, kSlabWidthMm);
    layout.addEdge(sw, se,
        {south_a.panels_mm, south_b.panels_mm},
        {slab});

    layout.addEdge(se, ne, ew.panels_mm);   // east  — no opening
    layout.addEdge(ne, nw, ns.panels_mm);   // north — no opening
    layout.addEdge(nw, sw, ew.panels_mm);   // west  — no opening

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
