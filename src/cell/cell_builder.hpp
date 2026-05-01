#pragma once
#include <cmath>
#include <memory>
#include <threepp/geometries/BoxGeometry.hpp>
#include <threepp/materials/MeshPhongMaterial.hpp>
#include <threepp/objects/Group.hpp>
#include <threepp/objects/Mesh.hpp>
#include "cell_layout.hpp"
#include "fence_catalog.hpp"

// Builds threepp geometry from a CellLayout + CatalogProtos.
//
// Key contract:
//   buildEdgeGroup  — panels + inter-panel posts + opening boxes, NO end posts,
//                     centered at origin along X
//   buildNodeGroup  — geometry for one node (post, rounded corner, etc.)
//   buildCell       — positions and assembles all edges and nodes into one root group
//
// Edge groups are rotated by -atan2(dz, dx) around Y so local X aligns with the
// node-to-node direction, then translated to the midpoint.

namespace cell {

// Renders one fence span (panels + inter-panel posts), advancing cursor.
// Returns visual width consumed in metres.
inline float buildSpanInto(
    threepp::Group& grp,
    const std::vector<int>& span,
    const CatalogProtos& protos,
    float cursor)
{
    const int n = static_cast<int>(span.size());
    for (int i = 0; i < n; ++i) {
        int w = span[i];
        auto panel = protos.panels.at(w)->clone();
        panel->position.set(cursor + w * 0.0005f, 0.f, 0.f);
        grp.add(panel);
        cursor += w * 0.001f;
        if (i < n - 1) {
            auto post = protos.post->clone();
            post->position.set(cursor + 0.025f, 0.f, 0.f);
            grp.add(post);
            cursor += 0.050f;
        }
    }
    return cursor;
}

// Computes the visual width (mm) of a single fence span (no end posts).
inline int spanVisualMm(const std::vector<int>& span) {
    if (span.empty()) return 0;
    int total = -50;
    for (int w : span) total += w + 50;
    return total;
}

// Computes total visual width (mm) of an edge: all spans + all openings.
inline int edgeVisualMm(const Edge& edge) {
    int total = 0;
    for (const auto& s : edge.spans_mm) total += spanVisualMm(s);
    for (const auto& op : edge.user_openings) total += op->getWidth();
    return total;
}

// Panels + inter-panel posts + opening boxes; NO end posts; centered on X.
// Opening boxes span the full edge height and a nominal 50mm depth.
inline std::shared_ptr<threepp::Object3D> buildEdgeGroup(
    const Edge& edge,
    const CatalogProtos& protos)
{
    using namespace threepp;

    const int total_mm  = edgeVisualMm(edge);
    const float half_m  = total_mm * 0.0005f;
    float cursor        = -half_m;

    auto grp = Group::create();

    for (size_t i = 0; i < edge.spans_mm.size(); ++i) {
        cursor = buildSpanInto(*grp, edge.spans_mm[i], protos, cursor);

        if (i < edge.user_openings.size()) {
            const auto& op    = edge.user_openings[i];
            const float ow_m  = op->getWidth() * 0.001f;
            const float oh_m  = protos.edge_height_mm * 0.001f;

            auto geo = BoxGeometry::create(ow_m, oh_m, 0.05f);
            auto mat = MeshPhongMaterial::create();
            mat->color = Color(0x4a7fc1);
            mat->transparent = true;
            mat->opacity = 0.85f;
            auto box = Mesh::create(geo, mat);
            box->position.set(cursor + ow_m * 0.5f, oh_m * 0.5f, 0.f);
            grp->add(box);
            cursor += ow_m;
        }
    }
    return grp;
}

// Geometry for a single node, centred at the local origin.
inline std::shared_ptr<threepp::Object3D> buildNodeGroup(
    NodeType type,
    const CatalogProtos& protos)
{
    using namespace threepp;
    auto grp = Group::create();
    switch (type) {
        case NodeType::Post:
            grp->add(protos.post->clone());
            break;
    }
    return grp;
}

// Assembles the full cell. Each edge is placed at the midpoint between its two nodes
// and rotated so local X aligns with the node-to-node direction. Each node gets its
// own geometry at its exact position.
inline std::shared_ptr<threepp::Object3D> buildCell(
    const CellLayout& layout,
    const CatalogProtos& protos)
{
    using namespace threepp;
    auto root = Group::create();

    for (const auto& edge : layout.edges) {
        const auto& na = layout.nodes[edge.nodeA];
        const auto& nb = layout.nodes[edge.nodeB];

        float dx   = nb.x - na.x;
        float dz   = nb.z - na.z;
        float midX = (na.x + nb.x) * 0.5f;
        float midZ = (na.z + nb.z) * 0.5f;

        auto eg = buildEdgeGroup(edge, protos);
        eg->position.set(midX, 0.f, midZ);
        eg->rotation.y = -std::atan2(dz, dx);
        root->add(eg);
    }

    for (const auto& node : layout.nodes) {
        auto ng = buildNodeGroup(node.type, protos);
        ng->position.set(node.x, 0.f, node.z);
        root->add(ng);
    }

    return root;
}

} // namespace cell
