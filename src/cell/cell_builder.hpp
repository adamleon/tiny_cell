#pragma once
#include <cmath>
#include <memory>
#include <threepp/objects/Group.hpp>
#include "cell_layout.hpp"
#include "fence_catalog.hpp"

// Builds threepp geometry from a CellLayout + CatalogProtos.
//
// Key contract:
//   buildEdgeGroup  — panels + inter-panel posts only, NO end posts, centered at origin along X
//   buildNodeGroup  — geometry for one node (post, rounded corner, etc.), centered at origin
//   buildCell       — positions and assembles all edges and nodes into one root group
//
// Edge groups are rotated by -atan2(dz, dx) around Y so local X aligns with the
// node-to-node direction, then translated to the midpoint. This handles any angle.

namespace cell {

// Panels + inter-panel posts, no end posts, spanning [-visual_half, +visual_half] on X.
inline std::shared_ptr<threepp::Object3D> buildEdgeGroup(
    const std::vector<int>& panels_mm,
    const CatalogProtos& protos)
{
    using namespace threepp;
    const int n = static_cast<int>(panels_mm.size());

    int visual_mm = -50;
    for (int w : panels_mm) visual_mm += w + 50;

    auto grp    = Group::create();
    float cursor = visual_mm * -0.0005f;

    for (int i = 0; i < n; ++i) {
        int w = panels_mm[i];

        auto panel = protos.panels.at(w)->clone();
        panel->position.set(cursor + w * 0.0005f, 0.0f, 0.0f);
        grp->add(panel);
        cursor += w * 0.001f;

        if (i < n - 1) {
            auto post = protos.post->clone();
            post->position.set(cursor + 0.025f, 0.0f, 0.0f);
            grp->add(post);
            cursor += 0.050f;
        }
    }
    return grp;
}

// Geometry for a single node, centred at the local origin.
// Add new cases here as NodeType grows — no other code needs to change.
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
        // Future:
        // case NodeType::RoundedCorner:
        //     grp->add(protos.roundedCorner->clone());
        //     break;
    }
    return grp;
}

// Assembles the full cell. Each edge is placed at the midpoint between its two nodes
// and rotated so local X aligns with the node-to-node direction. Each node gets its
// own geometry at its exact position. Result is a single root group at the world origin.
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

        auto eg = buildEdgeGroup(edge.panels_mm, protos);
        eg->position.set(midX, 0.0f, midZ);
        eg->rotation.y = -std::atan2(dz, dx);
        root->add(eg);
    }

    for (const auto& node : layout.nodes) {
        auto ng = buildNodeGroup(node.type, protos);
        ng->position.set(node.x, 0.0f, node.z);
        root->add(ng);
    }

    return root;
}

} // namespace cell
