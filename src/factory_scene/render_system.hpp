#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

#include <entt/entt.hpp>
#include <threepp/geometries/BoxGeometry.hpp>
#include <threepp/materials/MeshPhongMaterial.hpp>
#include <threepp/objects/Group.hpp>
#include <threepp/objects/Mesh.hpp>

#include "factory_scene.hpp"
#include "cell/fence_catalog.hpp"

// Temporary combined VisualUpdateSystem + RenderSystem.
// Will split once VisualComponent is introduced: VisualUpdateSystem writes asset keys,
// RenderSystem reads VisualComponent + PoseComponent only.
//
// Coordinate conversion: ECS is Z-up (pos.x = world X, pos.y = world Z floor, pos.z = height).
// threepp is Y-up: threepp(x, y, z) = ECS(pos.x, pos.z, pos.y) in metres.

namespace render {

namespace detail {

inline int spanVisualMm(const std::vector<int>& span) {
    if (span.empty()) return 0;
    int total = -50;
    for (int w : span) total += w + 50;
    return total;
}

}  // namespace detail

inline std::shared_ptr<threepp::Object3D> buildScene(
    const factory::FactoryScene& scene,
    const cell::CatalogProtos&   protos)
{
    using namespace threepp;
    const auto& reg = scene.registry();
    auto root = Group::create();

    // ── Edges ─────────────────────────────────────────────────────────────────
    reg.view<factory::EdgeComponent, factory::PoseComponent>().each(
        [&](entt::entity edge_e,
            const factory::EdgeComponent& ec,
            const factory::PoseComponent& ep)
        {
            // Collect openings for this edge, sorted left-to-right by local X.
            struct OpInfo { float local_x; int width_mm; };
            std::vector<OpInfo> ops;
            reg.view<factory::DeclaredOpeningComponent,
                     factory::PoseComponent>().each(
                [&](entt::entity,
                    const factory::DeclaredOpeningComponent& oc,
                    const factory::PoseComponent& op)
                {
                    if (oc.parent_edge == edge_e)
                        ops.push_back({op.position.x, oc.width_mm});
                });
            std::sort(ops.begin(), ops.end(),
                      [](const OpInfo& a, const OpInfo& b) { return a.local_x < b.local_x; });

            // Total visual width (mm): all spans + all openings.
            int total_mm = 0;
            for (const auto& s : ec.spans_mm) total_mm += detail::spanVisualMm(s);
            for (const auto& op : ops) total_mm += op.width_mm;

            float cursor = -total_mm * 0.0005f;  // start at -half_m in edge-local X
            auto grp = Group::create();

            const int n_spans = static_cast<int>(ec.spans_mm.size());
            for (int i = 0; i < n_spans; ++i) {
                // Panels + inter-panel posts for span[i].
                const auto& span = ec.spans_mm[i];
                const int   np   = static_cast<int>(span.size());
                for (int j = 0; j < np; ++j) {
                    float w_m = span[j] * 0.001f;
                    auto panel = protos.panels.at(span[j])->clone();
                    panel->position.set(cursor + w_m * 0.5f, 0.f, 0.f);
                    grp->add(panel);
                    cursor += w_m;
                    if (j < np - 1) {
                        auto post = protos.post->clone();
                        post->position.set(cursor + 0.025f, 0.f, 0.f);
                        grp->add(post);
                        cursor += 0.05f;
                    }
                }
                // Opening between span[i] and span[i+1].
                if (i < static_cast<int>(ops.size())) {
                    float ow_m = ops[i].width_mm * 0.001f;
                    float oh_m = protos.edge_height_mm * 0.001f;
                    auto geo   = BoxGeometry::create(ow_m, oh_m, 0.05f);
                    auto mat   = MeshPhongMaterial::create();
                    mat->color       = Color(0x4a7fc1);
                    mat->transparent = true;
                    mat->opacity     = 0.85f;
                    auto box = Mesh::create(geo, mat);
                    box->position.set(cursor + ow_m * 0.5f, oh_m * 0.5f, 0.f);
                    grp->add(box);
                    cursor += ow_m;
                }
            }

            // Position: Z-up ECS → Y-up threepp (swap .y and .z, scale mm → m).
            // Recompute yaw from node poses — cleaner than extracting from quaternion.
            const auto& pa  = reg.get<factory::PoseComponent>(ec.node_a);
            const auto& pb  = reg.get<factory::PoseComponent>(ec.node_b);
            float yaw = std::atan2(pb.position.y - pa.position.y,
                                   pb.position.x - pa.position.x);
            grp->position.set(ep.position.x * 0.001f, 0.f, ep.position.y * 0.001f);
            grp->rotation.y = -yaw;
            root->add(grp);
        });

    // ── Nodes (corner posts) ──────────────────────────────────────────────────
    reg.view<factory::NodeComponent, factory::PoseComponent>().each(
        [&](entt::entity,
            const factory::NodeComponent&,
            const factory::PoseComponent& np)
        {
            auto ng = Group::create();
            ng->add(protos.post->clone());
            ng->position.set(np.position.x * 0.001f, 0.f, np.position.y * 0.001f);
            root->add(ng);
        });

    return root;
}

}  // namespace render
