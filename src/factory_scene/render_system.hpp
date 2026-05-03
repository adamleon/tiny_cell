#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

#include <entt/entt.hpp>
#include <threepp/geometries/BoxGeometry.hpp>
#include <threepp/geometries/PlaneGeometry.hpp>
#include <threepp/materials/MeshPhysicalMaterial.hpp>
#include <threepp/materials/MeshStandardMaterial.hpp>
#include <threepp/objects/Group.hpp>
#include <threepp/objects/Mesh.hpp>
#include <threepp/textures/DataTexture.hpp>

#include "factory_scene.hpp"
#include "cell/fence_catalog.hpp"

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

// 64×64 RGBA DataTexture with 45° yellow/black hazard stripes.
inline std::shared_ptr<threepp::DataTexture> makeHazardTexture() {
    using namespace threepp;
    constexpr int W = 64, H = 64;
    std::vector<unsigned char> px(W * H * 4);
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            bool yellow = (((x + y) / 32) % 2) == 0;
            int i = (y * W + x) * 4;
            px[i+0] = yellow ? 240 : 25;
            px[i+1] = yellow ? 180 : 25;
            px[i+2] = yellow ?   0 : 25;
            px[i+3] = 255;
        }
    }
    auto tex = DataTexture::create(ImageData(std::move(px)), W, H);
    tex->wrapS = TextureWrapping::Repeat;
    tex->wrapT = TextureWrapping::Repeat;
    tex->needsUpdate();
    return tex;
}

}  // namespace detail

inline std::shared_ptr<threepp::Object3D> buildScene(
    const factory::FactoryScene& scene,
    const cell::CatalogProtos&   protos)
{
    using namespace threepp;
    const auto& reg = scene.registry();
    auto root = Group::create();

    auto hazardTex = detail::makeHazardTexture();

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

            const float pw_m = protos.post_width_mm * 0.001f;
            const float hw_m = pw_m * 0.5f;
            int total_mm = 0;
            for (const auto& s : ec.spans_mm) total_mm += detail::spanVisualMm(s);
            for (const auto& op : ops) total_mm += op.width_mm + 2 * protos.post_width_mm;

            float cursor = -total_mm * 0.0005f;
            auto grp = Group::create();

            const int n_spans = static_cast<int>(ec.spans_mm.size());
            for (int i = 0; i < n_spans; ++i) {
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
                        post->position.set(cursor + hw_m, 0.f, 0.f);
                        grp->add(post);
                        cursor += pw_m;
                    }
                }
                if (i < static_cast<int>(ops.size())) {
                    auto add_post = [&] {
                        auto post = protos.post->clone();
                        post->position.set(cursor + hw_m, 0.f, 0.f);
                        grp->add(post);
                        cursor += pw_m;
                    };

                    add_post();

                    float ow_m = ops[i].width_mm * 0.001f;
                    float oh_m = protos.edge_height_mm * 0.001f;
                    auto geo   = BoxGeometry::create(ow_m, oh_m, 0.05f);
                    auto mat = MeshPhysicalMaterial::create();
                    mat->color        = Color(0xc8a060);
                    mat->transmission = 0.75f;
                    mat->roughness    = 0.05f;
                    mat->metalness    = 0.0f;
                    mat->ior          = 1.5f;
                    auto box = Mesh::create(geo, mat);
                    box->position.set(cursor + ow_m * 0.5f, oh_m * 0.5f, 0.f);
                    grp->add(box);
                    cursor += ow_m;

                    add_post();
                }
            }

            // ── Hazard strip along this edge ──────────────────────────────────
            // Flat plane lying on the floor, spanning the full visual width of
            // the edge. UV repeat tiles the texture at ~300mm per stripe pitch.
            {
                const float stripWidth = 0.1f;   // 100 mm
                const float edgeLen    = total_mm * 0.001f;
                // Texture has 2 diagonal stripe cycles per tile; tile at 100mm
                // along the edge so each stripe is ~50mm wide.
                auto geo = PlaneGeometry::create(edgeLen, stripWidth);
                auto mat = MeshStandardMaterial::create();
                mat->map = hazardTex;
                mat->map->repeat.set(edgeLen / 0.1f, 1.0f);
                mat->roughness = 0.6f;
                mat->metalness = 0.0f;
                auto strip = Mesh::create(geo, mat);
                strip->rotation.x = -math::PI / 2.0f;
                // Local -Z is the cell exterior (consistent for all edges due
                // to winding order). Offset the strip so its inner edge sits on
                // the fence line and it extends outward.
                strip->position.set(0.0f, 0.002f, -stripWidth / 2.0f);
                strip->receiveShadow = true;
                grp->add(strip);
            }

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
