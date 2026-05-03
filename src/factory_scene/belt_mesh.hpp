#pragma once
#include <cmath>
#include <memory>
#include <vector>

#include <threepp/geometries/BoxGeometry.hpp>
#include <threepp/geometries/CylinderGeometry.hpp>
#include <threepp/geometries/PlaneGeometry.hpp>
#include <threepp/materials/MeshStandardMaterial.hpp>
#include <threepp/objects/Group.hpp>
#include <threepp/objects/Mesh.hpp>
#include <threepp/textures/DataTexture.hpp>

// Procedural belt mesh generator. Pure function — no ECS access.
//
// Local coordinate space (Y-up, threepp convention):
//   +X = travel direction (from end_a to end_b)
//   +Y = up
//   +Z = across the belt (left side when looking in travel direction)
//   Origin = center of belt footprint at floor level (Y = 0, base of legs)
//
// Belt surface sits at Y = belt_surface_height_mm * 0.001 metres.

namespace belt {

struct CatalogData {
    int frame_height_mm        = 50;
    int belt_surface_offset_mm = 3;    // top of frame to belt surface
    int frame_overhang_mm      = 20;   // aluminium overhang each side beyond belt width
    int max_leg_span_mm        = 1500;
    int leg_profile_mm         = 40;
    int motor_overhang_mm      = 180;
    int motor_height_mm        = 120;
    int tile_pitch_mm          = 200;

    threepp::Color belt_color    = threepp::Color(0x3a3a3a);
    float          belt_rough    = 0.7f;
    threepp::Color frame_color   = threepp::Color(0xcccccc);
    float          frame_rough   = 0.3f;
    float          frame_metal   = 0.7f;
    threepp::Color legs_color    = threepp::Color(0xbbbbbb);
    float          legs_rough    = 0.4f;
    float          legs_metal    = 0.6f;
};

static const CatalogData kGenericCatalog{};

// Striped belt texture: dark grey cleats perpendicular to travel direction.
// Tile pitch matches CatalogData::tile_pitch_mm when repeat is set accordingly.
inline std::shared_ptr<threepp::DataTexture> makeBeltTexture() {
    using namespace threepp;
    constexpr int W = 64, H = 16;
    std::vector<unsigned char> px(W * H * 4);
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            bool cleat = (x % 16 < 3);
            unsigned char c = cleat ? 80 : 48;
            int i = (y * W + x) * 4;
            px[i+0] = px[i+1] = px[i+2] = c;
            px[i+3] = 255;
        }
    }
    auto tex = DataTexture::create(ImageData(std::move(px)), W, H);
    tex->wrapS = TextureWrapping::Repeat;
    tex->wrapT = TextureWrapping::Repeat;
    tex->needsUpdate();
    return tex;
}

struct BeltMesh {
    std::shared_ptr<threepp::Object3D>             object;
    std::shared_ptr<threepp::MeshStandardMaterial> belt_mat;  // animate UV offset here
};

// width_mm, length_mm, surface_height_mm: belt dimensions (mm).
// tex: shared belt texture for UV animation (may be nullptr for untextured belt).
inline BeltMesh buildBeltMesh(
    int width_mm,
    int length_mm,
    int surface_height_mm,
    const CatalogData& cat = kGenericCatalog,
    std::shared_ptr<threepp::DataTexture> tex = nullptr)
{
    using namespace threepp;
    using math::PI;

    auto root = Group::create();

    const float len    = length_mm          * 0.001f;
    const float w      = width_mm           * 0.001f;
    const float sh     = surface_height_mm  * 0.001f;
    const float fh     = cat.frame_height_mm * 0.001f;
    const float fover  = cat.frame_overhang_mm * 0.001f;
    const float fw     = w + 2.0f * fover;
    const float fso    = cat.belt_surface_offset_mm * 0.001f;
    const float lp     = cat.leg_profile_mm  * 0.001f;
    const float mo     = cat.motor_overhang_mm * 0.001f;
    const float mh     = cat.motor_height_mm   * 0.001f;

    // ── Frame body ────────────────────────────────────────────────────────────
    {
        auto geo = BoxGeometry::create(len, fh, fw);
        auto mat = MeshStandardMaterial::create();
        mat->color     = cat.frame_color;
        mat->roughness = cat.frame_rough;
        mat->metalness = cat.frame_metal;
        auto mesh = Mesh::create(geo, mat);
        mesh->position.set(0.f, sh - fh * 0.5f, 0.f);
        mesh->castShadow    = true;
        mesh->receiveShadow = true;
        root->add(mesh);
    }

    // ── Belt surface ──────────────────────────────────────────────────────────
    auto belt_mat = MeshStandardMaterial::create();
    belt_mat->color     = cat.belt_color;
    belt_mat->roughness = cat.belt_rough;
    belt_mat->metalness = 0.0f;
    if (tex) {
        float reps_u = static_cast<float>(length_mm) / cat.tile_pitch_mm;
        float reps_v = static_cast<float>(width_mm)  / cat.tile_pitch_mm;
        tex->repeat.set(reps_u, reps_v);
        belt_mat->map = tex;
    }
    {
        auto geo  = PlaneGeometry::create(len, w);
        auto mesh = Mesh::create(geo, belt_mat);
        mesh->rotation.x    = -PI / 2.0f;  // lie flat in XZ plane
        mesh->position.set(0.f, sh + fso * 0.001f, 0.f);
        mesh->receiveShadow = true;
        root->add(mesh);
    }

    // ── End rollers (×2) ─────────────────────────────────────────────────────
    {
        float radius = cat.frame_height_mm * 0.35f * 0.001f;
        auto geo = CylinderGeometry::create(radius, radius, w);
        auto mat = MeshStandardMaterial::create();
        mat->color     = cat.frame_color;
        mat->roughness = cat.frame_rough;
        mat->metalness = cat.frame_metal;
        for (int side : {-1, 1}) {
            auto mesh = Mesh::create(geo, mat);
            mesh->rotation.x = PI / 2.0f;  // align with Z axis
            mesh->position.set(side * len * 0.5f, sh - radius, 0.f);
            mesh->castShadow    = true;
            mesh->receiveShadow = true;
            root->add(mesh);
        }
    }

    // ── Drive unit (at end_b = +X end) ───────────────────────────────────────
    {
        auto geo = BoxGeometry::create(mo, mh, fw);
        auto mat = MeshStandardMaterial::create();
        mat->color     = cat.frame_color;
        mat->roughness = cat.frame_rough;
        mat->metalness = cat.frame_metal;
        auto mesh = Mesh::create(geo, mat);
        mesh->position.set(len * 0.5f + mo * 0.5f, sh - fh * 0.5f, 0.f);
        mesh->castShadow    = true;
        mesh->receiveShadow = true;
        root->add(mesh);
    }

    // ── Legs ─────────────────────────────────────────────────────────────────
    {
        float leg_h = sh - fh - fso;  // floor to underside of frame
        if (leg_h > 0.0f) {
            int n_legs = static_cast<int>(
                std::ceil(static_cast<float>(length_mm) / cat.max_leg_span_mm)) + 1;
            if (n_legs < 2) n_legs = 2;

            auto geo = BoxGeometry::create(lp, leg_h, lp);
            auto mat = MeshStandardMaterial::create();
            mat->color     = cat.legs_color;
            mat->roughness = cat.legs_rough;
            mat->metalness = cat.legs_metal;

            for (int k = 0; k < n_legs; ++k) {
                float x = -len * 0.5f + k * len / (n_legs - 1);
                auto mesh = Mesh::create(geo, mat);
                mesh->position.set(x, leg_h * 0.5f, 0.f);
                mesh->castShadow    = true;
                mesh->receiveShadow = true;
                root->add(mesh);
            }
        }
    }

    return {root, belt_mat};
}

}  // namespace belt
