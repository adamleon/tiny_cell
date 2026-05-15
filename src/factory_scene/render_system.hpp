#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <unordered_map>

#include <entt/entt.hpp>
#include <threepp/geometries/BoxGeometry.hpp>
#include <threepp/geometries/PlaneGeometry.hpp>
#include <threepp/geometries/SphereGeometry.hpp>
#include <threepp/loaders/ModelLoader.hpp>
#include <threepp/loaders/URDFLoader.hpp>
#include <threepp/materials/MeshBasicMaterial.hpp>
#include <threepp/materials/MeshPhysicalMaterial.hpp>
#include <threepp/materials/MeshStandardMaterial.hpp>
#include <threepp/math/MathUtils.hpp>
#include <threepp/objects/Group.hpp>
#include <threepp/objects/Mesh.hpp>
#include <threepp/objects/Robot.hpp>
#include <threepp/scenes/Scene.hpp>
#include <threepp/textures/DataTexture.hpp>

#include "factory_scene.hpp"
#include "ik_dls.hpp"
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
                    if (oc.parent_edge() == edge_e)
                        ops.push_back({op.position.x, oc.width_mm()});
                });
            std::sort(ops.begin(), ops.end(),
                      [](const OpInfo& a, const OpInfo& b) { return a.local_x < b.local_x; });

            const float pw_m = protos.post_width_mm * 0.001f;
            const float hw_m = pw_m * 0.5f;
            int total_mm = 0;
            for (const auto& s : ec.spans_mm()) total_mm += detail::spanVisualMm(s);
            for (const auto& op : ops) total_mm += op.width_mm + 2 * protos.post_width_mm;

            float cursor = -total_mm * 0.0005f;
            auto grp = Group::create();

            const int n_spans = static_cast<int>(ec.spans_mm().size());
            for (int i = 0; i < n_spans; ++i) {
                const auto& span = ec.spans_mm()[i];
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

            const auto& pa  = reg.get<factory::PoseComponent>(ec.node_a());
            const auto& pb  = reg.get<factory::PoseComponent>(ec.node_b());
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

// ── Magic-transport particle effects ────────────────────────────────────────
//
// Tiny glowing spheres that drift and fade out, spawned at every active
// MagicTransportComponent's world position. The MagicTransport is a visual
// placeholder; particles make its motion legible.
//
// Usage per render-loop tick:
//   magic_particles.spawn_for_active_transports(reg, *scene);
//   magic_particles.step(*scene, dt);

namespace magic {

struct Particle {
    std::shared_ptr<threepp::Mesh>              mesh;
    std::shared_ptr<threepp::MeshBasicMaterial> mat;
    threepp::Vector3                            velocity;
    float                                       lifetime_s;
    float                                       total_life_s;
};

class ParticleSystem {
public:
    // Spawn `count` particles at the given ECS world position. ECS Z-up to
    // threepp Y-up conversion is handled here.
    void spawn(threepp::Scene& scene, factory::Vec3 ecs_world, int count = 3) {
        static const uint32_t kPalette[] =
            { 0xff66cc, 0x66ccff, 0xffcc66, 0xcc99ff, 0xffff99 };
        for (int i = 0; i < count; ++i) {
            auto geo = threepp::SphereGeometry::create(0.04f, 6, 4);
            auto mat = threepp::MeshBasicMaterial::create();
            mat->color       = threepp::Color(kPalette[std::rand() % 5]);
            mat->transparent = true;
            mat->opacity     = 1.0f;

            auto mesh = threepp::Mesh::create(geo, mat);
            const float wx = ecs_world.x * 0.001f;
            const float wy = ecs_world.z * 0.001f;
            const float wz = ecs_world.y * 0.001f;
            const float jx = (std::rand() / float(RAND_MAX) - 0.5f) * 0.18f;
            const float jy = (std::rand() / float(RAND_MAX) - 0.5f) * 0.18f;
            const float jz = (std::rand() / float(RAND_MAX) - 0.5f) * 0.18f;
            mesh->position.set(wx + jx, wy + jy, wz + jz);
            scene.add(mesh);

            const float vx = (std::rand() / float(RAND_MAX) - 0.5f) * 0.6f;
            const float vy =  0.3f + (std::rand() / float(RAND_MAX)) * 0.8f;
            const float vz = (std::rand() / float(RAND_MAX) - 0.5f) * 0.6f;
            particles_.push_back({mesh, mat,
                                  threepp::Vector3(vx, vy, vz),
                                  0.9f, 0.9f});
        }
    }

    // Convenience: spawn particles at every active MagicTransportComponent
    // (skips idle ones). Call once per tick.
    void spawn_for_active_transports(entt::registry& reg, threepp::Scene& scene,
                                     int count_per_active = 3) {
        for (auto&& [mt_e, mt, mpose] :
             reg.view<factory::MagicTransportComponent,
                      factory::PoseComponent>().each())
        {
            (void)mpose;
            if (mt.state() == factory::PickerState::Idle) continue;
            auto wmat = factory::world_transform(mt_e, reg);
            spawn(scene, glm::vec3(wmat[3]), count_per_active);
        }
    }

    // Advance, fade, and reap expired particles.
    void step(threepp::Scene& scene, float dt) {
        for (auto& p : particles_) {
            p.mesh->position.x += p.velocity.x * dt;
            p.mesh->position.y += p.velocity.y * dt;
            p.mesh->position.z += p.velocity.z * dt;
            p.lifetime_s -= dt;
            const float life_frac = std::max(0.f, p.lifetime_s / p.total_life_s);
            p.mesh->scale.setScalar(life_frac);
            p.mat->opacity = life_frac;
        }
        particles_.erase(
            std::remove_if(particles_.begin(), particles_.end(),
                [&](const Particle& p) {
                    if (p.lifetime_s <= 0.f) {
                        scene.remove(*p.mesh);
                        return true;
                    }
                    return false;
                }),
            particles_.end());
    }

private:
    std::vector<Particle> particles_;
};

}  // namespace magic

// ── Robot URDF + IK rendering ───────────────────────────────────────────────
//
// One Registry per render loop. On each update():
//   - Lazily loads the URDF for every entity with a RobotComponent and adds
//     the resulting threepp::Robot to the scene (oriented Z-up → Y-up).
//   - Syncs the robot's base position to RobotComponent.base_position.
//   - For robots that have a `tracks` entity, builds an IK target from that
//     entity's world position and applies the solved joint values.
//
// All robot state (URDF visual, IK solver, joint cache) lives in the Registry.
// The demo only needs one declaration + one update() call per frame.

namespace robot {

// Geometry loader that wraps threepp's default ModelLoader and strips the
// up-axis rotation that ColladaLoader bakes into Z_UP DAE files. The KUKA
// URDFs we use are Z-up natively and we already rotate the whole robot at
// the root, so ColladaLoader's extra rotation just offsets each visual
// segment from its collision counterpart. STL files don't need this — they
// pass through unmodified.
class FlatGeometryLoader : public threepp::Loader<threepp::Group> {
public:
    std::shared_ptr<threepp::Group> load(const std::filesystem::path& path) override {
        auto group = inner_.load(path);
        if (!group) return nullptr;
        if (path.extension() == ".dae" || path.extension() == ".DAE") {
            group->rotation.set(0.f, 0.f, 0.f);
            group->quaternion.set(0.f, 0.f, 0.f, 1.f);
        }
        return group;
    }
private:
    threepp::ModelLoader inner_;
};

struct Instance {
    std::shared_ptr<threepp::Robot> visual;
    std::unique_ptr<factory::ik::Solver> solver;
    std::vector<float>              joints;
};

class Registry {
public:
    void update(factory::FactoryScene& scene, threepp::Scene& threepp_scene) {
        auto& reg = scene.registry();

        for (auto&& [e, rc] : reg.view<factory::RobotComponent>().each()) {
            auto it = instances_.find(e);
            if (it == instances_.end()) {
                it = load(e, rc, threepp_scene);
                if (it == instances_.end()) continue;
            }

            Instance& inst = it->second;

            // Keep the visual's base position in sync with the component, and
            // refresh `matrix` immediately — `computeEndEffectorTransform`
            // premultiplies by *matrix*, which is otherwise only refreshed at
            // render time. Without this, FK runs in URDF-local frame while
            // the IK target is in world frame and the solver chases garbage.
            const auto bp = rc.base_position();
            inst.visual->position.set(bp.x * 0.001f,
                                      bp.z * 0.001f,
                                      bp.y * 0.001f);
            inst.visual->updateMatrix();

            // IK if tracking an entity. Position-only for now — gripper
            // orientation is left to whatever the solver finds, which avoids
            // contorting the arm to match an arbitrary world-frame identity
            // quaternion. Set target.orientation later for true 6-DOF.
            if (rc.tracks() != entt::null && reg.valid(rc.tracks()) && inst.solver) {
                const auto wmat = factory::world_transform(rc.tracks(), reg);
                factory::ik::Target target;
                target.position = factory::Vec3{wmat[3][0], wmat[3][1], wmat[3][2]};
                inst.joints = inst.solver->solve(target, inst.joints);
                for (size_t i = 0; i < inst.joints.size(); ++i) {
                    inst.visual->setJointValue(i, inst.joints[i]);
                }
            }
        }
    }

private:
    std::unordered_map<entt::entity, Instance> instances_;

    std::unordered_map<entt::entity, Instance>::iterator
    load(entt::entity e, const factory::RobotComponent& rc, threepp::Scene& threepp_scene)
    {
        std::filesystem::path path(rc.urdf_path());
        if (!std::filesystem::exists(path)) return instances_.end();

        threepp::URDFLoader loader;
        loader.setGeometryLoader(std::make_shared<FlatGeometryLoader>());
        auto robot = loader.load(path);
        if (!robot) return instances_.end();

        // URDFs are conventionally Z-up; threepp's scene is Y-up. Rotate so
        // the robot's "up" matches the camera's.
        robot->rotation.x = -threepp::math::PI / 2.f;

        // Place at the component's base position immediately so the first
        // FK call sees the correct *matrix* (computeEndEffectorTransform
        // premultiplies by it — relying on render-time refresh would put the
        // first IK call in URDF-local frame).
        const auto bp = rc.base_position();
        robot->position.set(bp.x * 0.001f, bp.z * 0.001f, bp.y * 0.001f);
        robot->updateMatrix();

        // Colliders are now hidden again — both visuals and collision use
        // the same .stl files (see urdf_with_stl_visuals); the two are
        // overlapping geometry, so showing the wireframe collider over the
        // shaded visual would just look like z-fighting noise.
        robot->showColliders(false);
        robot->traverseType<threepp::Mesh>([](threepp::Mesh& m) {
            m.castShadow    = true;
            m.receiveShadow = true;
        });

        threepp_scene.add(robot);

        Instance inst;
        inst.visual = robot;
        inst.solver = factory::ik::make_dls_solver(robot);

        // Start joints at the midpoint of each joint's range. All-zeros is
        // a poor warm-start for KUKAs: joint_2's range is asymmetric, so
        // zero sits near its upper limit and the solver immediately fights
        // clamping. Mid-range gives a safe, well-conditioned starting pose.
        const auto infos = robot->getArticulatedJointInfo();
        inst.joints.resize(infos.size());
        for (size_t i = 0; i < infos.size(); ++i) {
            inst.joints[i] = infos[i].range.has_value() ? infos[i].range->mid() : 0.f;
            robot->setJointValue(i, inst.joints[i]);
        }
        return instances_.emplace(e, std::move(inst)).first;
    }
};

}  // namespace robot

}  // namespace render
