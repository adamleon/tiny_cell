#pragma once
#include <entt/entt.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

// Axis convention for all poses: x = forward, y = left, z = up (right-hand, Z-up).
// This matches the industrial / ROS convention. The RenderSystem applies a
// Z-up → Y-up conversion when building threepp scene objects.
//
// To swap the math library, change the three using-declarations below and
// update the world_transform implementation. Nothing outside this file should
// reference glm:: directly.

namespace factory {

using Vec3 = glm::vec3;
using Quat = glm::quat;
using Mat4 = glm::mat4;

// PoseComponent: position and orientation in the parent entity's reference frame.
//
//   parent == entt::null  → unallocated — entity exists but has no world position yet
//   parent == self        → world root  — the scene entity (exactly one per FactoryScene)
//   parent == other       → placed relative to that entity's frame
struct PoseComponent {
    Vec3         position    = Vec3{0.f};
    Quat         orientation = Quat{1.f, 0.f, 0.f, 0.f};  // w=1, x=y=z=0 (identity)
    entt::entity parent      = entt::null;
};

// World-space transform matrix (positions in mm).
// Walks the parent chain until parent == self (world root).
// Returns identity for unallocated entities (parent == null).
inline Mat4 world_transform(entt::entity entity, const entt::registry& reg) {
    const auto* pose = reg.try_get<PoseComponent>(entity);
    if (!pose || pose->parent == entt::null) return Mat4{1.f};

    Mat4 local = glm::translate(Mat4{1.f}, pose->position)
               * glm::mat4_cast(pose->orientation);

    if (pose->parent == entity) return local;  // world root — stop here
    return world_transform(pose->parent, reg) * local;
}

}  // namespace factory
