#pragma once
#include <cmath>
#include <entt/entt.hpp>
#include <unordered_map>
#include "components.hpp"
#include "solver/solver.hpp"

// FactoryScene is the single source of truth for all scene state.
// It wraps an EnTT registry and owns the current solver output (the "CellLayout").
//
// Every entity has a PoseComponent. parent == entt::null means unallocated.
// The scene entity (kSceneEntityId = 1) is always created at construction and
// serves as the world root — its parent points to itself.
//
// apply(SolverOutput) maps every solver element to an entt::entity:
//   - Known EntityId → updates the entity's components in place
//   - Unknown EntityId → creates a new entity, registers the mapping

namespace factory {

static constexpr solver::EntityId kSceneEntityId = 1u;

class FactoryScene {
    entt::registry                                     registry_;
    std::unordered_map<solver::EntityId, entt::entity> id_map_;
    solver::SolverOutput                               current_layout_;

    entt::entity get_or_create(solver::EntityId id) {
        auto [it, inserted] = id_map_.emplace(id, entt::null);
        if (inserted) it->second = registry_.create();
        return it->second;
    }

public:
    FactoryScene() {
        // Scene entity: world root, always ID 1, parent == self.
        auto scene_ent  = registry_.create();
        id_map_.emplace(kSceneEntityId, scene_ent);
        auto& pose      = registry_.emplace<PoseComponent>(scene_ent);
        pose.parent     = scene_ent;
    }

    // Apply a solver output: populate/update components from every element.
    // Idempotent — calling with the same output does not create duplicate entities.
    void apply(const solver::SolverOutput& output) {
        current_layout_    = output;
        auto scene_ent     = id_map_.at(kSceneEntityId);

        // Nodes first — edges need their PoseComponents to compute midpoints.
        for (const auto& n : output.nodes) {
            auto e  = get_or_create(n.entity_id);
            registry_.emplace_or_replace<NodeComponent>(e).type = n.type;

            // Nodes lie on the factory floor (z = 0).
            // Solver x_mm/z_mm map to Vec3 x/y in the Z-up floor plane.
            auto& pose    = registry_.emplace_or_replace<PoseComponent>(e);
            pose.position = Vec3{n.x_mm, n.z_mm, 0.f};
            pose.parent   = scene_ent;
        }

        for (const auto& edge : output.edges) {
            auto e  = get_or_create(edge.entity_id);
            auto& ec       = registry_.emplace_or_replace<EdgeComponent>(e);
            ec.node_a      = get_or_create(edge.node_a_id);
            ec.node_b      = get_or_create(edge.node_b_id);
            ec.spans_mm    = edge.spans_mm;
            ec.catalog_ref = edge.catalog_ref;

            // Edge pose: origin at midpoint, x-axis along the edge direction.
            const auto& pa = registry_.get<PoseComponent>(ec.node_a);
            const auto& pb = registry_.get<PoseComponent>(ec.node_b);
            Vec3  dir      = pb.position - pa.position;
            float len      = glm::length(dir);
            Vec3  dnorm    = (len > 0.f) ? dir / len : Vec3{1.f, 0.f, 0.f};
            float yaw      = std::atan2(dnorm.y, dnorm.x);

            auto& ep       = registry_.emplace_or_replace<PoseComponent>(e);
            ep.position    = (pa.position + pb.position) * 0.5f;
            ep.orientation = glm::angleAxis(yaw, Vec3{0.f, 0.f, 1.f});
            ep.parent      = scene_ent;

            for (const auto& op : edge.openings) {
                auto oe  = get_or_create(op.entity_id);
                auto& oc = registry_.emplace_or_replace<DeclaredOpeningComponent>(oe);
                oc.parent_edge = e;
                oc.width_mm    = op.width_mm;
                // desired_position_mm stays absent: solver-assigned, not yet anchored.

                // Local x in edge frame: op.position_mm from node_a; edge origin is midpoint.
                float local_x       = static_cast<float>(op.position_mm) - len * 0.5f;
                auto& op_pose       = registry_.emplace_or_replace<PoseComponent>(oe);
                op_pose.position    = Vec3{local_x, 0.f, 0.f};
                op_pose.orientation = Quat{1.f, 0.f, 0.f, 0.f};
                op_pose.parent      = e;
            }
        }
    }

    // Translate a solver EntityId to its entt::entity. Returns entt::null if unknown.
    entt::entity find(solver::EntityId id) const {
        auto it = id_map_.find(id);
        return it != id_map_.end() ? it->second : entt::null;
    }

    // Total entities in this scene (includes the scene entity itself).
    std::size_t entity_count() const { return id_map_.size(); }

    const solver::SolverOutput& current_layout() const { return current_layout_; }
    entt::registry&             registry()              { return registry_; }
    const entt::registry&       registry()        const { return registry_; }
};

}  // namespace factory
