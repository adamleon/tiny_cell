#pragma once
#include <entt/entt.hpp>
#include <unordered_map>
#include "components.hpp"
#include "solver/solver.hpp"

// FactoryScene is the single source of truth for all scene state.
// It wraps an EnTT registry and owns the current solver output (the "CellLayout").
//
// apply(SolverOutput) maps every element in the output to an entt::entity:
//   - Known EntityId → updates the existing entity's LayoutComponent
//   - Unknown EntityId → creates a new entity, registers the ID
//
// The RenderSystem (future) reads current_layout() for actual positions and panel combos.
// The SolverInputBuilder (future) reads LayoutComponents to build the next SolverInput.

namespace factory {

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
    // Apply a solver output: populate/update LayoutComponents from every element.
    // Idempotent when called with the same output — entities are updated, not duplicated.
    void apply(const solver::SolverOutput& output) {
        current_layout_ = output;

        // Nodes first — edges reference them
        for (const auto& n : output.nodes) {
            auto e  = get_or_create(n.entity_id);
            auto& lc = registry_.emplace_or_replace<LayoutComponent>(e);
            lc.role      = LayoutRole::Node;
            lc.x_mm      = n.x_mm;
            lc.z_mm      = n.z_mm;
            lc.node_type = n.type;
        }

        for (const auto& edge : output.edges) {
            auto e  = get_or_create(edge.entity_id);
            auto& lc = registry_.emplace_or_replace<LayoutComponent>(e);
            lc.role        = LayoutRole::Edge;
            lc.node_a      = get_or_create(edge.node_a_id);
            lc.node_b      = get_or_create(edge.node_b_id);
            lc.spans_mm    = edge.spans_mm;
            lc.catalog_ref = edge.catalog_ref;

            // Openings on this edge: edge-allocated (parent set, position absent)
            for (const auto& op : edge.openings) {
                auto oe  = get_or_create(op.entity_id);
                auto& olc = registry_.emplace_or_replace<LayoutComponent>(oe);
                olc.role       = LayoutRole::DeclaredOpening;
                olc.parent_edge = e;
                // desired_position_mm stays absent: solver suggested this position,
                // but the user hasn't anchored it yet.
                olc.width_mm   = op.width_mm;
            }
        }
    }

    // Translate a solver EntityId to its entt::entity. Returns entt::null if unknown.
    entt::entity find(solver::EntityId id) const {
        auto it = id_map_.find(id);
        return it != id_map_.end() ? it->second : entt::null;
    }

    // Number of entities created by this scene.
    std::size_t entity_count() const { return id_map_.size(); }

    const solver::SolverOutput& current_layout() const { return current_layout_; }
    entt::registry&             registry()              { return registry_; }
    const entt::registry&       registry()        const { return registry_; }
};

}  // namespace factory
