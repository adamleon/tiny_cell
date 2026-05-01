#pragma once
#include <entt/entt.hpp>
#include <optional>
#include <string>
#include <vector>
#include "solver/types.hpp"
#include "pose_component.hpp"

// ECS components for the factory scene. All layout data lives here.
// The solver reads a snapshot (SolverInput), never these components directly.
// The RenderSystem reads the stored SolverOutput (current layout), never LayoutComponent.
//
// Position and orientation live in PoseComponent, not here.

namespace factory {

enum class LayoutRole { Node, Edge, DeclaredOpening };

// LayoutComponent carries an entity's role in the fence layout.
// Each entity type uses a subset of the fields; unused fields keep zero-value defaults.
struct LayoutComponent {
    LayoutRole role = LayoutRole::Node;

    // ── Node ─────────────────────────────────────────────────────────────────
    solver::NodeType node_type = solver::NodeType::Post;

    // ── Edge ─────────────────────────────────────────────────────────────────
    entt::entity                  node_a      = entt::null;
    entt::entity                  node_b      = entt::null;
    std::vector<std::vector<int>> spans_mm;
    std::string                   catalog_ref;

    // ── DeclaredOpening ──────────────────────────────────────────────────────
    // Allocation state is encoded by which optional fields are set (see ENTITY_SYSTEM.md):
    //   parent_edge absent              → Unallocated
    //   parent_edge set, position absent → Edge-allocated
    //   both set                        → Anchored
    entt::entity       parent_edge         = entt::null;
    std::optional<int> desired_position_mm;  // user intent; absent when solver-assigned
    int                width_mm            = 0;
    float              mobility            = 0.0f;  // 0.0 = immovable fail-safe
};

}  // namespace factory
