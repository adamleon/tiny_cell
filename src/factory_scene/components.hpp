#pragma once
#include <entt/entt.hpp>
#include <optional>
#include <string>
#include <vector>
#include "solver/types.hpp"

// ECS components for the factory scene. All layout data lives here.
// The solver reads a snapshot (SolverInput), never these components directly.
// The RenderSystem reads the stored SolverOutput (the current layout), never LayoutComponent.

namespace factory {

enum class LayoutRole { Node, Edge, DeclaredOpening };

// LayoutComponent carries an entity's role in the fence layout.
// Each entity type uses a subset of the fields; unused fields keep their zero-value defaults.
//
// Node: position is the single source of truth. Design intent: once threepp scene objects
// are introduced, position will be read from the Object3D world transform instead of x_mm/z_mm.
struct LayoutComponent {
    LayoutRole role = LayoutRole::Node;

    // ── Node fields ──────────────────────────────────────────────────────────
    float            x_mm      = 0.f;
    float            z_mm      = 0.f;
    solver::NodeType node_type = solver::NodeType::Post;

    // ── Edge fields ──────────────────────────────────────────────────────────
    entt::entity                  node_a      = entt::null;
    entt::entity                  node_b      = entt::null;
    std::vector<std::vector<int>> spans_mm;   // panel combos per fence span
    std::string                   catalog_ref;

    // ── DeclaredOpening fields ───────────────────────────────────────────────
    // Allocation state is determined by which optional fields are set:
    //   parent_edge absent         → Unallocated (solver chooses edge + position)
    //   parent_edge set, position absent → Edge-allocated (solver chooses position)
    //   both set                   → Anchored (user confirmed; solver displaces within mobility)
    entt::entity       parent_edge          = entt::null;
    std::optional<int> desired_position_mm;             // user intent; absent when solver-assigned
    int                width_mm             = 0;
    float              mobility             = 0.0f;     // 0.0 = immovable fail-safe
};

}  // namespace factory
