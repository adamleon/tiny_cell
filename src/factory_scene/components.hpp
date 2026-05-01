#pragma once
#include <entt/entt.hpp>
#include <optional>
#include <string>
#include <vector>
#include "solver/types.hpp"
#include "pose_component.hpp"

// One struct per entity type. An entity's role is determined by which components
// it has — there is no discriminator field. Adding new entity types (robot, belt,
// pallet) means adding new structs here; existing components are never modified.
//
// FactoryScene is the only place that creates entities and attaches components,
// which prevents nonsense combinations.

namespace factory {

// ── Fence layout ──────────────────────────────────────────────────────────────

struct NodeComponent {
    solver::NodeType type = solver::NodeType::Post;
};

struct EdgeComponent {
    entt::entity                  node_a      = entt::null;
    entt::entity                  node_b      = entt::null;
    std::vector<std::vector<int>> spans_mm;
    std::string                   catalog_ref;
};

// Allocation state is encoded by which optional fields are set (see ENTITY_SYSTEM.md):
//   parent_edge absent              → Unallocated  (solver chooses edge + position)
//   parent_edge set, position absent → Edge-allocated (solver chooses position)
//   both set                        → Anchored     (user confirmed)
struct DeclaredOpeningComponent {
    entt::entity       parent_edge         = entt::null;
    std::optional<int> desired_position_mm;  // absent = solver-assigned, not yet anchored
    int                width_mm            = 0;
    float              mobility            = 0.0f;  // 0.0 = immovable fail-safe
};

// ── Future entity types go here (robot, belt, pallet, …) ─────────────────────

}  // namespace factory
