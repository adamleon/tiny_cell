#pragma once
#include <cstdint>
#include <string>
#include <vector>

// Solver-world types. No EnTT dependency.
// The SolverInputBuilder (ECS layer) maps entt::entity → EntityId before calling the solver.
// The FactoryScene maps EntityId back to entt::entity after, creating new entities as needed.

namespace solver {

// Opaque stable handle. kNewEntity signals "factory scene should create a new entity here."
using EntityId = uint32_t;
static constexpr EntityId kNewEntity = ~EntityId{0};  // 0xFFFFFFFF — matches entt::null's raw value

// ── Input types ───────────────────────────────────────────────────────────────

struct NodeSpec {
    EntityId entity_id = kNewEntity;
    float    x_mm      = 0.f;
    float    z_mm      = 0.f;
};

struct OpeningSpec {
    EntityId entity_id = kNewEntity;
    int      width_mm  = 0;
};

struct SolverInput {
    std::vector<NodeSpec>    nodes;
    std::vector<OpeningSpec> unallocated_openings;
    std::string              catalog_path;
};

// ── Output types ──────────────────────────────────────────────────────────────

// NodeType is always solver output, never user input.
// Priority list (only Post implemented now): None > Straight > RoundedCorner > Post > AngledPost > Gap
enum class NodeType { Post };

struct NodeOut {
    EntityId entity_id;
    float    x_mm;
    float    z_mm;
    NodeType type;
};

struct OpeningOut {
    EntityId entity_id;
    EntityId edge_id;
    int      position_mm;  // center of opening measured from edge start
    int      width_mm;
};

struct EdgeOut {
    EntityId                      entity_id;
    EntityId                      node_a_id;
    EntityId                      node_b_id;
    std::vector<std::vector<int>> spans_mm;  // panel combos per fence span
    std::vector<OpeningOut>       openings;  // in left-to-right order on edge
    std::string                   catalog_ref;
};

struct SolverOutput {
    std::vector<NodeOut> nodes;
    std::vector<EdgeOut> edges;
};

}  // namespace solver
