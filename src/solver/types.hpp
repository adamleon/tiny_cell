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

// An opening anchored to a specific edge at a known position (e.g. a belt pass-through).
struct AnchoredOpeningSpec {
    EntityId entity_id   = kNewEntity;
    int      width_mm    = 0;
    int      edge_index  = 0;   // index into SolverInput::nodes ring (edge i connects node i → node i+1)
    int      position_mm = 0;   // center of opening measured from node_a, mm
};

struct SolverInput {
    std::vector<NodeSpec>            nodes;
    std::vector<OpeningSpec>         unallocated_openings;
    std::vector<AnchoredOpeningSpec> anchored_openings;    // belt pass-throughs, fixed edge+position
    std::string                      catalog_path;
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
