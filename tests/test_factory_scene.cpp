// Tests for FactoryScene: entity creation, LayoutComponent population, ID mapping.
// No threepp dependency. Uses the real solver + combinations.json.

#include <cstdio>
#include <stdexcept>
#include <string>

#include "cell/fence_solver.hpp"
#include "solver/solver.hpp"
#include "factory_scene/factory_scene.hpp"

// ── Test harness ──────────────────────────────────────────────────────────────
static int g_run = 0, g_fail = 0;

#define REQUIRE(cond) \
    do { if (!(cond)) throw std::runtime_error("REQUIRE failed: " #cond); } while(0)

#define REQUIRE_EQ(a, b) \
    do { if (!((a) == (b))) throw std::runtime_error( \
        std::string("REQUIRE_EQ: ") + std::to_string(a) + " != " + std::to_string(b)); } while(0)

#define TEST(name) static void test_##name()
#define RUN(name) \
    do { ++g_run; try { test_##name(); printf("  PASS  " #name "\n"); } \
         catch (const std::exception& e) { ++g_fail; printf("  FAIL  " #name " — %s\n", e.what()); } \
    } while(0)

// ── Fixture ───────────────────────────────────────────────────────────────────
static solver::SolverOutput solve_rect(bool with_opening = false) {
    auto table = loadTable("assets/components/fences/axelent_x-guard/combinations.json");
    solver::SolverInput in;
    in.nodes = {
        {solver::kNewEntity, -2000.f, -1500.f},
        {solver::kNewEntity,  2000.f, -1500.f},
        {solver::kNewEntity,  2000.f,  1500.f},
        {solver::kNewEntity, -2000.f,  1500.f},
    };
    if (with_opening)
        in.unallocated_openings = {{solver::kNewEntity, 500}};
    in.catalog_path = "assets/components/fences/axelent_x-guard";
    return solver::solve(in, table);
}

// ── Tests ─────────────────────────────────────────────────────────────────────

// 4 nodes + 4 edges + 1 opening = 9 entities created
TEST(apply_creates_correct_entity_count) {
    factory::FactoryScene scene;
    scene.apply(solve_rect(true));
    REQUIRE_EQ(scene.entity_count(), 9u);
}

// Without an opening: 4 nodes + 4 edges = 8 entities
TEST(apply_without_opening_creates_eight_entities) {
    factory::FactoryScene scene;
    scene.apply(solve_rect(false));
    REQUIRE_EQ(scene.entity_count(), 8u);
}

// apply() is idempotent: calling it twice with the same output must not create new entities
TEST(apply_twice_is_idempotent) {
    factory::FactoryScene scene;
    auto out = solve_rect(true);
    scene.apply(out);
    scene.apply(out);
    REQUIRE_EQ(scene.entity_count(), 9u);
}

// find() returns a valid entity for an ID present in the output
TEST(find_known_id_returns_valid_entity) {
    factory::FactoryScene scene;
    auto out = solve_rect(false);
    scene.apply(out);
    auto id     = out.nodes[0].entity_id;
    auto entity = scene.find(id);
    REQUIRE(entity != entt::null);
}

// find() returns entt::null for an ID never seen
TEST(find_unknown_id_returns_null) {
    factory::FactoryScene scene;
    scene.apply(solve_rect(false));
    REQUIRE(scene.find(solver::kNewEntity) == entt::null);
    REQUIRE(scene.find(99999u) == entt::null);
}

// Every node in the output has a LayoutComponent with role Node
TEST(node_entities_have_correct_role) {
    factory::FactoryScene scene;
    auto out = solve_rect(false);
    scene.apply(out);
    for (const auto& n : out.nodes) {
        auto e = scene.find(n.entity_id);
        REQUIRE(e != entt::null);
        const auto& lc = scene.registry().get<factory::LayoutComponent>(e);
        REQUIRE(lc.role == factory::LayoutRole::Node);
    }
}

// Node positions are preserved from solver output
TEST(node_positions_in_layout_component) {
    factory::FactoryScene scene;
    auto out = solve_rect(false);
    scene.apply(out);
    for (const auto& n : out.nodes) {
        const auto& lc = scene.registry().get<factory::LayoutComponent>(scene.find(n.entity_id));
        REQUIRE(lc.x_mm == n.x_mm);
        REQUIRE(lc.z_mm == n.z_mm);
    }
}

// Node types are preserved
TEST(node_type_is_post) {
    factory::FactoryScene scene;
    auto out = solve_rect(false);
    scene.apply(out);
    for (const auto& n : out.nodes) {
        const auto& lc = scene.registry().get<factory::LayoutComponent>(scene.find(n.entity_id));
        REQUIRE(lc.node_type == solver::NodeType::Post);
    }
}

// Edge LayoutComponents have role Edge
TEST(edge_entities_have_correct_role) {
    factory::FactoryScene scene;
    auto out = solve_rect(false);
    scene.apply(out);
    for (const auto& e : out.edges) {
        const auto& lc = scene.registry().get<factory::LayoutComponent>(scene.find(e.entity_id));
        REQUIRE(lc.role == factory::LayoutRole::Edge);
    }
}

// Edge LayoutComponent node_a/node_b point to the correct node entities
TEST(edge_node_references_are_correct) {
    factory::FactoryScene scene;
    auto out = solve_rect(false);
    scene.apply(out);
    for (const auto& e : out.edges) {
        const auto& lc = scene.registry().get<factory::LayoutComponent>(scene.find(e.entity_id));
        REQUIRE(lc.node_a == scene.find(e.node_a_id));
        REQUIRE(lc.node_b == scene.find(e.node_b_id));
        REQUIRE(lc.node_a != entt::null);
        REQUIRE(lc.node_b != entt::null);
    }
}

// Edge spans_mm are copied verbatim
TEST(edge_spans_preserved) {
    factory::FactoryScene scene;
    auto out = solve_rect(false);
    scene.apply(out);
    for (const auto& e : out.edges) {
        const auto& lc = scene.registry().get<factory::LayoutComponent>(scene.find(e.entity_id));
        REQUIRE(lc.spans_mm == e.spans_mm);
    }
}

// Opening entity has role DeclaredOpening
TEST(opening_entity_has_correct_role) {
    factory::FactoryScene scene;
    auto out = solve_rect(true);
    scene.apply(out);
    const auto& op_out = out.edges[0].openings[0];
    const auto& lc = scene.registry().get<factory::LayoutComponent>(scene.find(op_out.entity_id));
    REQUIRE(lc.role == factory::LayoutRole::DeclaredOpening);
}

// Opening parent_edge points to the edge that contains it
TEST(opening_parent_edge_is_correct) {
    factory::FactoryScene scene;
    auto out = solve_rect(true);
    scene.apply(out);
    const auto& op_out   = out.edges[0].openings[0];
    const auto& op_lc    = scene.registry().get<factory::LayoutComponent>(scene.find(op_out.entity_id));
    REQUIRE(op_lc.parent_edge == scene.find(out.edges[0].entity_id));
    REQUIRE(op_lc.parent_edge != entt::null);
}

// Opening is edge-allocated: parent_edge set, desired_position_mm absent
TEST(opening_is_edge_allocated_not_anchored) {
    factory::FactoryScene scene;
    auto out = solve_rect(true);
    scene.apply(out);
    const auto& op_out = out.edges[0].openings[0];
    const auto& lc = scene.registry().get<factory::LayoutComponent>(scene.find(op_out.entity_id));
    REQUIRE(lc.parent_edge != entt::null);
    REQUIRE(!lc.desired_position_mm.has_value());
}

// Opening width is preserved
TEST(opening_width_preserved) {
    factory::FactoryScene scene;
    auto out = solve_rect(true);
    scene.apply(out);
    const auto& op_out = out.edges[0].openings[0];
    const auto& lc = scene.registry().get<factory::LayoutComponent>(scene.find(op_out.entity_id));
    REQUIRE_EQ(lc.width_mm, 500);
}

// Opening mobility defaults to 0.0 (immovable fail-safe)
TEST(opening_mobility_defaults_to_zero) {
    factory::FactoryScene scene;
    auto out = solve_rect(true);
    scene.apply(out);
    const auto& op_out = out.edges[0].openings[0];
    const auto& lc = scene.registry().get<factory::LayoutComponent>(scene.find(op_out.entity_id));
    REQUIRE(lc.mobility == 0.0f);
}

// current_layout() reflects the most recently applied solver output
TEST(current_layout_reflects_last_apply) {
    factory::FactoryScene scene;
    auto out = solve_rect(true);
    scene.apply(out);
    REQUIRE(scene.current_layout().nodes.size() == out.nodes.size());
    REQUIRE(scene.current_layout().edges.size() == out.edges.size());
}

// ── Main ──────────────────────────────────────────────────────────────────────
int main() {
    printf("Running factory scene tests...\n");

    RUN(apply_creates_correct_entity_count);
    RUN(apply_without_opening_creates_eight_entities);
    RUN(apply_twice_is_idempotent);
    RUN(find_known_id_returns_valid_entity);
    RUN(find_unknown_id_returns_null);
    RUN(node_entities_have_correct_role);
    RUN(node_positions_in_layout_component);
    RUN(node_type_is_post);
    RUN(edge_entities_have_correct_role);
    RUN(edge_node_references_are_correct);
    RUN(edge_spans_preserved);
    RUN(opening_entity_has_correct_role);
    RUN(opening_parent_edge_is_correct);
    RUN(opening_is_edge_allocated_not_anchored);
    RUN(opening_width_preserved);
    RUN(opening_mobility_defaults_to_zero);
    RUN(current_layout_reflects_last_apply);

    printf("\n%d/%d passed", g_run - g_fail, g_run);
    if (g_fail) printf("  (%d FAILED)", g_fail);
    printf("\n");
    return g_fail ? 1 : 0;
}
