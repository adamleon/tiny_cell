// Tests for FactoryScene: entity creation, LayoutComponent, PoseComponent, ID mapping.
// No threepp dependency. Uses the real solver + combinations.json.

#include <cmath>
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

#define REQUIRE_NEAR(a, b, eps) \
    do { if (std::abs(float(a) - float(b)) > float(eps)) throw std::runtime_error( \
        std::string("REQUIRE_NEAR: ") + std::to_string(a) + " vs " + std::to_string(b)); } while(0)

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
        {solver::kNewEntity, -2000.f, -1500.f},  // SW
        {solver::kNewEntity,  2000.f, -1500.f},  // SE
        {solver::kNewEntity,  2000.f,  1500.f},  // NE
        {solver::kNewEntity, -2000.f,  1500.f},  // NW
    };
    if (with_opening)
        in.unallocated_openings = {{solver::kNewEntity, 500}};
    in.catalog_path = "assets/components/fences/axelent_x-guard";
    return solver::solve(in, table);
}

// ── Entity count / lifecycle ──────────────────────────────────────────────────

// Scene entity (ID=1) is always created at construction — counts as 1 entity.
// With opening: 1 scene + 4 nodes + 4 edges + 1 opening = 10.
TEST(apply_creates_correct_entity_count) {
    factory::FactoryScene scene;
    scene.apply(solve_rect(true));
    REQUIRE_EQ(scene.entity_count(), 10u);
}

// Without an opening: 1 scene + 4 nodes + 4 edges = 9.
TEST(apply_without_opening_creates_nine_entities) {
    factory::FactoryScene scene;
    scene.apply(solve_rect(false));
    REQUIRE_EQ(scene.entity_count(), 9u);
}

// apply() is idempotent: same output applied twice must not duplicate entities.
TEST(apply_twice_is_idempotent) {
    factory::FactoryScene scene;
    auto out = solve_rect(true);
    scene.apply(out);
    scene.apply(out);
    REQUIRE_EQ(scene.entity_count(), 10u);
}

// ── ID mapping ────────────────────────────────────────────────────────────────

TEST(find_known_id_returns_valid_entity) {
    factory::FactoryScene scene;
    auto out = solve_rect(false);
    scene.apply(out);
    REQUIRE(scene.find(out.nodes[0].entity_id) != entt::null);
}

TEST(find_unknown_id_returns_null) {
    factory::FactoryScene scene;
    scene.apply(solve_rect(false));
    REQUIRE(scene.find(solver::kNewEntity) == entt::null);
    REQUIRE(scene.find(99999u) == entt::null);
}

// ── Scene entity ──────────────────────────────────────────────────────────────

// Scene entity always exists, has PoseComponent with parent == self (world root).
TEST(scene_entity_is_world_root) {
    factory::FactoryScene scene;
    auto se = scene.find(factory::kSceneEntityId);
    REQUIRE(se != entt::null);
    const auto& pose = scene.registry().get<factory::PoseComponent>(se);
    REQUIRE(pose.parent == se);
}

// Scene entity exists before any apply() call.
TEST(scene_entity_exists_before_apply) {
    factory::FactoryScene scene;
    REQUIRE(scene.entity_count() == 1u);
    REQUIRE(scene.find(factory::kSceneEntityId) != entt::null);
}

// ── Components ────────────────────────────────────────────────────────────────

// Node entities carry NodeComponent (not LayoutComponent).
TEST(node_entities_have_node_component) {
    factory::FactoryScene scene;
    auto out = solve_rect(false);
    scene.apply(out);
    for (const auto& n : out.nodes)
        REQUIRE(scene.registry().try_get<factory::NodeComponent>(scene.find(n.entity_id)) != nullptr);
}

TEST(node_type_is_post) {
    factory::FactoryScene scene;
    auto out = solve_rect(false);
    scene.apply(out);
    for (const auto& n : out.nodes) {
        const auto& nc = scene.registry().get<factory::NodeComponent>(scene.find(n.entity_id));
        REQUIRE(nc.type == solver::NodeType::Post);
    }
}

// Node entities must NOT have EdgeComponent or DeclaredOpeningComponent.
TEST(node_has_no_other_layout_components) {
    factory::FactoryScene scene;
    auto out = solve_rect(false);
    scene.apply(out);
    for (const auto& n : out.nodes) {
        auto e = scene.find(n.entity_id);
        REQUIRE(scene.registry().try_get<factory::EdgeComponent>(e)            == nullptr);
        REQUIRE(scene.registry().try_get<factory::DeclaredOpeningComponent>(e) == nullptr);
    }
}

TEST(edge_entities_have_edge_component) {
    factory::FactoryScene scene;
    auto out = solve_rect(false);
    scene.apply(out);
    for (const auto& e : out.edges)
        REQUIRE(scene.registry().try_get<factory::EdgeComponent>(scene.find(e.entity_id)) != nullptr);
}

TEST(edge_node_references_are_correct) {
    factory::FactoryScene scene;
    auto out = solve_rect(false);
    scene.apply(out);
    for (const auto& e : out.edges) {
        const auto& ec = scene.registry().get<factory::EdgeComponent>(scene.find(e.entity_id));
        REQUIRE(ec.node_a == scene.find(e.node_a_id));
        REQUIRE(ec.node_b == scene.find(e.node_b_id));
        REQUIRE(ec.node_a != entt::null);
        REQUIRE(ec.node_b != entt::null);
    }
}

TEST(edge_spans_preserved) {
    factory::FactoryScene scene;
    auto out = solve_rect(false);
    scene.apply(out);
    for (const auto& e : out.edges) {
        const auto& ec = scene.registry().get<factory::EdgeComponent>(scene.find(e.entity_id));
        REQUIRE(ec.spans_mm == e.spans_mm);
    }
}

TEST(opening_entity_has_declared_opening_component) {
    factory::FactoryScene scene;
    auto out = solve_rect(true);
    scene.apply(out);
    const auto& op_out = out.edges[0].openings[0];
    REQUIRE(scene.registry().try_get<factory::DeclaredOpeningComponent>(
        scene.find(op_out.entity_id)) != nullptr);
}

TEST(opening_parent_edge_is_correct) {
    factory::FactoryScene scene;
    auto out = solve_rect(true);
    scene.apply(out);
    const auto& op_out = out.edges[0].openings[0];
    const auto& oc = scene.registry().get<factory::DeclaredOpeningComponent>(
        scene.find(op_out.entity_id));
    REQUIRE(oc.parent_edge == scene.find(out.edges[0].entity_id));
}

// Edge-allocated: parent_edge set, desired_position_mm absent.
TEST(opening_is_edge_allocated_not_anchored) {
    factory::FactoryScene scene;
    auto out = solve_rect(true);
    scene.apply(out);
    const auto& oc = scene.registry().get<factory::DeclaredOpeningComponent>(
        scene.find(out.edges[0].openings[0].entity_id));
    REQUIRE(oc.parent_edge != entt::null);
    REQUIRE(!oc.desired_position_mm.has_value());
}

TEST(opening_width_preserved) {
    factory::FactoryScene scene;
    auto out = solve_rect(true);
    scene.apply(out);
    const auto& oc = scene.registry().get<factory::DeclaredOpeningComponent>(
        scene.find(out.edges[0].openings[0].entity_id));
    REQUIRE_EQ(oc.width_mm, 500);
}

TEST(opening_mobility_defaults_to_zero) {
    factory::FactoryScene scene;
    auto out = solve_rect(true);
    scene.apply(out);
    const auto& oc = scene.registry().get<factory::DeclaredOpeningComponent>(
        scene.find(out.edges[0].openings[0].entity_id));
    REQUIRE(oc.mobility == 0.0f);
}

// ── PoseComponent ─────────────────────────────────────────────────────────────

// Nodes lie on the factory floor (z == 0) and are children of the scene entity.
TEST(node_pose_on_floor_with_scene_parent) {
    factory::FactoryScene scene;
    auto out = solve_rect(false);
    scene.apply(out);
    auto se = scene.find(factory::kSceneEntityId);
    for (const auto& n : out.nodes) {
        const auto& pose = scene.registry().get<factory::PoseComponent>(scene.find(n.entity_id));
        REQUIRE(pose.position.z == 0.f);
        REQUIRE(pose.parent == se);
    }
}

// Node positions match the solver output (x_mm → pos.x, z_mm → pos.y).
TEST(node_pose_position_matches_solver) {
    factory::FactoryScene scene;
    auto out = solve_rect(false);
    scene.apply(out);
    for (const auto& n : out.nodes) {
        const auto& pose = scene.registry().get<factory::PoseComponent>(scene.find(n.entity_id));
        REQUIRE(pose.position.x == n.x_mm);
        REQUIRE(pose.position.y == n.z_mm);
    }
}

// Edge origin is the midpoint of its two nodes.
TEST(edge_pose_at_midpoint_of_nodes) {
    factory::FactoryScene scene;
    auto out = solve_rect(false);
    scene.apply(out);
    // Edge 0: SW(-2000,-1500) → SE(2000,-1500); midpoint = (0, -1500)
    const auto& ep = scene.registry().get<factory::PoseComponent>(scene.find(out.edges[0].entity_id));
    REQUIRE_NEAR(ep.position.x,   0.f, 0.01f);
    REQUIRE_NEAR(ep.position.y, -1500.f, 0.01f);
    REQUIRE_NEAR(ep.position.z,   0.f, 0.01f);
}

// Edge parent is the scene entity.
TEST(edge_pose_parent_is_scene_entity) {
    factory::FactoryScene scene;
    auto out = solve_rect(false);
    scene.apply(out);
    auto se = scene.find(factory::kSceneEntityId);
    for (const auto& e : out.edges) {
        const auto& pose = scene.registry().get<factory::PoseComponent>(scene.find(e.entity_id));
        REQUIRE(pose.parent == se);
    }
}

// Opening's parent in PoseComponent is the edge entity (not the scene entity).
TEST(opening_pose_parent_is_edge) {
    factory::FactoryScene scene;
    auto out = solve_rect(true);
    scene.apply(out);
    const auto& op_pose = scene.registry().get<factory::PoseComponent>(
        scene.find(out.edges[0].openings[0].entity_id));
    REQUIRE(op_pose.parent == scene.find(out.edges[0].entity_id));
}

// Opening is centered on a 4000mm edge: local x should be ~0mm (midpoint = center).
TEST(opening_pose_centered_on_edge) {
    factory::FactoryScene scene;
    auto out = solve_rect(true);
    scene.apply(out);
    // Edge 0 is 4000mm; opening center is at 2000mm from start → 0mm from midpoint.
    const auto& op_pose = scene.registry().get<factory::PoseComponent>(
        scene.find(out.edges[0].openings[0].entity_id));
    REQUIRE_NEAR(op_pose.position.x, 0.f, 0.01f);
    REQUIRE_NEAR(op_pose.position.y, 0.f, 0.01f);
    REQUIRE_NEAR(op_pose.position.z, 0.f, 0.01f);
}

// A PoseComponent with parent == entt::null means unallocated.
TEST(default_pose_parent_is_null) {
    factory::FactoryScene scene;
    auto& reg = scene.registry();
    auto  e   = reg.create();
    auto& p   = reg.emplace<factory::PoseComponent>(e);
    REQUIRE(p.parent == entt::null);
}

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
    RUN(apply_without_opening_creates_nine_entities);
    RUN(apply_twice_is_idempotent);
    RUN(find_known_id_returns_valid_entity);
    RUN(find_unknown_id_returns_null);
    RUN(scene_entity_is_world_root);
    RUN(scene_entity_exists_before_apply);
    RUN(node_entities_have_node_component);
    RUN(node_type_is_post);
    RUN(node_has_no_other_layout_components);
    RUN(edge_entities_have_edge_component);
    RUN(edge_node_references_are_correct);
    RUN(edge_spans_preserved);
    RUN(opening_entity_has_declared_opening_component);
    RUN(opening_parent_edge_is_correct);
    RUN(opening_is_edge_allocated_not_anchored);
    RUN(opening_width_preserved);
    RUN(opening_mobility_defaults_to_zero);
    RUN(node_pose_on_floor_with_scene_parent);
    RUN(node_pose_position_matches_solver);
    RUN(edge_pose_at_midpoint_of_nodes);
    RUN(edge_pose_parent_is_scene_entity);
    RUN(opening_pose_parent_is_edge);
    RUN(opening_pose_centered_on_edge);
    RUN(default_pose_parent_is_null);
    RUN(current_layout_reflects_last_apply);

    printf("\n%d/%d passed", g_run - g_fail, g_run);
    if (g_fail) printf("  (%d FAILED)", g_fail);
    printf("\n");
    return g_fail ? 1 : 0;
}
