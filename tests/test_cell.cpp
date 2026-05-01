// Unit tests for the cell layout data model, solver, and catalog helpers.
// No threepp dependency — covers the design invariants in docs/CELL_LAYOUT_DESIGN.md.

#include <cassert>
#include <cstdio>
#include <memory>
#include <stdexcept>
#include <string>

#include "cell/cell_layout.hpp"
#include "cell/fence_catalog_data.hpp"
#include "cell/fence_solver.hpp"

// ── Minimal test harness ───────────────────────────────────────────────────────
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

// ── Design invariant: Opening mobility default is 0.0 (immovable fail-safe) ──
// Per the design doc: "Default is 0.0 (immovable). If an opening is incorrectly
// deserialised with a missing mobility value, it stays fixed rather than drifting."
TEST(opening_default_mobility_is_zero) {
    // Construct a DeclaredOpening without specifying mobility
    cell::DeclaredOpening op(500, 400);
    REQUIRE_EQ(op.mobility(), 0.0f);
}

// ── Design invariant: Base Opening mobility default is 0.0 ────────────────────
// WorldFeatureOpening is not yet implemented, but the base class must already
// default to 0.0 for any subclass that forgets to override.
TEST(opening_base_default_mobility_is_zero) {
    struct MinimalOpening : cell::Opening {
        int getWidth() const override { return 100; }
    };
    MinimalOpening op;
    REQUIRE_EQ(op.mobility(), 0.0f);
}

// ── Design invariant: Base Opening is not editable and has empty collision box ─
TEST(opening_base_not_editable_no_collisionbox) {
    struct MinimalOpening : cell::Opening {
        int getWidth() const override { return 100; }
    };
    MinimalOpening op;
    REQUIRE(!op.isEditable());
    REQUIRE(op.collisionBox().empty());
}

// ── DeclaredOpening: editable, correct width, mobility stored separately ─────
TEST(user_placed_opening_properties) {
    cell::DeclaredOpening op(1000, 600, 0.75f);
    REQUIRE_EQ(op.getWidth(), 600);
    REQUIRE_EQ(op.desired_position_mm, 1000);
    REQUIRE(op.isEditable());
    REQUIRE_EQ(op.mobility(), 0.75f);
}

// ── DeclaredOpening: non-empty collision box ────────────────────────────────
TEST(user_placed_opening_has_collision_box) {
    cell::DeclaredOpening op(500, 400);
    REQUIRE(!op.collisionBox().empty());
}

// ── DeclaredOpening: desired_position_mm is user intent, not altered by ctor ─
TEST(user_placed_opening_desired_position_preserved) {
    cell::DeclaredOpening op(1234, 300);
    REQUIRE_EQ(op.desired_position_mm, 1234);
}

// ── Edge invariant: single-span addEdge produces spans.size()==1, no openings ──
TEST(add_edge_single_span_no_openings) {
    cell::CellLayout layout;
    int a = layout.addNode(0.f, 0.f);
    int b = layout.addNode(1.f, 0.f);
    layout.addEdge(a, b, {1000, 700});

    REQUIRE_EQ(layout.edges.size(), 1u);
    const auto& edge = layout.edges[0];
    REQUIRE_EQ(edge.spans_mm.size(), 1u);
    REQUIRE(edge.user_openings.empty());
    REQUIRE_EQ(edge.spans_mm[0][0], 1000);
    REQUIRE_EQ(edge.spans_mm[0][1], 700);
}

// ── Edge invariant: spans.size() == openings.size() + 1 ─────────────────────
TEST(edge_span_opening_count_invariant) {
    cell::CellLayout layout;
    int a = layout.addNode(0.f, 0.f);
    int b = layout.addNode(4.f, 0.f);

    auto slab = std::make_shared<cell::DeclaredOpening>(1750, 500);
    layout.addEdge(a, b,
        {{1000, 700}, {1000, 700}},
        {slab});

    const auto& edge = layout.edges[0];
    REQUIRE_EQ(edge.spans_mm.size(), edge.user_openings.size() + 1);
}

// ── Edge: openings stored by shared_ptr — identity preserved ─────────────────
TEST(edge_opening_identity_preserved) {
    cell::CellLayout layout;
    int a = layout.addNode(0.f, 0.f);
    int b = layout.addNode(4.f, 0.f);

    auto slab = std::make_shared<cell::DeclaredOpening>(1750, 500);
    layout.addEdge(a, b, {{1000, 700}, {1000, 700}}, {slab});

    REQUIRE(layout.edges[0].user_openings[0] == slab);
}

// ── NodeType: default is Post; adding a node uses correct default ─────────────
TEST(add_node_defaults_to_post) {
    cell::CellLayout layout;
    int idx = layout.addNode(1.f, 2.f);
    REQUIRE(layout.nodes[idx].type == cell::NodeType::Post);
    REQUIRE_EQ(layout.nodes[idx].x, 1.f);
    REQUIRE_EQ(layout.nodes[idx].z, 2.f);
}

// ── catalogEdgeHeight: reads post.height_mm from real catalog ────────────────
TEST(catalog_edge_height_from_post) {
    auto catalog = cell::loadCatalog("assets/components/fences/axelent_x-guard/catalog.json");
    int h = cell::catalogEdgeHeight(catalog);
    REQUIRE_EQ(h, 1400);  // Axelent X-Guard post is 1400mm
}

// ── Solver: solve() actual_mm equals sum(panels) + (n-1)*50 ──────────────────
TEST(solver_actual_mm_matches_visual_formula) {
    auto table = loadTable("assets/components/fences/axelent_x-guard/combinations.json");
    auto result = solve(table, 2000, true);

    int expected = -50;
    for (int w : result.panels_mm) expected += w + 50;
    REQUIRE_EQ(result.actual_mm, expected);
}

// ── Solver: prefer_over returns actual_mm >= target ───────────────────────────
TEST(solver_prefer_over_rounds_up) {
    auto table = loadTable("assets/components/fences/axelent_x-guard/combinations.json");
    for (int target : {250, 500, 1000, 1500, 2000, 3000, 4000}) {
        auto r = solve(table, target, true);
        REQUIRE(r.actual_mm >= target);
    }
}

// ── Solver: prefer_under returns actual_mm <= target ─────────────────────────
TEST(solver_prefer_under_rounds_down) {
    auto table = loadTable("assets/components/fences/axelent_x-guard/combinations.json");
    for (int target : {500, 1000, 1500, 2000, 3000, 4000}) {
        auto r = solve(table, target, false);
        REQUIRE(r.actual_mm <= target);
    }
}

// ── Solver: lookup prefer_over hits exact entry when target is in table ───────
TEST(solver_prefer_over_exact_target) {
    auto table = loadTable("assets/components/fences/axelent_x-guard/combinations.json");
    // 250mm is the smallest entry — must return 250
    auto r = lookup(table, 250, true);
    REQUIRE_EQ(r.actual_mm, 250);
}

// ── Solver: panels_mm is non-empty for any achievable target ─────────────────
TEST(solver_panels_nonempty) {
    auto table = loadTable("assets/components/fences/axelent_x-guard/combinations.json");
    auto r = solve(table, 1000, true);
    REQUIRE(!r.panels_mm.empty());
}

// ── Demo assumption: south edge sum with slab equals ns.actual_mm at 4000mm ───
// This validates the specific arithmetic the demo relies on.
TEST(demo_south_edge_closure) {
    auto table  = loadTable("assets/components/fences/axelent_x-guard/combinations.json");
    auto ns     = solve(table, 4000, true);
    int slab_w  = 500;
    int remaining  = ns.actual_mm - slab_w;
    auto south_a   = solve(table, remaining / 2, true);
    auto south_b   = solve(table, remaining / 2, true);
    int south_visual = south_a.actual_mm + slab_w + south_b.actual_mm;
    REQUIRE_EQ(south_visual, ns.actual_mm);
}

// ── Main ──────────────────────────────────────────────────────────────────────
int main() {
    printf("Running cell layout tests...\n");

    RUN(opening_default_mobility_is_zero);
    RUN(opening_base_default_mobility_is_zero);
    RUN(opening_base_not_editable_no_collisionbox);
    RUN(user_placed_opening_properties);
    RUN(user_placed_opening_has_collision_box);
    RUN(user_placed_opening_desired_position_preserved);
    RUN(add_edge_single_span_no_openings);
    RUN(edge_span_opening_count_invariant);
    RUN(edge_opening_identity_preserved);
    RUN(add_node_defaults_to_post);
    RUN(catalog_edge_height_from_post);
    RUN(solver_actual_mm_matches_visual_formula);
    RUN(solver_prefer_over_rounds_up);
    RUN(solver_prefer_under_rounds_down);
    RUN(solver_prefer_over_exact_target);
    RUN(solver_panels_nonempty);
    RUN(demo_south_edge_closure);

    printf("\n%d/%d passed", g_run - g_fail, g_run);
    if (g_fail) printf("  (%d FAILED)", g_fail);
    printf("\n");
    return g_fail ? 1 : 0;
}
