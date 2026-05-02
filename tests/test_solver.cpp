// Unit tests for the layout solver.
// Covers entity ID assignment, topology, opening placement, and span structure.
// No threepp dependency. Loads combinations.json from the asset directory.

#include <algorithm>
#include <cassert>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <vector>

#include "cell/fence_solver.hpp"
#include "solver/solver.hpp"

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

// ── Test fixture ──────────────────────────────────────────────────────────────
// 4-node rectangle matching the fence_walls demo dimensions.
// Edge 0 south 4000mm, edge 1 east 3000mm, edge 2 north 4000mm, edge 3 west 3000mm.
static solver::SolverInput make_rect_input(bool with_opening = false) {
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
    return in;
}

static LookupTable load_table() {
    return loadTable("assets/components/fences/axelent_x-guard/combinations.json");
}

static int visual_mm(const std::vector<int>& panels) {
    if (panels.empty()) return 0;
    int total = -50;
    for (int w : panels) total += w + 50;
    return total;
}

// ── Tests ─────────────────────────────────────────────────────────────────────

// Every element in the output must have an assigned ID (not kNewEntity).
TEST(entity_ids_assigned) {
    auto out = solver::solve(make_rect_input(true), load_table());
    for (const auto& node : out.nodes)
        REQUIRE(node.entity_id != solver::kNewEntity);
    for (const auto& edge : out.edges) {
        REQUIRE(edge.entity_id != solver::kNewEntity);
        for (const auto& op : edge.openings)
            REQUIRE(op.entity_id != solver::kNewEntity);
    }
}

// All assigned IDs within a single solve must be unique.
TEST(entity_ids_unique) {
    auto out = solver::solve(make_rect_input(true), load_table());
    std::vector<solver::EntityId> ids;
    for (const auto& n : out.nodes) ids.push_back(n.entity_id);
    for (const auto& e : out.edges) {
        ids.push_back(e.entity_id);
        for (const auto& op : e.openings) ids.push_back(op.entity_id);
    }
    std::sort(ids.begin(), ids.end());
    REQUIRE(std::adjacent_find(ids.begin(), ids.end()) == ids.end());
}

// 4-node input → 4 nodes and 4 edges in output.
TEST(node_and_edge_counts) {
    auto out = solver::solve(make_rect_input(), load_table());
    REQUIRE_EQ(out.nodes.size(), 4u);
    REQUIRE_EQ(out.edges.size(), 4u);
}

// Input node positions must pass through unchanged.
TEST(node_positions_preserved) {
    auto out = solver::solve(make_rect_input(), load_table());
    REQUIRE(out.nodes[0].x_mm == -2000.f && out.nodes[0].z_mm == -1500.f);
    REQUIRE(out.nodes[1].x_mm ==  2000.f && out.nodes[1].z_mm == -1500.f);
    REQUIRE(out.nodes[2].x_mm ==  2000.f && out.nodes[2].z_mm ==  1500.f);
    REQUIRE(out.nodes[3].x_mm == -2000.f && out.nodes[3].z_mm ==  1500.f);
}

// NodeType is Post for all corners (Layer 4 simplified).
TEST(all_nodes_are_post) {
    auto out = solver::solve(make_rect_input(), load_table());
    for (const auto& n : out.nodes)
        REQUIRE(n.type == solver::NodeType::Post);
}

// Edge i connects node[i] → node[(i+1)%4].
TEST(edge_topology_consecutive_nodes) {
    auto out = solver::solve(make_rect_input(), load_table());
    for (int i = 0; i < 4; ++i) {
        REQUIRE(out.edges[i].node_a_id == out.nodes[i].entity_id);
        REQUIRE(out.edges[i].node_b_id == out.nodes[(i + 1) % 4].entity_id);
    }
}

// Unallocated opening ends up on edge 0 only.
TEST(opening_placed_on_first_edge) {
    auto out = solver::solve(make_rect_input(true), load_table());
    REQUIRE(out.edges[0].openings.size() == 1u);
    REQUIRE(out.edges[1].openings.empty());
    REQUIRE(out.edges[2].openings.empty());
    REQUIRE(out.edges[3].openings.empty());
}

// Opening width is preserved verbatim.
TEST(opening_width_preserved) {
    auto out = solver::solve(make_rect_input(true), load_table());
    REQUIRE_EQ(out.edges[0].openings[0].width_mm, 500);
}

// Opening's edge_id must match the edge it lives on.
TEST(opening_edge_id_matches_edge) {
    auto out = solver::solve(make_rect_input(true), load_table());
    REQUIRE(out.edges[0].openings[0].edge_id == out.edges[0].entity_id);
}

// Edge with opening has 2 spans; edges without opening have 1 span.
TEST(span_count_per_edge) {
    auto out = solver::solve(make_rect_input(true), load_table());
    REQUIRE_EQ(out.edges[0].spans_mm.size(), 2u);  // split by opening
    REQUIRE_EQ(out.edges[1].spans_mm.size(), 1u);
    REQUIRE_EQ(out.edges[2].spans_mm.size(), 1u);
    REQUIRE_EQ(out.edges[3].spans_mm.size(), 1u);
}

// Every span must contain at least one panel.
TEST(spans_nonempty) {
    auto out = solver::solve(make_rect_input(true), load_table());
    for (const auto& edge : out.edges)
        for (const auto& span : edge.spans_mm)
            REQUIRE(!span.empty());
}

// Opening center position = left_span_actual + post + opening_width/2.
TEST(opening_position_at_center) {
    auto tbl = load_table();
    auto out = solver::solve(make_rect_input(true), tbl);
    const auto& edge = out.edges[0];
    const auto& op   = edge.openings[0];
    int left_actual  = visual_mm(edge.spans_mm[0]);
    REQUIRE_EQ(op.position_mm, left_actual + tbl.post_width_mm + op.width_mm / 2);
}

// Left and right flanking spans are equal (symmetric placement).
TEST(flanking_spans_equal) {
    auto out = solver::solve(make_rect_input(true), load_table());
    const auto& spans = out.edges[0].spans_mm;
    REQUIRE_EQ(visual_mm(spans[0]), visual_mm(spans[1]));
}

// Total visual length of edge 0 = left + post + opening + post + right = desired edge length.
TEST(edge_with_opening_visual_length_correct) {
    auto tbl = load_table();
    auto out = solver::solve(make_rect_input(true), tbl);
    const auto& edge = out.edges[0];
    int total = visual_mm(edge.spans_mm[0])
              + tbl.post_width_mm + edge.openings[0].width_mm + tbl.post_width_mm
              + visual_mm(edge.spans_mm[1]);
    REQUIRE_EQ(total, 4000);
}

// No opening → one span per edge and no opening list.
TEST(no_opening_single_span_all_edges) {
    auto out = solver::solve(make_rect_input(false), load_table());
    for (const auto& edge : out.edges) {
        REQUIRE_EQ(edge.spans_mm.size(), 1u);
        REQUIRE(edge.openings.empty());
    }
}

// Entity IDs provided in input must be echoed back unchanged.
TEST(existing_entity_ids_preserved) {
    auto in = make_rect_input(true);
    in.nodes[0].entity_id = 10u;
    in.nodes[1].entity_id = 11u;
    in.nodes[2].entity_id = 12u;
    in.nodes[3].entity_id = 13u;
    in.unallocated_openings[0].entity_id = 20u;

    auto out = solver::solve(in, load_table());
    REQUIRE_EQ(out.nodes[0].entity_id, 10u);
    REQUIRE_EQ(out.nodes[1].entity_id, 11u);
    REQUIRE_EQ(out.nodes[2].entity_id, 12u);
    REQUIRE_EQ(out.nodes[3].entity_id, 13u);
    REQUIRE_EQ(out.edges[0].openings[0].entity_id, 20u);
}

// ── Main ─────────────────────────────────────────────────────────────────────
int main() {
    printf("Running solver tests...\n");

    RUN(entity_ids_assigned);
    RUN(entity_ids_unique);
    RUN(node_and_edge_counts);
    RUN(node_positions_preserved);
    RUN(all_nodes_are_post);
    RUN(edge_topology_consecutive_nodes);
    RUN(opening_placed_on_first_edge);
    RUN(opening_width_preserved);
    RUN(opening_edge_id_matches_edge);
    RUN(span_count_per_edge);
    RUN(spans_nonempty);
    RUN(opening_position_at_center);
    RUN(flanking_spans_equal);
    RUN(edge_with_opening_visual_length_correct);
    RUN(no_opening_single_span_all_edges);
    RUN(existing_entity_ids_preserved);

    printf("\n%d/%d passed", g_run - g_fail, g_run);
    if (g_fail) printf("  (%d FAILED)", g_fail);
    printf("\n");
    return g_fail ? 1 : 0;
}
