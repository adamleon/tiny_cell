#pragma once
#include <algorithm>
#include <cmath>
#include "types.hpp"
#include "../cell/fence_solver.hpp"  // LookupTable, ::solve(), ::lookup()

// Pure function: receives a SolverInput and a pre-loaded panel lookup table,
// returns a SolverOutput with entity IDs for every element. IDs that were
// kNewEntity in the input are assigned fresh sequential IDs. Existing IDs
// are echoed back unchanged so the factory scene can match them to entities.
//
// Layers implemented here:
//   Layer 1 — topology from node positions
//   Layer 2 — per-span panel combinations via lookup table
//   Layer 3a — unallocated opening placed on edge 0, centered (legacy/simple case)
//   Layer 3b — anchored openings placed on their specified edges at their positions
//   Layer 4  — all corners NodeType::Post

namespace solver {

inline SolverOutput solve(const SolverInput& input, const LookupTable& table) {
    EntityId next_id = 2;
    for (const auto& n : input.nodes)
        if (n.entity_id < kNewEntity) next_id = std::max(next_id, n.entity_id + 1);
    for (const auto& op : input.unallocated_openings)
        if (op.entity_id < kNewEntity) next_id = std::max(next_id, op.entity_id + 1);
    for (const auto& op : input.anchored_openings)
        if (op.entity_id < kNewEntity) next_id = std::max(next_id, op.entity_id + 1);
    auto assign = [&](EntityId id) -> EntityId {
        return (id == kNewEntity) ? next_id++ : id;
    };

    const int n = static_cast<int>(input.nodes.size());
    SolverOutput out;

    // Layer 1 — assign node IDs; all corners Post (Layer 4)
    std::vector<EntityId> node_ids(n);
    for (int i = 0; i < n; ++i) {
        node_ids[i] = assign(input.nodes[i].entity_id);
        out.nodes.push_back({node_ids[i], input.nodes[i].x_mm, input.nodes[i].z_mm, NodeType::Post});
    }

    // Euclidean distance between node i and its successor
    auto edge_desired = [&](int i) -> int {
        const auto& a = input.nodes[i];
        const auto& b = input.nodes[(i + 1) % n];
        const float dx = b.x_mm - a.x_mm;
        const float dz = b.z_mm - a.z_mm;
        return static_cast<int>(std::lround(std::sqrt(dx * dx + dz * dz)));
    };

    // Partition anchored openings by edge index, sorted by position
    std::vector<std::vector<const AnchoredOpeningSpec*>> edge_ops(n);
    for (const auto& aop : input.anchored_openings) {
        if (aop.edge_index >= 0 && aop.edge_index < n)
            edge_ops[aop.edge_index].push_back(&aop);
    }
    for (auto& ops : edge_ops)
        std::sort(ops.begin(), ops.end(),
                  [](const AnchoredOpeningSpec* a, const AnchoredOpeningSpec* b) {
                      return a->position_mm < b->position_mm;
                  });

    // Layers 2 + 3 — panel combinations per edge
    const int post_w = table.post_width_mm;
    for (int i = 0; i < n; ++i) {
        EdgeOut edge;
        edge.entity_id   = next_id++;
        edge.node_a_id   = node_ids[i];
        edge.node_b_id   = node_ids[(i + 1) % n];
        edge.catalog_ref = input.catalog_path;

        const int desired = edge_desired(i);

        if (!edge_ops[i].empty()) {
            // Layer 3b: anchored openings at specified positions on this edge
            int cursor = 0;
            for (const auto* aop : edge_ops[i]) {
                int span_desired = aop->position_mm - aop->width_mm / 2 - post_w - cursor;
                auto result = ::solve(table, span_desired, /*prefer_over=*/true);
                edge.spans_mm.push_back(result.panels_mm);
                cursor = aop->position_mm + aop->width_mm / 2 + post_w;

                OpeningOut op;
                op.entity_id   = assign(aop->entity_id);
                op.edge_id     = edge.entity_id;
                op.position_mm = aop->position_mm;
                op.width_mm    = aop->width_mm;
                edge.openings.push_back(op);
            }
            auto result = ::solve(table, desired - cursor, /*prefer_over=*/true);
            edge.spans_mm.push_back(result.panels_mm);

        } else if (i == 0 && !input.unallocated_openings.empty()) {
            // Layer 3a: first unallocated opening placed at center of edge 0
            const auto& op_spec = input.unallocated_openings[0];
            const int   op_w    = op_spec.width_mm;
            const int   half    = (desired - op_w - 2 * post_w) / 2;

            auto left  = ::solve(table, half, /*prefer_over=*/true);
            auto right = ::solve(table, half, /*prefer_over=*/true);

            OpeningOut op;
            op.entity_id   = assign(op_spec.entity_id);
            op.edge_id     = edge.entity_id;
            op.position_mm = left.actual_mm + post_w + op_w / 2;
            op.width_mm    = op_w;

            edge.spans_mm = {left.panels_mm, right.panels_mm};
            edge.openings = {op};

        } else {
            // Layer 2: single span, no opening
            auto result = ::solve(table, desired, /*prefer_over=*/true);
            edge.spans_mm = {result.panels_mm};
        }

        out.edges.push_back(std::move(edge));
    }

    return out;
}

}  // namespace solver
