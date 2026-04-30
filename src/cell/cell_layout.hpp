#pragma once
#include <vector>

// Pure data model — no threepp dependency.
// Nodes are corners/junctions; edges are fence spans between them.
// The builder (cell_builder.hpp) reads this and produces 3D geometry.

namespace cell {

// Extensible: add new types without changing buildNodeGroup callers.
// Each type maps to a different geometry strategy in cell_builder.hpp.
enum class NodeType {
    Post,           // standard 50mm post
    // Future:
    // RoundedCorner,  // Axelent rounded corner piece (mesh between two posts)
    // Pillar,         // wall stops before pillar; posts on both sides
    // FreeEnd,        // open end, post on one side only
    // None,           // concrete wall / no hardware needed
};

struct Node {
    float x = 0.0f;   // scene X (metres)
    float z = 0.0f;   // scene Z (metres)
    NodeType type = NodeType::Post;
};

struct Edge {
    int nodeA;
    int nodeB;
    std::vector<int> panels_mm;   // solved panel widths, largest first
    // Future: per-edge fence type / catalog reference
};

struct CellLayout {
    std::vector<Node> nodes;
    std::vector<Edge> edges;

    int addNode(float x, float z, NodeType type = NodeType::Post) {
        nodes.push_back({x, z, type});
        return static_cast<int>(nodes.size()) - 1;
    }

    void addEdge(int a, int b, std::vector<int> panels) {
        edges.push_back({a, b, std::move(panels)});
    }
};

} // namespace cell
