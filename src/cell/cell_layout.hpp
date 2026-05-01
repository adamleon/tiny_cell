#pragma once
#include <memory>
#include <vector>

// Pure data model — no threepp dependency.
// Nodes are corners; edges are fence sides between them.
// The builder (cell_builder.hpp) reads this and produces 3D geometry.

namespace cell {

// Axis-aligned bounding rectangle in edge-local 2D coordinates (mm).
// Used by the selection system. An empty box means not independently selectable.
struct LocalBox {
    float min_x = 0.f, max_x = 0.f;
    float min_z = 0.f, max_z = 0.f;
    bool empty() const { return max_x <= min_x; }
    static LocalBox make_empty() { return {}; }
};

// Reserved space on an edge where no fence is built.
// Polymorphic: subclass determines origin, selectability, and mobility.
// Default implementations provide the fail-safe behaviour described in the design:
//   mobility() = 0.0 (immovable) — safe if incorrectly loaded
//   collisionBox() = empty     — not independently selectable
//   isEditable()   = false
struct Opening {
    int desired_position_mm = 0;  // user intent; persists across solves

    virtual ~Opening() = default;
    virtual int      getWidth()     const = 0;
    virtual float    mobility()     const { return 0.0f; }
    virtual bool     isEditable()   const { return false; }
    virtual LocalBox collisionBox() const { return LocalBox::make_empty(); }
};

// Placed by the user on a specific edge. Stored on the edge.
// desired_position_mm stores user intent; actual solved position lives in the solve cache.
struct DeclaredOpening : public Opening {
    int   width_mm  = 0;
    float mobility_ = 0.0f;

    DeclaredOpening(int position_mm, int width, float mob = 0.0f)
        : width_mm(width), mobility_(mob)
    {
        desired_position_mm = position_mm;
    }

    int   getWidth()   const override { return width_mm; }
    float mobility()   const override { return mobility_; }
    bool  isEditable() const override { return true; }

    LocalBox collisionBox() const override {
        float half = static_cast<float>(width_mm) / 2.f;
        float pos  = static_cast<float>(desired_position_mm);
        return { pos - half, pos + half, -25.f, 25.f };
    }
};

// WorldFeatureOpening is derived at solve time from any world feature (machine, pillar,
// solid wall, conveyor) intersecting the edge. Not stored on edges.
// Inherits mobility() = 0.0 and collisionBox() = empty from Opening.

// NodeType is always solver output, never user input.
enum class NodeType {
    Post,
    // Future (priority order from design): None, Straight, RoundedCorner, AngledPost, Gap
};

struct Node {
    float    x    = 0.f;
    float    z    = 0.f;
    NodeType type = NodeType::Post;
};

struct Edge {
    int nodeA;
    int nodeB;

    // One fence-panel span per entry. user_openings sit between adjacent spans.
    // Invariant: spans_mm.size() == user_openings.size() + 1
    std::vector<std::vector<int>>                   spans_mm;
    std::vector<std::shared_ptr<DeclaredOpening>> user_openings;
};

struct CellLayout {
    std::vector<Node> nodes;
    std::vector<Edge> edges;

    int addNode(float x, float z, NodeType type = NodeType::Post) {
        nodes.push_back({x, z, type});
        return static_cast<int>(nodes.size()) - 1;
    }

    // Single span, no openings — backward-compatible convenience overload.
    void addEdge(int a, int b, std::vector<int> panels) {
        edges.push_back({a, b, {std::move(panels)}, {}});
    }

    // Full form: multiple spans interleaved with openings.
    // Callers must satisfy spans.size() == openings.size() + 1.
    void addEdge(int a, int b,
                 std::vector<std::vector<int>> spans,
                 std::vector<std::shared_ptr<DeclaredOpening>> openings)
    {
        edges.push_back({a, b, std::move(spans), std::move(openings)});
    }
};

} // namespace cell
