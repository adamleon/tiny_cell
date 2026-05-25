#pragma once

// 2D geometry primitives in core/. Frame-aware as of step 5 B.2 (the
// step that introduces continuous placement). Earlier code that
// pre-dates frames continues to use Vec2 / Polygon as bare values in
// an implicit local frame (e.g. catalog footprints) — that's fine; a
// polygon gains an explicit frame when a placement step puts it
// somewhere.
//
// Frame system rules (architecture.md §4):
//   * FrameId is its own id space, NEVER an entt::entity.
//   * Pose2D ALWAYS carries a FrameId.
//   * Resolution composes transforms; triplet addition is the #1 bug
//     to prevent (see the §4.3 regression test in tests/).

#include <cassert>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <mp-units/systems/si.h>
#include <stdexcept>
#include <tinycell/units.hpp>
#include <vector>

namespace tinycell::core {

struct Vec2 {
    Length x;
    Length y;
};

// Polygon vertices in declaration order. Closure is implicit (first
// vertex == last is NOT required by callers; adapters that need closed
// rings close on demand).
using Polygon = std::vector<Vec2>;

// ---- Frame system ------------------------------------------------------

// FrameId — its own id space (architecture.md §4.1). Wraps a uint64_t.
//   * FrameId{0} — sentinel for "no frame" / "null parent" (only the
//                  world frame has a null parent).
//   * FrameId{1} — convention: the world frame.
//   * FrameId{N} for N >= 2 — solver-minted frames.
//
// A strong typedef so a raw uint64_t can never accidentally be passed
// where a FrameId is expected (and vice versa). Comparable and
// hashable; otherwise opaque.
struct FrameId {
    std::uint64_t value;

    constexpr bool operator==(const FrameId&) const = default;
    constexpr auto operator<=>(const FrameId&) const = default;
};

inline constexpr FrameId kNullFrame{0};
inline constexpr FrameId kWorldFrame{1};

constexpr bool is_null(FrameId id) { return id == kNullFrame; }

// Pose2D — a position + orientation IN A SPECIFIC FRAME
// (architecture.md §4.2). Frame carriage is the load-bearing part: a
// "pose" without a frame is ambiguous and lets every spatial bug back
// in.
struct Pose2D {
    Length x;
    Length y;
    Angle theta;
    FrameId frame;
};

// Transform2D — the rigid 2D transform (rotation + translation) as a
// pure math object. No FrameId. Compose, invert, apply to Vec2 / Pose2D.
//
// Semantics: applying T to point p produces
//     p' = R(theta) * p + (tx, ty)
// where R(theta) is the standard 2D rotation matrix (right-handed,
// CCW positive, consistent with core's right-handed math convention).
struct Transform2D {
    Length tx;
    Length ty;
    Angle theta;

    static Transform2D identity() {
        using namespace mp_units;
        using namespace mp_units::si;
        return Transform2D{0.0 * metre, 0.0 * metre, 0.0 * radian};
    }
};

inline Vec2 apply(const Transform2D& t, const Vec2& p) {
    using namespace mp_units;
    const double c = std::cos(t.theta.numerical_value_in(si::radian));
    const double s = std::sin(t.theta.numerical_value_in(si::radian));
    // Strip and reattach metres at the math boundary; mp-units doesn't
    // multiply a scalar trig coefficient with a Length quantity directly
    // without an explicit `*` form, so this stays explicit and obvious.
    const double px = p.x.numerical_value_in(si::metre);
    const double py = p.y.numerical_value_in(si::metre);
    const double tx_m = t.tx.numerical_value_in(si::metre);
    const double ty_m = t.ty.numerical_value_in(si::metre);
    return Vec2{
        (c * px - s * py + tx_m) * si::metre,
        (s * px + c * py + ty_m) * si::metre,
    };
}

// Compose two transforms — applying compose(A, B) to a point p produces
// the same result as applying B first then A:
//     apply(compose(A, B), p) == apply(A, apply(B, p))
// Mirrors matrix multiplication A * B; the leftmost transform is applied
// last. Frame composition uses this with A = parent's transform-to-world
// and B = child's transform-in-parent (i.e. compose-with-parent-on-left).
inline Transform2D compose(const Transform2D& a, const Transform2D& b) {
    using namespace mp_units;
    const double ca = std::cos(a.theta.numerical_value_in(si::radian));
    const double sa = std::sin(a.theta.numerical_value_in(si::radian));
    const double bx = b.tx.numerical_value_in(si::metre);
    const double by = b.ty.numerical_value_in(si::metre);
    const double ax = a.tx.numerical_value_in(si::metre);
    const double ay = a.ty.numerical_value_in(si::metre);
    return Transform2D{
        (ca * bx - sa * by + ax) * si::metre,
        (sa * bx + ca * by + ay) * si::metre,
        a.theta + b.theta,
    };
}

// Inverse of a rigid transform. inverse(T) ∘ T == identity within fp eps.
inline Transform2D inverse(const Transform2D& t) {
    using namespace mp_units;
    const double c = std::cos(t.theta.numerical_value_in(si::radian));
    const double s = std::sin(t.theta.numerical_value_in(si::radian));
    const double tx_m = t.tx.numerical_value_in(si::metre);
    const double ty_m = t.ty.numerical_value_in(si::metre);
    return Transform2D{
        (-c * tx_m - s * ty_m) * si::metre,
        ( s * tx_m - c * ty_m) * si::metre,
        -t.theta,
    };
}

// FrameTable — the parent-link store (architecture.md §4.1). One row
// per FrameId. World is implicit at FrameId{1}; the caller does NOT
// have to add it, but if they do, they pass {parent = kNullFrame,
// pose = identity}. New frames are minted by add() returning the new
// FrameId; FrameIds are monotonic within a table's lifetime (no recycle).
//
// Storage: vector indexed by FrameId.value. Index 0 (kNullFrame) is
// unused; index 1 (kWorldFrame) is always present with null parent.
class FrameTable {
public:
    FrameTable() {
        // Pre-allocate the null-sentinel and the world row.
        rows_.resize(2);
        rows_[kNullFrame.value] = Row{kNullFrame, Transform2D::identity()};
        rows_[kWorldFrame.value] = Row{kNullFrame, Transform2D::identity()};
    }

    // Add a new frame parented at `parent`, with transform `pose_in_parent`
    // expressed as a Transform2D (translation + rotation of the child
    // frame's origin in the parent frame's coords). Returns the new
    // FrameId. Solver minting a child of a non-existent parent is a
    // defect per CLAUDE.md §1 — reject loudly.
    FrameId add(FrameId parent, const Transform2D& pose_in_parent) {
        if (is_null(parent)) {
            throw std::invalid_argument("FrameTable::add: parent must not be the null frame");
        }
        if (parent.value >= rows_.size()) {
            throw std::invalid_argument("FrameTable::add: parent frame does not exist");
        }
        const FrameId id{static_cast<std::uint64_t>(rows_.size())};
        rows_.push_back(Row{parent, pose_in_parent});
        return id;
    }

    // Resolve `id` to a transform that takes coordinates in `id`'s frame
    // to coordinates in the world frame. Walks the parent chain composing
    // transforms left-to-right (root-most first). Cycle and depth guard
    // assert/throw — they cannot happen if the table is constructed via
    // add() and never edited directly, but the check costs nothing and
    // catches malformed input.
    Transform2D resolve_to_world(FrameId id) const {
        if (is_null(id)) {
            throw std::invalid_argument("FrameTable::resolve_to_world: cannot resolve the null frame");
        }
        if (id.value >= rows_.size()) {
            throw std::invalid_argument("FrameTable::resolve_to_world: frame id out of range");
        }
        if (id == kWorldFrame) {
            return Transform2D::identity();
        }

        // Walk to root, accumulating transforms. We walk child→parent and
        // build T_world_id = T_world_parent * T_parent_id incrementally.
        Transform2D acc = Transform2D::identity();
        FrameId cursor = id;
        std::size_t hops = 0;
        constexpr std::size_t kMaxDepth = 64;  // ample for any realistic nesting
        while (cursor != kWorldFrame) {
            if (hops++ >= kMaxDepth) {
                throw std::runtime_error("FrameTable::resolve_to_world: parent chain exceeds depth bound — cycle or malformed table");
            }
            const Row& row = rows_[cursor.value];
            acc = compose(row.pose_in_parent, acc);
            cursor = row.parent;
            if (is_null(cursor)) {
                throw std::runtime_error("FrameTable::resolve_to_world: parent chain reached null without hitting world — orphan frame");
            }
        }
        return acc;
    }

    std::size_t size() const { return rows_.size(); }

private:
    struct Row {
        FrameId parent;
        Transform2D pose_in_parent;
    };
    std::vector<Row> rows_;
};

// express(pose, target_frame, table) — re-express `pose` in
// `target_frame`'s coordinates (architecture.md §4.3). Composition of
// transforms, NEVER component-wise (xa+xb, ya+yb, θa+θb) — that's the
// #1 bug this code exists to prevent.
//
// CLAUDE.md §1: do not call this per object/pair inside a collision /
// clearance / NLP kernel. Footprints get flattened once at cache build
// time; the cached world copy is read flat.
inline Pose2D express(const Pose2D& pose, FrameId target_frame, const FrameTable& table) {
    using namespace mp_units;
    if (pose.frame == target_frame) {
        return pose;
    }
    const Transform2D pose_to_world = table.resolve_to_world(pose.frame);
    const Transform2D target_to_world = table.resolve_to_world(target_frame);

    // The pose-as-transform is { pose.x, pose.y, pose.theta } in pose.frame.
    // To get the pose's POSITION in world we apply pose_to_world to the
    // pose's translation, and add pose.theta to the world rotation of
    // pose.frame.
    const Transform2D pose_in_world =
        compose(pose_to_world, Transform2D{pose.x, pose.y, pose.theta});

    // Then invert target_to_world to bring it into target_frame coords.
    const Transform2D pose_in_target =
        compose(inverse(target_to_world), pose_in_world);

    return Pose2D{
        pose_in_target.tx,
        pose_in_target.ty,
        pose_in_target.theta,
        target_frame,
    };
}

} // namespace tinycell::core
