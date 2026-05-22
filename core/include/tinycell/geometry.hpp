#pragma once

// 2D geometry primitives in core/. These are frame-LESS for step 1 — a polygon
// is just a list of points in some implicit local frame (e.g. the owning
// equipment's frame). Explicit FrameId tracking and Pose2D arrive when the
// solver starts composing transforms across nested frames (architecture.md §4).

#include <tinycell/units.hpp>
#include <vector>

namespace tinycell::core {

struct Vec2 {
    Length x;
    Length y;
};

// Polygon vertices in declaration order. Not required to be convex; closure
// is implicit (first vertex == last vertex is not required). Geometry kernels
// (Boost.Geometry, in step 5) will close at the call site.
using Polygon = std::vector<Vec2>;

} // namespace tinycell::core
