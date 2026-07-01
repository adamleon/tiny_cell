#pragma once

// Boost.Geometry adapter. Per CLAUDE.md §0 + §6: this is the ONLY place
// boost/* may be #included; the public API speaks only core types
// (core::Polygon, core::Length). Units are stripped to SI meters at the
// adapter boundary and reattached on return — callers never see raw doubles.
//
// The operations here are step-5 prerequisites: convex hull (broad-phase
// fast-reject for collision; cheap "floor wanted" proxy for Layer-2 and
// the positional prior) and outward buffer (clearance bake into the
// station footprint at build time, per solver.md "Clearance baked into
// the cached footprint" + data-model.md §3.1).

#include <tinycell/geometry.hpp>
#include <tinycell/units.hpp>

namespace tinycell::adapters {

// Convex hull of the vertices of `in`. The result is a single closed
// polygon in core::Polygon (vertices in CCW order, first vertex NOT
// duplicated as last — that's the caller's choice if they want closure).
//
// Degenerate input (≤2 vertices, or all collinear) returns the input
// vertices unchanged: not a hull in the geometric sense, but the
// invariant "the result encloses every input point" still holds and
// downstream collision tests behave correctly.
core::Polygon convex_hull(const core::Polygon& in);

// Inflate `in` outward by `amount`. Used to bake the governing standards
// clearance into a footprint once at build time, so per-collision-check
// code can do plain overlap on already-buffered polygons (solver.md
// "Clearance baked into the cached footprint").
//
// Requires `amount > 0`; negative or zero amounts are a defect (the
// solver should not be calling buffer with a non-positive clearance —
// reject loudly per CLAUDE.md §1 "never clamp a bad value").
//
// The result is the outer boundary of the buffered region. For a
// convex input, the result is always a single convex polygon (the
// buffer of a convex set is convex); for a non-convex input the
// buffer can in principle produce holes or multiple components, in
// which case this function asserts — narrow-phase buffering of
// non-convex unions arrives with the StationFootprint cache and
// will need a richer return type then.
core::Polygon buffer_outward(const core::Polygon& in, core::Length amount);

} // namespace tinycell::adapters
