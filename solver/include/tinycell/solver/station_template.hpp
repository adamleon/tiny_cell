#pragma once

// Intra-station layout for a palletizer cell (step-6 M4 capstone). This is
// the "second placement problem" the architecture has always anticipated
// (solver.md "Geometry flattening & caching"): once a station is a CELL of
// equipment rather than a single machine, the equipment must be laid out
// WITHIN the station frame — separately from where the station sits on the
// floor (the inter-station Layer-3 placer).
//
// A robotic palletizer cell is an arm at the station origin serving one or
// more PALLET BUILD POSITIONS. The realistic configuration is TWO positions
// (the arm alternates between them to hide pallet-change dead time), which
// is what first makes this a genuine layout problem rather than a single
// fixed offset: the pallet zones must all sit within the arm's reach AND
// not collide with each other.
//
// CRUDEST-CONCRETE form (roadmap.md M4): the layout is a fixed TEMPLATE plus
// a FEASIBILITY CHECK — "if the templated approach is enough, the
// intra-station NLP shrinks to a feasibility check." The pallets are placed
// on a common reach-radius arc on the arm's +x working side, spread just far
// enough not to collide. If the template cannot fit them (pallet too large
// for the reach band, or too many to fit around the arm), it reports
// infeasible with a diagnostic rather than fabricating an unreachable layout
// (CLAUDE.md §1 — never invent past a constraint). A free NLP over the slot
// positions, rotated/facing zones, and a richer reachability model are LATER
// increments, earned when a real cell defeats the template.
//
// Pure core logic — no foreign lib (CLAUDE.md §1): the caller turns these
// slot centres into footprints and accumulates/buffers the hull via the
// adapter. This module only decides WHERE the pallets go and whether they
// fit.

#include <string>
#include <tinycell/geometry.hpp>
#include <tinycell/units.hpp>
#include <vector>

namespace tinycell::solver {

// One pallet build position within a palletizer cell, in the arm's STATION
// frame (arm at the origin). Axis-aligned at MVP; a facing/rotation refinement
// is deferred.
struct PalletSlot {
    core::Vec2 centre;  // pallet zone centre, station frame
};

// Result of laying out a palletizer cell. `feasible` is false (and `slots`
// empty) when the template cannot place the pallets within reach without
// collision; `diagnostic` then says why.
struct PalletizerCellLayout {
    std::vector<PalletSlot> slots;
    bool feasible = false;
    std::string diagnostic;
};

// Lay out `pallet_count` equal pallet build positions for an arm whose reach
// band is [reach_min, reach_max]. Each pallet occupies a footprint of
// (pallet_w + 2*margin) × (pallet_l + 2*margin). The slots are placed at a
// common radius on the +x side and spread over a symmetric arc so no two
// collide (conservatively: their bounding circles stay apart). Feasibility
// fails — with a diagnostic — when a pallet's footprint cannot fit inside the
// reach band, or when `pallet_count` pallets cannot be spread around the arm
// without collision. `pallet_count <= 0` returns an empty, feasible layout.
PalletizerCellLayout layout_palletizer_cell(core::Length reach_min,
                                            core::Length reach_max,
                                            core::Length pallet_w,
                                            core::Length pallet_l,
                                            core::Length margin,
                                            int pallet_count);

} // namespace tinycell::solver
