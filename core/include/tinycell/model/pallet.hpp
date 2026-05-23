#pragma once

// PalletSpec — description of a pallet ITEM TYPE (the surface that receives
// stacked items during palletizing; itself an item the cell can manipulate
// in principle, even if MVP strategies only place onto it). Composes
// ItemPhysical so pallets carry the same physical-property shape as boxes
// and any future item type. Stays a distinct type from BoxSpec — see the
// note in box.hpp about role-separation.
//
// Pallet-specific fields will accrue here over time (stacking-pattern
// hints, deck-feature flags, top-vs-side load capacity, …). The stacking
// pattern itself is computed downstream from box + pallet dimensions
// (data-model.md §1) and is not stored on the spec.

#include <tinycell/model/item.hpp>

namespace tinycell::core {

struct PalletSpec {
    ItemPhysical physical;
};

} // namespace tinycell::core
