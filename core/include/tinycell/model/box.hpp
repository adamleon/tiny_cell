#pragma once

// BoxSpec — description of a box ITEM TYPE (a "box" being one of the items
// that flows through the cell). Composes ItemPhysical so the box gets the
// same physical-property shape every item type carries (width, length,
// height, mass, symmetry). BoxSpec stays a distinct type so PalletizeParams
// can express "the BoxSpec being placed" vs "the PalletSpec receiving it"
// without accidentally swapping the roles at call sites.
//
// Box-specific fields will accrue here over time (label class, surface
// texture, allowed grip strategies, …). For now BoxSpec is the bare
// ItemPhysical composition; the named type carries the role.

#include <tinycell/model/item.hpp>

namespace tinycell::core {

struct BoxSpec {
    ItemPhysical physical;
};

} // namespace tinycell::core
