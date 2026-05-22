#pragma once

// PalletSpec — physical description of a pallet (the surface that receives a
// stack of boxes during palletizing). Width and length define the deck area;
// the stacking pattern (grid today; more later) is computed downstream from
// the pallet + box dimensions, not stored here.

#include <tinycell/units.hpp>

namespace tinycell::core {

struct PalletSpec {
    Length width;
    Length length;
};

} // namespace tinycell::core
