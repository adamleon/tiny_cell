#pragma once

// BoxSpec — physical description of a single item (a "box") that flows through
// the cell. Pure value type, no behavior. Used as a task parameter today;
// later it will also describe items the sources spawn and the sinks consume.

#include <tinycell/units.hpp>

namespace tinycell::core {

struct BoxSpec {
    Length width;
    Length length;
    Length height;
    Mass mass;
};

} // namespace tinycell::core
