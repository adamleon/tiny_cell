#pragma once

#include <tinycell/units.hpp>
#include <vector>

namespace tinycell::core {

struct Vec2 {
    Length x;
    Length y;
};

using Polygon = std::vector<Vec2>;

} // namespace tinycell::core
