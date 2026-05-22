#pragma once

// Build version, exposed so demos and tests can identify the binary they're
// running against. Bumped manually with each release; not derived from git.

#include <string_view>

namespace tinycell::version {

inline constexpr int major = 0;
inline constexpr int minor = 1;
inline constexpr int patch = 0;
inline constexpr std::string_view string = "0.1.0";

} // namespace tinycell::version
