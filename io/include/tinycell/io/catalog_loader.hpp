#pragma once

#include <filesystem>
#include <tinycell/model/arm.hpp>
#include <vector>

namespace tinycell::io {

std::vector<core::ArmEntry> load_arm_catalog(const std::filesystem::path& path);

} // namespace tinycell::io
