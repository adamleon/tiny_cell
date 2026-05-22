#pragma once

#include <filesystem>
#include <tinycell/model/arm.hpp>
#include <vector>

// One loader per equipment category, returning that category's strongly-typed
// entry vector. The polymorphic-variant alternative is rejected for as long as
// every downstream consumer reads catalogs per category (strategies match tasks
// by capability and consume only their own category). See docs/decisions.md
// for when this is revisited.
namespace tinycell::io {

std::vector<core::ArmEntry> load_arm_catalog(const std::filesystem::path& path);

} // namespace tinycell::io
