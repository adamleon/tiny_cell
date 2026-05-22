#pragma once

// Catalog loaders — one per equipment category, each returning a strongly-
// typed vector of specs. The polymorphic-variant alternative was rejected
// for as long as every downstream consumer reads catalogs per category
// (strategies match tasks by capability and only read their own category).
// See docs/decisions.md for when this is revisited.

#include <filesystem>
#include <tinycell/model/arm.hpp>
#include <tinycell/model/pusher.hpp>
#include <vector>

namespace tinycell::io {

// Loads an arm catalog from JSON at `path`. The file must declare
// `category: "arm"` and an `entries` array following the schema in
// data-model.md §3 + the asset's own `_notes`. Validates range invariants
// (payloads/reaches/powers positive, footprint has ≥ 3 vertices, etc.)
// and throws ParseError with a source-locating message on any violation.
// On success, returns the entries in catalog declaration order.
std::vector<core::ArmSpec> load_arm_catalog(const std::filesystem::path& path);

// Loads a pusher catalog from JSON at `path`. Same contract as
// load_arm_catalog (per-category loader — decisions.md): file must declare
// `category: "pusher"` and an `entries` array; range invariants are checked
// at parse time; ParseError thrown on any violation with a source-locating
// message. Returned in catalog declaration order.
std::vector<core::PusherSpec> load_pusher_catalog(const std::filesystem::path& path);

} // namespace tinycell::io
