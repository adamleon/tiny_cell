#pragma once
#include <fstream>
#include <stdexcept>
#include <string>
#include <nlohmann/json.hpp>

// Pure catalog data helpers — no threepp dependency.
// Include this header in tests and tools that need catalog data
// without pulling in the 3D rendering layer.

namespace cell {

inline nlohmann::json loadCatalog(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("Cannot open catalog: " + path);
    return nlohmann::json::parse(f);
}

// The height of the fence edge (floor to panel top) equals the post height.
// Post height is the canonical source; panels may vary in height within a catalog,
// but the post defines the structural height of the cell boundary.
inline int catalogEdgeHeight(const nlohmann::json& catalog) {
    return catalog.at("post").at("height_mm").get<int>();
}

inline int catalogPostWidth(const nlohmann::json& catalog) {
    return catalog.at("post").at("width_mm").get<int>();
}

} // namespace cell
