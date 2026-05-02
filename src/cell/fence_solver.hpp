#pragma once
#include <algorithm>
#include <fstream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>
#include <nlohmann/json.hpp>

struct LookupResult {
    int              actual_mm;
    std::vector<int> panels_mm;
};

struct LookupTable {
    std::map<int, std::vector<int>> entries;
    int post_width_mm = 50;
};

inline LookupTable loadTable(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("Cannot open: " + path);
    auto j = nlohmann::json::parse(f);
    LookupTable table;
    table.post_width_mm = j.value("post_width_mm", 50);
    for (const auto& e : j["entries"])
        table.entries[e["total_mm"].get<int>()] = e["panels_mm"].get<std::vector<int>>();
    return table;
}

// prefer_over=true  → nearest combination >= target (rounds up)
// prefer_over=false → nearest combination <= target (rounds down)
inline LookupResult lookup(const LookupTable& table, int target_mm, bool prefer_over) {
    const auto& m = table.entries;
    if (prefer_over) {
        auto it = m.lower_bound(target_mm);
        if (it == m.end()) --it;
        return {it->first, it->second};
    } else {
        auto it = m.upper_bound(target_mm);
        if (it != m.begin()) --it;
        else ++it;
        return {it->first, it->second};
    }
}

// For desired lengths > max_lookup (2100mm), prepend 1000mm panels greedily,
// then look up the remainder. Result is sorted largest-first.
inline LookupResult solve(const LookupTable& table, int desired_mm, bool prefer_over) {
    std::vector<int> prefix;
    const int post_w = table.post_width_mm;
    while (desired_mm > 2100) {
        prefix.push_back(1000);
        desired_mm -= (1000 + post_w);
    }
    auto tail = lookup(table, desired_mm, prefer_over);
    prefix.insert(prefix.end(), tail.panels_mm.begin(), tail.panels_mm.end());
    std::sort(prefix.begin(), prefix.end(), std::greater<int>());

    int n      = static_cast<int>(prefix.size());
    int actual = (n == 0) ? 0 : -post_w;
    for (int w : prefix) actual += w + post_w;
    return {actual, prefix};
}
