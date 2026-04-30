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

using LookupTable = std::map<int, std::vector<int>>;

inline LookupTable loadTable(const std::string& path) {
    std::ifstream f(path);
    if (!f) throw std::runtime_error("Cannot open: " + path);
    auto j = nlohmann::json::parse(f);
    LookupTable table;
    for (const auto& e : j["entries"])
        table[e["total_mm"].get<int>()] = e["panels_mm"].get<std::vector<int>>();
    return table;
}

// prefer_over=true  → nearest combination >= target (rounds up)
// prefer_over=false → nearest combination <= target (rounds down)
inline LookupResult lookup(const LookupTable& table, int target_mm, bool prefer_over) {
    if (prefer_over) {
        auto it = table.lower_bound(target_mm);
        if (it == table.end()) --it;
        return {it->first, it->second};
    } else {
        auto it = table.upper_bound(target_mm);
        if (it != table.begin()) --it;
        else ++it;
        return {it->first, it->second};
    }
}

// For desired lengths > max_lookup (2100mm), prepend 1000mm panels greedily,
// then look up the remainder. Result is sorted largest-first.
inline LookupResult solve(const LookupTable& table, int desired_mm, bool prefer_over) {
    std::vector<int> prefix;
    while (desired_mm > 2100) {
        prefix.push_back(1000);
        desired_mm -= 1050;   // 1000mm panel + 50mm post
    }
    auto tail = lookup(table, desired_mm, prefer_over);
    prefix.insert(prefix.end(), tail.panels_mm.begin(), tail.panels_mm.end());
    std::sort(prefix.begin(), prefix.end(), std::greater<int>());

    int n      = static_cast<int>(prefix.size());
    int actual = (n == 0) ? 0 : -50;
    for (int w : prefix) actual += w + 50;
    return {actual, prefix};
}
