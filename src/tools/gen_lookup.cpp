#include <nlohmann/json.hpp>

#include <algorithm>
#include <fstream>
#include <iostream>
#include <map>
#include <numeric>
#include <string>
#include <vector>

using json = nlohmann::json;
using Seq  = std::vector<int>;

static bool isBetter(const Seq& candidate, const Seq& existing) {
    if (candidate.size() < existing.size()) return true;
    if (candidate.size() > existing.size()) return false;
    // Same count: prefer larger panels (higher sum = fewer, bigger panels)
    int sc = std::accumulate(candidate.begin(), candidate.end(), 0);
    int se = std::accumulate(existing.begin(),  existing.end(),  0);
    return sc > se;
}

int main(int argc, char* argv[]) {
    const std::string catalogPath = (argc > 1)
        ? argv[1]
        : "assets/components/fences/axelent_x-guard/catalog.json";
    const std::string outputPath = (argc > 2)
        ? argv[2]
        : "assets/components/fences/axelent_x-guard/combinations.json";

    std::ifstream f(catalogPath);
    if (!f) {
        std::cerr << "Cannot open: " << catalogPath << "\n";
        return 1;
    }
    json catalog = json::parse(f);

    const int post_w = catalog["post"]["width_mm"].get<int>();
    constexpr int MAX_MM = 2100;

    std::vector<int> panels;
    for (const auto& p : catalog["panels"])
        panels.push_back(p["width_mm"].get<int>());

    // Iterative DP: dp[visual_mm] = best panel sequence
    std::map<int, Seq> dp;

    // Seed with single panels
    for (int w : panels)
        dp[w] = {w};

    // Extend: sweep in ascending order; nxt > t always so no revisits needed
    for (auto& [t, seq] : dp) {
        for (int w : panels) {
            int nxt = t + post_w + w;
            if (nxt > MAX_MM) continue;
            Seq cand = seq;
            cand.push_back(w);
            auto it = dp.find(nxt);
            if (it == dp.end() || isBetter(cand, it->second))
                dp[nxt] = cand;
        }
    }

    // Build output
    json entries = json::array();
    for (const auto& [total, seq] : dp) {
        json e;
        e["total_mm"]  = total;
        e["panels_mm"] = seq;
        entries.push_back(e);
    }

    json out;
    out["post_width_mm"]  = post_w;
    out["max_lookup_mm"]  = MAX_MM;
    out["entries"]        = entries;

    std::ofstream o(outputPath);
    if (!o) {
        std::cerr << "Cannot write: " << outputPath << "\n";
        return 1;
    }
    o << out.dump(2) << "\n";
    std::cout << "Wrote " << dp.size() << " entries to " << outputPath << "\n";
    return 0;
}
