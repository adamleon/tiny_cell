#pragma once
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>

// Robot-arm catalog: a per-arm POD spec and a JSON loader.
//
// The spec is the commercial + mechanical data needed by the cost model. JSON
// fields that are `null` get resolved to documented defaults at load time, so
// downstream code (the cost model, the solver) always sees concrete values.
//
// Units throughout: SI (m, kg, s, W, EUR). This is the cost-model boundary;
// callers in ECS-mm land convert at the call site.

namespace factory::robot_arm_catalog {

struct RobotArmSpec {
    std::string name;
    std::string urdf_path;                  // relative to the catalog file's directory
    float       price_purchase_eur;
    float       mass_robot_kg;
    float       payload_max_kg;
    float       reach_max_m;
    float       reach_min_m;                // 0 if not constrained
    float       speed_max_m_s;              // TCP
    float       power_idle_w;               // resolved from controller_class if absent
    float       power_peak_w;               // 0 if unknown (informational; not used by model)
    float       acceleration_max_m_s2;      // 0 = no constraint
    float       maintenance_cost_annual_eur;// resolved: 4% of price if absent
    float       lifetime_years;             // resolved: 14 if absent
    bool        regen_capable;
};

// Default lookups for fields that may be null in the JSON.
inline float power_idle_from_class(const std::string& cls) {
    if (cls == "small")  return 150.0f;
    if (cls == "medium") return 250.0f;
    if (cls == "large")  return 400.0f;
    return 250.0f;  // unknown class → assume medium
}

inline constexpr float kDefaultLifetimeYears        = 14.0f;
inline constexpr float kDefaultMaintenanceFraction  = 0.04f;  // of price_purchase_eur

namespace detail {

inline float opt_float(const nlohmann::json& j, const char* key, float fallback) {
    auto it = j.find(key);
    if (it == j.end() || it->is_null()) return fallback;
    return it->get<float>();
}

inline std::string opt_string(const nlohmann::json& j, const char* key,
                              const std::string& fallback) {
    auto it = j.find(key);
    if (it == j.end() || it->is_null()) return fallback;
    return it->get<std::string>();
}

inline bool opt_bool(const nlohmann::json& j, const char* key, bool fallback) {
    auto it = j.find(key);
    if (it == j.end() || it->is_null()) return fallback;
    return it->get<bool>();
}

}  // namespace detail

// Parse one JSON object into a RobotArmSpec, resolving optional fields to
// their documented defaults. Required fields are dereferenced directly and
// will throw nlohmann's exception if missing — that's intentional, a missing
// price or mass is a catalog error worth surfacing.
inline RobotArmSpec parse_entry(const nlohmann::json& j) {
    RobotArmSpec s{};
    s.name                = j.at("name").get<std::string>();
    s.urdf_path           = detail::opt_string(j, "urdf", "");
    s.price_purchase_eur  = j.at("price_purchase_eur").get<float>();
    s.mass_robot_kg       = j.at("mass_robot_kg").get<float>();
    s.payload_max_kg      = j.at("payload_max_kg").get<float>();
    s.reach_max_m         = j.at("reach_max_m").get<float>();
    s.reach_min_m         = detail::opt_float(j, "reach_min_m", 0.0f);
    s.speed_max_m_s       = j.at("speed_max_m_s").get<float>();

    const std::string controller_class =
        detail::opt_string(j, "controller_class", "medium");
    s.power_idle_w = detail::opt_float(j, "power_idle_w",
                                       power_idle_from_class(controller_class));

    s.power_peak_w        = detail::opt_float(j, "power_peak_w", 0.0f);
    s.acceleration_max_m_s2 = detail::opt_float(j, "acceleration_max_m_s2", 0.0f);
    s.lifetime_years      = detail::opt_float(j, "lifetime_years",
                                              kDefaultLifetimeYears);
    s.maintenance_cost_annual_eur =
        detail::opt_float(j, "maintenance_cost_annual_eur",
                          s.price_purchase_eur * kDefaultMaintenanceFraction);
    s.regen_capable       = detail::opt_bool(j, "regen_capable", false);
    return s;
}

// Load a catalog JSON file into a vector of resolved specs.
// Throws std::runtime_error on file-open / parse failure.
inline std::vector<RobotArmSpec> load(const std::filesystem::path& path) {
    std::ifstream in(path);
    if (!in) {
        throw std::runtime_error("robot_arm_catalog::load: cannot open "
                                 + path.string());
    }
    nlohmann::json doc;
    in >> doc;

    std::vector<RobotArmSpec> out;
    const auto& arr = doc.at("robots");
    out.reserve(arr.size());
    for (const auto& entry : arr) {
        out.push_back(parse_entry(entry));
    }
    return out;
}

}  // namespace factory::robot_arm_catalog
