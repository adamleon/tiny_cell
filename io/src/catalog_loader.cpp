#include "tinycell/io/catalog_loader.hpp"
#include "tinycell/io/parse_error.hpp"

#include <fstream>
#include <nlohmann/json.hpp>
#include <string>
#include <vector>

namespace tinycell::io {

namespace {

using nlohmann::json;
using namespace mp_units;
namespace tc = tinycell::core;

// Throws ParseError if `key` is missing, wrong-typed, or fails `predicate`.
double read_positive_double(const json& j, const char* key, const std::filesystem::path& src) {
    if (!j.contains(key)) {
        throw ParseError(src, std::string("missing field: ") + key);
    }
    if (!j.at(key).is_number()) {
        throw ParseError(src, std::string("field is not a number: ") + key);
    }
    double v = j.at(key).get<double>();
    if (v <= 0.0) {
        throw ParseError(src, std::string("field must be > 0: ") + key);
    }
    return v;
}

double read_non_negative_double(const json& j, const char* key, const std::filesystem::path& src) {
    if (!j.contains(key)) {
        throw ParseError(src, std::string("missing field: ") + key);
    }
    if (!j.at(key).is_number()) {
        throw ParseError(src, std::string("field is not a number: ") + key);
    }
    double v = j.at(key).get<double>();
    if (v < 0.0) {
        throw ParseError(src, std::string("field must be >= 0: ") + key);
    }
    return v;
}

std::string read_non_empty_string(const json& j, const char* key, const std::filesystem::path& src) {
    if (!j.contains(key)) {
        throw ParseError(src, std::string("missing field: ") + key);
    }
    if (!j.at(key).is_string()) {
        throw ParseError(src, std::string("field is not a string: ") + key);
    }
    std::string v = j.at(key).get<std::string>();
    if (v.empty()) {
        throw ParseError(src, std::string("field must not be empty: ") + key);
    }
    return v;
}

tc::Polygon read_footprint_polygon(const json& j, const std::filesystem::path& src) {
    if (!j.contains("footprint_polygon_m")) {
        throw ParseError(src, "missing field: footprint_polygon_m");
    }
    const auto& arr = j.at("footprint_polygon_m");
    if (!arr.is_array()) {
        throw ParseError(src, "footprint_polygon_m must be an array of [x,y] pairs");
    }
    if (arr.size() < 3) {
        throw ParseError(src, "footprint_polygon_m must have at least 3 vertices");
    }
    tc::Polygon polygon;
    polygon.reserve(arr.size());
    for (std::size_t i = 0; i < arr.size(); ++i) {
        const auto& vertex = arr[i];
        if (!vertex.is_array() || vertex.size() != 2) {
            throw ParseError(src, "footprint_polygon_m vertex must be [x,y]");
        }
        if (!vertex[0].is_number() || !vertex[1].is_number()) {
            throw ParseError(src, "footprint_polygon_m vertex coordinates must be numbers");
        }
        polygon.push_back(tc::Vec2{
            .x = vertex[0].get<double>() * si::metre,
            .y = vertex[1].get<double>() * si::metre,
        });
    }
    return polygon;
}

tc::ReachEnvelope read_reach_envelope(const json& j, const std::filesystem::path& src) {
    if (!j.contains("reach_envelope")) {
        throw ParseError(src, "missing field: reach_envelope");
    }
    const auto& re = j.at("reach_envelope");
    if (!re.is_object() || !re.contains("type")) {
        throw ParseError(src, "reach_envelope must be an object with a 'type' field");
    }
    const auto type = re.at("type").get<std::string>();
    if (type != "radius") {
        throw ParseError(src, "reach_envelope.type only 'radius' is supported in step 1");
    }
    if (!re.contains("min_m") || !re.contains("max_m")) {
        throw ParseError(src, "reach_envelope radius requires min_m and max_m");
    }
    const auto min_m = re.at("min_m").get<double>();
    const auto max_m = re.at("max_m").get<double>();
    if (min_m < 0.0) {
        throw ParseError(src, "reach_envelope.min_m must be >= 0");
    }
    if (max_m <= min_m) {
        throw ParseError(src, "reach_envelope.max_m must be > min_m");
    }
    return tc::ReachEnvelope{
        .min_radius = min_m * si::metre,
        .max_radius = max_m * si::metre,
    };
}

bool read_required_bool(const json& j, const char* key, const std::filesystem::path& src) {
    if (!j.contains(key) || !j.at(key).is_boolean()) {
        throw ParseError(src, std::string("missing or non-boolean field: ") + key);
    }
    return j.at(key).get<bool>();
}

tc::ArmEntry parse_arm_entry(const json& j, const std::filesystem::path& src) {
    return tc::ArmEntry{
        .id = read_non_empty_string(j, "id", src),
        .model_name = read_non_empty_string(j, "model_name", src),
        .family = read_non_empty_string(j, "family", src),
        .controller_class = read_non_empty_string(j, "controller_class", src),
        .footprint = read_footprint_polygon(j, src),
        .reach = read_reach_envelope(j, src),
        .payload_max = read_positive_double(j, "payload_max_kg", src) * si::kilogram,
        .mass = read_positive_double(j, "mass_kg", src) * si::kilogram,
        .max_speed = read_positive_double(j, "max_speed_m_s", src) * (si::metre / si::second),
        .max_acceleration = read_positive_double(j, "max_acceleration_m_s2", src)
                            * (si::metre / (si::second * si::second)),
        .repeatability = read_positive_double(j, "repeatability_mm", src) * 0.001 * si::metre,
        .power_peak = read_positive_double(j, "power_peak_w", src) * si::watt,
        .power_idle = read_non_negative_double(j, "power_idle_w", src) * si::watt,
        .list_price_eur = read_positive_double(j, "list_price_eur", src),
        // 365.25 days/year, 86400 s/day.
        .lifetime = read_positive_double(j, "lifetime_years", src) * 365.25 * 86400.0 * si::second,
        .maintenance_annual_fraction =
            read_non_negative_double(j, "maintenance_annual_fraction", src),
        .regen_capable = read_required_bool(j, "regen_capable", src),
    };
}

} // namespace

std::vector<core::ArmEntry> load_arm_catalog(const std::filesystem::path& path) {
    std::ifstream input(path);
    if (!input.is_open()) {
        throw ParseError(path, "could not open file");
    }

    json doc;
    try {
        input >> doc;
    } catch (const json::parse_error& e) {
        throw ParseError(path, std::string("JSON parse error: ") + e.what());
    }

    if (!doc.is_object()) {
        throw ParseError(path, "top-level JSON must be an object");
    }
    if (!doc.contains("category") || doc.at("category").get<std::string>() != "arm") {
        throw ParseError(path, "expected category=\"arm\"");
    }
    if (!doc.contains("entries") || !doc.at("entries").is_array()) {
        throw ParseError(path, "missing or non-array field: entries");
    }

    std::vector<core::ArmEntry> out;
    out.reserve(doc.at("entries").size());
    for (std::size_t i = 0; i < doc.at("entries").size(); ++i) {
        try {
            out.push_back(parse_arm_entry(doc.at("entries")[i], path));
        } catch (const ParseError&) {
            throw;
        } catch (const json::exception& e) {
            throw ParseError(path,
                             "entries[" + std::to_string(i) + "]: " + e.what());
        }
    }
    return out;
}

} // namespace tinycell::io
