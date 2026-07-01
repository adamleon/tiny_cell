#include "tinycell/io/workflow_loader.hpp"
#include "tinycell/io/parse_error.hpp"

#include <fstream>
#include <nlohmann/json.hpp>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

#include <tinycell/model/item.hpp>
#include <tinycell/model/port.hpp>

namespace tinycell::io {

namespace {

using nlohmann::json;
using namespace mp_units;
namespace tc = tinycell::core;

// read_* helpers mirror catalog_loader.cpp (each: presence -> JSON type ->
// range -> value; ParseError naming the field on any failure). Kept TU-local
// per the per-loader pattern (decisions.md — loaders share only these helpers,
// not control flow); catalog_loader.cpp carries its own copies. Units are
// attached by the caller after the helper returns.

double read_positive_double(const json& j, const char* key, const std::filesystem::path& src) {
    if (!j.contains(key)) throw ParseError(src, std::string("missing field: ") + key);
    if (!j.at(key).is_number()) throw ParseError(src, std::string("field is not a number: ") + key);
    const double v = j.at(key).get<double>();
    if (v <= 0.0) throw ParseError(src, std::string("field must be > 0: ") + key);
    return v;
}

// Any-sign number — for world coordinates, which are legitimately negative.
double read_number(const json& j, const char* key, const std::filesystem::path& src) {
    if (!j.contains(key)) throw ParseError(src, std::string("missing field: ") + key);
    if (!j.at(key).is_number()) throw ParseError(src, std::string("field is not a number: ") + key);
    return j.at(key).get<double>();
}

std::string read_non_empty_string(const json& j, const char* key, const std::filesystem::path& src) {
    if (!j.contains(key)) throw ParseError(src, std::string("missing field: ") + key);
    if (!j.at(key).is_string()) throw ParseError(src, std::string("field is not a string: ") + key);
    std::string v = j.at(key).get<std::string>();
    if (v.empty()) throw ParseError(src, std::string("field must not be empty: ") + key);
    return v;
}

int read_positive_int(const json& j, const char* key, const std::filesystem::path& src) {
    if (!j.contains(key)) throw ParseError(src, std::string("missing field: ") + key);
    if (!j.at(key).is_number_integer()) throw ParseError(src, std::string("field is not an integer: ") + key);
    const int v = j.at(key).get<int>();
    if (v <= 0) throw ParseError(src, std::string("field must be > 0: ") + key);
    return v;
}

const json& require_object(const json& j, const char* key, const std::filesystem::path& src) {
    if (!j.contains(key)) throw ParseError(src, std::string("missing field: ") + key);
    if (!j.at(key).is_object()) throw ParseError(src, std::string("field must be an object: ") + key);
    return j.at(key);
}

// Degrees at the file boundary -> radians in core (units.hpp convention).
tc::Angle read_angle_deg(const json& j, const char* key, const std::filesystem::path& src) {
    const double deg = read_number(j, key, src);// any sign
    return (deg * std::numbers::pi / 180.0) * si::radian;
}

tc::RotationalSymmetry parse_symmetry(const json& j, const std::filesystem::path& src) {
    const std::string kind = read_non_empty_string(j, "kind", src);
    if (kind == "continuous") return tc::symmetry::continuous();
    if (kind == "asymmetric") return tc::symmetry::asymmetric();
    if (kind == "discrete") {
        const int period = read_positive_int(j, "period_deg", src);
        // symmetry::discrete enforces period in (0, 360); rewrap its throw as a
        // ParseError so the diagnostic names the source file.
        try {
            return tc::symmetry::discrete(period);
        } catch (const std::invalid_argument& e) {
            throw ParseError(src, std::string("symmetry: ") + e.what());
        }
    }
    throw ParseError(src, "symmetry.kind must be continuous | discrete | asymmetric, got: " + kind);
}

tc::ItemPhysical parse_item_physical(const json& j, const std::filesystem::path& src) {
    return tc::ItemPhysical{
        .width = read_positive_double(j, "width_m", src) * si::metre,
        .length = read_positive_double(j, "length_m", src) * si::metre,
        .height = read_positive_double(j, "height_m", src) * si::metre,
        .mass = read_positive_double(j, "mass_kg", src) * si::kilogram,
        .symmetry = parse_symmetry(require_object(j, "symmetry", src), src),
    };
}

tc::PortDirection parse_role(const std::string& role, const std::filesystem::path& src) {
    if (role == "output") return tc::PortDirection::Output;
    if (role == "input") return tc::PortDirection::Input;
    throw ParseError(src, "anchor.role must be \"input\" or \"output\", got: " + role);
}

tc::TaskParams parse_params(const std::string& kind, const json& j, const std::filesystem::path& src) {
    if (kind == "palletize") {
        return tc::PalletizeParams{
            .item_id = read_non_empty_string(j, "item_id", src),
            .item = tc::BoxSpec{.physical = parse_item_physical(require_object(j, "item", src), src)},
            .pallet = tc::PalletSpec{.physical = parse_item_physical(require_object(j, "pallet", src), src)},
            .box_count = read_positive_int(j, "box_count", src),
        };
    }
    if (kind == "anchor") {
        return tc::AnchorParams{
            .name = read_non_empty_string(j, "name", src),
            .role = parse_role(read_non_empty_string(j, "role", src), src),
            .world_x = read_number(j, "world_x_m", src) * si::metre,
            .world_y = read_number(j, "world_y_m", src) * si::metre,
            .world_theta = read_angle_deg(j, "world_theta_deg", src),
        };
    }
    if (kind == "transport") {
        return tc::TransportParams{
            .source_task_id = read_non_empty_string(j, "source_task_id", src),
            .source_port_name = read_non_empty_string(j, "source_port_name", src),
            .sink_task_id = read_non_empty_string(j, "sink_task_id", src),
            .sink_port_name = read_non_empty_string(j, "sink_port_name", src),
        };
    }
    throw ParseError(src, "task.kind must be palletize | anchor | transport, got: " + kind);
}

tc::Task parse_task(const json& j, const std::filesystem::path& src) {
    return tc::Task{
        .id = read_non_empty_string(j, "id", src),
        .params = parse_params(read_non_empty_string(j, "kind", src), j, src),
        .target_ct_per_item =
                tc::CycleTimePerItem{read_positive_double(j, "target_ct_per_item_s", src) * si::second},
    };
}

} // namespace

std::vector<core::Task> load_workflow(const std::filesystem::path& path) {
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
    if (!doc.contains("workflow") || !doc.at("workflow").is_array()) {
        throw ParseError(path, "missing or non-array field: workflow");
    }

    const auto& arr = doc.at("workflow");
    std::vector<core::Task> out;
    out.reserve(arr.size());
    for (std::size_t i = 0; i < arr.size(); ++i) {
        try {
            out.push_back(parse_task(arr[i], path));
        } catch (const ParseError&) {
            throw; // already source-located
        } catch (const json::exception& e) {
            throw ParseError(path, "workflow[" + std::to_string(i) + "]: " + e.what());
        }
    }
    return out;
}

} // namespace tinycell::io
