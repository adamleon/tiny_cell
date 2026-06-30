#include <filesystem>
#include <fstream>
#include <gtest/gtest.h>
#include <mp-units/systems/si.h>
#include <tinycell/io/catalog_loader.hpp>
#include <tinycell/io/parse_error.hpp>

namespace {

using namespace mp_units;
namespace io = tinycell::io;
namespace tc = tinycell::core;

std::filesystem::path repo_root() {
    return std::filesystem::path(TINYCELL_REPO_ROOT);
}

class TempJsonFile {
public:
    explicit TempJsonFile(const std::string& content) {
        path_ = std::filesystem::temp_directory_path()
                / ("tinycell_test_" + std::to_string(counter_++) + ".json");
        std::ofstream{path_} << content;
    }
    ~TempJsonFile() {
        std::error_code ec;
        std::filesystem::remove(path_, ec);
    }
    const std::filesystem::path& path() const { return path_; }

private:
    std::filesystem::path path_;
    static inline int counter_ = 0;
};

} // namespace

TEST(ArmCatalog, LoadsRealKukaCatalog) {
    auto entries = io::load_arm_catalog(repo_root() / "assets" / "arm" / "kuka" / "catalog.json");
    EXPECT_EQ(entries.size(), 11U);
}

TEST(ArmCatalog, FieldsRoundTrip) {
    auto entries = io::load_arm_catalog(repo_root() / "assets" / "arm" / "kuka" / "catalog.json");
    const auto& kr4 = entries.front();
    EXPECT_EQ(kr4.id, "kuka_kr4_r600");
    EXPECT_EQ(kr4.family, "agilus");
    EXPECT_NEAR(kr4.payload_max.value().numerical_value_in(si::kilogram), 4.0, 1e-9);
    EXPECT_NEAR(kr4.reach.max_radius.value().numerical_value_in(si::metre), 0.601, 1e-9);
    EXPECT_NEAR(kr4.list_price_eur, 25000.0, 1e-9);
    EXPECT_EQ(kr4.regen_capable, false);
}

TEST(ArmCatalog, FootprintHasFourVertices) {
    auto entries = io::load_arm_catalog(repo_root() / "assets" / "arm" / "kuka" / "catalog.json");
    for (const auto& arm : entries) {
        EXPECT_EQ(arm.footprint.size(), 4U) << "arm " << arm.id;
    }
}

TEST(ArmCatalog, RejectsMissingFile) {
    EXPECT_THROW(io::load_arm_catalog("/no/such/file.json"), io::ParseError);
}

TEST(ArmCatalog, RejectsWrongCategory) {
    TempJsonFile file(R"({"category":"gripper","entries":[]})");
    EXPECT_THROW(io::load_arm_catalog(file.path()), io::ParseError);
}

TEST(ArmCatalog, RejectsNegativePayload) {
    TempJsonFile file(R"({
      "category":"arm",
      "entries":[{
        "id":"bad","model_name":"bad","family":"x","controller_class":"small",
        "footprint_polygon_m":[[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]],
        "reach_envelope":{"type":"radius","min_m":0.0,"max_m":1.0},
        "payload_max_kg":-1.0,"mass_kg":10.0,
        "max_speed_m_s":1.0,"max_acceleration_m_s2":1.0,"repeatability_mm":0.05,
        "power_peak_w":1000.0,"power_idle_w":100.0,
        "list_price_eur":1000.0,"lifetime_years":10,
        "maintenance_annual_fraction":0.04,"regen_capable":false
      }]
    })");
    EXPECT_THROW(io::load_arm_catalog(file.path()), io::ParseError);
}

TEST(ArmCatalog, RejectsMalformedFootprint) {
    TempJsonFile file(R"({
      "category":"arm",
      "entries":[{
        "id":"bad","model_name":"bad","family":"x","controller_class":"small",
        "footprint_polygon_m":[[-0.1,-0.1],[0.1,-0.1]],
        "reach_envelope":{"type":"radius","min_m":0.0,"max_m":1.0},
        "payload_max_kg":1.0,"mass_kg":10.0,
        "max_speed_m_s":1.0,"max_acceleration_m_s2":1.0,"repeatability_mm":0.05,
        "power_peak_w":1000.0,"power_idle_w":100.0,
        "list_price_eur":1000.0,"lifetime_years":10,
        "maintenance_annual_fraction":0.04,"regen_capable":false
      }]
    })");
    EXPECT_THROW(io::load_arm_catalog(file.path()), io::ParseError);
}

TEST(PusherCatalog, LoadsRealGenericCatalog) {
    auto entries = io::load_pusher_catalog(
        repo_root() / "assets" / "pusher" / "generic" / "catalog.json");
    EXPECT_EQ(entries.size(), 4U);
}

TEST(PusherCatalog, FieldsRoundTrip) {
    auto entries = io::load_pusher_catalog(
        repo_root() / "assets" / "pusher" / "generic" / "catalog.json");
    const auto& first = entries.front();
    EXPECT_EQ(first.id, "pusher_short_light");
    EXPECT_EQ(first.family, "pneumatic_short");
    EXPECT_NEAR(first.stroke.numerical_value_in(si::metre), 0.2, 1e-9);
    EXPECT_NEAR(first.payload_max.value().numerical_value_in(si::kilogram), 5.0, 1e-9);
    EXPECT_NEAR(first.cycle_time_per_push.numerical_value_in(si::second), 1.2, 1e-9);
    EXPECT_NEAR(first.list_price_eur, 3200.0, 1e-9);
}

TEST(PusherCatalog, RejectsWrongCategory) {
    TempJsonFile file(R"({"category":"arm","entries":[]})");
    EXPECT_THROW(io::load_pusher_catalog(file.path()), io::ParseError);
}

TEST(PusherCatalog, RejectsNegativeStroke) {
    TempJsonFile file(R"({
      "category":"pusher",
      "entries":[{
        "id":"bad","model_name":"bad","family":"x","controller_class":"small",
        "footprint_polygon_m":[[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]],
        "stroke_m":-0.1,"payload_max_kg":5.0,
        "cycle_time_per_push_s":1.0,
        "power_peak_w":500.0,"power_idle_w":50.0,
        "list_price_eur":1000.0
      }]
    })");
    EXPECT_THROW(io::load_pusher_catalog(file.path()), io::ParseError);
}

TEST(PusherCatalog, RejectsMissingPayload) {
    TempJsonFile file(R"({
      "category":"pusher",
      "entries":[{
        "id":"bad","model_name":"bad","family":"x","controller_class":"small",
        "footprint_polygon_m":[[-0.1,-0.1],[0.1,-0.1],[0.1,0.1],[-0.1,0.1]],
        "stroke_m":0.5,
        "cycle_time_per_push_s":1.0,
        "power_peak_w":500.0,"power_idle_w":50.0,
        "list_price_eur":1000.0
      }]
    })");
    EXPECT_THROW(io::load_pusher_catalog(file.path()), io::ParseError);
}

// ---- Belt catalog (step-6 M2) -------------------------------------------

TEST(BeltCatalog, LoadsRealGenericCatalog) {
    auto entries = io::load_belt_catalog(
        repo_root() / "assets" / "belt" / "generic" / "catalog.json");
    EXPECT_EQ(entries.size(), 4U);
}

TEST(BeltCatalog, FieldsRoundTrip) {
    auto entries = io::load_belt_catalog(
        repo_root() / "assets" / "belt" / "generic" / "catalog.json");
    const auto& first = entries.front();
    EXPECT_EQ(first.id, "belt_flat_400_short");
    EXPECT_EQ(first.family, "flat");
    EXPECT_NEAR(first.belt_width.numerical_value_in(si::metre), 0.4, 1e-9);
    EXPECT_NEAR(first.min_length.numerical_value_in(si::metre), 1.0, 1e-9);
    EXPECT_NEAR(first.max_length.numerical_value_in(si::metre), 5.0, 1e-9);
    EXPECT_NEAR(first.max_speed.numerical_value_in(si::metre / si::second), 0.5, 1e-9);
    EXPECT_NEAR(first.list_price_eur, 2600.0, 1e-9);
}

TEST(BeltCatalog, LengthRangesAreValid) {
    auto entries = io::load_belt_catalog(
        repo_root() / "assets" / "belt" / "generic" / "catalog.json");
    for (const auto& belt : entries) {
        EXPECT_GT(belt.max_length.numerical_value_in(si::metre),
                  belt.min_length.numerical_value_in(si::metre))
            << "belt " << belt.id;
    }
}

TEST(BeltCatalog, RejectsWrongCategory) {
    TempJsonFile file(R"({"category":"arm","entries":[]})");
    EXPECT_THROW(io::load_belt_catalog(file.path()), io::ParseError);
}

TEST(BeltCatalog, RejectsInvertedLengthRange) {
    TempJsonFile file(R"({
      "category":"belt",
      "entries":[{
        "id":"bad","model_name":"bad","family":"flat",
        "belt_width_m":0.6,"min_length_m":9.0,"max_length_m":3.0,
        "max_speed_m_s":0.6,"list_price_eur":4800.0
      }]
    })");
    EXPECT_THROW(io::load_belt_catalog(file.path()), io::ParseError);
}

TEST(BeltCatalog, RejectsMissingWidth) {
    TempJsonFile file(R"({
      "category":"belt",
      "entries":[{
        "id":"bad","model_name":"bad","family":"flat",
        "min_length_m":3.0,"max_length_m":9.0,
        "max_speed_m_s":0.6,"list_price_eur":4800.0
      }]
    })");
    EXPECT_THROW(io::load_belt_catalog(file.path()), io::ParseError);
}
