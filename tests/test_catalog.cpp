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
    EXPECT_NEAR(kr4.payload_max.numerical_value_in(si::kilogram), 4.0, 1e-9);
    EXPECT_NEAR(kr4.reach.max_radius.numerical_value_in(si::metre), 0.601, 1e-9);
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
