#include <filesystem>
#include <gtest/gtest.h>
#include <mp-units/systems/si.h>
#include <tinycell/io/catalog_loader.hpp>
#include <tinycell/model/task.hpp>
#include <tinycell/solver/arm_strategy.hpp>

namespace {

using namespace mp_units;
namespace tc = tinycell::core;
namespace ts = tinycell::solver;

std::filesystem::path repo_root() {
    return std::filesystem::path(TINYCELL_REPO_ROOT);
}

std::vector<tc::ArmSpec> kuka_arms() {
    return tinycell::io::load_arm_catalog(
        repo_root() / "assets" / "arm" / "kuka" / "catalog.json");
}

tc::Task small_palletize_task() {
    return tc::Task{
        .id = "task_palletize_small",
        .params = tc::PalletizeParams{
            .item_id = "box_a",
            .item = tc::BoxSpec{.width = 0.3 * si::metre,
                                .length = 0.4 * si::metre,
                                .height = 0.2 * si::metre,
                                .mass = 5.0 * si::kilogram},
            .pallet = tc::PalletSpec{.width = 1.2 * si::metre, .length = 0.8 * si::metre},
            .box_count = 24,
        },
    };
}

} // namespace

TEST(ArmStrategy, AppliesToPalletize) {
    auto arms = kuka_arms();
    ts::ArmStrategy strategy(arms);
    EXPECT_TRUE(strategy.applies_to(small_palletize_task()));
}

TEST(ArmStrategy, NameMatchesConvention) {
    auto arms = kuka_arms();
    ts::ArmStrategy strategy(arms);
    EXPECT_EQ(strategy.name(), "ArmStrategy");
}

TEST(ArmStrategy, FeasibleForLightBox) {
    auto arms = kuka_arms();
    ts::ArmStrategy strategy(arms);
    auto result = strategy.evaluate(small_palletize_task());
    EXPECT_EQ(result.feasibility, ts::Feasibility::FULL);
    ASSERT_TRUE(result.equipment.has_value());
    EXPECT_GT(result.cycle_time.numerical_value_in(si::second), 0.0);
    EXPECT_GT(result.energy_per_cycle.numerical_value_in(si::joule), 0.0);
}

TEST(ArmStrategy, PicksCheapestFeasibleArm) {
    auto arms = kuka_arms();
    ts::ArmStrategy strategy(arms);
    auto result = strategy.evaluate(small_palletize_task());
    ASSERT_TRUE(result.equipment.has_value());
    // 5 kg payload + 1.2x0.8 m pallet (diagonal ~1.442 m, half ~0.721 m):
    // KR4_R600 (4 kg) fails payload; KR6_R700_2 (6 kg, 0.706 m reach) fails
    // reach by ~15 mm; KR6_R900_2 (0.901 m reach, EUR 36000) is the cheapest
    // arm that satisfies both.
    EXPECT_EQ(result.equipment->catalog_id, "kuka_kr6_r900_2");
}

TEST(ArmStrategy, InfeasibleWhenPayloadExceedsAllArms) {
    auto arms = kuka_arms();
    ts::ArmStrategy strategy(arms);
    auto task = small_palletize_task();
    std::get<tc::PalletizeParams>(task.params).item.mass = 500.0 * si::kilogram;
    auto result = strategy.evaluate(task);
    EXPECT_EQ(result.feasibility, ts::Feasibility::INFEASIBLE);
    EXPECT_FALSE(result.equipment.has_value());
}

TEST(ArmStrategy, PicksLargerArmForLargePallet) {
    auto arms = kuka_arms();
    ts::ArmStrategy strategy(arms);
    auto task = small_palletize_task();
    auto& p = std::get<tc::PalletizeParams>(task.params);
    p.pallet.width = 2.0 * si::metre;
    p.pallet.length = 2.0 * si::metre;
    auto result = strategy.evaluate(task);
    EXPECT_EQ(result.feasibility, ts::Feasibility::FULL);
    ASSERT_TRUE(result.equipment.has_value());
    // Pallet diagonal ≈ 2.83 m → reach/2 ≥ 1.41 m → needs Cybertech.
    EXPECT_TRUE(result.equipment->catalog_id == "kuka_kr16_r1610" ||
                result.equipment->catalog_id == "kuka_kr22_r1610")
        << "got " << result.equipment->catalog_id;
}
