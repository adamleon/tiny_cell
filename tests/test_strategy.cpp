#include <algorithm>
#include <cmath>
#include <filesystem>
#include <gtest/gtest.h>
#include <mp-units/systems/si.h>
#include <numbers>
#include <tinycell/io/catalog_loader.hpp>
#include <tinycell/model/port.hpp>
#include <tinycell/model/task.hpp>
#include <tinycell/solver/arm_strategy.hpp>
#include <tinycell/solver/pusher_strategy.hpp>

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

std::vector<tc::PusherSpec> generic_pushers() {
    return tinycell::io::load_pusher_catalog(
        repo_root() / "assets" / "pusher" / "generic" / "catalog.json");
}

tc::Task small_palletize_task() {
    return tc::Task{
        .id = "task_palletize_small",
        .params = tc::PalletizeParams{
            .item_id = "box_a",
            .item = tc::BoxSpec{
                .physical = tc::ItemPhysical{
                    .width = 0.3 * si::metre,
                    .length = 0.4 * si::metre,
                    .height = 0.2 * si::metre,
                    .mass = 5.0 * si::kilogram,
                    .symmetry = tc::symmetry::discrete(180),
                },
            },
            .pallet = tc::PalletSpec{
                .physical = tc::ItemPhysical{
                    .width = 1.2 * si::metre,
                    .length = 0.8 * si::metre,
                    .height = 0.15 * si::metre,
                    .mass = 25.0 * si::kilogram,
                    .symmetry = tc::symmetry::discrete(180),
                },
            },
            .box_count = 24,
        },
        // Loose target so existing FULL tests remain FULL once PARTIAL
        // emission lands; PARTIAL-specific tests will set tighter targets.
        .target_ct_per_item = tc::CycleTimePerItem{5.0 * si::second},
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
    std::get<tc::PalletizeParams>(task.params).item.physical.mass = 500.0 * si::kilogram;
    auto result = strategy.evaluate(task);
    EXPECT_EQ(result.feasibility, ts::Feasibility::INFEASIBLE);
    EXPECT_FALSE(result.equipment.has_value());
}

TEST(ArmStrategy, PicksLargerArmForLargePallet) {
    auto arms = kuka_arms();
    ts::ArmStrategy strategy(arms);
    auto task = small_palletize_task();
    auto& p = std::get<tc::PalletizeParams>(task.params);
    p.pallet.physical.width = 2.0 * si::metre;
    p.pallet.physical.length = 2.0 * si::metre;
    auto result = strategy.evaluate(task);
    EXPECT_EQ(result.feasibility, ts::Feasibility::FULL);
    ASSERT_TRUE(result.equipment.has_value());
    // Pallet diagonal ≈ 2.83 m → reach/2 ≥ 1.41 m → needs Cybertech.
    EXPECT_TRUE(result.equipment->catalog_id == "kuka_kr16_r1610" ||
                result.equipment->catalog_id == "kuka_kr22_r1610")
        << "got " << result.equipment->catalog_id;
}

TEST(PusherStrategy, NameAndAppliesTo) {
    auto pushers = generic_pushers();
    ts::PusherStrategy strategy(pushers);
    EXPECT_EQ(strategy.name(), "PusherStrategy");
    EXPECT_TRUE(strategy.applies_to(small_palletize_task()));
}

TEST(PusherStrategy, FeasibleForSmallPalletizeTask) {
    auto pushers = generic_pushers();
    ts::PusherStrategy strategy(pushers);
    // 1.2 x 0.8 m pallet → row dim = 0.8 m (smaller pallet dim). 5 kg item,
    // box 0.3 x 0.4 m → boxes_per_row = floor(0.8/0.3) = 2 → row payload
    // = 10 kg. pusher_long_medium (1.0 m stroke, 25 kg payload, EUR 9800)
    // is the cheapest passing both filters; pusher_mid_medium (0.5 m) and
    // pusher_short_light (0.2 m) fail the stroke check.
    auto result = strategy.evaluate(small_palletize_task());
    EXPECT_EQ(result.feasibility, ts::Feasibility::FULL);
    ASSERT_TRUE(result.equipment.has_value());
    EXPECT_EQ(result.equipment->catalog_id, "pusher_long_medium");
    EXPECT_GT(result.cycle_time.numerical_value_in(si::second), 0.0);
    EXPECT_GT(result.energy_per_cycle.numerical_value_in(si::joule), 0.0);
}

TEST(PusherStrategy, PicksCheapestFeasibleForShortPallet) {
    auto pushers = generic_pushers();
    ts::PusherStrategy strategy(pushers);
    auto task = small_palletize_task();
    auto& p = std::get<tc::PalletizeParams>(task.params);
    // Shrink the pallet so a cheaper short-stroke pusher becomes feasible.
    // 0.4 x 0.3 m pallet → row dim = 0.3 m, boxes_per_row = 1, row payload
    // = 5 kg. pusher_mid_medium (0.5 m stroke, 15 kg payload, EUR 5500) is
    // the cheapest passing both; pusher_short_light (0.2 m stroke) fails.
    p.pallet.physical.width = 0.4 * si::metre;
    p.pallet.physical.length = 0.3 * si::metre;
    auto result = strategy.evaluate(task);
    EXPECT_EQ(result.feasibility, ts::Feasibility::FULL);
    ASSERT_TRUE(result.equipment.has_value());
    EXPECT_EQ(result.equipment->catalog_id, "pusher_mid_medium");
}

TEST(PusherStrategy, CycleTimeCountsRowsNotBoxes) {
    auto pushers = generic_pushers();
    ts::PusherStrategy strategy(pushers);
    // small task: 24 boxes, boxes_per_row = 2 → 12 rows.
    // pusher_long_medium cycle_time_per_push = 2.4 s → expected cycle_time
    // = 28.8 s. (NOT 24 × 2.4 = 57.6 s, the per-box mistake.)
    auto result = strategy.evaluate(small_palletize_task());
    ASSERT_EQ(result.feasibility, ts::Feasibility::FULL);
    EXPECT_NEAR(result.cycle_time.numerical_value_in(si::second), 28.8, 1e-9);
}

TEST(PusherStrategy, InfeasibleForLargePallet) {
    auto pushers = generic_pushers();
    ts::PusherStrategy strategy(pushers);
    auto task = small_palletize_task();
    auto& p = std::get<tc::PalletizeParams>(task.params);
    // 2 m pallet > largest stroke (1.5 m).
    p.pallet.physical.width = 2.0 * si::metre;
    p.pallet.physical.length = 2.0 * si::metre;
    auto result = strategy.evaluate(task);
    EXPECT_EQ(result.feasibility, ts::Feasibility::INFEASIBLE);
    EXPECT_FALSE(result.equipment.has_value());
}

TEST(PusherStrategy, InfeasibleForHeavyItem) {
    auto pushers = generic_pushers();
    ts::PusherStrategy strategy(pushers);
    auto task = small_palletize_task();
    auto& p = std::get<tc::PalletizeParams>(task.params);
    p.item.physical.mass = 500.0 * si::kilogram; // a single box exceeds every payload
    auto result = strategy.evaluate(task);
    EXPECT_EQ(result.feasibility, ts::Feasibility::INFEASIBLE);
    EXPECT_FALSE(result.equipment.has_value());
}

TEST(PusherStrategy, InfeasibleWhenRowPayloadExceedsAllPushers) {
    auto pushers = generic_pushers();
    ts::PusherStrategy strategy(pushers);
    auto task = small_palletize_task();
    auto& p = std::get<tc::PalletizeParams>(task.params);
    // Single box is light (35 kg) but the row carries multiple: pallet
    // 1.2 x 0.8 m → row dim = 0.8 m, box 0.3 x 0.4 m → boxes_per_row = 2,
    // row payload = 70 kg. Heaviest pusher in catalog is 60 kg → infeasible
    // even though no single box exceeds payload.
    p.item.physical.mass = 35.0 * si::kilogram;
    auto result = strategy.evaluate(task);
    EXPECT_EQ(result.feasibility, ts::Feasibility::INFEASIBLE);
}

TEST(PusherStrategy, InfeasibleWhenBoxTooWideForPalletRow) {
    auto pushers = generic_pushers();
    ts::PusherStrategy strategy(pushers);
    auto task = small_palletize_task();
    auto& p = std::get<tc::PalletizeParams>(task.params);
    // Box wider than the row direction → boxes_per_row = 0 → pattern
    // infeasibility, reported before any catalog lookup.
    p.pallet.physical.width = 0.5 * si::metre;
    p.pallet.physical.length = 0.5 * si::metre;
    p.item.physical.width = 0.6 * si::metre;
    p.item.physical.length = 0.6 * si::metre;
    auto result = strategy.evaluate(task);
    EXPECT_EQ(result.feasibility, ts::Feasibility::INFEASIBLE);
}

// ---- Port constraints emitted by strategies (T.3) -----------------------

namespace {
const tc::PortConstraint* find_port(const std::vector<tc::PortConstraint>& v,
                                    const std::string& name) {
    auto it = std::find_if(v.begin(), v.end(),
        [&](const tc::PortConstraint& c) { return c.port_name == name; });
    return it == v.end() ? nullptr : &*it;
}
} // namespace

TEST(ArmStrategy, FeasibleResultEmitsLoosePortConstraints) {
    auto arms = kuka_arms();
    ts::ArmStrategy strategy(arms);
    auto result = strategy.evaluate(small_palletize_task());
    ASSERT_EQ(result.feasibility, ts::Feasibility::FULL);
    // One PortConstraint per LogicalPort the Palletize task declares.
    EXPECT_EQ(result.port_constraints.size(), 3u);
    EXPECT_NE(find_port(result.port_constraints, "item_in"),    nullptr);
    EXPECT_NE(find_port(result.port_constraints, "pallet_in"),  nullptr);
    EXPECT_NE(find_port(result.port_constraints, "pallet_out"), nullptr);
    // Arm direction tolerance is pi/2 — items can come from any
    // direction within the reach envelope.
    for (const auto& c : result.port_constraints) {
        EXPECT_NEAR(c.direction_tolerance.numerical_value_in(si::radian),
                    std::numbers::pi / 2.0, 1e-9);
    }
}

TEST(ArmStrategy, ItemInIsReachAnnulusPalletPortsAreOffsetPoints) {
    auto arms = kuka_arms();
    ts::ArmStrategy strategy(arms);
    auto result = strategy.evaluate(small_palletize_task());
    ASSERT_EQ(result.feasibility, ts::Feasibility::FULL);
    ASSERT_TRUE(result.equipment.has_value());

    // Look up the arm the strategy actually chose, to compare its reach.
    auto arm_it = std::find_if(arms.begin(), arms.end(),
        [&](const tc::ArmSpec& a) { return a.id == result.equipment->catalog_id; });
    ASSERT_NE(arm_it, arms.end());

    const auto* item_in = find_port(result.port_constraints, "item_in");
    const auto* pallet_in = find_port(result.port_constraints, "pallet_in");
    const auto* pallet_out = find_port(result.port_constraints, "pallet_out");
    ASSERT_NE(item_in, nullptr);
    ASSERT_NE(pallet_in, nullptr);
    ASSERT_NE(pallet_out, nullptr);

    // item_in is an ANNULUS: its reach band equals the chosen arm's reach.
    ASSERT_TRUE(item_in->reach_min.has_value());
    ASSERT_TRUE(item_in->reach_max.has_value());
    EXPECT_NEAR(item_in->reach_min->numerical_value_in(si::metre),
                arm_it->reach.min_radius.numerical_value_in(si::metre), 1e-9);
    EXPECT_NEAR(item_in->reach_max->numerical_value_in(si::metre),
                arm_it->reach.max_radius.value().numerical_value_in(si::metre), 1e-9);

    // pallet_in / pallet_out are fixed POINTS (no annulus) at a non-zero
    // +x offset, co-located (single pallet belt passes through the station).
    EXPECT_FALSE(pallet_in->reach_max.has_value());
    EXPECT_FALSE(pallet_out->reach_max.has_value());
    EXPECT_GT(pallet_in->x.numerical_value_in(si::metre), 0.0);
    EXPECT_NEAR(pallet_in->x.numerical_value_in(si::metre),
                pallet_out->x.numerical_value_in(si::metre), 1e-9);
}

TEST(ArmStrategy, InfeasibleResultHasNoPortConstraints) {
    auto arms = kuka_arms();
    ts::ArmStrategy strategy(arms);
    auto task = small_palletize_task();
    auto& p = std::get<tc::PalletizeParams>(task.params);
    p.item.physical.mass = 1000.0 * si::kilogram;  // exceeds every catalog payload
    auto result = strategy.evaluate(task);
    ASSERT_EQ(result.feasibility, ts::Feasibility::INFEASIBLE);
    EXPECT_TRUE(result.port_constraints.empty());
}

TEST(PusherStrategy, FeasibleResultEmitsStrictPortsWithPerpendicularPalletDirection) {
    auto pushers = generic_pushers();
    ts::PusherStrategy strategy(pushers);
    auto result = strategy.evaluate(small_palletize_task());
    ASSERT_EQ(result.feasibility, ts::Feasibility::FULL);
    ASSERT_EQ(result.port_constraints.size(), 3u);

    const auto* item_in = find_port(result.port_constraints, "item_in");
    const auto* pallet_in = find_port(result.port_constraints, "pallet_in");
    const auto* pallet_out = find_port(result.port_constraints, "pallet_out");
    ASSERT_NE(item_in, nullptr);
    ASSERT_NE(pallet_in, nullptr);
    ASSERT_NE(pallet_out, nullptr);

    // All directions strict: zero tolerance.
    EXPECT_NEAR(item_in->direction_tolerance.numerical_value_in(si::radian),    0.0, 1e-9);
    EXPECT_NEAR(pallet_in->direction_tolerance.numerical_value_in(si::radian),  0.0, 1e-9);
    EXPECT_NEAR(pallet_out->direction_tolerance.numerical_value_in(si::radian), 0.0, 1e-9);

    // item_in along the pusher's "behind" axis (theta = 0); pallet_in
    // and pallet_out perpendicular (theta = pi/2). This is the
    // pusher-must-be-attached-to-item-belt-with-perpendicular-pallet-belt
    // rule expressed in the type system.
    EXPECT_NEAR(item_in->theta.numerical_value_in(si::radian),    0.0, 1e-9);
    EXPECT_NEAR(pallet_in->theta.numerical_value_in(si::radian),  std::numbers::pi / 2.0, 1e-9);
    EXPECT_NEAR(pallet_out->theta.numerical_value_in(si::radian), std::numbers::pi / 2.0, 1e-9);
}

TEST(PusherStrategy, InfeasibleResultHasNoPortConstraints) {
    auto pushers = generic_pushers();
    ts::PusherStrategy strategy(pushers);
    auto task = small_palletize_task();
    auto& p = std::get<tc::PalletizeParams>(task.params);
    p.item.physical.mass = 1000.0 * si::kilogram;
    auto result = strategy.evaluate(task);
    ASSERT_EQ(result.feasibility, ts::Feasibility::INFEASIBLE);
    EXPECT_TRUE(result.port_constraints.empty());
}

