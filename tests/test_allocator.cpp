// Smoke tests for the Layer-2 allocator (solver/allocator.cpp).
//
// Intent: exercise the binding + sharing math on hand-constructed
// TaskEnumeration inputs. Integration tests that feed the real
// enumerator output through the allocator land in commit 9 alongside
// PARTIAL chain support.
//
// Each test builds a minimal Task + StrategyResult pair, wraps them in
// a TaskEnumeration, and asserts on the resulting instance set.

#include <gtest/gtest.h>
#include <mp-units/systems/si.h>
#include <tinycell/model/box.hpp>
#include <tinycell/model/pallet.hpp>
#include <tinycell/model/task.hpp>
#include <tinycell/solver/allocator.hpp>
#include <tinycell/solver/enumerator.hpp>
#include <tinycell/solver/strategy.hpp>

namespace {

using namespace mp_units;
namespace tc = tinycell::core;
namespace ts = tinycell::solver;

// Build a Palletize task with a given target rate. Content of the
// PalletizeParams doesn't matter to the allocator — only id and
// target_ct_per_item are read — but valid params keep the Task
// constructor happy.
tc::Task make_task(const std::string& id, double target_s_per_item) {
    return tc::Task{
        .id = id,
        .params = tc::PalletizeParams{
            .item_id = "box",
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
        .target_ct_per_item = tc::CycleTimePerItem{target_s_per_item * si::second},
    };
}

// Build a FULL StrategyResult that names a specific catalog id and
// achievable rate. Energy/cycle_time aren't load-bearing for the
// allocator's sharing math; we set them to plausible non-zero values.
ts::StrategyResult make_full_result(const std::string& strategy_name,
                                    const std::string& catalog_id,
                                    double achievable_s_per_item) {
    return ts::StrategyResult{
        .feasibility = ts::Feasibility::FULL,
        .strategy_name = strategy_name,
        .equipment = ts::EquipmentRef{.catalog_id = catalog_id},
        .energy_per_cycle = 1000.0 * si::joule,
        .cycle_time = 24.0 * achievable_s_per_item * si::second,
        .achievable_ct_per_item = achievable_s_per_item * si::second,
        .partial_info = std::nullopt,
        .preconditions = {},
        .requires_state = [](const tc::ItemState&) { return true; },
        .effect = [](const tc::ItemState& s) { return s; },
    };
}

ts::StrategyResult make_infeasible_result(const std::string& strategy_name) {
    return ts::StrategyResult{
        .feasibility = ts::Feasibility::INFEASIBLE,
        .strategy_name = strategy_name,
        .equipment = std::nullopt,
        .energy_per_cycle = 0.0 * si::joule,
        .cycle_time = 0.0 * si::second,
        .achievable_ct_per_item = 0.0 * si::second,
        .partial_info = std::nullopt,
        .preconditions = {},
        .requires_state = [](const tc::ItemState&) { return true; },
        .effect = [](const tc::ItemState& s) { return s; },
    };
}

ts::TaskEnumeration make_enumeration(tc::Task task,
                                     ts::StrategyResult result,
                                     bool with_winner) {
    ts::TaskEnumeration te{.task = std::move(task)};
    te.proposals.push_back(std::move(result));
    te.winner_index = with_winner ? std::optional<std::size_t>{0} : std::nullopt;
    return te;
}

} // namespace

TEST(Allocator, EmptyInputProducesEmptyResult) {
    auto result = ts::allocate({});
    EXPECT_TRUE(result.instances.empty());
    EXPECT_TRUE(result.unallocated.empty());
}

TEST(Allocator, TaskWithNoWinnerIsUnallocated) {
    auto task = make_task("orphan", 5.0);
    auto te = make_enumeration(task, make_infeasible_result("ArmStrategy"),
                               /*with_winner=*/false);
    std::vector<ts::TaskEnumeration> input{std::move(te)};

    auto result = ts::allocate(input);
    EXPECT_TRUE(result.instances.empty());
    ASSERT_EQ(result.unallocated.size(), 1U);
    EXPECT_EQ(result.unallocated[0], "orphan");
}

TEST(Allocator, SingleFullTaskMintsSingleInstance) {
    auto task = make_task("t1", 5.0);
    auto te = make_enumeration(task,
                               make_full_result("ArmStrategy", "arm_a", 2.0),
                               /*with_winner=*/true);
    std::vector<ts::TaskEnumeration> input{std::move(te)};

    auto result = ts::allocate(input);
    ASSERT_EQ(result.instances.size(), 1U);
    EXPECT_TRUE(result.unallocated.empty());
    const auto& inst = result.instances[0];
    EXPECT_EQ(inst.catalog_id, "arm_a");
    EXPECT_EQ(inst.strategy_name, "ArmStrategy");
    ASSERT_EQ(inst.served.size(), 1U);
    EXPECT_EQ(inst.served[0].task_id, "t1");
}

TEST(Allocator, DistinctCatalogIdsDoNotShare) {
    auto task_a = make_task("ta", 5.0);
    auto task_b = make_task("tb", 5.0);
    auto te_a = make_enumeration(task_a,
                                 make_full_result("ArmStrategy", "arm_a", 2.0),
                                 true);
    auto te_b = make_enumeration(task_b,
                                 make_full_result("ArmStrategy", "arm_b", 2.0),
                                 true);
    std::vector<ts::TaskEnumeration> input;
    input.push_back(std::move(te_a));
    input.push_back(std::move(te_b));

    auto result = ts::allocate(input);
    EXPECT_EQ(result.instances.size(), 2U);
}

TEST(Allocator, SameCatalogIdSharesWhenCapacityAllows) {
    // Achievable 2 s/item → 0.5 items/sec capacity.
    // Two tasks each at target 5 s/item → 0.2 items/sec demand each,
    // total 0.4 — fits within 0.5. One instance, both served.
    auto task_a = make_task("ta", 5.0);
    auto task_b = make_task("tb", 5.0);
    auto te_a = make_enumeration(task_a,
                                 make_full_result("ArmStrategy", "arm_a", 2.0),
                                 true);
    auto te_b = make_enumeration(task_b,
                                 make_full_result("ArmStrategy", "arm_a", 2.0),
                                 true);
    std::vector<ts::TaskEnumeration> input;
    input.push_back(std::move(te_a));
    input.push_back(std::move(te_b));

    auto result = ts::allocate(input);
    ASSERT_EQ(result.instances.size(), 1U);
    EXPECT_TRUE(result.unallocated.empty());
    const auto& inst = result.instances[0];
    ASSERT_EQ(inst.served.size(), 2U);
    EXPECT_EQ(inst.served[0].task_id, "ta");
    EXPECT_EQ(inst.served[1].task_id, "tb");
}

TEST(Allocator, SameCatalogIdDoesNotShareWhenCapacityExceeded) {
    // Achievable 2 s/item → 0.5 items/sec capacity.
    // Two tasks each at target 3 s/item → 0.333 items/sec demand each,
    // total 0.666 — exceeds 0.5. Should not share; two instances.
    auto task_a = make_task("ta", 3.0);
    auto task_b = make_task("tb", 3.0);
    auto te_a = make_enumeration(task_a,
                                 make_full_result("ArmStrategy", "arm_a", 2.0),
                                 true);
    auto te_b = make_enumeration(task_b,
                                 make_full_result("ArmStrategy", "arm_a", 2.0),
                                 true);
    std::vector<ts::TaskEnumeration> input;
    input.push_back(std::move(te_a));
    input.push_back(std::move(te_b));

    auto result = ts::allocate(input);
    EXPECT_EQ(result.instances.size(), 2U);
    EXPECT_TRUE(result.unallocated.empty());
}
