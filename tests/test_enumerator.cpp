#include <filesystem>
#include <gtest/gtest.h>
#include <mp-units/systems/si.h>
#include <tinycell/io/catalog_loader.hpp>
#include <tinycell/model/task.hpp>
#include <tinycell/solver/allocator.hpp>
#include <tinycell/solver/arm_strategy.hpp>
#include <tinycell/solver/enumerator.hpp>
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

tc::Task make_palletize(const std::string& id, double item_mass_kg,
                        double pallet_w_m, double pallet_l_m, int box_count) {
    return tc::Task{
        .id = id,
        .params = tc::PalletizeParams{
            .item_id = "box",
            .item = tc::BoxSpec{
                .physical = tc::ItemPhysical{
                    .width = 0.3 * si::metre,
                    .length = 0.4 * si::metre,
                    .height = 0.2 * si::metre,
                    .mass = item_mass_kg * si::kilogram,
                    .symmetry = tc::symmetry::discrete(180),
                },
            },
            .pallet = tc::PalletSpec{
                .physical = tc::ItemPhysical{
                    .width = pallet_w_m * si::metre,
                    .length = pallet_l_m * si::metre,
                    .height = 0.15 * si::metre,
                    .mass = 25.0 * si::kilogram,
                    .symmetry = tc::symmetry::discrete(180),
                },
            },
            .box_count = box_count,
        },
        .target_ct_per_item = tc::CycleTimePerItem{5.0 * si::second},
    };
}

} // namespace

TEST(Enumerator, RunsBothStrategiesPerTask) {
    auto arms = kuka_arms();
    auto pushers = generic_pushers();
    ts::ArmStrategy arm(arms);
    ts::PusherStrategy pusher(pushers);
    const std::vector<const ts::Strategy*> strategies{&arm, &pusher};

    const std::vector<tc::Task> workflow{make_palletize("t0", 5.0, 1.2, 0.8, 24)};
    auto results = ts::enumerate(workflow, strategies);

    ASSERT_EQ(results.size(), 1U);
    EXPECT_EQ(results[0].task.id, workflow[0].id);
    EXPECT_EQ(results[0].proposals.size(), 2U);
    // Both strategies applies_to Palletize, so both proposals must be present.
    EXPECT_EQ(results[0].proposals[0].strategy_name, "ArmStrategy");
    EXPECT_EQ(results[0].proposals[1].strategy_name, "PusherStrategy");
}

TEST(Enumerator, PicksWinnerByLowestEnergy) {
    auto arms = kuka_arms();
    auto pushers = generic_pushers();
    ts::ArmStrategy arm(arms);
    ts::PusherStrategy pusher(pushers);
    const std::vector<const ts::Strategy*> strategies{&arm, &pusher};

    const std::vector<tc::Task> workflow{make_palletize("t0", 5.0, 1.2, 0.8, 24)};
    auto results = ts::enumerate(workflow, strategies);

    ASSERT_EQ(results.size(), 1U);
    ASSERT_TRUE(results[0].winner_index.has_value());
    const auto& winner = results[0].proposals[*results[0].winner_index];
    EXPECT_EQ(winner.feasibility, ts::Feasibility::FULL);
    // Winner must be the proposal with the minimum energy_per_cycle among
    // FULL proposals.
    for (const auto& p : results[0].proposals) {
        if (p.feasibility == ts::Feasibility::FULL) {
            EXPECT_LE(winner.energy_per_cycle.numerical_value_in(si::joule),
                      p.energy_per_cycle.numerical_value_in(si::joule));
        }
    }
}

TEST(Enumerator, NoWinnerWhenAllInfeasible) {
    auto arms = kuka_arms();
    auto pushers = generic_pushers();
    ts::ArmStrategy arm(arms);
    ts::PusherStrategy pusher(pushers);
    const std::vector<const ts::Strategy*> strategies{&arm, &pusher};

    // 500 kg item exceeds every arm and every pusher's payload.
    const std::vector<tc::Task> workflow{make_palletize("t0", 500.0, 1.2, 0.8, 1)};
    auto results = ts::enumerate(workflow, strategies);

    ASSERT_EQ(results.size(), 1U);
    EXPECT_FALSE(results[0].winner_index.has_value());
    for (const auto& p : results[0].proposals) {
        EXPECT_EQ(p.feasibility, ts::Feasibility::INFEASIBLE);
    }
}

TEST(Enumerator, ArmWinsWhenPusherInfeasible) {
    auto arms = kuka_arms();
    auto pushers = generic_pushers();
    ts::ArmStrategy arm(arms);
    ts::PusherStrategy pusher(pushers);
    const std::vector<const ts::Strategy*> strategies{&arm, &pusher};

    // 2 m pallet — no pusher has stroke ≥ 2 m, but arms reach it.
    const std::vector<tc::Task> workflow{make_palletize("t0", 8.0, 2.0, 2.0, 24)};
    auto results = ts::enumerate(workflow, strategies);

    ASSERT_EQ(results.size(), 1U);
    ASSERT_TRUE(results[0].winner_index.has_value());
    EXPECT_EQ(results[0].proposals[*results[0].winner_index].strategy_name, "ArmStrategy");
}

TEST(EnumeratorPartial, EmitsResidualChainWhenNoFullExists) {
    // ArmStrategy-only, with a pallet small enough for KR4 R600
    // (cheapest arm in the catalog) to qualify on reach but a tight
    // target so its cycle time falls just short of FULL. The residual
    // target is loose enough that the same arm hits FULL → chain
    // length 2 (original PARTIAL + residual FULL).
    auto arms = kuka_arms();
    ts::ArmStrategy arm(arms);
    const std::vector<const ts::Strategy*> strategies{&arm};

    auto t = make_palletize("t0", 5.0, 0.7, 0.7, 24);
    t.target_ct_per_item = tc::CycleTimePerItem{1.2 * si::second};
    const std::vector<tc::Task> workflow{t};

    auto results = ts::enumerate(workflow, strategies);

    ASSERT_EQ(results.size(), 2U);

    EXPECT_FALSE(results[0].task.is_residual);
    ASSERT_TRUE(results[0].winner_index.has_value());
    EXPECT_EQ(results[0].proposals[*results[0].winner_index].feasibility,
              ts::Feasibility::PARTIAL);

    EXPECT_TRUE(results[1].task.is_residual);
    ASSERT_TRUE(results[1].winner_index.has_value());
    EXPECT_EQ(results[1].proposals[*results[1].winner_index].feasibility,
              ts::Feasibility::FULL);
}

TEST(EnumeratorPartial, ChainTerminatesAtDepthCap) {
    // Extreme target — KR4's 1.4 s/item is ~14× too slow for a 0.1 s
    // target. ArmStrategy's residual math (a*t)/(a-t) converges toward
    // `achievable` only asymptotically, so the chain cannot reach FULL
    // within MAX_PARTIAL_CHAIN_DEPTH. The cap halts recursion and the
    // deepest entry stays PARTIAL with its residual NOT enumerated.
    auto arms = kuka_arms();
    ts::ArmStrategy arm(arms);
    const std::vector<const ts::Strategy*> strategies{&arm};

    auto t = make_palletize("t0", 5.0, 0.7, 0.7, 24);
    t.target_ct_per_item = tc::CycleTimePerItem{0.1 * si::second};
    const std::vector<tc::Task> workflow{t};

    auto results = ts::enumerate(workflow, strategies);

    // Original + MAX_PARTIAL_CHAIN_DEPTH residuals; the deepest residual
    // is enumerated (it's PARTIAL) but its OWN residual is not.
    EXPECT_EQ(results.size(), ts::MAX_PARTIAL_CHAIN_DEPTH + 1);
    for (const auto& te : results) {
        ASSERT_TRUE(te.winner_index.has_value());
        EXPECT_EQ(te.proposals[*te.winner_index].feasibility,
                  ts::Feasibility::PARTIAL);
    }
}

TEST(EnumeratorAllocator, SharingWinsOnIdenticalTasks) {
    // Two identical palletize tasks. With a loose target (5 s/item),
    // pusher_long_medium reaches FULL on both at the same per-item
    // rate. Each task demands 0.2 items/sec; the pusher's capacity is
    // well above 0.4. Allocator should bind both tasks to ONE shared
    // instance.
    auto arms = kuka_arms();
    auto pushers = generic_pushers();
    ts::ArmStrategy arm(arms);
    ts::PusherStrategy pusher(pushers);
    const std::vector<const ts::Strategy*> strategies{&arm, &pusher};

    const std::vector<tc::Task> workflow{
        make_palletize("t_a", 5.0, 1.2, 0.8, 24),
        make_palletize("t_b", 5.0, 1.2, 0.8, 24),
    };
    auto results = ts::enumerate(workflow, strategies);
    auto allocation = ts::allocate(results);

    ASSERT_EQ(allocation.instances.size(), 1U);
    EXPECT_EQ(allocation.instances[0].served.size(), 2U);
    EXPECT_TRUE(allocation.unallocated.empty());
}

TEST(Enumerator, PreservesWorkflowOrder) {
    auto arms = kuka_arms();
    auto pushers = generic_pushers();
    ts::ArmStrategy arm(arms);
    ts::PusherStrategy pusher(pushers);
    const std::vector<const ts::Strategy*> strategies{&arm, &pusher};

    const std::vector<tc::Task> workflow{
        make_palletize("first",  5.0, 1.2, 0.8, 24),
        make_palletize("second", 8.0, 1.2, 1.0, 12),
        make_palletize("third",  3.0, 0.4, 0.3, 6),
    };
    auto results = ts::enumerate(workflow, strategies);

    ASSERT_EQ(results.size(), 3U);
    EXPECT_EQ(results[0].task.id, "first");
    EXPECT_EQ(results[1].task.id, "second");
    EXPECT_EQ(results[2].task.id, "third");
}
