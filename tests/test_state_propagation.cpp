#include <filesystem>
#include <gtest/gtest.h>
#include <mp-units/systems/si.h>
#include <stdexcept>
#include <tinycell/io/catalog_loader.hpp>
#include <tinycell/model/item.hpp>
#include <tinycell/model/task.hpp>
#include <tinycell/solver/arm_strategy.hpp>
#include <tinycell/solver/pusher_strategy.hpp>
#include <tinycell/solver/state_propagation.hpp>
#include <variant>

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

tc::Task small_palletize_task(const std::string& id = "t") {
    return tc::Task{
        .id = id,
        .params = tc::PalletizeParams{
            .item_id = "box",
            .item = tc::BoxSpec{.width = 0.3 * si::metre,
                                .length = 0.4 * si::metre,
                                .height = 0.2 * si::metre,
                                .mass = 5.0 * si::kilogram},
            .pallet = tc::PalletSpec{.width = 1.2 * si::metre,
                                     .length = 0.8 * si::metre},
            .box_count = 24,
        },
    };
}

} // namespace

// -- Strategy state-flow contract -------------------------------------------

TEST(ArmStrategyState, EmitsRequiresStateAndEffect) {
    auto arms = kuka_arms();
    ts::ArmStrategy strategy(arms);
    auto result = strategy.evaluate(small_palletize_task());
    EXPECT_TRUE(static_cast<bool>(result.requires_state));
    EXPECT_TRUE(static_cast<bool>(result.effect));
}

TEST(ArmStrategyState, RequiresPositionKnown) {
    auto arms = kuka_arms();
    ts::ArmStrategy strategy(arms);
    auto result = strategy.evaluate(small_palletize_task());
    tc::ItemState unknown_pose{
        .position_known = false, .orientation_resolved_to = {},
        .on_carrier = tc::OnCarrier::Belt};
    tc::ItemState known_pose{
        .position_known = true, .orientation_resolved_to = {},
        .on_carrier = tc::OnCarrier::Free};
    EXPECT_FALSE(result.requires_state(unknown_pose));
    EXPECT_TRUE(result.requires_state(known_pose));
}

TEST(ArmStrategyState, EffectPlacesItemOnPallet) {
    auto arms = kuka_arms();
    ts::ArmStrategy strategy(arms);
    auto result = strategy.evaluate(small_palletize_task());
    tc::ItemState input{.position_known = true,
                        .orientation_resolved_to = {},
                        .on_carrier = tc::OnCarrier::Belt};
    auto output = result.effect(input);
    EXPECT_TRUE(output.position_known);
    EXPECT_EQ(output.on_carrier, tc::OnCarrier::Pallet);
}

TEST(PusherStrategyState, RequiresPositionKnownAndOnBelt) {
    auto pushers = generic_pushers();
    ts::PusherStrategy strategy(pushers);
    auto result = strategy.evaluate(small_palletize_task());
    tc::ItemState belt_known{.position_known = true,
                             .orientation_resolved_to = {},
                             .on_carrier = tc::OnCarrier::Belt};
    tc::ItemState belt_unknown{.position_known = false,
                               .orientation_resolved_to = {},
                               .on_carrier = tc::OnCarrier::Belt};
    tc::ItemState pallet_known{.position_known = true,
                               .orientation_resolved_to = {},
                               .on_carrier = tc::OnCarrier::Pallet};
    EXPECT_TRUE(result.requires_state(belt_known));
    EXPECT_FALSE(result.requires_state(belt_unknown));
    EXPECT_FALSE(result.requires_state(pallet_known));
}

TEST(PusherStrategyState, EffectPlacesItemOnPallet) {
    auto pushers = generic_pushers();
    ts::PusherStrategy strategy(pushers);
    auto result = strategy.evaluate(small_palletize_task());
    tc::ItemState input{.position_known = true,
                        .orientation_resolved_to = {},
                        .on_carrier = tc::OnCarrier::Belt};
    auto output = result.effect(input);
    EXPECT_TRUE(output.position_known);
    EXPECT_EQ(output.on_carrier, tc::OnCarrier::Pallet);
}

// -- Propagation pass --------------------------------------------------------

TEST(StatePropagation, SingleTaskSuccessReturnsTwoStateTrajectory) {
    auto arms = kuka_arms();
    ts::ArmStrategy strategy(arms);
    const std::vector<tc::Task> workflow{small_palletize_task("t0")};
    const auto r0 = strategy.evaluate(workflow[0]);
    const std::vector<const ts::StrategyResult*> chosen{&r0};
    const tc::ItemState initial{.position_known = true,
                                .orientation_resolved_to = {},
                                .on_carrier = tc::OnCarrier::Belt};

    auto result = ts::propagate_state(workflow, chosen, initial);
    const auto* success = std::get_if<ts::PropagationSuccess>(&result);
    ASSERT_NE(success, nullptr);
    ASSERT_EQ(success->trajectory.size(), 2U);
    EXPECT_EQ(success->trajectory[0].position_known, true);
    EXPECT_EQ(success->trajectory[0].on_carrier, tc::OnCarrier::Belt);
    EXPECT_EQ(success->trajectory[1].on_carrier, tc::OnCarrier::Pallet);
}

TEST(StatePropagation, FirstTrajectoryStateIsInitial) {
    auto arms = kuka_arms();
    ts::ArmStrategy strategy(arms);
    const std::vector<tc::Task> workflow{small_palletize_task("t0")};
    const auto r0 = strategy.evaluate(workflow[0]);
    const std::vector<const ts::StrategyResult*> chosen{&r0};
    const tc::ItemState initial{.position_known = true,
                                .orientation_resolved_to = {},
                                .on_carrier = tc::OnCarrier::Fixture};

    auto result = ts::propagate_state(workflow, chosen, initial);
    const auto* success = std::get_if<ts::PropagationSuccess>(&result);
    ASSERT_NE(success, nullptr);
    EXPECT_EQ(success->trajectory[0].on_carrier, tc::OnCarrier::Fixture);
}

TEST(StatePropagation, FailsAtFirstTaskWhenInitialStateRejected) {
    auto arms = kuka_arms();
    ts::ArmStrategy strategy(arms);
    const std::vector<tc::Task> workflow{small_palletize_task("t0")};
    const auto r0 = strategy.evaluate(workflow[0]);
    const std::vector<const ts::StrategyResult*> chosen{&r0};
    const tc::ItemState initial{.position_known = false,
                                .orientation_resolved_to = {},
                                .on_carrier = tc::OnCarrier::Belt};

    auto result = ts::propagate_state(workflow, chosen, initial);
    const auto* failure = std::get_if<ts::PropagationFailure>(&result);
    ASSERT_NE(failure, nullptr);
    EXPECT_EQ(failure->task_index, 0U);
    EXPECT_EQ(failure->strategy_name, "ArmStrategy");
    EXPECT_FALSE(failure->inbound_state.position_known);
}

TEST(StatePropagation, FailsMidWorkflowWhenPriorEffectBreaksNextRequirement) {
    // Two-task workflow: arm then pusher. The arm's effect leaves the
    // item on PALLET; the pusher needs BELT → FAIL at task 1. This is
    // the case that justifies the propagator (cannot be caught by
    // initial-state validation alone).
    auto arms = kuka_arms();
    auto pushers = generic_pushers();
    ts::ArmStrategy arm(arms);
    ts::PusherStrategy pusher(pushers);
    const std::vector<tc::Task> workflow{
        small_palletize_task("t0"),
        small_palletize_task("t1"),
    };
    const auto r0 = arm.evaluate(workflow[0]);
    const auto r1 = pusher.evaluate(workflow[1]);
    const std::vector<const ts::StrategyResult*> chosen{&r0, &r1};
    const tc::ItemState initial{.position_known = true,
                                .orientation_resolved_to = {},
                                .on_carrier = tc::OnCarrier::Belt};

    auto result = ts::propagate_state(workflow, chosen, initial);
    const auto* failure = std::get_if<ts::PropagationFailure>(&result);
    ASSERT_NE(failure, nullptr);
    EXPECT_EQ(failure->task_index, 1U);
    EXPECT_EQ(failure->strategy_name, "PusherStrategy");
    EXPECT_EQ(failure->inbound_state.on_carrier, tc::OnCarrier::Pallet);
}

TEST(StatePropagation, MismatchedSizesThrows) {
    auto arms = kuka_arms();
    ts::ArmStrategy strategy(arms);
    const std::vector<tc::Task> workflow{small_palletize_task("t0")};
    const std::vector<const ts::StrategyResult*> chosen{};
    const tc::ItemState initial{};

    EXPECT_THROW(ts::propagate_state(workflow, chosen, initial),
                 std::invalid_argument);
}

TEST(StatePropagation, NullChosenEntryThrows) {
    const std::vector<tc::Task> workflow{small_palletize_task("t0")};
    const std::vector<const ts::StrategyResult*> chosen{nullptr};
    const tc::ItemState initial{};

    EXPECT_THROW(ts::propagate_state(workflow, chosen, initial),
                 std::logic_error);
}

TEST(StatePropagation, MissingStateFlowContractThrows) {
    // A StrategyResult without requires_state / effect set is a defect in
    // the caller's selection logic; the propagator throws rather than
    // fabricating identity behaviour past it.
    const std::vector<tc::Task> workflow{small_palletize_task("t0")};
    ts::StrategyResult bare{
        .feasibility = ts::Feasibility::FULL,
        .strategy_name = "Bare",
        .equipment = std::nullopt,
        .energy_per_cycle = 0.0 * si::joule,
        .cycle_time = 0.0 * si::second,
        // requires_state and effect intentionally left default-constructed
    };
    const std::vector<const ts::StrategyResult*> chosen{&bare};
    const tc::ItemState initial{};

    EXPECT_THROW(ts::propagate_state(workflow, chosen, initial),
                 std::logic_error);
}
