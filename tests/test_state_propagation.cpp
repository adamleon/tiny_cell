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

// small_palletize_task — defaults to a continuously-symmetric (cylinder)
// box so tests of the position/carrier gates aren't accidentally also
// failing the orientation gate. Tests that exercise the orientation
// gate pass an explicit symmetry.
tc::Task small_palletize_task(const std::string& id = "t",
                              tc::RotationalSymmetry box_symmetry = tc::symmetry::continuous()) {
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
                    .symmetry = box_symmetry,
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
        .target_ct_per_item = tc::CycleTimePerItem{5.0 * si::second},
    };
}

tc::ItemState fully_pinned_on_belt(int alignment = 0) {
    return tc::ItemState{
        .position_known = true,
        .orientation = {
            .knowledge = tc::knowledge::known(alignment),
            .control   = tc::control::constrained(alignment),
        },
        .on_carrier = tc::OnCarrier::Belt,
    };
}

} // namespace

// -- distinct_alignments -----------------------------------------------------

TEST(DistinctAlignments, ContinuousIsAlwaysOne) {
    EXPECT_EQ(tc::distinct_alignments(tc::symmetry::continuous()), 1);
}

TEST(DistinctAlignments, RectangleAt90Cardinals) {
    // Rectangle's 2-fold symmetry collapses 4 cardinals into 2 orbits.
    EXPECT_EQ(tc::distinct_alignments(tc::symmetry::discrete(180)), 2);
}

TEST(DistinctAlignments, SquareAt90Cardinals) {
    // Square's 4-fold symmetry collapses all 4 cardinals into 1 orbit.
    EXPECT_EQ(tc::distinct_alignments(tc::symmetry::discrete(90)), 1);
}

TEST(DistinctAlignments, HexagonAt90Cardinals) {
    // Hexagon's 6-fold symmetry: 60° period, so cardinals {0,180} pair
    // up and {90,270} pair up — 2 orbits.
    EXPECT_EQ(tc::distinct_alignments(tc::symmetry::discrete(60)), 2);
}

TEST(DistinctAlignments, TriangleAt90Cardinals) {
    // Triangle's 3-fold symmetry: 120° period, doesn't divide 90° —
    // no cardinal pairs in the same orbit, all 4 distinct.
    EXPECT_EQ(tc::distinct_alignments(tc::symmetry::discrete(120)), 4);
}

TEST(DistinctAlignments, AsymmetricAt90Cardinals) {
    EXPECT_EQ(tc::distinct_alignments(tc::symmetry::asymmetric()), 4);
}

TEST(DistinctAlignments, RejectsBadSnapStep) {
    EXPECT_THROW(tc::distinct_alignments(tc::symmetry::discrete(180), 17),
                 std::invalid_argument);
    EXPECT_THROW(tc::distinct_alignments(tc::symmetry::discrete(180), 0),
                 std::invalid_argument);
}

// -- Factories validate ------------------------------------------------------

TEST(SymmetryFactories, RejectOutOfRangePeriod) {
    EXPECT_THROW(tc::symmetry::discrete(0), std::invalid_argument);
    EXPECT_THROW(tc::symmetry::discrete(-90), std::invalid_argument);
    EXPECT_THROW(tc::symmetry::discrete(360), std::invalid_argument);
    EXPECT_THROW(tc::symmetry::discrete(540), std::invalid_argument);
}

TEST(KnowledgeFactories, RejectNegativeAlignment) {
    EXPECT_THROW(tc::knowledge::known(-1), std::invalid_argument);
}

TEST(ControlFactories, RejectNegativeAlignment) {
    EXPECT_THROW(tc::control::constrained(-1), std::invalid_argument);
}

// -- orientation_known / orientation_pinned / *_matches_alignment ------------

TEST(OrientationKnown, ContinuousIsAlwaysKnown) {
    EXPECT_TRUE(tc::orientation_known(tc::symmetry::continuous(),
                                      tc::knowledge::unknown()));
    EXPECT_TRUE(tc::orientation_known(tc::symmetry::continuous(),
                                      tc::knowledge::known(0)));
}

TEST(OrientationKnown, KnownPassesUnknownFails) {
    EXPECT_FALSE(tc::orientation_known(tc::symmetry::discrete(180),
                                       tc::knowledge::unknown()));
    EXPECT_TRUE(tc::orientation_known(tc::symmetry::discrete(180),
                                      tc::knowledge::known(0)));
    EXPECT_TRUE(tc::orientation_known(tc::symmetry::discrete(180),
                                      tc::knowledge::known(1)));
}

TEST(OrientationPinned, ContinuousIsAlwaysPinned) {
    EXPECT_TRUE(tc::orientation_pinned(tc::symmetry::continuous(),
                                       tc::control::free()));
    EXPECT_TRUE(tc::orientation_pinned(tc::symmetry::continuous(),
                                       tc::control::constrained(0)));
}

TEST(OrientationPinned, ConstrainedPassesFreeFails) {
    EXPECT_FALSE(tc::orientation_pinned(tc::symmetry::discrete(180),
                                        tc::control::free()));
    EXPECT_TRUE(tc::orientation_pinned(tc::symmetry::discrete(180),
                                       tc::control::constrained(0)));
}

TEST(ControlMatchesAlignment, RequiresExactMatch) {
    const auto sym = tc::symmetry::discrete(180);
    EXPECT_TRUE(tc::control_matches_alignment(sym, tc::control::constrained(0), 0));
    EXPECT_FALSE(tc::control_matches_alignment(sym, tc::control::constrained(1), 0));
    EXPECT_FALSE(tc::control_matches_alignment(sym, tc::control::free(), 0));
}

TEST(ControlMatchesAlignment, ContinuousAcceptsAnyRequirement) {
    EXPECT_TRUE(tc::control_matches_alignment(tc::symmetry::continuous(),
                                              tc::control::free(), 0));
    EXPECT_TRUE(tc::control_matches_alignment(tc::symmetry::continuous(),
                                              tc::control::constrained(0), 1));
}

// -- ArmStrategy reads Knowledge -------------------------------------------

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
    auto result = strategy.evaluate(small_palletize_task("t", tc::symmetry::continuous()));
    tc::ItemState no_pose{
        .position_known = false,
        .orientation = {tc::knowledge::known(0), tc::control::constrained(0)},
        .on_carrier = tc::OnCarrier::Belt,
    };
    EXPECT_FALSE(result.requires_state(no_pose));
}

TEST(ArmStrategyState, ReadsKnowledgeNotControl) {
    // Arm passes when knowledge resolves the alignment, regardless of
    // whether control is Free or Constrained. The arm plans its motion
    // from the planner's belief.
    auto arms = kuka_arms();
    ts::ArmStrategy strategy(arms);
    auto result = strategy.evaluate(small_palletize_task("t", tc::symmetry::discrete(180)));
    tc::ItemState known_no_constraint{
        .position_known = true,
        .orientation = {tc::knowledge::known(0), tc::control::free()},
        .on_carrier = tc::OnCarrier::Belt,
    };
    tc::ItemState unknown_with_constraint{
        .position_known = true,
        .orientation = {tc::knowledge::unknown(), tc::control::constrained(0)},
        .on_carrier = tc::OnCarrier::Belt,
    };
    EXPECT_TRUE(result.requires_state(known_no_constraint));
    EXPECT_FALSE(result.requires_state(unknown_with_constraint));
}

TEST(ArmStrategyState, CarrierAgnostic) {
    auto arms = kuka_arms();
    ts::ArmStrategy strategy(arms);
    auto result = strategy.evaluate(small_palletize_task("t", tc::symmetry::continuous()));
    for (auto c : {tc::OnCarrier::Belt, tc::OnCarrier::Pallet,
                   tc::OnCarrier::Free, tc::OnCarrier::Fixture}) {
        tc::ItemState s{
            .position_known = true,
            .orientation = {tc::knowledge::known(0), tc::control::free()},
            .on_carrier = c,
        };
        EXPECT_TRUE(result.requires_state(s))
            << "arm should accept any carrier (got rejection for index "
            << static_cast<int>(c) << ")";
    }
}

TEST(ArmStrategyState, EffectPinsBothAxes) {
    auto arms = kuka_arms();
    ts::ArmStrategy strategy(arms);
    auto result = strategy.evaluate(small_palletize_task());
    auto output = result.effect(fully_pinned_on_belt());
    EXPECT_TRUE(output.position_known);
    EXPECT_EQ(output.on_carrier, tc::OnCarrier::Pallet);
    EXPECT_TRUE(std::holds_alternative<tc::OrientationKnowledge::Known>(
        output.orientation.knowledge.kind));
    EXPECT_TRUE(std::holds_alternative<tc::OrientationControl::Constrained>(
        output.orientation.control.kind));
}

// -- PusherStrategy reads Control ------------------------------------------

TEST(PusherStrategyState, RequiresPositionKnownAndOnBelt) {
    auto pushers = generic_pushers();
    ts::PusherStrategy strategy(pushers);
    auto result = strategy.evaluate(small_palletize_task("t", tc::symmetry::continuous()));
    tc::ItemState belt_pinned{
        .position_known = true,
        .orientation = {tc::knowledge::known(0), tc::control::constrained(0)},
        .on_carrier = tc::OnCarrier::Belt,
    };
    tc::ItemState pallet_pinned{
        .position_known = true,
        .orientation = {tc::knowledge::known(0), tc::control::constrained(0)},
        .on_carrier = tc::OnCarrier::Pallet,
    };
    tc::ItemState belt_no_position{
        .position_known = false,
        .orientation = {tc::knowledge::known(0), tc::control::constrained(0)},
        .on_carrier = tc::OnCarrier::Belt,
    };
    EXPECT_TRUE(result.requires_state(belt_pinned));
    EXPECT_FALSE(result.requires_state(pallet_pinned));
    EXPECT_FALSE(result.requires_state(belt_no_position));
}

TEST(PusherStrategyState, RejectsKnowledgeOnlyForDiscreteSymmetry) {
    // The K/C-split motivator: rectangle with Knowledge=Known but
    // Control=Free → pusher rejects because the item could rotate
    // between observation and contact.
    auto pushers = generic_pushers();
    ts::PusherStrategy strategy(pushers);
    auto result = strategy.evaluate(small_palletize_task("t", tc::symmetry::discrete(180)));
    // Probe both alignments — pusher's required alignment is whichever
    // its row math picked.
    bool any_passes_with_free = false;
    for (int a : {0, 1}) {
        tc::ItemState known_only{
            .position_known = true,
            .orientation = {tc::knowledge::known(a), tc::control::free()},
            .on_carrier = tc::OnCarrier::Belt,
        };
        if (result.requires_state(known_only)) any_passes_with_free = true;
    }
    EXPECT_FALSE(any_passes_with_free)
        << "Pusher should reject Knowledge-only (Control=Free) for a "
           "rectangle — the physical hold is required for the open-loop "
           "stroke.";
}

TEST(PusherStrategyState, AcceptsConstrainedAtRequiredAlignment) {
    auto pushers = generic_pushers();
    ts::PusherStrategy strategy(pushers);
    auto result = strategy.evaluate(small_palletize_task("t", tc::symmetry::discrete(180)));
    // Try both alignments — exactly one must satisfy.
    int passing = 0;
    for (int a : {0, 1}) {
        tc::ItemState pinned{
            .position_known = true,
            .orientation = {tc::knowledge::known(a), tc::control::constrained(a)},
            .on_carrier = tc::OnCarrier::Belt,
        };
        if (result.requires_state(pinned)) ++passing;
    }
    EXPECT_EQ(passing, 1)
        << "Pusher should accept Constrained at exactly one alignment "
           "(its row-math choice).";
}

TEST(PusherStrategyState, ContinuousSymmetryShortCircuits) {
    // Cylinder: pusher passes despite Unknown knowledge and Free control.
    auto pushers = generic_pushers();
    ts::PusherStrategy strategy(pushers);
    auto result = strategy.evaluate(small_palletize_task("t", tc::symmetry::continuous()));
    tc::ItemState belt_unaligned{
        .position_known = true,
        .orientation = {tc::knowledge::unknown(), tc::control::free()},
        .on_carrier = tc::OnCarrier::Belt,
    };
    EXPECT_TRUE(result.requires_state(belt_unaligned));
}

TEST(PusherStrategyState, EffectPinsBothAxes) {
    auto pushers = generic_pushers();
    ts::PusherStrategy strategy(pushers);
    auto result = strategy.evaluate(small_palletize_task());
    auto output = result.effect(fully_pinned_on_belt());
    EXPECT_TRUE(output.position_known);
    EXPECT_EQ(output.on_carrier, tc::OnCarrier::Pallet);
    EXPECT_TRUE(std::holds_alternative<tc::OrientationKnowledge::Known>(
        output.orientation.knowledge.kind));
    EXPECT_TRUE(std::holds_alternative<tc::OrientationControl::Constrained>(
        output.orientation.control.kind));
}

// -- Propagation pass --------------------------------------------------------

TEST(StatePropagation, SingleTaskSuccessReturnsTwoStateTrajectory) {
    auto arms = kuka_arms();
    ts::ArmStrategy strategy(arms);
    const std::vector<tc::Task> workflow{
        small_palletize_task("t0", tc::symmetry::continuous())};
    const auto r0 = strategy.evaluate(workflow[0]);
    const std::vector<const ts::StrategyResult*> chosen{&r0};
    const auto initial = fully_pinned_on_belt();

    auto result = ts::propagate_state(workflow, chosen, initial);
    const auto* success = std::get_if<ts::PropagationSuccess>(&result);
    ASSERT_NE(success, nullptr);
    ASSERT_EQ(success->trajectory.size(), 2U);
    EXPECT_EQ(success->trajectory[0].on_carrier, tc::OnCarrier::Belt);
    EXPECT_EQ(success->trajectory[1].on_carrier, tc::OnCarrier::Pallet);
}

TEST(StatePropagation, FirstTrajectoryStateIsInitial) {
    auto arms = kuka_arms();
    ts::ArmStrategy strategy(arms);
    const std::vector<tc::Task> workflow{
        small_palletize_task("t0", tc::symmetry::continuous())};
    const auto r0 = strategy.evaluate(workflow[0]);
    const std::vector<const ts::StrategyResult*> chosen{&r0};
    tc::ItemState initial = fully_pinned_on_belt();
    initial.on_carrier = tc::OnCarrier::Fixture;

    auto result = ts::propagate_state(workflow, chosen, initial);
    const auto* success = std::get_if<ts::PropagationSuccess>(&result);
    ASSERT_NE(success, nullptr);
    EXPECT_EQ(success->trajectory[0].on_carrier, tc::OnCarrier::Fixture);
}

TEST(StatePropagation, FailsAtFirstTaskWhenPositionUnknown) {
    auto arms = kuka_arms();
    ts::ArmStrategy strategy(arms);
    const std::vector<tc::Task> workflow{
        small_palletize_task("t0", tc::symmetry::continuous())};
    const auto r0 = strategy.evaluate(workflow[0]);
    const std::vector<const ts::StrategyResult*> chosen{&r0};
    tc::ItemState initial = fully_pinned_on_belt();
    initial.position_known = false;

    auto result = ts::propagate_state(workflow, chosen, initial);
    const auto* failure = std::get_if<ts::PropagationFailure>(&result);
    ASSERT_NE(failure, nullptr);
    EXPECT_EQ(failure->task_index, 0U);
    EXPECT_EQ(failure->strategy_name, "ArmStrategy");
    EXPECT_FALSE(failure->inbound_state.position_known);
}

TEST(StatePropagation, PusherRejectsKnowledgeOnlyOnRectangle) {
    // The headline scenario for the K/C split: rectangle observed but
    // not physically held → pusher rejects in propagation.
    auto pushers = generic_pushers();
    ts::PusherStrategy strategy(pushers);
    const std::vector<tc::Task> workflow{
        small_palletize_task("t0", tc::symmetry::discrete(180))};
    const auto r0 = strategy.evaluate(workflow[0]);
    const std::vector<const ts::StrategyResult*> chosen{&r0};
    tc::ItemState initial{
        .position_known = true,
        .orientation = {tc::knowledge::known(0), tc::control::free()},
        .on_carrier = tc::OnCarrier::Belt,
    };

    auto result = ts::propagate_state(workflow, chosen, initial);
    ASSERT_TRUE(std::holds_alternative<ts::PropagationFailure>(result));
    EXPECT_EQ(std::get<ts::PropagationFailure>(result).strategy_name,
              "PusherStrategy");
}

TEST(StatePropagation, FailsMidWorkflowWhenPriorEffectChangesCarrier) {
    // Two-task workflow: arm then pusher. The arm's effect leaves the
    // item on PALLET; the pusher needs BELT → FAIL at task 1.
    auto arms = kuka_arms();
    auto pushers = generic_pushers();
    ts::ArmStrategy arm(arms);
    ts::PusherStrategy pusher(pushers);
    const std::vector<tc::Task> workflow{
        small_palletize_task("t0", tc::symmetry::continuous()),
        small_palletize_task("t1", tc::symmetry::continuous()),
    };
    const auto r0 = arm.evaluate(workflow[0]);
    const auto r1 = pusher.evaluate(workflow[1]);
    const std::vector<const ts::StrategyResult*> chosen{&r0, &r1};
    const auto initial = fully_pinned_on_belt();

    auto result = ts::propagate_state(workflow, chosen, initial);
    const auto* failure = std::get_if<ts::PropagationFailure>(&result);
    ASSERT_NE(failure, nullptr);
    EXPECT_EQ(failure->task_index, 1U);
    EXPECT_EQ(failure->strategy_name, "PusherStrategy");
    EXPECT_EQ(failure->inbound_state.on_carrier, tc::OnCarrier::Pallet);
}

TEST(StatePropagation, MismatchedSizesThrows) {
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
    // A StrategyResult without requires_state / effect set is a defect
    // in the caller's selection logic; the propagator throws rather
    // than fabricating identity behaviour past it.
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
