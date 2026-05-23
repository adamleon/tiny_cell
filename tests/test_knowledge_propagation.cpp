#include <filesystem>
#include <gtest/gtest.h>
#include <limits>
#include <mp-units/systems/si.h>
#include <stdexcept>
#include <tinycell/io/catalog_loader.hpp>
#include <tinycell/model/item.hpp>
#include <tinycell/model/task.hpp>
#include <tinycell/solver/arm_strategy.hpp>
#include <tinycell/solver/knowledge_propagation.hpp>
#include <tinycell/solver/pusher_strategy.hpp>
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
// box so tests of the carrier/position gates aren't accidentally also
// failing the orientation gate. Tests that want to exercise the
// orientation gate pass an explicit symmetry.
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
    };
}

tc::ItemKnowledge fully_known_on_belt() {
    return tc::ItemKnowledge{
        .position_known = true,
        .orientation = tc::orientation::exact(),
        .on_carrier = tc::OnCarrier::Belt,
    };
}

} // namespace

// -- Property helper: distinct_orientations + orientation_resolved -----------

TEST(DistinctOrientations, ContinuousIsAlwaysOne) {
    EXPECT_EQ(tc::distinct_orientations(tc::symmetry::continuous(),
                                        tc::orientation::unknown()), 1);
    EXPECT_EQ(tc::distinct_orientations(tc::symmetry::continuous(),
                                        tc::orientation::snapped(90)), 1);
    EXPECT_EQ(tc::distinct_orientations(tc::symmetry::continuous(),
                                        tc::orientation::exact()), 1);
}

TEST(DistinctOrientations, ExactKnowledgeIsAlwaysOne) {
    EXPECT_EQ(tc::distinct_orientations(tc::symmetry::asymmetric(),
                                        tc::orientation::exact()), 1);
    EXPECT_EQ(tc::distinct_orientations(tc::symmetry::discrete(180),
                                        tc::orientation::exact()), 1);
}

TEST(DistinctOrientations, UnknownKnowledgeIsUnbounded) {
    EXPECT_EQ(tc::distinct_orientations(tc::symmetry::asymmetric(),
                                        tc::orientation::unknown()),
              std::numeric_limits<int>::max());
    EXPECT_EQ(tc::distinct_orientations(tc::symmetry::discrete(180),
                                        tc::orientation::unknown()),
              std::numeric_limits<int>::max());
}

TEST(DistinctOrientations, RectangleAfterFixtureLeavesTwoClasses) {
    // Rectangle (G=180) + fixture (K=90) → 180/90 = 2. The user's
    // motivating example for the model.
    EXPECT_EQ(tc::distinct_orientations(tc::symmetry::discrete(180),
                                        tc::orientation::snapped(90)), 2);
}

TEST(DistinctOrientations, RectangleAfterCameraResolvesToOne) {
    EXPECT_EQ(tc::distinct_orientations(tc::symmetry::discrete(180),
                                        tc::orientation::snapped(180)), 1);
}

TEST(DistinctOrientations, SquareAtAnyCardinalSnapResolvesToOne) {
    EXPECT_EQ(tc::distinct_orientations(tc::symmetry::discrete(90),
                                        tc::orientation::snapped(90)), 1);
}

TEST(DistinctOrientations, AsymmetricUnderSnappedScalesWithSnapStep) {
    EXPECT_EQ(tc::distinct_orientations(tc::symmetry::asymmetric(),
                                        tc::orientation::snapped(90)), 4);
    EXPECT_EQ(tc::distinct_orientations(tc::symmetry::asymmetric(),
                                        tc::orientation::snapped(180)), 2);
}

TEST(OrientationResolved, IsTrueIffDistinctOrientationsEqualsOne) {
    EXPECT_TRUE(tc::orientation_resolved(tc::symmetry::continuous(),
                                         tc::orientation::unknown()));
    EXPECT_TRUE(tc::orientation_resolved(tc::symmetry::discrete(180),
                                         tc::orientation::snapped(180)));
    EXPECT_FALSE(tc::orientation_resolved(tc::symmetry::discrete(180),
                                          tc::orientation::snapped(90)));
    EXPECT_FALSE(tc::orientation_resolved(tc::symmetry::discrete(180),
                                          tc::orientation::unknown()));
}

TEST(SymmetryFactories, RejectOutOfRangeDiscreteFold) {
    EXPECT_THROW(tc::symmetry::discrete(0), std::invalid_argument);
    EXPECT_THROW(tc::symmetry::discrete(-90), std::invalid_argument);
    EXPECT_THROW(tc::symmetry::discrete(360), std::invalid_argument);
    EXPECT_THROW(tc::symmetry::discrete(540), std::invalid_argument);
}

TEST(OrientationFactories, RejectOutOfRangeSnapStep) {
    EXPECT_THROW(tc::orientation::snapped(0), std::invalid_argument);
    EXPECT_THROW(tc::orientation::snapped(-30), std::invalid_argument);
    EXPECT_THROW(tc::orientation::snapped(360), std::invalid_argument);
}

// -- Strategy knowledge-flow contract ---------------------------------------

TEST(ArmStrategyKnowledge, EmitsRequiresKnowledgeAndEffect) {
    auto arms = kuka_arms();
    ts::ArmStrategy strategy(arms);
    auto result = strategy.evaluate(small_palletize_task());
    EXPECT_TRUE(static_cast<bool>(result.requires_knowledge));
    EXPECT_TRUE(static_cast<bool>(result.effect));
}

TEST(ArmStrategyKnowledge, RequiresPositionKnown) {
    auto arms = kuka_arms();
    ts::ArmStrategy strategy(arms);
    auto result = strategy.evaluate(small_palletize_task("t", tc::symmetry::continuous()));
    tc::ItemKnowledge no_pose{
        .position_known = false,
        .orientation = tc::orientation::exact(),
        .on_carrier = tc::OnCarrier::Belt,
    };
    tc::ItemKnowledge with_pose{
        .position_known = true,
        .orientation = tc::orientation::exact(),
        .on_carrier = tc::OnCarrier::Free,
    };
    EXPECT_FALSE(result.requires_knowledge(no_pose));
    EXPECT_TRUE(result.requires_knowledge(with_pose));
}

TEST(ArmStrategyKnowledge, RequiresOrientationResolved) {
    auto arms = kuka_arms();
    ts::ArmStrategy strategy(arms);
    // Rectangle (Discrete(180)) — unknown orientation FAILs; snapped(180)
    // resolves it under the symmetry → passes.
    auto result = strategy.evaluate(
        small_palletize_task("t", tc::symmetry::discrete(180)));
    tc::ItemKnowledge unknown_orient{
        .position_known = true,
        .orientation = tc::orientation::unknown(),
        .on_carrier = tc::OnCarrier::Belt,
    };
    tc::ItemKnowledge resolved_orient{
        .position_known = true,
        .orientation = tc::orientation::snapped(180),
        .on_carrier = tc::OnCarrier::Belt,
    };
    EXPECT_FALSE(result.requires_knowledge(unknown_orient));
    EXPECT_TRUE(result.requires_knowledge(resolved_orient));
}

TEST(ArmStrategyKnowledge, CarrierAgnostic) {
    auto arms = kuka_arms();
    ts::ArmStrategy strategy(arms);
    auto result = strategy.evaluate(small_palletize_task("t", tc::symmetry::continuous()));
    for (auto c : {tc::OnCarrier::Belt, tc::OnCarrier::Pallet,
                   tc::OnCarrier::Free, tc::OnCarrier::Fixture}) {
        tc::ItemKnowledge k{
            .position_known = true,
            .orientation = tc::orientation::exact(),
            .on_carrier = c,
        };
        EXPECT_TRUE(result.requires_knowledge(k))
            << "arm should accept any carrier (got rejection for carrier index "
            << static_cast<int>(c) << ")";
    }
}

TEST(ArmStrategyKnowledge, EffectPlacesItemOnPalletWithExactOrientation) {
    auto arms = kuka_arms();
    ts::ArmStrategy strategy(arms);
    auto result = strategy.evaluate(small_palletize_task());
    auto output = result.effect(fully_known_on_belt());
    EXPECT_TRUE(output.position_known);
    EXPECT_EQ(output.on_carrier, tc::OnCarrier::Pallet);
    EXPECT_TRUE(std::holds_alternative<tc::OrientationKnowledge::Exact>(
        output.orientation.kind));
}

TEST(PusherStrategyKnowledge, RequiresPositionKnownAndOnBelt) {
    auto pushers = generic_pushers();
    ts::PusherStrategy strategy(pushers);
    auto result = strategy.evaluate(small_palletize_task("t", tc::symmetry::continuous()));
    tc::ItemKnowledge belt_known{
        .position_known = true,
        .orientation = tc::orientation::exact(),
        .on_carrier = tc::OnCarrier::Belt,
    };
    tc::ItemKnowledge belt_unknown_pose{
        .position_known = false,
        .orientation = tc::orientation::exact(),
        .on_carrier = tc::OnCarrier::Belt,
    };
    tc::ItemKnowledge pallet_known{
        .position_known = true,
        .orientation = tc::orientation::exact(),
        .on_carrier = tc::OnCarrier::Pallet,
    };
    EXPECT_TRUE(result.requires_knowledge(belt_known));
    EXPECT_FALSE(result.requires_knowledge(belt_unknown_pose));
    EXPECT_FALSE(result.requires_knowledge(pallet_known));
}

TEST(PusherStrategyKnowledge, RequiresOrientationResolved) {
    auto pushers = generic_pushers();
    ts::PusherStrategy strategy(pushers);
    auto result = strategy.evaluate(
        small_palletize_task("t", tc::symmetry::discrete(180)));
    tc::ItemKnowledge unresolved{
        .position_known = true,
        .orientation = tc::orientation::unknown(),
        .on_carrier = tc::OnCarrier::Belt,
    };
    tc::ItemKnowledge resolved{
        .position_known = true,
        .orientation = tc::orientation::snapped(180),
        .on_carrier = tc::OnCarrier::Belt,
    };
    EXPECT_FALSE(result.requires_knowledge(unresolved));
    EXPECT_TRUE(result.requires_knowledge(resolved));
}

TEST(PusherStrategyKnowledge, EffectPlacesItemOnPalletWithExactOrientation) {
    auto pushers = generic_pushers();
    ts::PusherStrategy strategy(pushers);
    auto result = strategy.evaluate(small_palletize_task());
    auto output = result.effect(fully_known_on_belt());
    EXPECT_TRUE(output.position_known);
    EXPECT_EQ(output.on_carrier, tc::OnCarrier::Pallet);
    EXPECT_TRUE(std::holds_alternative<tc::OrientationKnowledge::Exact>(
        output.orientation.kind));
}

// -- Propagation pass --------------------------------------------------------

TEST(KnowledgePropagation, SingleTaskSuccessReturnsTwoStateTrajectory) {
    auto arms = kuka_arms();
    ts::ArmStrategy strategy(arms);
    const std::vector<tc::Task> workflow{
        small_palletize_task("t0", tc::symmetry::continuous())};
    const auto r0 = strategy.evaluate(workflow[0]);
    const std::vector<const ts::StrategyResult*> chosen{&r0};
    const auto initial = fully_known_on_belt();

    auto result = ts::propagate_knowledge(workflow, chosen, initial);
    const auto* success = std::get_if<ts::PropagationSuccess>(&result);
    ASSERT_NE(success, nullptr);
    ASSERT_EQ(success->trajectory.size(), 2U);
    EXPECT_EQ(success->trajectory[0].position_known, true);
    EXPECT_EQ(success->trajectory[0].on_carrier, tc::OnCarrier::Belt);
    EXPECT_EQ(success->trajectory[1].on_carrier, tc::OnCarrier::Pallet);
}

TEST(KnowledgePropagation, FirstTrajectoryStateIsInitial) {
    auto arms = kuka_arms();
    ts::ArmStrategy strategy(arms);
    const std::vector<tc::Task> workflow{
        small_palletize_task("t0", tc::symmetry::continuous())};
    const auto r0 = strategy.evaluate(workflow[0]);
    const std::vector<const ts::StrategyResult*> chosen{&r0};
    const tc::ItemKnowledge initial{
        .position_known = true,
        .orientation = tc::orientation::exact(),
        .on_carrier = tc::OnCarrier::Fixture,
    };

    auto result = ts::propagate_knowledge(workflow, chosen, initial);
    const auto* success = std::get_if<ts::PropagationSuccess>(&result);
    ASSERT_NE(success, nullptr);
    EXPECT_EQ(success->trajectory[0].on_carrier, tc::OnCarrier::Fixture);
}

TEST(KnowledgePropagation, FailsAtFirstTaskWhenInitialKnowledgeRejected) {
    auto arms = kuka_arms();
    ts::ArmStrategy strategy(arms);
    const std::vector<tc::Task> workflow{
        small_palletize_task("t0", tc::symmetry::continuous())};
    const auto r0 = strategy.evaluate(workflow[0]);
    const std::vector<const ts::StrategyResult*> chosen{&r0};
    const tc::ItemKnowledge initial{
        .position_known = false,
        .orientation = tc::orientation::exact(),
        .on_carrier = tc::OnCarrier::Belt,
    };

    auto result = ts::propagate_knowledge(workflow, chosen, initial);
    const auto* failure = std::get_if<ts::PropagationFailure>(&result);
    ASSERT_NE(failure, nullptr);
    EXPECT_EQ(failure->task_index, 0U);
    EXPECT_EQ(failure->strategy_name, "ArmStrategy");
    EXPECT_FALSE(failure->inbound_knowledge.position_known);
}

TEST(KnowledgePropagation, FailsOnOrientationGate) {
    // Rectangle + unknown orientation → arm's orientation gate fires.
    auto arms = kuka_arms();
    ts::ArmStrategy strategy(arms);
    const std::vector<tc::Task> workflow{
        small_palletize_task("t0", tc::symmetry::discrete(180))};
    const auto r0 = strategy.evaluate(workflow[0]);
    const std::vector<const ts::StrategyResult*> chosen{&r0};
    const tc::ItemKnowledge initial{
        .position_known = true,
        .orientation = tc::orientation::unknown(),
        .on_carrier = tc::OnCarrier::Belt,
    };

    auto result = ts::propagate_knowledge(workflow, chosen, initial);
    ASSERT_TRUE(std::holds_alternative<ts::PropagationFailure>(result));
    EXPECT_EQ(std::get<ts::PropagationFailure>(result).strategy_name,
              "ArmStrategy");
}

TEST(KnowledgePropagation, FailsMidWorkflowWhenPriorEffectBreaksNextRequirement) {
    // Two-task workflow: arm then pusher. The arm's effect leaves the
    // item on PALLET; the pusher needs BELT → FAIL at task 1. The case
    // that justifies the propagator (cannot be caught by initial-state
    // validation alone). Cylinder symmetry keeps the orientation gate
    // trivially satisfied so the carrier failure is isolated.
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
    const auto initial = fully_known_on_belt();

    auto result = ts::propagate_knowledge(workflow, chosen, initial);
    const auto* failure = std::get_if<ts::PropagationFailure>(&result);
    ASSERT_NE(failure, nullptr);
    EXPECT_EQ(failure->task_index, 1U);
    EXPECT_EQ(failure->strategy_name, "PusherStrategy");
    EXPECT_EQ(failure->inbound_knowledge.on_carrier, tc::OnCarrier::Pallet);
}

TEST(KnowledgePropagation, MismatchedSizesThrows) {
    const std::vector<tc::Task> workflow{small_palletize_task("t0")};
    const std::vector<const ts::StrategyResult*> chosen{};
    const tc::ItemKnowledge initial{};

    EXPECT_THROW(ts::propagate_knowledge(workflow, chosen, initial),
                 std::invalid_argument);
}

TEST(KnowledgePropagation, NullChosenEntryThrows) {
    const std::vector<tc::Task> workflow{small_palletize_task("t0")};
    const std::vector<const ts::StrategyResult*> chosen{nullptr};
    const tc::ItemKnowledge initial{};

    EXPECT_THROW(ts::propagate_knowledge(workflow, chosen, initial),
                 std::logic_error);
}

TEST(KnowledgePropagation, MissingKnowledgeFlowContractThrows) {
    // A StrategyResult without requires_knowledge / effect set is a
    // defect in the caller's selection logic; the propagator throws
    // rather than fabricating identity behaviour past it.
    const std::vector<tc::Task> workflow{small_palletize_task("t0")};
    ts::StrategyResult bare{
        .feasibility = ts::Feasibility::FULL,
        .strategy_name = "Bare",
        .equipment = std::nullopt,
        .energy_per_cycle = 0.0 * si::joule,
        .cycle_time = 0.0 * si::second,
        // requires_knowledge and effect intentionally left default-constructed
    };
    const std::vector<const ts::StrategyResult*> chosen{&bare};
    const tc::ItemKnowledge initial{};

    EXPECT_THROW(ts::propagate_knowledge(workflow, chosen, initial),
                 std::logic_error);
}
