// Tests for AnchorStrategy — the trivial fulfiller of Anchor tasks
// that pins workflow nodes to world poses (feeders, dispatches).

#include <gtest/gtest.h>
#include <cmath>
#include <mp-units/systems/si.h>
#include <numbers>
#include <tinycell/model/port.hpp>
#include <tinycell/model/task.hpp>
#include <tinycell/solver/anchor_strategy.hpp>

namespace tc = tinycell::core;
namespace ts = tinycell::solver;
using mp_units::si::metre;
using mp_units::si::radian;
using mp_units::si::second;
using mp_units::si::kilogram;

namespace {

tc::Task make_anchor(double world_x, double world_y, double world_theta,
                    tc::PortDirection role = tc::PortDirection::Output) {
    return tc::Task{
        .id = "t_anchor",
        .params = tc::AnchorParams{
            .name = "feeder",
            .role = role,
            .world_x = world_x * metre,
            .world_y = world_y * metre,
            .world_theta = world_theta * radian,
        },
        .target_ct_per_item = tc::CycleTimePerItem{1.0 * second},
    };
}

tc::Task make_palletize() {
    return tc::Task{
        .id = "t_p",
        .params = tc::PalletizeParams{
            .item_id = "box",
            .item = tc::BoxSpec{
                .physical = tc::ItemPhysical{
                    .width = 0.3 * metre, .length = 0.4 * metre,
                    .height = 0.2 * metre, .mass = 5.0 * kilogram,
                    .symmetry = tc::symmetry::discrete(180),
                },
            },
            .pallet = tc::PalletSpec{
                .physical = tc::ItemPhysical{
                    .width = 1.2 * metre, .length = 0.8 * metre,
                    .height = 0.15 * metre, .mass = 25.0 * kilogram,
                    .symmetry = tc::symmetry::discrete(180),
                },
            },
            .box_count = 24,
        },
        .target_ct_per_item = tc::CycleTimePerItem{5.0 * second},
    };
}

} // namespace

TEST(AnchorStrategy, NameMatchesConvention) {
    ts::AnchorStrategy s;
    EXPECT_EQ(s.name(), "AnchorStrategy");
}

TEST(AnchorStrategy, AppliesToAnchorTasksOnly) {
    ts::AnchorStrategy s;
    EXPECT_TRUE(s.applies_to(make_anchor(0.0, 0.0, 0.0)));
    EXPECT_FALSE(s.applies_to(make_palletize()));
}

TEST(AnchorStrategy, EmitsPortConstraintAtWorldPose) {
    ts::AnchorStrategy s;
    const double theta = std::numbers::pi / 4.0;  // 45 deg
    auto result = s.evaluate(make_anchor(3.0, -2.0, theta));
    EXPECT_EQ(result.feasibility, ts::Feasibility::FULL);
    EXPECT_FALSE(result.equipment.has_value());

    ASSERT_EQ(result.port_constraints.size(), 1u);
    const auto& pc = result.port_constraints[0];
    EXPECT_EQ(pc.port_name, "port");
    EXPECT_NEAR(pc.x.numerical_value_in(metre), 3.0, 1e-9);
    EXPECT_NEAR(pc.y.numerical_value_in(metre), -2.0, 1e-9);
    EXPECT_NEAR(pc.theta.numerical_value_in(radian), theta, 1e-9);
    EXPECT_NEAR(pc.direction_tolerance.numerical_value_in(radian), 0.0, 1e-9);
}
