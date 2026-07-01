// Tests for TransferStrategy — the magical fulfiller of Transport
// tasks. Verifies the applies_to filter, the FULL response shape, and
// that it doesn't accidentally match non-Transport tasks.

#include <gtest/gtest.h>
#include <mp-units/systems/si.h>
#include <tinycell/model/item.hpp>
#include <tinycell/model/task.hpp>
#include <tinycell/solver/transfer_strategy.hpp>

namespace tc = tinycell::core;
namespace ts = tinycell::solver;
using mp_units::si::metre;
using mp_units::si::second;
using mp_units::si::kilogram;
using mp_units::si::joule;

namespace {

tc::Task make_transport() {
    return tc::Task{
        .id = "t_xport",
        .params = tc::TransportParams{
            .source_task_id = "t_src",
            .source_port_name = "pallet_out",
            .sink_task_id = "t_dst",
            .sink_port_name = "pallet_in",
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

TEST(TransferStrategy, NameMatchesConvention) {
    ts::TransferStrategy s;
    EXPECT_EQ(s.name(), "TransferStrategy");
}

TEST(TransferStrategy, AppliesToTransportTasksOnly) {
    ts::TransferStrategy s;
    EXPECT_TRUE(s.applies_to(make_transport()));
    EXPECT_FALSE(s.applies_to(make_palletize()));
}

TEST(TransferStrategy, EvaluateReturnsFullWithZeroCost) {
    ts::TransferStrategy s;
    auto r = s.evaluate(make_transport());
    EXPECT_EQ(r.feasibility, ts::Feasibility::FULL);
    EXPECT_EQ(r.strategy_name, "TransferStrategy");
    EXPECT_FALSE(r.equipment.has_value());
    EXPECT_NEAR(r.energy_per_cycle.numerical_value_in(joule),       0.0, 1e-9);
    EXPECT_NEAR(r.cycle_time.numerical_value_in(second),            0.0, 1e-9);
    EXPECT_NEAR(r.achievable_ct_per_item.numerical_value_in(second), 0.0, 1e-9);
    EXPECT_FALSE(r.partial_info.has_value());
    EXPECT_TRUE(r.preconditions.empty());
    EXPECT_TRUE(r.port_constraints.empty());
}

TEST(TransferStrategy, StateFunctionsArePermissiveAndIdentity) {
    ts::TransferStrategy s;
    auto r = s.evaluate(make_transport());
    // Any inbound state passes the requires_state check.
    tc::ItemState st{
        .position_known = false,
        .orientation = {
            .knowledge = tc::knowledge::unknown(),
            .control = tc::control::free(),
        },
        .on_carrier = tc::OnCarrier::Belt,
    };
    EXPECT_TRUE(r.requires_state(st));
    auto st2 = r.effect(st);
    EXPECT_EQ(st2.position_known, st.position_known);
    // Identity effect — out == in.
}
