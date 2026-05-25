// Tests for the task-side LogicalPort declaration + task_ports() helper.
// Strategy-side PortConstraint lands in T.3 with its own tests.

#include <gtest/gtest.h>
#include <algorithm>
#include <mp-units/systems/si.h>
#include <tinycell/model/port.hpp>
#include <tinycell/model/task.hpp>

namespace tc = tinycell::core;
using mp_units::si::metre;
using mp_units::si::second;
using mp_units::si::kilogram;

namespace {

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

bool has_port(const std::vector<tc::LogicalPort>& ports,
              const std::string& name, tc::PortDirection dir) {
    return std::any_of(ports.begin(), ports.end(),
        [&](const tc::LogicalPort& p) {
            return p.name == name && p.direction == dir;
        });
}

} // namespace

TEST(Ports, PalletizeHasItemInPalletInPalletOut) {
    auto t = make_palletize();
    auto ports = tc::task_ports(t);
    ASSERT_EQ(ports.size(), 3u);
    EXPECT_TRUE(has_port(ports, "item_in",    tc::PortDirection::Input));
    EXPECT_TRUE(has_port(ports, "pallet_in",  tc::PortDirection::Input));
    EXPECT_TRUE(has_port(ports, "pallet_out", tc::PortDirection::Output));
}

TEST(Ports, PalletizePortsAreStableAcrossInstances) {
    // The port set is a property of the kind, not of any per-instance
    // params — two Palletize tasks with different items/pallets/counts
    // declare the same ports.
    auto t1 = make_palletize();
    auto t2 = make_palletize();
    t2.id = "t_p2";
    std::get<tc::PalletizeParams>(t2.params).box_count = 99;

    auto p1 = tc::task_ports(t1);
    auto p2 = tc::task_ports(t2);
    ASSERT_EQ(p1.size(), p2.size());
    for (std::size_t i = 0; i < p1.size(); ++i) {
        EXPECT_EQ(p1[i].name, p2[i].name);
        EXPECT_EQ(p1[i].direction, p2[i].direction);
    }
}

tc::Task make_transport(const std::string& id,
                        const std::string& src_task, const std::string& src_port,
                        const std::string& dst_task, const std::string& dst_port) {
    return tc::Task{
        .id = id,
        .params = tc::TransportParams{
            .source_task_id = src_task,
            .source_port_name = src_port,
            .sink_task_id = dst_task,
            .sink_port_name = dst_port,
        },
        // Transport tasks are "magical" at MVP - TransferStrategy will
        // hit any positive target. Workflow author can refine later.
        .target_ct_per_item = tc::CycleTimePerItem{1.0 * second},
    };
}

TEST(Ports, TransportTaskHasNoLogicalPorts) {
    // A Transport is an edge between two other tasks' ports - it
    // doesn't expose ports of its own.
    auto t = make_transport("t_xport", "t_src", "pallet_out", "t_dst", "pallet_in");
    EXPECT_EQ(t.kind(), tc::TaskKind::Transport);
    EXPECT_TRUE(tc::task_ports(t).empty());
}

TEST(Ports, TransportParamsCarryEndpointReferences) {
    auto t = make_transport("t_xport", "t_pallet_a", "pallet_out", "t_pallet_b", "pallet_in");
    const auto& p = std::get<tc::TransportParams>(t.params);
    EXPECT_EQ(p.source_task_id, "t_pallet_a");
    EXPECT_EQ(p.source_port_name, "pallet_out");
    EXPECT_EQ(p.sink_task_id, "t_pallet_b");
    EXPECT_EQ(p.sink_port_name, "pallet_in");
}

TEST(Ports, AngleAliasCompilesAndStoresRadians) {
    // Smoke: the Angle alias works the same way as the other unit
    // aliases (mp_units quantity in radians).
    tc::Angle a = 1.5708 * mp_units::si::radian;
    EXPECT_NEAR(a.numerical_value_in(mp_units::si::radian), 1.5708, 1e-9);
}
