// Smoke tests for Layer-2 cost accounting (solver/cost.cpp).
//
// Tests are arithmetic-driven: each test sets up an allocation with
// hand-chosen numbers so per-workflow and lifetime quantities are
// computable by hand and can be asserted exactly.
//
// Key invariant under test: no double-counting between within-cycle
// idle (already inside energy_per_cycle) and between-cycle standby
// (the gap term applied by estimate_lifetime_cost).

#include <gtest/gtest.h>
#include <mp-units/systems/si.h>
#include <tinycell/model/arm.hpp>
#include <tinycell/solver/allocator.hpp>
#include <tinycell/solver/cost.hpp>

namespace {

using namespace mp_units;
namespace tc = tinycell::core;
namespace ts = tinycell::solver;

// Synthetic arm with round numbers so test arithmetic is hand-checkable.
//   list_price = 100 000 EUR
//   power_idle = 1000 W
//   power_peak = 2500 W (consistent with full_speed_power = 1000 W and
//                       active_duty 0.6 in motion_model:
//                       2500 × 0.6 + 1000 × 0.4 = 1500 + 400 = 1900 W
//                       — but tests don't care about peak; only list
//                       price and idle are looked up.)
tc::ArmSpec test_arm_spec() {
    return tc::ArmSpec{
        .id = "test_arm",
        .model_name = "TestArm",
        .family = "test",
        .controller_class = "small",
        .footprint = {},
        .reach = tc::ReachEnvelope{
            .min_radius = 0.0 * si::metre,
            .max_radius = tc::ReachRadius{1.0 * si::metre},
        },
        .payload_max = tc::Payload{10.0 * si::kilogram},
        .mass = 50.0 * si::kilogram,
        .max_speed = 2.0 * (si::metre / si::second),
        .max_acceleration = 5.0 * (si::metre / (si::second * si::second)),
        .repeatability = 0.0001 * si::metre,
        .power_peak = 2500.0 * si::watt,
        .power_idle = 1000.0 * si::watt,
        .list_price_eur = 100000.0,
        .lifetime = 14.0 * 365.25 * 86400.0 * si::second,
        .maintenance_annual_fraction = 0.04,
        .regen_capable = false,
    };
}

// One bound instance, one served task. Numbers chosen so:
//   full_speed_power = energy_per_cycle / cycle_time = 100000 J / 100 s = 1000 W
//   utilization      = achievable / target = 5 / 10 = 0.5
//   active_power     = 1000 W × 0.5 = 500 W
//   gap_power        = power_idle × (1 − util) = 1000 × 0.5 = 500 W
//   total_power      = 1000 W
ts::AllocationResult one_instance_half_utilized() {
    ts::AllocationResult r;
    ts::BoundInstance::ServedTask st{
        .task_id = "t",
        .target_ct_per_item = 10.0 * si::second,
        .achievable_ct_per_item = 5.0 * si::second,
        .cycle_time = 100.0 * si::second,
        .energy_per_cycle = 100000.0 * si::joule,
    };
    r.instances.push_back(ts::BoundInstance{
        .id = 0,
        .catalog_id = "test_arm",
        .strategy_name = "ArmStrategy",
        .achievable_ct_per_item = 5.0 * si::second,
        .served = {std::move(st)},
    });
    return r;
}

} // namespace

TEST(Cost, ComputeCostSumsCapexAndEnergyPerWorkflow) {
    auto arms = std::vector<tc::ArmSpec>{test_arm_spec()};
    auto pushers = std::vector<tc::PusherSpec>{};
    auto allocation = one_instance_half_utilized();

    auto report = ts::compute_cost(allocation, arms, pushers);

    ASSERT_EQ(report.per_instance.size(), 1U);
    EXPECT_DOUBLE_EQ(report.total_capex_eur, 100000.0);
    EXPECT_DOUBLE_EQ(
        report.total_energy_per_workflow.numerical_value_in(si::joule),
        100000.0);
    EXPECT_DOUBLE_EQ(report.per_instance[0].utilization, 0.5);
    EXPECT_DOUBLE_EQ(
        report.per_instance[0].standby_power.numerical_value_in(si::watt),
        1000.0);
}

TEST(Cost, UnknownCatalogIdIsSkipped) {
    auto arms = std::vector<tc::ArmSpec>{};  // empty — lookup misses
    auto pushers = std::vector<tc::PusherSpec>{};
    auto allocation = one_instance_half_utilized();

    auto report = ts::compute_cost(allocation, arms, pushers);

    EXPECT_TRUE(report.per_instance.empty());
    EXPECT_DOUBLE_EQ(report.total_capex_eur, 0.0);
}

TEST(Cost, LifetimeMathProducesExpectedSplit) {
    // With the synthetic instance:
    //   total_power = 1000 W
    //   op_seconds  = 14 yr × 4000 h/yr × 3600 s/h = 201 600 000 s
    //   lifetime_kWh = 1000 W × 201 600 000 s / 3.6e6 = 56 000 kWh
    //   lifetime_energy_eur = 56 000 × 0.22 EUR/kWh = 12 320 EUR
    //   capex = 100 000 EUR
    //   maintenance = (100 000 + 12 320) × 0.515 = 57 844.8 EUR
    //   total = 170 164.8 EUR
    auto arms = std::vector<tc::ArmSpec>{test_arm_spec()};
    auto pushers = std::vector<tc::PusherSpec>{};
    auto allocation = one_instance_half_utilized();
    auto report = ts::compute_cost(allocation, arms, pushers);

    auto estimate = ts::estimate_lifetime_cost(allocation, report);

    ASSERT_EQ(estimate.per_instance.size(), 1U);
    EXPECT_NEAR(estimate.total_capex_eur, 100000.0, 1e-6);
    EXPECT_NEAR(estimate.total_lifetime_energy_eur, 12320.0, 1e-3);
    EXPECT_NEAR(estimate.total_maintenance_other_eur, 57844.8, 1e-2);
    EXPECT_NEAR(estimate.total_eur, 170164.8, 1e-2);
}

TEST(Cost, NoDoubleCountingAtFullUtilization) {
    // At utilization = 1.0 the standby-gap term is zero, so total
    // power equals only the active term. This verifies the gap
    // (between-cycle) standby is not being added on top of the
    // within-cycle idle that's already inside energy_per_cycle.
    ts::AllocationResult r;
    ts::BoundInstance::ServedTask st{
        .task_id = "t_full",
        .target_ct_per_item = 5.0 * si::second,
        .achievable_ct_per_item = 5.0 * si::second,   // util = 1
        .cycle_time = 100.0 * si::second,
        .energy_per_cycle = 100000.0 * si::joule,     // full_speed = 1000 W
    };
    r.instances.push_back(ts::BoundInstance{
        .id = 0,
        .catalog_id = "test_arm",
        .strategy_name = "ArmStrategy",
        .achievable_ct_per_item = 5.0 * si::second,
        .served = {std::move(st)},
    });
    auto arms = std::vector<tc::ArmSpec>{test_arm_spec()};
    auto pushers = std::vector<tc::PusherSpec>{};

    auto report = ts::compute_cost(r, arms, pushers);
    auto estimate = ts::estimate_lifetime_cost(r, report);

    // total_power = active 1000 + gap 0 = 1000 W
    // lifetime_kWh = 1000 × 201 600 000 / 3.6e6 = 56 000 kWh
    // lifetime_energy_eur = 56 000 × 0.22 = 12 320 EUR
    // (Same as the half-utilized case — coincidence here: half-util has
    //  active 500 + gap 500 = 1000, vs full-util active 1000 + gap 0
    //  = 1000. By construction, this synthetic arm's full_speed_power
    //  equals its idle power, so total power is rate-invariant. That's
    //  what makes the no-double-count assertion direct: adding the
    //  gap term doesn't OVERSHOOT the rate-invariant value.)
    ASSERT_EQ(estimate.per_instance.size(), 1U);
    EXPECT_NEAR(estimate.per_instance[0].lifetime_energy_eur, 12320.0, 1e-3);
}

TEST(Cost, SharedInstanceCountedOnceForCapex) {
    // Two tasks on one instance, each at util 0.25 → total util 0.5.
    // Capex must be counted ONCE; energy_per_workflow sums both tasks.
    ts::AllocationResult r;
    ts::BoundInstance::ServedTask t1{
        .task_id = "t1",
        .target_ct_per_item = 20.0 * si::second,
        .achievable_ct_per_item = 5.0 * si::second,   // util = 0.25
        .cycle_time = 100.0 * si::second,
        .energy_per_cycle = 100000.0 * si::joule,
    };
    ts::BoundInstance::ServedTask t2 = t1;
    t2.task_id = "t2";

    r.instances.push_back(ts::BoundInstance{
        .id = 0,
        .catalog_id = "test_arm",
        .strategy_name = "ArmStrategy",
        .achievable_ct_per_item = 5.0 * si::second,
        .served = {t1, t2},
    });
    auto arms = std::vector<tc::ArmSpec>{test_arm_spec()};
    auto pushers = std::vector<tc::PusherSpec>{};

    auto report = ts::compute_cost(r, arms, pushers);

    ASSERT_EQ(report.per_instance.size(), 1U);
    EXPECT_DOUBLE_EQ(report.total_capex_eur, 100000.0);      // not 200 000
    EXPECT_DOUBLE_EQ(
        report.total_energy_per_workflow.numerical_value_in(si::joule),
        200000.0);                                            // both tasks
    EXPECT_DOUBLE_EQ(report.per_instance[0].utilization, 0.5);
}
