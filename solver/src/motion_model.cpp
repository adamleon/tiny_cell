#include "motion_model.hpp"

#include <mp-units/systems/si.h>

namespace tinycell::solver {

namespace {

using namespace mp_units;
namespace tc = tinycell::core;

// Coarse constants sized for sensible orderings on toy problems, not
// realistic absolutes. Real calibrated numbers arrive when calibration
// data lands (see decisions.md "Model-vs-reality safety margin
// deferred"); the motion-model seam does not change.
constexpr double arm_pick_place_overhead_seconds = 1.0;
constexpr double arm_active_duty_fraction = 0.6;
constexpr double pusher_active_duty_fraction = 0.4;

// Energy for one motion cycle, split across active-power and idle-power
// durations. `active_seconds = cycle_time × active_duty_fraction`; the
// rest is idle. Shared helper so the arm and pusher bodies use the
// same convention.
tc::Energy active_idle_energy(tc::Duration cycle_time,
                              double active_duty_fraction,
                              tc::Power active_power,
                              tc::Power idle_power) {
    const auto seconds = cycle_time.numerical_value_in(si::second);
    const auto active_seconds = seconds * active_duty_fraction;
    const auto idle_seconds = seconds - active_seconds;
    const auto joules =
        active_power.numerical_value_in(si::watt) * active_seconds +
        idle_power.numerical_value_in(si::watt) * idle_seconds;
    return joules * si::joule;
}

} // namespace

MotionCycle arm_motion_cycle(tc::Length one_way_distance,
                             tc::Mass /*payload*/,
                             const tc::ArmSpec& arm) {
    const auto distance_m = one_way_distance.numerical_value_in(si::metre);
    const auto max_speed_m_s =
        arm.max_speed.numerical_value_in(si::metre / si::second);
    // 2× one-way distance = round-trip pick→drop→return-to-pick.
    const auto travel_seconds = 2.0 * distance_m / max_speed_m_s;
    const auto cycle_time =
        (travel_seconds + arm_pick_place_overhead_seconds) * si::second;
    const auto energy = active_idle_energy(
        cycle_time, arm_active_duty_fraction, arm.power_peak, arm.power_idle);
    return MotionCycle{.cycle_time = cycle_time, .energy_per_cycle = energy};
}

MotionCycle pusher_motion_cycle(tc::Mass /*row_payload*/,
                                const tc::PusherSpec& pusher) {
    const auto cycle_time = pusher.cycle_time_per_push;
    const auto energy = active_idle_energy(
        cycle_time, pusher_active_duty_fraction,
        pusher.power_peak, pusher.power_idle);
    return MotionCycle{.cycle_time = cycle_time, .energy_per_cycle = energy};
}

} // namespace tinycell::solver
