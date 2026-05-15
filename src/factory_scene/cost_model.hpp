#pragma once
#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>

#include "robot_arm_catalog.hpp"

// Robot-arm annual lifecycle cost model.
//
// Given a RobotArmSpec (commercial + mechanical data), a Task (move payload
// from A to B in a cycle time), and ExternalParams (electricity price,
// operating hours), estimate the annual cost in EUR. Used by the optimisation
// layer to rank robot-arm candidates for a given task; absolute accuracy is
// secondary to relative ranking between candidates.
//
// Energy decomposition per cycle:
//   E_cycle = E_kinetic + E_gravity_lift + E_gravity_hold + E_friction + E_standby
//
// Feasibility is checked first; infeasible candidates return
// {feasible = false, annual_cost_eur = +infinity, infeasible_reason = ...}.
//
// Units: SI throughout (m, kg, s, W, J, EUR).
//
// Pure functions over POD types — no ECS, no I/O.

namespace factory::cost {

// ── Model constants (not user-tunable per spec) ──────────────────────────────

inline constexpr float kEffectiveMassFraction   = 0.3f;   // α
inline constexpr float kReachPenaltyCoeff       = 1.8f;   // β
inline constexpr float kFrictionPerKgM          = 3.0f;   // k_friction, J/(m·kg)
inline constexpr float kDrivetrainEfficiency    = 0.8f;   // η_drive
inline constexpr float kRegenEfficiencyOff      = 0.05f;  // η_regen when regen_capable = false
inline constexpr float kRegenEfficiencyOn       = 0.30f;  // η_regen when regen_capable = true
inline constexpr float kGravity                 = 9.81f;
inline constexpr float kPeakSpeedFactor         = 1.5f;   // v_peak = factor × d / t
inline constexpr float kAccelEstimateFactor     = 4.5f;   // required_accel ≈ factor × d / t²

// ── Inputs ───────────────────────────────────────────────────────────────────

struct ExternalParams {
    float electricity_price_eur_kwh;
    float operating_hours_per_year;
};

struct TaskParams {
    float distance_m;
    float payload_mass_kg;
    float cycle_time_s;
    float vertical_lift_m  = 0.0f;     // signed (negative = downward)
    float gravity_factor   = 0.5f;     // 0.5 for A↔B cyclic return; 1.0 for one-way
    float operating_distance_m = 0.0f; // 0 → defaults to (reach_max + reach_min) / 2
};

// ── Output ───────────────────────────────────────────────────────────────────

struct CostBreakdown {
    bool        feasible;
    float       annual_cost_eur;             // primary metric for ranking

    // Per-cycle / annual breakdown (zeroed when infeasible):
    float       energy_per_cycle_j;
    float       annual_kwh;
    float       annual_energy_eur;
    float       annual_maintenance_eur;
    float       annualised_capex_eur;        // price_purchase_eur / lifetime_years

    // Kinematics used (zeroed when infeasible):
    float       v_peak_m_s;
    float       required_accel_m_s2;
    float       config_penalty;              // 1 + β · (reach_util)²

    // Diagnostic — non-null only when infeasible:
    const char* infeasible_reason = nullptr;
};

// ── Implementation ───────────────────────────────────────────────────────────

namespace detail {

inline CostBreakdown infeasible(const char* reason) {
    CostBreakdown b{};
    b.feasible            = false;
    b.annual_cost_eur     = std::numeric_limits<float>::infinity();
    b.infeasible_reason   = reason;
    return b;
}

inline float resolve_operating_distance(const TaskParams& t,
                                        const robot_arm_catalog::RobotArmSpec& s)
{
    if (t.operating_distance_m > 0.0f) return t.operating_distance_m;
    return 0.5f * (s.reach_max_m + s.reach_min_m);
}

inline float regen_efficiency(const robot_arm_catalog::RobotArmSpec& s) {
    return s.regen_capable ? kRegenEfficiencyOn : kRegenEfficiencyOff;
}

}  // namespace detail

inline CostBreakdown estimate(const robot_arm_catalog::RobotArmSpec& spec,
                              const TaskParams&                     task,
                              const ExternalParams&                 ext)
{
    using namespace detail;

    assert(task.cycle_time_s   > 0.0f);
    assert(task.distance_m    >= 0.0f);
    assert(task.payload_mass_kg >= 0.0f);
    assert(ext.electricity_price_eur_kwh   >= 0.0f);
    assert(ext.operating_hours_per_year     > 0.0f);
    assert(spec.lifetime_years              > 0.0f);

    const float operating_distance = resolve_operating_distance(task, spec);

    // ── Kinematics ──────────────────────────────────────────────────────────
    const float v_peak         = kPeakSpeedFactor    * task.distance_m / task.cycle_time_s;
    const float required_accel = kAccelEstimateFactor * task.distance_m
                                                      / (task.cycle_time_s * task.cycle_time_s);

    // ── Feasibility filter ──────────────────────────────────────────────────
    if (task.payload_mass_kg > spec.payload_max_kg)
        return infeasible("payload exceeds robot.payload_max");
    if (operating_distance > spec.reach_max_m)
        return infeasible("operating_distance exceeds robot.reach_max");
    if (spec.reach_min_m > 0.0f && operating_distance < spec.reach_min_m)
        return infeasible("operating_distance below robot.reach_min");
    if (v_peak > spec.speed_max_m_s)
        return infeasible("required v_peak exceeds robot.speed_max");
    if (spec.acceleration_max_m_s2 > 0.0f &&
        required_accel > spec.acceleration_max_m_s2)
        return infeasible("required acceleration exceeds robot.acceleration_max");

    // ── Energy per cycle ───────────────────────────────────────────────────
    const float m_eff         = task.payload_mass_kg
                              + kEffectiveMassFraction * spec.mass_robot_kg;
    const float reach_util    = (spec.reach_max_m > 0.0f)
                              ? operating_distance / spec.reach_max_m : 0.0f;
    const float config_penalty = 1.0f + kReachPenaltyCoeff * reach_util * reach_util;

    const float eta_regen = regen_efficiency(spec);

    const float E_kinetic =
        0.5f * m_eff * v_peak * v_peak
            * (1.0f - eta_regen) / kDrivetrainEfficiency
            * config_penalty;

    const float E_gravity_lift =
        std::max(0.0f, task.payload_mass_kg * kGravity * task.vertical_lift_m)
            * task.gravity_factor;

    const float E_gravity_hold =
        task.payload_mass_kg * kGravity * operating_distance
            * task.cycle_time_s * 0.5f
            / kDrivetrainEfficiency;

    const float E_friction =
        kFrictionPerKgM * m_eff * task.distance_m
            / kDrivetrainEfficiency;

    const float E_standby =
        spec.power_idle_w * task.cycle_time_s;

    const float E_cycle = E_kinetic + E_gravity_lift + E_gravity_hold
                        + E_friction + E_standby;

    // ── Annual figures ──────────────────────────────────────────────────────
    const float cycles_per_year =
        ext.operating_hours_per_year * 3600.0f / task.cycle_time_s;

    const float annual_kwh        = E_cycle * cycles_per_year / 3.6e6f;
    const float annual_energy_eur = annual_kwh * ext.electricity_price_eur_kwh;
    const float annualised_capex  = spec.price_purchase_eur / spec.lifetime_years;
    const float annual_maint      = spec.maintenance_cost_annual_eur;
    const float annual_total      = annualised_capex + annual_maint + annual_energy_eur;

    CostBreakdown b{};
    b.feasible                = true;
    b.annual_cost_eur         = annual_total;
    b.energy_per_cycle_j      = E_cycle;
    b.annual_kwh              = annual_kwh;
    b.annual_energy_eur       = annual_energy_eur;
    b.annual_maintenance_eur  = annual_maint;
    b.annualised_capex_eur    = annualised_capex;
    b.v_peak_m_s              = v_peak;
    b.required_accel_m_s2     = required_accel;
    b.config_penalty          = config_penalty;
    b.infeasible_reason       = nullptr;
    return b;
}

}  // namespace factory::cost
