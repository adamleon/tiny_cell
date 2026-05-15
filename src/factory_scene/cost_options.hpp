#pragma once
#include <algorithm>
#include <cassert>
#include <vector>

#include "cost_model.hpp"
#include "robot_arm_catalog.hpp"
#include "throughput.hpp"

// Cost-option enumerators: given a task and a catalog, return every feasible
// candidate configuration with its annual cost. Built on top of cost::estimate
// (single-arm) and throughput:: (cycle-time math). Returned vectors are sorted
// by ascending annual cost — the lowest-cost feasible option is index 0.
//
// These are the primitives the (future) outer solver iterates over when
// making archetype-and-placement decisions. Each function returns all feasible
// options (not just the cheapest) so the solver can apply layout, space,
// or other secondary constraints before committing.
//
// Pure functions over POD types — no ECS, no I/O.

namespace factory::cost {

// ── Robot-arm options (standalone picker) ───────────────────────────────────
//
// For a pickup-and-place task done by a single arm at the given demand rate,
// enumerate every arm in the catalog that meets the feasibility constraints.

struct RobotArmOption {
    robot_arm_catalog::RobotArmSpec spec;
    CostBreakdown                   breakdown;
};

// `base_task` provides distance, payload, vertical_lift, gravity_factor, and
// operating_distance. The function fills in cycle_time = 60 / demand_per_min.
inline std::vector<RobotArmOption>
robot_arm_options(float demand_per_min,
                  const std::vector<robot_arm_catalog::RobotArmSpec>& catalog,
                  const TaskParams& base_task,
                  const ExternalParams& ext)
{
    assert(demand_per_min > 0.f);

    TaskParams task = base_task;
    task.cycle_time_s = 60.f / demand_per_min;

    std::vector<RobotArmOption> out;
    out.reserve(catalog.size());
    for (const auto& spec : catalog) {
        auto r = estimate(spec, task, ext);
        if (!r.feasible) continue;
        out.push_back({spec, r});
    }

    std::sort(out.begin(), out.end(),
        [](const RobotArmOption& a, const RobotArmOption& b) {
            return a.breakdown.annual_cost_eur < b.breakdown.annual_cost_eur;
        });
    return out;
}

// ── Palletizer options (station + N pickers) ────────────────────────────────
//
// Composite: enumerate every (num_pickers, arm) combination that meets the
// pallet-completion demand. For each candidate, the per-arm box throughput is
// (pallets_per_min × boxes_per_pallet) / num_pickers, which sets the cycle
// time fed into cost::estimate.
//
// The station mechanism is a separate fixed cost — claim frame, pallet
// dispenser, controllers — independent of arm choice.

struct PalletizerContext {
    // Pallet + box geometry → boxes_per_pallet via throughput::boxes_per_pallet
    throughput::PalletizerGeometry geometry;

    // Per-arm task geometry (distance, lift, etc.) — the cycle_time field is
    // ignored and overwritten by the enumerator.
    TaskParams per_arm_task;

    // Station-mechanism cost, amortised over its own lifetime (years).
    float station_mech_capex_eur;
    float station_mech_power_w;
    float station_mech_maintenance_annual_eur;
    float station_mech_lifetime_years = 14.f;

    // Upper bound on number of arms to try. 3 is typical for paper-class
    // palletizers; raise for higher-throughput cells.
    int max_pickers = 3;
};

struct PalletizerOption {
    int                              num_pickers;
    robot_arm_catalog::RobotArmSpec  arm;
    CostBreakdown                    per_arm_breakdown;

    // Station mechanism contribution (independent of arm choice):
    float                            station_mech_annualised_capex_eur;
    float                            station_mech_maintenance_annual_eur;
    float                            station_mech_annual_energy_eur;

    // Composite annual cost = (N × per_arm) + station_mech contribution:
    float                            annual_total_cost_eur;
};

inline std::vector<PalletizerOption>
palletizer_options(float pallets_per_min,
                   const std::vector<robot_arm_catalog::RobotArmSpec>& catalog,
                   const PalletizerContext& ctx,
                   const ExternalParams& ext)
{
    assert(pallets_per_min > 0.f);
    assert(ctx.max_pickers >= 1);
    assert(ctx.station_mech_lifetime_years > 0.f);

    const int boxes_per_pallet = throughput::boxes_per_pallet(ctx.geometry);
    assert(boxes_per_pallet > 0);

    const float box_per_min_total = pallets_per_min * float(boxes_per_pallet);

    // Station-mechanism annualised contributions (independent of arm).
    // Energy from idle power running over operating hours:
    const float station_mech_annual_kwh =
        ctx.station_mech_power_w * ext.operating_hours_per_year / 1000.f;
    const float station_mech_annual_energy_eur =
        station_mech_annual_kwh * ext.electricity_price_eur_kwh;
    const float station_mech_annualised_capex =
        ctx.station_mech_capex_eur / ctx.station_mech_lifetime_years;

    std::vector<PalletizerOption> out;
    out.reserve(catalog.size() * static_cast<size_t>(ctx.max_pickers));

    for (int n = 1; n <= ctx.max_pickers; ++n) {
        const float per_arm_box_per_min = box_per_min_total / float(n);
        TaskParams task                 = ctx.per_arm_task;
        task.cycle_time_s               = 60.f / per_arm_box_per_min;

        for (const auto& spec : catalog) {
            auto per_arm = estimate(spec, task, ext);
            if (!per_arm.feasible) continue;

            const float total =
                float(n) * per_arm.annual_cost_eur
              + station_mech_annualised_capex
              + ctx.station_mech_maintenance_annual_eur
              + station_mech_annual_energy_eur;

            out.push_back(PalletizerOption{
                n, spec, per_arm,
                station_mech_annualised_capex,
                ctx.station_mech_maintenance_annual_eur,
                station_mech_annual_energy_eur,
                total});
        }
    }

    std::sort(out.begin(), out.end(),
        [](const PalletizerOption& a, const PalletizerOption& b) {
            return a.annual_total_cost_eur < b.annual_total_cost_eur;
        });
    return out;
}

}  // namespace factory::cost
