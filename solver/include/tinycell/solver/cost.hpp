#pragma once

// Layer-2 cost accounting (decisions.md "Compute cost against bound
// instances after allocation, not summed over the tree"). Two layers:
//
//   compute_cost(allocation, arm_catalog, pusher_catalog)
//     → CostReport
//   Per-workflow-execution capex (sum of catalog list_price_eur across
//   bound instances) and active energy (sum of energy_per_cycle across
//   served tasks). Always counts a shared instance once.
//
//   estimate_lifetime_cost(report, assumptions)
//     → LifetimeEstimate
//   Applies operating hours + lifetime + electricity price + a
//   maintenance/other ratio to produce a 14-year total cost estimate.
//
// Energy double-counting prevention is critical here — see decisions.md
// "Energy model: three distinct power concepts; cost calc sums two
// without double-counting." Briefly:
//   - energy_per_cycle ALREADY includes within-cycle idle (the motion
//     model integrates peak × active + idle × idle across one cycle).
//   - power_idle as a SEPARATE term applies only to between-cycle
//     gaps when an instance is on but not actively cycling.
// compute_cost sums energy_per_cycle (per workflow); estimate_lifetime_cost
// adds the between-cycle gap term (power_idle × gap_seconds) on top.
// They are NOT the same idle period.

#include <cstddef>
#include <span>
#include <string>
#include <tinycell/model/arm.hpp>
#include <tinycell/model/pusher.hpp>
#include <tinycell/solver/allocator.hpp>
#include <tinycell/units.hpp>
#include <vector>

namespace tinycell::solver {

// Per-instance capex + active energy for one workflow execution.
// `utilization` is Σ_T (achievable_T / target_T) summed over served
// tasks — used by the lifetime estimator to size the gap (standby)
// term. `standby_power` is the catalog's `power_idle`, the always-on
// baseline; carried here so the lifetime estimator doesn't need to
// look up catalogs again.
struct InstanceCost {
    std::size_t instance_id;
    std::string catalog_id;
    std::string strategy_name;
    double capex_eur;
    core::Power standby_power;
    double utilization;
    core::Energy energy_per_workflow;
};

struct CostReport {
    std::vector<InstanceCost> per_instance;
    double total_capex_eur;
    core::Energy total_energy_per_workflow;
};

// Look up each bound instance's catalog entry by (strategy_name,
// catalog_id) and assemble the per-workflow report. Instances whose
// catalog entries are not found are skipped silently — the caller is
// responsible for passing the same catalogs the strategies hold.
CostReport compute_cost(const AllocationResult& allocation,
                        std::span<const core::ArmSpec> arm_catalog,
                        std::span<const core::PusherSpec> pusher_catalog);

// Defaults match a KUKA 14-year lifetime survey (39 % capex / 27 %
// energy / 34 % maintenance+other), calibrated against German
// industrial electricity prices and a two-shift schedule. See
// decisions.md "Lifetime cost defaults from a KUKA 14-year survey".
struct LifetimeAssumptions {
    double lifetime_years = 14.0;
    double op_hours_per_year = 4000.0;        // 2 shifts × 250 days × 8 h
    double electricity_eur_per_kwh = 0.22;    // German industrial mid-range
    double maintenance_other_ratio = 0.515;   // 34 / (39 + 27)
};

struct InstanceLifetime {
    std::size_t instance_id;
    std::string catalog_id;
    std::string strategy_name;
    double capex_eur;
    double lifetime_energy_eur;       // active + between-cycle standby
    double maintenance_other_eur;     // (capex + lifetime_energy_eur) × ratio
    double total_eur;
};

struct LifetimeEstimate {
    std::vector<InstanceLifetime> per_instance;
    double total_capex_eur;
    double total_lifetime_energy_eur;
    double total_maintenance_other_eur;
    double total_eur;
};

// Apply lifetime assumptions to an Allocation + CostReport pair.
// Walks the allocation's per-task cycle data directly to compute
// active power; uses the report's capex and standby power so catalogs
// don't need re-passing here.
//
//   active_power_per_task = (energy_per_cycle / cycle_time) × (achievable / target)
//   active_power_per_instance = Σ_T active_power_per_task
//   standby_power_gap = standby_power × max(0, 1 - utilization)
//   lifetime_energy_J = (active + standby_gap) × op_seconds
LifetimeEstimate estimate_lifetime_cost(
    const AllocationResult& allocation,
    const CostReport& report,
    const LifetimeAssumptions& assumptions = LifetimeAssumptions{});

} // namespace tinycell::solver
