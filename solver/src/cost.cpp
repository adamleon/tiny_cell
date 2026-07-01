#include "tinycell/solver/cost.hpp"

#include <algorithm>
#include <mp-units/systems/si.h>
#include <optional>

namespace tinycell::solver {

namespace {

using namespace mp_units;
namespace tc = tinycell::core;

// Capex + standby_power lookup by (strategy_name, catalog_id). Strategy
// name disambiguates which catalog to search.
struct CatalogLookup {
    double capex_eur;
    tc::Power standby_power;
};

std::optional<CatalogLookup> lookup(
        const std::string& strategy_name,
        const std::string& catalog_id,
        std::span<const tc::ArmSpec> arm_catalog,
        std::span<const tc::PusherSpec> pusher_catalog) {
    if (strategy_name == "ArmStrategy") {
        for (const auto& a : arm_catalog) {
            if (a.id == catalog_id) {
                return CatalogLookup{
                    .capex_eur = a.list_price_eur,
                    .standby_power = a.power_idle,
                };
            }
        }
    } else if (strategy_name == "PusherStrategy") {
        for (const auto& p : pusher_catalog) {
            if (p.id == catalog_id) {
                return CatalogLookup{
                    .capex_eur = p.list_price_eur,
                    .standby_power = p.power_idle,
                };
            }
        }
    }
    return std::nullopt;
}

// Utilization on one instance = Σ_T (achievable_T / target_T) across
// served tasks. Capped at 1 — defensive against any over-pack the
// allocator's first-fit tolerance might admit.
double instance_utilization(const BoundInstance& inst) {
    double u = 0.0;
    for (const auto& s : inst.served) {
        const double a = s.achievable_ct_per_item.numerical_value_in(si::second);
        const double t = s.target_ct_per_item.numerical_value_in(si::second);
        u += a / t;
    }
    return std::min(u, 1.0);
}

// Average power one instance draws across all its served tasks plus
// between-cycle standby. Identity:
//   full_speed_power_for_T = energy_per_cycle_T / cycle_time_T
//   active_power_T_on_I    = full_speed_power_T × (achievable_T / target_T)
//   active_power_on_I      = Σ_T active_power_T_on_I
//   standby_gap_on_I       = power_idle × max(0, 1 − Σ_T util_T)
//   total_power_on_I       = active_power_on_I + standby_gap_on_I
//
// energy_per_cycle ALREADY includes within-cycle idle (motion model
// integrates peak × active + idle × idle); the standby_gap_on_I term
// applies to between-cycle gaps only. See decisions.md "Energy model:
// three distinct power concepts; cost calc sums two without double-
// counting."
double total_power_w(const BoundInstance& inst, tc::Power standby_power) {
    double active_w = 0.0;
    double util_sum = 0.0;
    for (const auto& s : inst.served) {
        const double e_per_cycle_j = s.energy_per_cycle.numerical_value_in(si::joule);
        const double cycle_s = s.cycle_time.numerical_value_in(si::second);
        const double a = s.achievable_ct_per_item.numerical_value_in(si::second);
        const double t = s.target_ct_per_item.numerical_value_in(si::second);
        const double full_speed_w = e_per_cycle_j / cycle_s;
        const double util = a / t;
        active_w += full_speed_w * util;
        util_sum += util;
    }
    const double gap_fraction = std::max(0.0, 1.0 - util_sum);
    const double standby_gap_w =
        standby_power.numerical_value_in(si::watt) * gap_fraction;
    return active_w + standby_gap_w;
}

} // namespace

CostReport compute_cost(const AllocationResult& allocation,
                        std::span<const tc::ArmSpec> arm_catalog,
                        std::span<const tc::PusherSpec> pusher_catalog) {
    CostReport report;
    report.total_capex_eur = 0.0;
    double total_energy_j = 0.0;

    for (const auto& inst : allocation.instances) {
        const auto info = lookup(
            inst.strategy_name, inst.catalog_id, arm_catalog, pusher_catalog);
        if (!info) continue;

        double energy_j = 0.0;
        for (const auto& s : inst.served) {
            energy_j += s.energy_per_cycle.numerical_value_in(si::joule);
        }

        report.per_instance.push_back(InstanceCost{
            .instance_id = inst.id,
            .catalog_id = inst.catalog_id,
            .strategy_name = inst.strategy_name,
            .capex_eur = info->capex_eur,
            .standby_power = info->standby_power,
            .utilization = instance_utilization(inst),
            .energy_per_workflow = energy_j * si::joule,
        });

        report.total_capex_eur += info->capex_eur;
        total_energy_j += energy_j;
    }

    report.total_energy_per_workflow = total_energy_j * si::joule;
    return report;
}

LifetimeEstimate estimate_lifetime_cost(
        const AllocationResult& allocation,
        const CostReport& report,
        const LifetimeAssumptions& assumptions) {
    LifetimeEstimate out;
    out.total_capex_eur = 0.0;
    out.total_lifetime_energy_eur = 0.0;
    out.total_maintenance_other_eur = 0.0;
    out.total_eur = 0.0;

    const double op_seconds =
        assumptions.lifetime_years * assumptions.op_hours_per_year * 3600.0;

    // Walk instances + per-instance cost in lockstep. CostReport only
    // contains instances whose catalog lookup succeeded, so iterate the
    // report and re-find the matching BoundInstance by id.
    for (const auto& ic : report.per_instance) {
        const BoundInstance* inst = nullptr;
        for (const auto& bi : allocation.instances) {
            if (bi.id == ic.instance_id) { inst = &bi; break; }
        }
        if (inst == nullptr) continue;

        const double power_w = total_power_w(*inst, ic.standby_power);
        const double lifetime_kwh = power_w * op_seconds / 3.6e6;
        const double lifetime_energy_eur =
            lifetime_kwh * assumptions.electricity_eur_per_kwh;
        const double maintenance_eur =
            (ic.capex_eur + lifetime_energy_eur) * assumptions.maintenance_other_ratio;
        const double total_eur =
            ic.capex_eur + lifetime_energy_eur + maintenance_eur;

        out.per_instance.push_back(InstanceLifetime{
            .instance_id = ic.instance_id,
            .catalog_id = ic.catalog_id,
            .strategy_name = ic.strategy_name,
            .capex_eur = ic.capex_eur,
            .lifetime_energy_eur = lifetime_energy_eur,
            .maintenance_other_eur = maintenance_eur,
            .total_eur = total_eur,
        });

        out.total_capex_eur += ic.capex_eur;
        out.total_lifetime_energy_eur += lifetime_energy_eur;
        out.total_maintenance_other_eur += maintenance_eur;
        out.total_eur += total_eur;
    }
    return out;
}

} // namespace tinycell::solver
