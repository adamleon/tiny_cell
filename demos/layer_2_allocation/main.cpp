// INTENT: Show the step-4 Layer-2 pipeline end-to-end on a toy
// workflow — enumerator + PARTIAL chain walking + allocator with
// cross-task sharing + cost report + 14-year lifetime estimate.
//
// Success means every section of the printout exhibits one of the
// layer's features:
//
//   1. Per-task proposals: each task gets a winning strategy (FULL
//      or PARTIAL), and PARTIAL chains emit residual TaskEnumerations
//      visible in the second part of the workflow listing.
//   2. Allocation summary: one bound instance serves BOTH t_pallet_a
//      and t_pallet_b — the sharing win, the central layer-2 value.
//      A separate instance covers t_pallet_tight's PARTIAL primary,
//      and a third instance covers its residual ("two arms in
//      parallel" emerging from the OR-tree + allocator interplay).
//   3. Cost report: capex is summed once per bound physical instance
//      (shared instance counted once), active energy sums across
//      served tasks for the workflow.
//   4. Lifetime estimate: capex + lifetime energy + maintenance/other
//      shares split per the KUKA-derived defaults (39 / 27 / 34 %
//      ballpark at full utilization, with shape changing with the
//      actual instance utilization).

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <mp-units/systems/si.h>
#include <string>
#include <vector>

#include <tinycell/io/catalog_loader.hpp>
#include <tinycell/model/task.hpp>
#include <tinycell/solver/allocator.hpp>
#include <tinycell/solver/arm_strategy.hpp>
#include <tinycell/solver/cost.hpp>
#include <tinycell/solver/enumerator.hpp>
#include <tinycell/solver/pusher_strategy.hpp>

namespace {

using namespace mp_units;
namespace tc = tinycell::core;
namespace ts = tinycell::solver;
namespace io = tinycell::io;

tc::Task make_palletize(const std::string& id,
                        double item_mass_kg,
                        double pallet_w_m, double pallet_l_m,
                        int box_count,
                        double target_s_per_item) {
    return tc::Task{
        .id = id,
        .params = tc::PalletizeParams{
            .item_id = "box",
            .item = tc::BoxSpec{
                .physical = tc::ItemPhysical{
                    .width = 0.3 * si::metre,
                    .length = 0.4 * si::metre,
                    .height = 0.2 * si::metre,
                    .mass = item_mass_kg * si::kilogram,
                    .symmetry = tc::symmetry::discrete(180),
                },
            },
            .pallet = tc::PalletSpec{
                .physical = tc::ItemPhysical{
                    .width = pallet_w_m * si::metre,
                    .length = pallet_l_m * si::metre,
                    .height = 0.15 * si::metre,
                    .mass = 25.0 * si::kilogram,
                    .symmetry = tc::symmetry::discrete(180),
                },
            },
            .box_count = box_count,
        },
        .target_ct_per_item = tc::CycleTimePerItem{target_s_per_item * si::second},
    };
}

const char* feasibility_str(ts::Feasibility f) {
    switch (f) {
    case ts::Feasibility::FULL:       return "FULL";
    case ts::Feasibility::PARTIAL:    return "PARTIAL";
    case ts::Feasibility::INFEASIBLE: return "INFEASIBLE";
    }
    return "?";
}

void print_enumeration_row(const ts::TaskEnumeration& te) {
    const std::string marker = te.task.is_residual ? "  └─" : "    ";
    std::cout << marker << " " << std::left << std::setw(24) << te.task.id;
    if (!te.winner_index) {
        std::cout << " no winner\n";
        return;
    }
    const auto& w = te.proposals[*te.winner_index];
    std::cout << " " << std::setw(11) << feasibility_str(w.feasibility)
              << " " << std::setw(16)
              << (w.equipment ? w.equipment->catalog_id : "-")
              << " " << std::right << std::fixed << std::setprecision(3)
              << std::setw(8)
              << w.achievable_ct_per_item.numerical_value_in(si::second)
              << " s/item"
              << " " << std::setw(8) << std::setprecision(1)
              << te.task.target_ct_per_item.value().numerical_value_in(si::second)
              << " s/item"
              << "\n";
}

void print_allocation(const ts::AllocationResult& alloc) {
    std::cout << "\nAllocation (" << alloc.instances.size()
              << " instance(s), " << alloc.unallocated.size()
              << " unallocated):\n";
    for (const auto& inst : alloc.instances) {
        const double committed = [&]() {
            double sum = 0.0;
            for (const auto& s : inst.served) {
                sum += 1.0 / s.target_ct_per_item.numerical_value_in(si::second);
            }
            return sum;
        }();
        const double capacity =
            1.0 / inst.achievable_ct_per_item.numerical_value_in(si::second);
        const double util = std::min(1.0, committed / capacity);
        std::cout << "  instance " << inst.id << "  "
                  << std::left << std::setw(16) << inst.catalog_id
                  << " " << std::setw(15) << inst.strategy_name
                  << "  util=" << std::fixed << std::setprecision(2) << util
                  << "  serves:";
        for (const auto& s : inst.served) {
            std::cout << " " << s.task_id;
        }
        std::cout << "\n";
    }
    for (const auto& u : alloc.unallocated) {
        std::cout << "  unallocated: " << u << "\n";
    }
}

void print_cost_report(const ts::CostReport& report) {
    std::cout << "\nCost report (one workflow execution):\n";
    std::cout << "  " << std::left << std::setw(28) << "instance"
              << std::right << std::setw(14) << "capex (EUR)"
              << std::setw(18) << "energy (kJ)\n";
    for (const auto& ic : report.per_instance) {
        std::cout << "  " << std::left << std::setw(28)
                  << (std::to_string(ic.instance_id) + ":" + ic.catalog_id)
                  << std::right << std::fixed << std::setprecision(0)
                  << std::setw(14) << ic.capex_eur
                  << std::setw(18) << std::setprecision(1)
                  << ic.energy_per_workflow.numerical_value_in(si::joule) / 1000.0
                  << "\n";
    }
    std::cout << "  " << std::left << std::setw(28) << "TOTAL"
              << std::right << std::setw(14) << std::setprecision(0)
              << report.total_capex_eur
              << std::setw(18) << std::setprecision(1)
              << report.total_energy_per_workflow.numerical_value_in(si::joule) / 1000.0
              << "\n";
}

void print_lifetime(const ts::LifetimeEstimate& est,
                    const ts::LifetimeAssumptions& a) {
    std::cout << "\nLifetime estimate (assumptions: "
              << a.lifetime_years << " yr, "
              << a.op_hours_per_year << " h/yr, "
              << a.electricity_eur_per_kwh << " EUR/kWh, "
              << "maintenance/other = " << (a.maintenance_other_ratio * 100.0)
              << "% of capex+energy):\n";
    std::cout << "  " << std::left << std::setw(28) << "instance"
              << std::right << std::setw(12) << "capex"
              << std::setw(12) << "energy"
              << std::setw(12) << "other"
              << std::setw(12) << "total\n";
    for (const auto& il : est.per_instance) {
        std::cout << "  " << std::left << std::setw(28)
                  << (std::to_string(il.instance_id) + ":" + il.catalog_id)
                  << std::right << std::fixed << std::setprecision(0)
                  << std::setw(12) << il.capex_eur
                  << std::setw(12) << il.lifetime_energy_eur
                  << std::setw(12) << il.maintenance_other_eur
                  << std::setw(12) << il.total_eur << "\n";
    }
    std::cout << "  " << std::left << std::setw(28) << "TOTAL"
              << std::right << std::setw(12) << std::setprecision(0)
              << est.total_capex_eur
              << std::setw(12) << est.total_lifetime_energy_eur
              << std::setw(12) << est.total_maintenance_other_eur
              << std::setw(12) << est.total_eur << "\n";
    // KUKA-survey reference split: shows the input assumptions are
    // calibrated against the 39/27/34 reference at full utilization.
    const double sum = est.total_eur;
    if (sum > 0.0) {
        const double capex_pct = 100.0 * est.total_capex_eur / sum;
        const double energy_pct = 100.0 * est.total_lifetime_energy_eur / sum;
        const double other_pct = 100.0 * est.total_maintenance_other_eur / sum;
        std::cout << "  " << std::left << std::setw(28) << "share (%)"
                  << std::right << std::setw(12) << std::setprecision(1) << capex_pct
                  << std::setw(12) << energy_pct
                  << std::setw(12) << other_pct << "\n";
    }
}

} // namespace

int main() {
    const auto repo = std::filesystem::path(TINYCELL_REPO_ROOT);
    const auto arms =
        io::load_arm_catalog(repo / "assets" / "arm" / "kuka" / "catalog.json");
    const auto pushers = io::load_pusher_catalog(
        repo / "assets" / "pusher" / "generic" / "catalog.json");

    ts::ArmStrategy arm_strategy(arms);
    ts::PusherStrategy pusher_strategy(pushers);
    const std::vector<const ts::Strategy*> strategies{
        &arm_strategy, &pusher_strategy};

    // Workflow:
    //   t_pallet_a, t_pallet_b — identical small palletize tasks at a
    //     loose 5 s/item target. The cheapest winning strategy is the
    //     same for both; the allocator should share one instance.
    //   t_pallet_tight — large pallet (no pusher reach), tight 2 s/item
    //     target. The cheapest reach-feasible arm cannot meet it
    //     alone, so ArmStrategy returns PARTIAL and the enumerator
    //     emits a residual sub-task; the residual's loosened target
    //     allows the same arm class to FULL on a second instance.
    const std::vector<tc::Task> workflow{
        make_palletize("t_pallet_a",     5.0, 1.2, 0.8, 24, 5.0),
        make_palletize("t_pallet_b",     5.0, 1.2, 0.8, 24, 5.0),
        make_palletize("t_pallet_tight", 8.0, 2.0, 2.0, 12, 2.0),
    };

    const auto enumeration = ts::enumerate(workflow, strategies);

    std::cout << "Workflow tasks: " << workflow.size()
              << "  (enumerated entries incl. residuals: "
              << enumeration.size() << ")\n";
    std::cout << "Strategies: ArmStrategy ("
              << arms.size() << " arms), PusherStrategy ("
              << pushers.size() << " pushers)\n\n";
    std::cout << "Per-task winners:\n";
    std::cout << "     " << std::left << std::setw(24) << "task"
              << " " << std::setw(11) << "feasibility"
              << " " << std::setw(16) << "equipment"
              << " " << std::right << std::setw(15) << "achievable"
              << " " << std::setw(15) << "target\n";
    for (const auto& te : enumeration) {
        print_enumeration_row(te);
    }

    const auto allocation = ts::allocate(enumeration);
    print_allocation(allocation);

    const auto report = ts::compute_cost(allocation, arms, pushers);
    print_cost_report(report);

    const ts::LifetimeAssumptions assumptions;
    const auto lifetime = ts::estimate_lifetime_cost(allocation, report, assumptions);
    print_lifetime(lifetime, assumptions);

    return 0;
}
