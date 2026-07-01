// demo_brute_force_enumerator — the strategy library on a small workflow.
//
// Demonstrates: the brute-force enumerator (roadmap.md step 2) iterating
// every registered strategy over every task in a small workflow. Loads
// both the KUKA arm catalog and the generic pusher catalog, registers
// ArmStrategy + PusherStrategy, runs `enumerate` over three Palletize
// tasks varying in box mass and pallet size, and prints per task the
// full proposal list plus the chosen winner.
//
// Success: prints three task blocks. For each task, both strategies
// appear in the proposals list with FULL or INFEASIBLE feasibility, and
// the winner row identifies which strategy + catalog entry was chosen.
// Expected qualitative outcome (energies are placeholders):
//   * task_small  — both feasible; PusherStrategy typically wins on
//                   placeholder energy. The pusher transfers a full row
//                   per stroke, so its cycle time is much lower than the
//                   per-box arm.
//   * task_heavy  — PusherStrategy INFEASIBLE: pushing two 40 kg boxes per
//                   stroke = 80 kg row payload, exceeding even the heavy
//                   pusher (60 kg). ArmStrategy wins with a Quantec PA arm.
//   * task_large_pallet — PusherStrategy INFEASIBLE: no pusher in the
//                   catalog has 2 m stroke. ArmStrategy wins with a
//                   Cybertech arm.
// If you see "no winner" on a task, every strategy returned non-FULL —
// inspect the proposals list to see which guard rejected each.

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <mp-units/systems/si.h>
#include <string>
#include <tinycell/io/catalog_loader.hpp>
#include <tinycell/model/task.hpp>
#include <tinycell/solver/arm_strategy.hpp>
#include <tinycell/solver/enumerator.hpp>
#include <tinycell/solver/pusher_strategy.hpp>
#include <vector>

namespace {

using namespace mp_units;
namespace tc = tinycell::core;
namespace ts = tinycell::solver;

const char* feasibility_str(ts::Feasibility f) {
    switch (f) {
        case ts::Feasibility::FULL:       return "FULL";
        case ts::Feasibility::PARTIAL:    return "PARTIAL";
        case ts::Feasibility::INFEASIBLE: return "INFEASIBLE";
    }
    return "?";
}

void print_task_summary(const tc::Task& task) {
    const auto& p = std::get<tc::PalletizeParams>(task.params);
    std::cout << "Task: " << task.id << "  (Palletize, "
              << p.box_count << " boxes @ "
              << p.item.physical.mass.numerical_value_in(si::kilogram) << " kg, pallet "
              << p.pallet.physical.width.numerical_value_in(si::metre) << " x "
              << p.pallet.physical.length.numerical_value_in(si::metre) << " m)\n";
}

void print_enumeration(const ts::TaskEnumeration& te) {
    print_task_summary(te.task);
    std::cout << std::left << std::setw(18) << "  strategy"
              << std::setw(14) << "feasibility"
              << std::setw(24) << "equipment"
              << std::right << std::setw(12) << "cycle (s)"
              << std::setw(14) << "energy (J)" << "\n";
    std::cout << "  " << std::string(80, '-') << "\n";
    for (std::size_t i = 0; i < te.proposals.size(); ++i) {
        const auto& r = te.proposals[i];
        const std::string marker = (te.winner_index && *te.winner_index == i) ? " *" : "  ";
        std::cout << marker << std::left << std::setw(16) << r.strategy_name
                  << std::setw(14) << feasibility_str(r.feasibility)
                  << std::setw(24)
                  << (r.equipment ? r.equipment->catalog_id : std::string{"-"})
                  << std::right << std::fixed << std::setprecision(1) << std::setw(12)
                  << r.cycle_time.numerical_value_in(si::second) << std::setw(14)
                  << r.energy_per_cycle.numerical_value_in(si::joule) << "\n";
    }
    if (te.winner_index) {
        std::cout << "  winner: " << te.proposals[*te.winner_index].strategy_name
                  << " (* row above)\n";
    } else {
        std::cout << "  winner: NONE — no strategy returned FULL\n";
    }
    std::cout << "\n";
}

std::vector<tc::Task> sample_workflow() {
    std::vector<tc::Task> tasks;
    // Helper to build a BoxSpec — boxes here use a rectangular footprint
    // (Discrete(180) symmetry) by default, so the task's catalog selection
    // is exercised without coupling to the orientation gate (which lives
    // in requires_knowledge, not applies_to / evaluate).
    auto box = [](double w, double l, double h, double m) {
        return tc::BoxSpec{
            .physical = tc::ItemPhysical{
                .width = w * si::metre,
                .length = l * si::metre,
                .height = h * si::metre,
                .mass = m * si::kilogram,
                .symmetry = tc::symmetry::discrete(180),
            },
        };
    };
    auto pallet = [](double w, double l) {
        return tc::PalletSpec{
            .physical = tc::ItemPhysical{
                .width = w * si::metre,
                .length = l * si::metre,
                .height = 0.15 * si::metre,
                .mass = 25.0 * si::kilogram,
                .symmetry = tc::symmetry::discrete(180),
            },
        };
    };

    tasks.push_back(tc::Task{
        .id = "task_small",
        .params = tc::PalletizeParams{
            .item_id = "box_400x300x200",
            .item = box(0.3, 0.4, 0.2, 5.0),
            .pallet = pallet(1.2, 0.8),
            .box_count = 24,
        },
        .target_ct_per_item = tc::CycleTimePerItem{5.0 * si::second},
    });
    tasks.push_back(tc::Task{
        .id = "task_heavy",
        .params = tc::PalletizeParams{
            .item_id = "crate_heavy",
            .item = box(0.5, 0.5, 0.3, 40.0),
            .pallet = pallet(1.2, 1.0),
            .box_count = 12,
        },
        .target_ct_per_item = tc::CycleTimePerItem{5.0 * si::second},
    });
    tasks.push_back(tc::Task{
        .id = "task_large_pallet",
        .params = tc::PalletizeParams{
            .item_id = "box_400x300x200",
            .item = box(0.3, 0.4, 0.2, 8.0),
            .pallet = pallet(2.0, 2.0),
            .box_count = 36,
        },
        .target_ct_per_item = tc::CycleTimePerItem{5.0 * si::second},
    });
    return tasks;
}

} // namespace

int main() {
    const auto repo = std::filesystem::path(TINYCELL_REPO_ROOT);
    auto arms = tinycell::io::load_arm_catalog(repo / "assets" / "arm" / "kuka" / "catalog.json");
    auto pushers =
        tinycell::io::load_pusher_catalog(repo / "assets" / "pusher" / "generic" / "catalog.json");

    ts::ArmStrategy arm_strategy(arms);
    ts::PusherStrategy pusher_strategy(pushers);

    const std::vector<const ts::Strategy*> strategies{&arm_strategy, &pusher_strategy};

    const auto workflow = sample_workflow();
    const auto enumerations = ts::enumerate(workflow, strategies);

    std::cout << "Brute-force enumerator: " << workflow.size() << " task(s) x "
              << strategies.size() << " strategies (" << arms.size() << " arms, "
              << pushers.size() << " pushers in catalog)\n\n";

    for (const auto& te : enumerations) {
        print_enumeration(te);
    }
    return 0;
}
