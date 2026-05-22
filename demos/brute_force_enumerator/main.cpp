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
//                   placeholder energy (lower duty fraction × cheaper
//                   short-stroke pusher).
//   * task_heavy  — PusherStrategy is feasible only at the heavy-duty end
//                   of the pusher catalog; ArmStrategy may pick a larger
//                   Cybertech / Quantec arm.
//   * task_large_pallet — PusherStrategy goes INFEASIBLE (no pusher's
//                   stroke covers a 2 m pallet); ArmStrategy wins by
//                   default with a Cybertech arm.
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
              << p.item.mass.numerical_value_in(si::kilogram) << " kg, pallet "
              << p.pallet.width.numerical_value_in(si::metre) << " x "
              << p.pallet.length.numerical_value_in(si::metre) << " m)\n";
}

void print_enumeration(const ts::TaskEnumeration& te) {
    print_task_summary(*te.task);
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
    tasks.push_back(tc::Task{
        .id = "task_small",
        .params = tc::PalletizeParams{
            .item_id = "box_400x300x200",
            .item = tc::BoxSpec{.width = 0.3 * si::metre,
                                .length = 0.4 * si::metre,
                                .height = 0.2 * si::metre,
                                .mass = 5.0 * si::kilogram},
            .pallet = tc::PalletSpec{.width = 1.2 * si::metre, .length = 0.8 * si::metre},
            .box_count = 24,
        },
    });
    tasks.push_back(tc::Task{
        .id = "task_heavy",
        .params = tc::PalletizeParams{
            .item_id = "crate_heavy",
            .item = tc::BoxSpec{.width = 0.5 * si::metre,
                                .length = 0.5 * si::metre,
                                .height = 0.3 * si::metre,
                                .mass = 40.0 * si::kilogram},
            .pallet = tc::PalletSpec{.width = 1.2 * si::metre, .length = 1.0 * si::metre},
            .box_count = 12,
        },
    });
    tasks.push_back(tc::Task{
        .id = "task_large_pallet",
        .params = tc::PalletizeParams{
            .item_id = "box_400x300x200",
            .item = tc::BoxSpec{.width = 0.3 * si::metre,
                                .length = 0.4 * si::metre,
                                .height = 0.2 * si::metre,
                                .mass = 8.0 * si::kilogram},
            .pallet = tc::PalletSpec{.width = 2.0 * si::metre, .length = 2.0 * si::metre},
            .box_count = 36,
        },
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
