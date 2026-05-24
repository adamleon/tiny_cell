// demo_arm_strategy — ArmStrategy evaluating a Palletize task.
//
// Demonstrates: the full chain io/ → core/ → solver/. Loads the KUKA arm
// catalog, constructs an ArmStrategy bound to it, builds a sample Palletize
// task (24 boxes of 5 kg each onto a 1.2 × 0.8 m pallet), and asks the
// strategy to evaluate. Prints the candidate equipment binding plus
// placeholder cycle-time and energy figures.
//
// Success: prints `feasibility: FULL`, picks `kuka_kr6_r900_2` (the
// cheapest catalog arm meeting both payload and the half-pallet-diagonal
// reach proxy), and labels the cycle-time / energy as PLACEHOLDER for
// step 4. If you see `feasibility: INFEASIBLE`, either the catalog can't
// satisfy the task (check the printed pallet/box numbers) or the reach
// proxy was tightened — re-read `select_arm`'s comments in
// solver/src/arm_strategy.cpp.

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <mp-units/systems/si.h>
#include <tinycell/io/catalog_loader.hpp>
#include <tinycell/model/task.hpp>
#include <tinycell/solver/arm_strategy.hpp>

int main() {
    using namespace mp_units;
    namespace tc = tinycell::core;
    namespace ts = tinycell::solver;

    auto arms = tinycell::io::load_arm_catalog(
        std::filesystem::path(TINYCELL_REPO_ROOT) / "assets" / "arm" / "kuka" / "catalog.json");

    ts::ArmStrategy strategy(arms);

    tc::Task palletize{
        .id = "demo_palletize",
        .params = tc::PalletizeParams{
            .item_id = "box_400x300x200",
            .item = tc::BoxSpec{
                .physical = tc::ItemPhysical{
                    .width = 0.3 * si::metre,
                    .length = 0.4 * si::metre,
                    .height = 0.2 * si::metre,
                    .mass = 5.0 * si::kilogram,
                    .symmetry = tc::symmetry::discrete(180),
                },
            },
            .pallet = tc::PalletSpec{
                .physical = tc::ItemPhysical{
                    .width = 1.2 * si::metre,
                    .length = 0.8 * si::metre,
                    .height = 0.15 * si::metre,
                    .mass = 25.0 * si::kilogram,
                    .symmetry = tc::symmetry::discrete(180),
                },
            },
            .box_count = 24,
        },
        .target_ct_per_item = 5.0 * si::second,
    };

    std::cout << "Task: " << palletize.id << " (Palletize, 24 boxes @ 5 kg each)\n\n";
    std::cout << "Strategy: " << strategy.name() << "\n";
    std::cout << "Applies: " << std::boolalpha << strategy.applies_to(palletize) << "\n\n";

    auto r = strategy.evaluate(palletize);
    std::cout << "Result:\n";
    std::cout << "  feasibility:      "
              << (r.feasibility == ts::Feasibility::FULL ? "FULL"
                  : r.feasibility == ts::Feasibility::PARTIAL ? "PARTIAL"
                  : "INFEASIBLE")
              << "\n";
    if (r.equipment.has_value()) {
        std::cout << "  candidate arm:    " << r.equipment->catalog_id << "\n";
    }
    std::cout << std::fixed << std::setprecision(2);
    std::cout << "  cycle_time:       " << r.cycle_time.numerical_value_in(si::second)
              << " s (placeholder, real model in step 4)\n";
    std::cout << "  energy_per_cycle: " << r.energy_per_cycle.numerical_value_in(si::joule)
              << " J (placeholder, real model in step 4)\n";
    return 0;
}
