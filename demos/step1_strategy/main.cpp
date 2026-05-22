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
            .item = {.width = 0.3 * si::metre,
                     .length = 0.4 * si::metre,
                     .height = 0.2 * si::metre,
                     .mass = 5.0 * si::kilogram},
            .pallet = {.width = 1.2 * si::metre, .length = 0.8 * si::metre},
            .box_count = 24,
        },
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
