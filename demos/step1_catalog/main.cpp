// demo_step1_catalog — milestone demo for step 1a (catalog loading).
//
// Demonstrates: the io/ catalog loader reads `assets/arm/kuka/catalog.json`,
// parses it through `load_arm_catalog`, validates each entry at the parse
// boundary (throwing ParseError on any structural or range violation), and
// returns a strongly-typed `std::vector<ArmSpec>` that core/solver code can
// consume without further checking.
//
// Success: prints a table of 11 KUKA arms with id, family, reach, payload,
// and price. The list spans Agilus (sub-1.1 m reach), Cybertech (1.6-2.1 m),
// and Quantec PA (3.2 m, palletizer arms). Exits 0; any non-zero exit means
// the loader rejected an entry — the message names which field and where.

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <mp-units/systems/si.h>
#include <tinycell/io/catalog_loader.hpp>
#include <tinycell/io/parse_error.hpp>

int main() {
    using namespace mp_units;
    namespace io = tinycell::io;

    std::filesystem::path catalog_path =
        std::filesystem::path(TINYCELL_REPO_ROOT) / "assets" / "arm" / "kuka" / "catalog.json";

    try {
        auto entries = io::load_arm_catalog(catalog_path);
        std::cout << "Loaded " << entries.size() << " arm entries from "
                  << catalog_path.filename() << "\n\n";
        std::cout << std::left << std::setw(22) << "id" << std::setw(12) << "family"
                  << std::right << std::setw(10) << "reach (m)" << std::setw(12) << "payload (kg)"
                  << std::setw(14) << "price (EUR)" << "\n";
        std::cout << std::string(70, '-') << "\n";
        for (const auto& arm : entries) {
            std::cout << std::left << std::setw(22) << arm.id << std::setw(12) << arm.family
                      << std::right << std::fixed << std::setprecision(3) << std::setw(10)
                      << arm.reach.max_radius.numerical_value_in(si::metre) << std::setprecision(1)
                      << std::setw(12) << arm.payload_max.numerical_value_in(si::kilogram)
                      << std::setw(14) << arm.list_price_eur << "\n";
        }
        return 0;
    } catch (const io::ParseError& e) {
        std::cerr << "ParseError: " << e.what() << "\n";
        return 1;
    }
}
