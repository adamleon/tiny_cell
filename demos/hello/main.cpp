// demo_hello — smoke test for the build chain.
//
// Demonstrates: the build infrastructure (CMake + FetchContent + mp-units +
// the core/ library) is alive and links cleanly. No domain logic — just
// version constants and a sample unit-typed quantity.
//
// Success: prints `TinyCell 0.1.0 — sample arm reach: 1.5 m` and exits 0.

#include <iostream>
#include <mp-units/systems/isq.h>
#include <mp-units/systems/si.h>
#include <tinycell/version.hpp>

int main() {
    using namespace mp_units;
    auto reach = 1.5 * isq::length[si::metre];
    std::cout << "TinyCell " << tinycell::version::string
              << " — sample arm reach: " << reach.numerical_value_in(si::metre) << " m\n";
    return 0;
}
