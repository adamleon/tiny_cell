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
