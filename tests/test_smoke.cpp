#include <gtest/gtest.h>
#include <mp-units/systems/isq.h>
#include <mp-units/systems/si.h>
#include <tinycell/version.hpp>

TEST(Smoke, VersionString) {
    EXPECT_EQ(tinycell::version::string, "0.1.0");
}

TEST(Smoke, MpUnitsQuantityConstructs) {
    using namespace mp_units;
    auto reach = 1.5 * isq::length[si::metre];
    EXPECT_GT(reach.numerical_value_in(si::metre), 0.0);
}
