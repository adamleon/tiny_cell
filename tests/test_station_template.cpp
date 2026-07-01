// Tests for layout_palletizer_cell() — the intra-station templated layout +
// feasibility check that turns a station into a multi-pallet CELL (step-6 M4
// capstone). Each case pins one behaviour: single pallet, a feasible
// dual-pallet cell (the realistic palletizer), and the two ways the template
// reports infeasible.

#include <gtest/gtest.h>

#include <cmath>
#include <mp-units/systems/si.h>

#include <tinycell/solver/station_template.hpp>

namespace tc = tinycell::core;
namespace ts = tinycell::solver;
using mp_units::si::metre;

namespace {

double radius_m(const tc::Vec2& v) {
    const double x = v.x.numerical_value_in(metre);
    const double y = v.y.numerical_value_in(metre);
    return std::sqrt(x * x + y * y);
}

double dist_m(const tc::Vec2& a, const tc::Vec2& b) {
    const double dx = (a.x - b.x).numerical_value_in(metre);
    const double dy = (a.y - b.y).numerical_value_in(metre);
    return std::sqrt(dx * dx + dy * dy);
}

} // namespace

TEST(StationTemplate, ZeroPalletsIsFeasibleAndEmpty) {
    const auto out = ts::layout_palletizer_cell(
        0.0 * metre, 3.195 * metre, 1.2 * metre, 0.8 * metre, 0.25 * metre, 0);
    EXPECT_TRUE(out.feasible);
    EXPECT_TRUE(out.slots.empty());
}

TEST(StationTemplate, SinglePalletSitsOnTheWorkingAxisInReach) {
    const auto out = ts::layout_palletizer_cell(
        0.0 * metre, 3.195 * metre, 1.2 * metre, 0.8 * metre, 0.25 * metre, 1);
    ASSERT_TRUE(out.feasible);
    ASSERT_EQ(out.slots.size(), 1u);
    // One pallet → on the +x axis (y ≈ 0), centre within the reach band.
    EXPECT_NEAR(out.slots[0].centre.y.numerical_value_in(metre), 0.0, 1e-9);
    EXPECT_GT(out.slots[0].centre.x.numerical_value_in(metre), 0.0);
    // Whole footprint reachable: centre + zone bounding radius <= reach_max.
    const double zone_br = std::hypot(0.6 + 0.25, 0.4 + 0.25);
    EXPECT_LE(radius_m(out.slots[0].centre) + zone_br, 3.195 + 1e-9);
}

TEST(StationTemplate, DualPalletCellIsFeasibleAndNonColliding) {
    // The realistic palletizer: a KR120-PA-class arm (3.195 m reach) serving
    // TWO EUR-ish pallets. Both must sit within reach and not collide.
    const double pallet_w = 1.2, pallet_l = 0.8, margin = 0.25;
    const auto out = ts::layout_palletizer_cell(
        0.0 * metre, 3.195 * metre, pallet_w * metre, pallet_l * metre,
        margin * metre, 2);
    ASSERT_TRUE(out.feasible) << out.diagnostic;
    ASSERT_EQ(out.slots.size(), 2u);

    const double zone_br = std::hypot(0.5 * pallet_w + margin, 0.5 * pallet_l + margin);
    // Both footprints within the reach band.
    for (const auto& s : out.slots) {
        EXPECT_LE(radius_m(s.centre) + zone_br, 3.195 + 1e-9);
    }
    // Symmetric about the +x axis (one north, one south).
    EXPECT_NEAR(out.slots[0].centre.y.numerical_value_in(metre),
                -out.slots[1].centre.y.numerical_value_in(metre), 1e-9);
    EXPECT_NE(out.slots[0].centre.y.numerical_value_in(metre),
              out.slots[1].centre.y.numerical_value_in(metre));
    // Non-colliding (conservative: centres ≥ 2× the zone bounding radius).
    EXPECT_GE(dist_m(out.slots[0].centre, out.slots[1].centre), 2.0 * zone_br - 1e-9);
}

TEST(StationTemplate, PalletTooLargeForReachIsInfeasible) {
    // A small arm (0.9 m reach) cannot fit a full 1.2 × 0.8 pallet in its
    // reach band — the feasibility check must catch it, not fabricate an
    // unreachable layout.
    const auto out = ts::layout_palletizer_cell(
        0.0 * metre, 0.901 * metre, 1.2 * metre, 0.8 * metre, 0.25 * metre, 1);
    EXPECT_FALSE(out.feasible);
    EXPECT_TRUE(out.slots.empty());
    EXPECT_FALSE(out.diagnostic.empty());
}

TEST(StationTemplate, TooManyPalletsToFitIsInfeasible) {
    // Even a long-reach arm can't ring itself with many large pallets within
    // the working arc.
    const auto out = ts::layout_palletizer_cell(
        0.0 * metre, 3.195 * metre, 1.2 * metre, 0.8 * metre, 0.25 * metre, 8);
    EXPECT_FALSE(out.feasible);
    EXPECT_FALSE(out.diagnostic.empty());
}
