#include "tinycell/solver/station_template.hpp"

#include <algorithm>
#include <cmath>
#include <numbers>

#include <mp-units/systems/si.h>

namespace tinycell::solver {

namespace {
using namespace mp_units;
namespace tc = tinycell::core;

// Largest absolute angle (from the +x axis) a pallet slot may sit at before
// it stops being on the arm's working side. Two pallets land at ±(spread/2);
// beyond this they would wrap behind the arm.
constexpr double kMaxSlotAngleRad = 75.0 * std::numbers::pi / 180.0;

} // namespace

PalletizerCellLayout layout_palletizer_cell(tc::Length reach_min,
                                            tc::Length reach_max,
                                            tc::Length pallet_w,
                                            tc::Length pallet_l,
                                            tc::Length margin,
                                            int pallet_count) {
    PalletizerCellLayout out;
    if (pallet_count <= 0) {
        out.feasible = true;  // nothing to place
        return out;
    }

    const double rmin = reach_min.numerical_value_in(si::metre);
    const double rmax = reach_max.numerical_value_in(si::metre);
    const double m = margin.numerical_value_in(si::metre);
    const double zone_hw = 0.5 * pallet_w.numerical_value_in(si::metre) + m;
    const double zone_hl = 0.5 * pallet_l.numerical_value_in(si::metre) + m;
    const double zone_br = std::sqrt(zone_hw * zone_hw + zone_hl * zone_hl);

    // The slot CENTRE radius R must keep the whole pallet footprint inside the
    // reach band: far edge R + zone_br <= reach_max, and (only when there is a
    // real reach hole, reach_min > 0) near edge R - zone_br >= reach_min. Also
    // keep R off the arm base (>= zone_hw) so the pallet sits in front of, not
    // around, the arm. Prefer ~60% of max reach, clamped into that window.
    const double r_lo = std::max(rmin + zone_br, zone_hw);
    const double r_hi = rmax - zone_br;
    if (r_lo > r_hi) {
        out.diagnostic =
            "pallet footprint does not fit the arm's reach band (need a "
            "longer-reach arm or a smaller pallet)";
        return out;
    }
    const double R = std::clamp(0.6 * rmax, r_lo, r_hi);

    // Angular spread so adjacent slots don't collide: keep their centres at
    // least 2*zone_br apart (bounding-circle non-overlap — conservative for
    // the axis-aligned rectangles). The chord between two centres on radius R
    // separated by dtheta is 2*R*sin(dtheta/2); require it >= 2*zone_br.
    double dtheta = 0.0;
    if (pallet_count > 1) {
        const double s = zone_br / R;  // need sin(dtheta/2) >= s
        if (s >= 1.0) {
            out.diagnostic =
                "pallets are too large relative to the reach to place more "
                "than one around the arm";
            return out;
        }
        dtheta = 2.0 * std::asin(s);
        const double half_arc = 0.5 * static_cast<double>(pallet_count - 1) * dtheta;
        if (half_arc > kMaxSlotAngleRad) {
            out.diagnostic =
                "too many pallets to fit around the arm within reach";
            return out;
        }
    }

    // Place slots symmetrically about the +x axis (angle 0).
    const double start = -0.5 * static_cast<double>(pallet_count - 1) * dtheta;
    out.slots.reserve(static_cast<std::size_t>(pallet_count));
    for (int i = 0; i < pallet_count; ++i) {
        const double a = start + static_cast<double>(i) * dtheta;
        out.slots.push_back(PalletSlot{
            .centre = tc::Vec2{R * std::cos(a) * si::metre,
                               R * std::sin(a) * si::metre}});
    }
    out.feasible = true;
    return out;
}

} // namespace tinycell::solver
