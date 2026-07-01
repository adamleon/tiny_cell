#pragma once

// Type aliases for the SI quantities used throughout core/. Each alias is a
// strongly-typed quantity (e.g. `Length{1.5 * si::metre}`) — assigning a Mass
// to a Length is a compile error. Range invariants (positivity, etc.) are NOT
// enforced here; those live in validated value-type wrappers — see
// `model/validated.hpp` for the wrappers currently in use (Payload,
// ReachRadius, CycleTimePerItem). Additional wrappers are added field-by-field
// as new solver code consumes each invariant.
//
// Stripping a quantity to a plain double is allowed ONLY at boundaries:
// io/ (parsing), render/ adapters (foreign libraries). Anywhere in core/ or
// solver/, keep the unit-typed form (CLAUDE.md §1).

#include <mp-units/systems/isq.h>
#include <mp-units/systems/si.h>

namespace tinycell::core {

using namespace mp_units;

using Length = quantity<isq::length[si::metre]>;
using Mass = quantity<isq::mass[si::kilogram]>;
using Speed = quantity<isq::speed[si::metre / si::second]>;

// mp-units treats isq::acceleration as a vector quantity, so we use the
// kind-less unit form here; the catalog stores scalar magnitude only.
using Acceleration = quantity<si::metre / (si::second * si::second)>;

using Power = quantity<isq::power[si::watt]>;
using Duration = quantity<isq::time[si::second]>;

// Internal energy unit is joules (data-model.md §6); convert to kWh only at
// the UI / cost-display boundary so unit-mixing bugs are impossible.
using Energy = quantity<isq::energy[si::joule]>;

// Angle: stored in radians. Catalog/UI conventions (degrees) are converted
// at the io/ and UI boundaries, never in core/ or solver/ (units.hpp top
// note). Used by port direction constraints + frame composition math
// (geometry.hpp for Pose2D / Transform2D / express, architecture.md §4).
using Angle = quantity<isq::angular_measure[si::radian]>;

} // namespace tinycell::core
