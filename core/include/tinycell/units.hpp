#pragma once

#include <mp-units/systems/isq.h>
#include <mp-units/systems/si.h>

namespace tinycell::core {

using namespace mp_units;

using Length = quantity<isq::length[si::metre]>;
using Mass = quantity<isq::mass[si::kilogram]>;
using Speed = quantity<isq::speed[si::metre / si::second]>;
// mp-units treats isq::acceleration as a vector quantity, so we use the kind-less
// unit form here; scalar magnitude is what the catalog stores.
using Acceleration = quantity<si::metre / (si::second * si::second)>;
using Power = quantity<isq::power[si::watt]>;
using Duration = quantity<isq::time[si::second]>;

using Price = double;

} // namespace tinycell::core
