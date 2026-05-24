#pragma once

// Pusher domain types. A pusher is a linear actuator that transfers an item
// across a short distance — typically off a feeder onto a pallet, conveyor, or
// fixture. Mirrors the ArmSpec shape but trimmed to the fields PusherStrategy
// actually consumes today; reach/orientation concepts that arms have don't
// apply, and TCO fields (mass / lifetime / maintenance / regen) are deferred
// to step 4 along with the real cost model (decisions.md).

#include <string>
#include <tinycell/geometry.hpp>
#include <tinycell/model/validated.hpp>
#include <tinycell/units.hpp>

namespace tinycell::core {

// PusherSpec — one row of the pusher catalog. Immutable description of one
// pusher MODEL (e.g. "GE-L1000-M"). Multiple placed pushers in a cell may
// share one PusherSpec.
//
// Feasibility-gating fields:
//   * payload_max — heaviest item the pusher can transfer in one stroke
//   * stroke      — maximum linear travel; gates how deep into a target
//                   surface a single push can place an item
// Cost-shaping fields:
//   * cycle_time_per_push — round-trip time for one box transfer
//   * power_peak / power_idle — used by the placeholder energy model
//   * list_price_eur — used by the cheapest-feasible selection inside
//                      PusherStrategy
struct PusherSpec {
    std::string id;
    std::string model_name;
    std::string family;
    std::string controller_class;
    Polygon footprint;
    Length stroke;
    Payload payload_max;
    Duration cycle_time_per_push;
    Power power_peak;
    Power power_idle;
    double list_price_eur;
};

} // namespace tinycell::core
