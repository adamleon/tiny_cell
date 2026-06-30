#pragma once

// Belt (conveyor) domain types. A belt is a LINEAR transport: it carries items
// in a straight line from a source port to a sink port. Unlike arms and pushers
// (point-placed equipment with a fixed footprint polygon), a belt's footprint
// IS its route — a rectangle `belt_width` wide along the placed length — so the
// spec carries a LENGTH RANGE the model supports rather than a static footprint.
// The concrete placed belt (PlacedBelt, step-6 M2) is built post-placement,
// once the source/sink port world positions are resolved by the Layer-3 solver.
//
// TCO / throughput fields stay deferred (as for PusherSpec): the Layer-2 cost
// model uses uniform lifetime assumptions, and belt throughput is derived from
// speed + item spacing by the analytic model rather than stored per spec. Add
// fields field-by-field when a real consumer (cost calibration, M3) needs them.

#include <string>
#include <tinycell/units.hpp>

namespace tinycell::core {

// BeltSpec — one row of the belt catalog. Immutable description of one belt
// MODEL (e.g. a 600 mm flat belt buildable from 3 to 9 m). Multiple placed
// belts in a cell may share one BeltSpec.
//
// Geometry-gating fields:
//   * belt_width — cross-belt width; the placed belt's footprint is this wide
//   * min_length / max_length — the routed straight-line length the model can
//                   be built to. Belt routing picks a spec whose length range
//                   covers the resolved source→sink distance.
// Cost-shaping field:
//   * max_speed      — belt surface speed (feeds the throughput model later)
//   * list_price_eur — used by cheapest-feasible selection during routing
struct BeltSpec {
    std::string id;
    std::string model_name;
    std::string family;
    Length belt_width;
    Length min_length;
    Length max_length;
    Speed max_speed;
    double list_price_eur;
};

} // namespace tinycell::core
