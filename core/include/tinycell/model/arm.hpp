#pragma once

// Arm domain types. Today: just ArmSpec — the static description of an arm
// model from the catalog. Later, when the layout solver starts producing
// placed equipment, this file will also hold ArmInstance (a specific arm at
// a specific pose, referring back to a spec by id). Runtime configuration
// (joint angles, current TCP) is much later and likely an ECS component.

#include <string>
#include <tinycell/geometry.hpp>
#include <tinycell/units.hpp>

namespace tinycell::core {

// Reach envelope as the catalog declares it. Today only the "radius" form
// (a circular workspace, in the floor plane) is parsed; richer polygonal
// envelopes can be added when needed without breaking callers that only
// read max_radius.
struct ReachEnvelope {
    Length min_radius;
    Length max_radius;
};

// ArmSpec — one row of the arm catalog. Immutable description of one arm
// MODEL (e.g. "KUKA KR6 R900-2"). Multiple placed arms in a cell may share
// one ArmSpec; their per-instance pose lives on a future ArmInstance type.
// Fields mirror data-model.md §3 plus arm-specific extras (mass, lifetime,
// regen capability) needed by the cost model in step 4.
struct ArmSpec {
    std::string id;
    std::string model_name;
    std::string family;
    std::string controller_class;
    Polygon footprint;
    ReachEnvelope reach;
    Mass payload_max;
    Mass mass;
    Speed max_speed;
    Acceleration max_acceleration;
    Length repeatability;
    Power power_peak;
    Power power_idle;
    double list_price_eur;
    Duration lifetime;
    double maintenance_annual_fraction;
    bool regen_capable;
};

} // namespace tinycell::core
