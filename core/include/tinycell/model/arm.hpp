#pragma once

#include <string>
#include <tinycell/geometry.hpp>
#include <tinycell/units.hpp>

namespace tinycell::core {

struct ReachEnvelope {
    Length min_radius;
    Length max_radius;
};

struct ArmEntry {
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
    Price list_price_eur;
    Duration lifetime;
    double maintenance_annual_fraction;
    bool regen_capable;
};

} // namespace tinycell::core
