#include "tinycell/solver/anchor_strategy.hpp"

#include <mp-units/systems/si.h>
#include <stdexcept>

namespace tinycell::solver {

namespace {
using namespace mp_units;
namespace tc = tinycell::core;
} // namespace

std::string_view AnchorStrategy::name() const { return "AnchorStrategy"; }

bool AnchorStrategy::applies_to(const tc::Task& task) const {
    return task.kind() == tc::TaskKind::Anchor;
}

StrategyResult AnchorStrategy::evaluate(const tc::Task& task) const {
    if (!applies_to(task)) {
        throw std::logic_error("AnchorStrategy::evaluate called on inapplicable task");
    }
    const auto& a = std::get<tc::AnchorParams>(task.params);

    // The "port" sits at the anchor station's origin in STATION FRAME
    // (port.hpp documents PortConstraint coordinates as station-frame).
    // The anchor's station is pinned at (a.world_x, a.world_y,
    // a.world_theta) by the consumer that builds the LayoutProblem, so
    // port_world = station_pose ∘ (0, 0, 0) recovers the world pose
    // AnchorParams names without doubling it. Direction tolerance 0:
    // an anchor's flow direction is fixed by the world setup.
    (void)a;
    return StrategyResult{
        .feasibility = Feasibility::FULL,
        .strategy_name = std::string{name()},
        .equipment = std::nullopt,
        .energy_per_cycle = 0.0 * si::joule,
        .cycle_time = 0.0 * si::second,
        .achievable_ct_per_item = 0.0 * si::second,
        .partial_info = std::nullopt,
        .preconditions = {},
        .requires_state = [](const tc::ItemState&) { return true; },
        .effect = [](const tc::ItemState& s) { return s; },
        .port_constraints = {
            {.port_name = "port",
             .x = 0.0 * si::metre,
             .y = 0.0 * si::metre,
             .theta = 0.0 * si::radian,
             .direction_tolerance = 0.0 * si::radian},
        },
    };
}

} // namespace tinycell::solver
