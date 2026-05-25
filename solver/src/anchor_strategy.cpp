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

    // The "port" PortConstraint is at (a.world_x, a.world_y, a.world_theta).
    // Note: these are WORLD coordinates, not station-frame. Anchors
    // are pinned, so the convention is that the anchor's "station"
    // sits at the world pose AnchorParams names, with the port at the
    // station's origin. The placer reads this PortConstraint and
    // pins the anchor station's world pose to the AnchorParams pose
    // (which it does NOT optimise over). At T.7 the demo just records
    // the values; Phase D wires the pinning.
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
             .x = a.world_x,
             .y = a.world_y,
             .theta = a.world_theta,
             .direction_tolerance = 0.0 * si::radian},
        },
    };
}

} // namespace tinycell::solver
