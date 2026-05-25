#include "tinycell/solver/transfer_strategy.hpp"

#include <mp-units/systems/si.h>

namespace tinycell::solver {

namespace {
using namespace mp_units;
namespace tc = tinycell::core;
} // namespace

std::string_view TransferStrategy::name() const { return "TransferStrategy"; }

bool TransferStrategy::applies_to(const tc::Task& task) const {
    return task.kind() == tc::TaskKind::Transport;
}

StrategyResult TransferStrategy::evaluate(const tc::Task& task) const {
    (void)task;
    // Magical: zero cost, zero time, no state precondition, identity
    // effect. The Layer-3 placer attaches geometric meaning to the
    // strategy's selection by reading the connected tasks'
    // PortConstraints (T.3); this strategy itself emits no
    // port_constraints (Transport tasks have no LogicalPorts of their
    // own — they reference other tasks' ports).
    return StrategyResult{
        .feasibility = Feasibility::FULL,
        .strategy_name = std::string{name()},
        .equipment = std::nullopt,            // no equipment to bind
        .energy_per_cycle = 0.0 * si::joule,
        .cycle_time = 0.0 * si::second,
        .achievable_ct_per_item = 0.0 * si::second,
        .partial_info = std::nullopt,
        .preconditions = {},
        .requires_state = [](const tc::ItemState&) { return true; },  // any state OK
        .effect = [](const tc::ItemState& s) { return s; },           // identity
        .port_constraints = {},  // Transport tasks have no own ports (T.2)
    };
}

} // namespace tinycell::solver
