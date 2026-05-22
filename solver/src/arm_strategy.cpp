#include "tinycell/solver/arm_strategy.hpp"

#include <algorithm>
#include <mp-units/systems/si.h>
#include <stdexcept>

namespace tinycell::solver {

namespace {

using namespace mp_units;
namespace tc = tinycell::core;

// Placeholder cycle-time and energy model. Replace with the analytic
// per-archetype throughput / cost models when step 4 (Layer 2 allocation)
// brings cycle-time as a hard constraint into the solver (see roadmap.md).
// These constants are not load-bearing for step 1 — they exercise the
// StrategyResult plumbing only.
constexpr double placeholder_seconds_per_box = 3.0;
constexpr double placeholder_arm_active_duty_fraction = 0.6;

// Pick the cheapest feasible arm — payload >= item mass, and the pallet
// footprint fits inside the arm's max reach when centred. The reach check
// is rough: the diagonal of the pallet bounding box must fit within reach.
const tc::ArmEntry* select_arm(const tc::PalletizeParams& p,
                               std::span<const tc::ArmEntry> catalog) {
    const auto pallet_diagonal_m = std::sqrt(
        std::pow(p.pallet.width.numerical_value_in(si::metre), 2) +
        std::pow(p.pallet.length.numerical_value_in(si::metre), 2));

    const tc::ArmEntry* best = nullptr;
    for (const auto& arm : catalog) {
        if (arm.payload_max < p.item.mass) {
            continue;
        }
        // Half the pallet diagonal is the worst-case reach distance from a
        // centred arm. Real reachability comes from precomputed workspace
        // tables in a later step.
        if (arm.reach.max_radius.numerical_value_in(si::metre) < 0.5 * pallet_diagonal_m) {
            continue;
        }
        if (best == nullptr || arm.list_price_eur < best->list_price_eur) {
            best = &arm;
        }
    }
    return best;
}

} // namespace

ArmStrategy::ArmStrategy(std::span<const tc::ArmEntry> catalog) : catalog_(catalog) {}

std::string_view ArmStrategy::name() const { return "ArmStrategy"; }

bool ArmStrategy::applies_to(const tc::Task& task) const {
    // For step 1, an arm covers Palletize. Add Transport / Grip / Assemble
    // as those TaskKinds come online.
    return task.kind() == tc::TaskKind::Palletize;
}

StrategyResult ArmStrategy::evaluate(const tc::Task& task) const {
    if (!applies_to(task)) {
        throw std::logic_error("ArmStrategy::evaluate called on inapplicable task");
    }
    const auto& p = std::get<tc::PalletizeParams>(task.params);

    const auto* arm = select_arm(p, catalog_);
    if (arm == nullptr) {
        return StrategyResult{
            .feasibility = Feasibility::INFEASIBLE,
            .strategy_name = std::string{name()},
            .equipment = std::nullopt,
            .energy_per_cycle = 0.0 * si::joule,
            .cycle_time = 0.0 * si::second,
        };
    }

    const auto cycle_time = placeholder_seconds_per_box
                            * static_cast<double>(p.box_count) * si::second;
    const auto active_seconds = cycle_time.numerical_value_in(si::second)
                                * placeholder_arm_active_duty_fraction;
    const auto idle_seconds = cycle_time.numerical_value_in(si::second) - active_seconds;
    const auto energy_j =
        arm->power_peak.numerical_value_in(si::watt) * active_seconds +
        arm->power_idle.numerical_value_in(si::watt) * idle_seconds;

    return StrategyResult{
        .feasibility = Feasibility::FULL,
        .strategy_name = std::string{name()},
        .equipment = EquipmentRef{.catalog_id = arm->id},
        .energy_per_cycle = energy_j * si::joule,
        .cycle_time = cycle_time,
    };
}

} // namespace tinycell::solver
