#include "tinycell/solver/arm_strategy.hpp"

#include <algorithm>
#include <cmath>
#include <mp-units/systems/si.h>
#include <stdexcept>

namespace tinycell::solver {

namespace {

using namespace mp_units;
namespace tc = tinycell::core;

// PLACEHOLDER (step 4): seconds-per-box is a flat 3 s and the duty cycle is
// a flat 0.6 (60% of cycle time at peak power, 40% at idle). Real analytic
// per-archetype throughput + energy models arrive at step 4 with Layer 2
// (decisions.md, "PARTIAL feasibility staged to step 4"). These constants
// exist only to exercise the StrategyResult plumbing and produce numbers
// you can eyeball in tests and the demo — they are NOT load-bearing.
constexpr double placeholder_seconds_per_box = 3.0;
constexpr double placeholder_arm_active_duty_fraction = 0.6;

// select_arm():
//   1. for each arm in the catalog
//   2.   reject if payload_max < item.mass (can't lift it)
//   3.   reject if reach.max_radius < half the pallet diagonal
//        (rough proxy — see PLACEHOLDER note below)
//   4. among remaining arms, return the one with the lowest list_price_eur
// Returns nullptr if no arm satisfies both filters.
//
// PLACEHOLDER (step 4-5): the reach check is the half-pallet-diagonal proxy.
// A centred arm with max_radius ≥ half-diagonal can reach the pallet corners
// in principle, but real reachability depends on the arm's specific work
// envelope (min reach, joint limits, ground/post obstacles), which comes
// from precomputed workspace tables (architecture.md §5, the COROS-2020
// sphere-collision approach). Arms close to the boundary here may turn out
// infeasible when real reach is computed.
const tc::ArmSpec* select_arm(const tc::PalletizeParams& p,
                              std::span<const tc::ArmSpec> catalog) {
    const auto pallet_diagonal_m = std::sqrt(
        std::pow(p.pallet.physical.width.numerical_value_in(si::metre), 2) +
        std::pow(p.pallet.physical.length.numerical_value_in(si::metre), 2));

    const tc::ArmSpec* best = nullptr;
    for (const auto& arm : catalog) {
        if (arm.payload_max < p.item.physical.mass) {
            continue;
        }
        if (arm.reach.max_radius.numerical_value_in(si::metre) < 0.5 * pallet_diagonal_m) {
            continue;
        }
        if (best == nullptr || arm.list_price_eur < best->list_price_eur) {
            best = &arm;
        }
    }
    return best;
}

// State-flow contract for ArmStrategy on a Palletize task:
//   * requires_state — the arm needs to know where to grasp the item AND
//     needs to know its orientation alignment. Arms read the KNOWLEDGE
//     axis of orientation (planner belief is enough — the arm plans its
//     motion from the belief and its end-effector compliance absorbs
//     minor drift between observation and grasp). ANY alignment is OK
//     as long as the arm knows which one; it parameterises the grasp
//     by what's known. Carrier-agnostic: an arm doesn't care whether
//     the item is on a belt, a fixture, or free — this carrier-
//     flexibility is what distinguishes an arm from a pusher in the
//     OR-tree.
//   * effect — after palletizing, the item rests on the pallet with
//     pose known AND physically held at the canonical alignment 0 (the
//     arm placed it deliberately at the pattern position; both belief
//     and physical control collapse to that alignment).
//
// Orientation is gated via `core::orientation_known`, which composes the
// item's symmetry and the current knowledge — strategies do not switch
// on item type (decisions.md "Item properties drive strategy gating,
// not item type").
RequiresStateFn arm_requires_state(tc::RotationalSymmetry item_symmetry) {
    return [item_symmetry](const tc::ItemState& s) {
        return s.position_known &&
               core::orientation_known(item_symmetry, s.orientation.knowledge);
    };
}

EffectFn arm_palletize_effect() {
    return [](const tc::ItemState& s) {
        tc::ItemState next = s;
        next.position_known = true;
        next.on_carrier = tc::OnCarrier::Pallet;
        // Arm-placed: item is BOTH observed (Knowledge) and physically
        // held (Control) at the canonical alignment 0 by the pallet's
        // grid pattern.
        next.orientation.knowledge = tc::knowledge::known(0);
        next.orientation.control = tc::control::constrained(0);
        return next;
    };
}

} // namespace

ArmStrategy::ArmStrategy(std::span<const tc::ArmSpec> catalog) : catalog_(catalog) {}

std::string_view ArmStrategy::name() const { return "ArmStrategy"; }

// applies_to(): the arm covers any task kind the strategy class is willing
// to solve. Today that's Palletize only; add Transport / Grip / Assemble
// here (and add the corresponding logic in evaluate) as those task kinds
// come online. Keep the switch over TaskKind exhaustive — when a new kind
// is added, the compiler will warn here if it isn't handled.
bool ArmStrategy::applies_to(const tc::Task& task) const {
    return task.kind() == tc::TaskKind::Palletize;
}

// evaluate(): produces a candidate proposal for the given task.
//   1. precondition check — task must be one we apply to
//   2. for a Palletize task, pick the cheapest feasible arm via select_arm
//   3. if no arm is feasible, return INFEASIBLE with zero metrics (still
//      carrying the state-flow contract — see arm_requires_state /
//      arm_palletize_effect above)
//   4. otherwise compute placeholder cycle_time and energy_per_cycle and
//      return FULL with the chosen arm as candidate equipment
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
            .requires_state = arm_requires_state(p.item.physical.symmetry),
            .effect = arm_palletize_effect(),
        };
    }

    // PLACEHOLDER (step 4): cycle_time and energy_per_cycle come from flat
    // constants, not a real model. See the placeholder_* constants above.
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
        .requires_state = arm_requires_state(p.item.physical.symmetry),
        .effect = arm_palletize_effect(),
    };
}

} // namespace tinycell::solver
