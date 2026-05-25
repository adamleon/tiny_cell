#include "tinycell/solver/arm_strategy.hpp"

#include "motion_model.hpp"

#include <algorithm>
#include <cmath>
#include <mp-units/systems/si.h>
#include <numbers>
#include <stdexcept>
#include <tinycell/model/port.hpp>
#include <vector>

namespace tinycell::solver {

namespace {

using namespace mp_units;
namespace tc = tinycell::core;

// Arm-emitted PortConstraints for a Palletize task. Arms can pick/place
// from anywhere within their reach envelope, so direction tolerance is
// pi/2 (any direction within +/- 90 degrees works) — the arm rotates to
// service the connected transport's direction. All three ports sit at
// the station origin (= the arm base for the one-instance-per-station
// pattern current at MVP); refinements to non-origin positions arrive
// when the placer needs more spatial precision (Phase D or later).
std::vector<tc::PortConstraint> arm_palletize_port_constraints() {
    constexpr auto kLoose = (std::numbers::pi / 2.0) * si::radian;
    return {
        {.port_name = "item_in",
         .x = 0.0 * si::metre, .y = 0.0 * si::metre,
         .theta = 0.0 * si::radian, .direction_tolerance = kLoose},
        {.port_name = "pallet_in",
         .x = 0.0 * si::metre, .y = 0.0 * si::metre,
         .theta = 0.0 * si::radian, .direction_tolerance = kLoose},
        {.port_name = "pallet_out",
         .x = 0.0 * si::metre, .y = 0.0 * si::metre,
         .theta = 0.0 * si::radian, .direction_tolerance = kLoose},
    };
}

// One feasible-on-physical-grounds arm candidate paired with its per-pick
// motion-model output. The strategy walks the candidate list in price
// order to choose between FULL (cheapest arm meeting the target rate)
// and PARTIAL (cheapest feasible arm if none meets the target).
//
// PLACEHOLDER (step 4-5): the reach feasibility uses the half-pallet-
// diagonal proxy; real reachability depends on the arm's specific work
// envelope (min reach, joint limits, ground/post obstacles), which comes
// from precomputed workspace tables (architecture.md §5). Arms close to
// the proxy boundary may turn out infeasible at Layer 3.
struct ArmCandidate {
    const tc::ArmSpec* arm;
    MotionCycle per_pick;
};

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
//   1. precondition check — task must be one we apply to.
//   2. gather feasible arms (passing payload + half-pallet-diagonal reach
//      proxy), each with its per-pick motion-model output.
//   3. if no arm is feasible → INFEASIBLE (carrying state-flow contract).
//   4. sort feasible arms cheapest-first.
//   5. walk in price order: first arm whose per-item cycle time meets the
//      task's target_ct_per_item → FULL with that arm.
//   6. if no arm meets target → PARTIAL with the cheapest feasible arm,
//      emitting a residual Palletize task whose target is tightened so
//      that another instance covering it (in parallel) completes the
//      original rate. Residual math: with achievable a and target t per
//      item, this arm covers fraction t/a of the demand; the residual
//      must cover (a-t)/a of the demand at the same rate, giving residual
//      per-item time (a*t)/(a-t).
StrategyResult ArmStrategy::evaluate(const tc::Task& task) const {
    if (!applies_to(task)) {
        throw std::logic_error("ArmStrategy::evaluate called on inapplicable task");
    }
    const auto& p = std::get<tc::PalletizeParams>(task.params);

    // One-way pick→drop distance as the half-pallet-diagonal proxy. Layer 3
    // will replace this with actual placed-geometry distance; same seam.
    const auto pallet_diagonal_m = std::sqrt(
        std::pow(p.pallet.physical.width.numerical_value_in(si::metre), 2) +
        std::pow(p.pallet.physical.length.numerical_value_in(si::metre), 2));
    const auto one_way_distance = (0.5 * pallet_diagonal_m) * si::metre;

    std::vector<ArmCandidate> candidates;
    for (const auto& arm : catalog_) {
        if (arm.payload_max.value() < p.item.physical.mass) {
            continue;
        }
        if (arm.reach.max_radius.value().numerical_value_in(si::metre)
            < 0.5 * pallet_diagonal_m) {
            continue;
        }
        candidates.push_back(ArmCandidate{
            .arm = &arm,
            .per_pick = arm_motion_cycle(one_way_distance, p.item.physical.mass, arm),
        });
    }

    const auto state_in = arm_requires_state(p.item.physical.symmetry);
    const auto effect_out = arm_palletize_effect();

    if (candidates.empty()) {
        return StrategyResult{
            .feasibility = Feasibility::INFEASIBLE,
            .strategy_name = std::string{name()},
            .equipment = std::nullopt,
            .energy_per_cycle = 0.0 * si::joule,
            .cycle_time = 0.0 * si::second,
            .achievable_ct_per_item = 0.0 * si::second,
            .partial_info = std::nullopt,
            .preconditions = {},
            .requires_state = state_in,
            .effect = effect_out,
            // INFEASIBLE: no binding, no concrete port constraints. The
            // port_constraints default-initialises to empty.
        };
    }

    // Strategy returns ONE proposal per call (data-model.md §2); the OR-tree
    // explores alternatives via other strategies on the same task, not by
    // fanning out here.
    std::sort(candidates.begin(), candidates.end(),
              [](const ArmCandidate& a, const ArmCandidate& b) {
                  return a.arm->list_price_eur < b.arm->list_price_eur;
              });

    const auto target = task.target_ct_per_item.value();
    const auto box_count = static_cast<double>(p.box_count);

    // FULL: cheapest arm whose per-item cycle time meets the target.
    for (const auto& c : candidates) {
        if (c.per_pick.cycle_time <= target) {
            return StrategyResult{
                .feasibility = Feasibility::FULL,
                .strategy_name = std::string{name()},
                .equipment = EquipmentRef{.catalog_id = c.arm->id},
                .energy_per_cycle = c.per_pick.energy_per_cycle * box_count,
                .cycle_time = c.per_pick.cycle_time * box_count,
                .achievable_ct_per_item = c.per_pick.cycle_time,
                .partial_info = std::nullopt,
                .preconditions = {},
                .requires_state = state_in,
                .effect = effect_out,
                .port_constraints = arm_palletize_port_constraints(),
            };
        }
    }

    // PARTIAL: no arm meets target — propose the cheapest feasible arm,
    // emit a residual whose target is tightened per the math above.
    const auto& best = candidates.front();
    const auto a_s = best.per_pick.cycle_time.numerical_value_in(si::second);
    const auto t_s = target.numerical_value_in(si::second);
    const auto residual_target_s = (a_s * t_s) / (a_s - t_s);

    tc::Task residual = task;
    residual.id = task.id + "_residual";
    residual.target_ct_per_item = tc::CycleTimePerItem{residual_target_s * si::second};
    residual.is_residual = true;

    return StrategyResult{
        .feasibility = Feasibility::PARTIAL,
        .strategy_name = std::string{name()},
        .equipment = EquipmentRef{.catalog_id = best.arm->id},
        .energy_per_cycle = best.per_pick.energy_per_cycle * box_count,
        .cycle_time = best.per_pick.cycle_time * box_count,
        .achievable_ct_per_item = best.per_pick.cycle_time,
        .partial_info = PartialInfo{
            .achievable_ct_per_item = best.per_pick.cycle_time,
            .target_ct_per_item = target,
        },
        .preconditions = {residual},
        .requires_state = state_in,
        .effect = effect_out,
        .port_constraints = arm_palletize_port_constraints(),
    };
}

} // namespace tinycell::solver
