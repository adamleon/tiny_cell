#include "tinycell/solver/pusher_strategy.hpp"

#include <algorithm>
#include <cmath>
#include <mp-units/systems/si.h>
#include <stdexcept>

namespace tinycell::solver {

namespace {

using namespace mp_units;
namespace tc = tinycell::core;

// PLACEHOLDER (step 4): the active duty fraction is a flat 0.4. A pusher
// extends + retracts + dwells; the active stroke is roughly the extend
// portion. Real per-archetype throughput / energy models arrive at step 4
// with Layer 2 (decisions.md). Constant exists only to exercise the
// StrategyResult plumbing — NOT load-bearing.
constexpr double placeholder_pusher_active_duty_fraction = 0.4;

// Equipment-model assumption: a row of boxes accumulates on an in-feed
// belt against an end stop. When the row is full, the belt stops and the
// pusher transfers the whole row sideways onto the pallet in one stroke.
// The pallet belt then advances perpendicular to the push by one box, so
// the next row has space. One push moves one ROW, not one box.
//
//   row direction      = the dimension the pusher pushes across
//                       = the smaller pallet dimension (cheaper stroke)
//   advance direction  = perpendicular to push; accumulated by belt index
//
// Picking the smaller pallet dim as the row direction is optimistic
// (favours feasibility / cost) and mirrors the half-diagonal optimism of
// ArmStrategy's reach proxy.

// boxes_per_row(): how many boxes pack along the row direction in a single
// push. Grid-pattern assumption only (data-model.md §1 — grid pattern is the
// step-1 scope). Box is oriented so its smaller floor dimension runs along
// the row, maximising boxes per row.
int boxes_per_row(const tc::PalletizeParams& p) {
    const auto row_dim_m =
        std::min(p.pallet.width, p.pallet.length).numerical_value_in(si::metre);
    const auto box_side_along_row_m =
        std::min(p.item.width, p.item.length).numerical_value_in(si::metre);
    return static_cast<int>(std::floor(row_dim_m / box_side_along_row_m));
}

// select_pusher():
//   1. for each pusher in the catalog
//   2.   reject if payload_max < item.mass × boxes_per_row
//        (the stroke moves a full row at once — all of those box masses
//        ride on the pusher face simultaneously)
//   3.   reject if stroke < row_dim
//        (the pusher must traverse one pallet row in a single extension)
//   4. among remaining pushers, return the one with the lowest list_price_eur
// Returns nullptr if no pusher satisfies both filters.
const tc::PusherSpec* select_pusher(const tc::PalletizeParams& p, int boxes_in_row,
                                    std::span<const tc::PusherSpec> catalog) {
    const auto required_stroke = std::min(p.pallet.width, p.pallet.length);
    const auto row_payload = p.item.mass * static_cast<double>(boxes_in_row);

    const tc::PusherSpec* best = nullptr;
    for (const auto& pusher : catalog) {
        if (pusher.payload_max < row_payload) {
            continue;
        }
        if (pusher.stroke < required_stroke) {
            continue;
        }
        if (best == nullptr || pusher.list_price_eur < best->list_price_eur) {
            best = &pusher;
        }
    }
    return best;
}

// State-flow contract for PusherStrategy on a Palletize task:
//   * requires_state — items must be on a feeder belt with their pose
//     known. The equipment model is "items accumulate on a belt against
//     an end stop until a row is queued, then one stroke transfers the
//     whole row" — both inputs (carrier = BELT, pose known to align
//     against the stop) are non-negotiable.
//   * effect — after palletizing, the item rests on the pallet with its
//     pose known (the stroke places it deterministically).
// "Item is pushable" / shape-regularity guards are STATIC item properties,
// not state predicates (data-model.md §2.1) — they live as INFEASIBLE
// branches inside evaluate (currently the boxes_per_row == 0 short-
// circuit). A pushable flag on BoxSpec would let pushable-INFEASIBLE be
// reported pre-catalog like the pattern guard, but BoxSpec carries no
// such flag yet.
// PLACEHOLDER (step 4): add `pushable: bool` to BoxSpec and short-
// circuit non-pushable items here before catalog lookup.
RequiresStateFn pusher_requires_state() {
    return [](const tc::ItemState& s) {
        return s.position_known && s.on_carrier == tc::OnCarrier::Belt;
    };
}

EffectFn pusher_palletize_effect() {
    return [](const tc::ItemState& s) {
        tc::ItemState next = s;
        next.position_known = true;
        next.on_carrier = tc::OnCarrier::Pallet;
        return next;
    };
}

} // namespace

PusherStrategy::PusherStrategy(std::span<const tc::PusherSpec> catalog) : catalog_(catalog) {}

std::string_view PusherStrategy::name() const { return "PusherStrategy"; }

// applies_to(): the pusher covers any task kind the strategy class is
// willing to solve. Today that's Palletize only; PushOff and short-distance
// Transport are the obvious additions when those task kinds come online.
bool PusherStrategy::applies_to(const tc::Task& task) const {
    return task.kind() == tc::TaskKind::Palletize;
}

// evaluate(): produces a candidate proposal for the given task.
//   1. precondition check — task must be one we apply to
//   2. compute boxes_per_row; if zero (box too wide for the pallet row),
//      no pusher can serve it — INFEASIBLE before catalog lookup
//   3. pick the cheapest pusher whose stroke covers one row and whose
//      payload covers the row's combined mass
//   4. if none is feasible, return INFEASIBLE with zero metrics
//   5. otherwise compute cycle_time (one push per row) and energy_per_cycle,
//      return FULL with the chosen pusher as candidate
StrategyResult PusherStrategy::evaluate(const tc::Task& task) const {
    if (!applies_to(task)) {
        throw std::logic_error("PusherStrategy::evaluate called on inapplicable task");
    }
    const auto& p = std::get<tc::PalletizeParams>(task.params);

    const int per_row = boxes_per_row(p);
    if (per_row <= 0) {
        // Box too wide for even one to fit in a pallet row — no pusher
        // configuration can serve this task. Equivalent to an INFEASIBLE
        // pattern guard (data-model.md §2.1) and reported the same way.
        return StrategyResult{
            .feasibility = Feasibility::INFEASIBLE,
            .strategy_name = std::string{name()},
            .equipment = std::nullopt,
            .energy_per_cycle = 0.0 * si::joule,
            .cycle_time = 0.0 * si::second,
            .requires_state = pusher_requires_state(),
            .effect = pusher_palletize_effect(),
        };
    }

    const auto* pusher = select_pusher(p, per_row, catalog_);
    if (pusher == nullptr) {
        return StrategyResult{
            .feasibility = Feasibility::INFEASIBLE,
            .strategy_name = std::string{name()},
            .equipment = std::nullopt,
            .energy_per_cycle = 0.0 * si::joule,
            .cycle_time = 0.0 * si::second,
            .requires_state = pusher_requires_state(),
            .effect = pusher_palletize_effect(),
        };
    }

    // PLACEHOLDER (step 4): cycle_time = cycle_time_per_push × num_rows
    // counts only push events; in-feed accumulation between pushes is
    // assumed to overlap perfectly. Real throughput model in step 4 will
    // couple in-feed rate, pusher cycle, and pallet advance.
    const int num_rows = (p.box_count + per_row - 1) / per_row;
    const auto cycle_time = pusher->cycle_time_per_push * static_cast<double>(num_rows);
    const auto active_seconds = cycle_time.numerical_value_in(si::second)
                                * placeholder_pusher_active_duty_fraction;
    const auto idle_seconds = cycle_time.numerical_value_in(si::second) - active_seconds;
    const auto energy_j =
        pusher->power_peak.numerical_value_in(si::watt) * active_seconds +
        pusher->power_idle.numerical_value_in(si::watt) * idle_seconds;

    return StrategyResult{
        .feasibility = Feasibility::FULL,
        .strategy_name = std::string{name()},
        .equipment = EquipmentRef{.catalog_id = pusher->id},
        .energy_per_cycle = energy_j * si::joule,
        .cycle_time = cycle_time,
        .requires_state = pusher_requires_state(),
        .effect = pusher_palletize_effect(),
    };
}

} // namespace tinycell::solver
