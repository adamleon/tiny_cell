#include "tinycell/solver/pusher_strategy.hpp"

#include <algorithm>
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

// select_pusher():
//   1. for each pusher in the catalog
//   2.   reject if payload_max < item.mass (can't push it)
//   3.   reject if stroke < max(pallet.width, pallet.length)
//        — the pusher must traverse the full pallet depth so the farthest
//        box reaches its target column. Placeholder proxy; the real check
//        depends on in-feed direction and pattern, neither modelled today.
//   4. among remaining pushers, return the one with the lowest list_price_eur
// Returns nullptr if no pusher satisfies both filters.
//
// PLACEHOLDER (step 4-5): the stroke check is a worst-case proxy that
// assumes the pusher must reach the far edge of the pallet in a single
// extension. With a multi-station feed or a re-positionable in-feed the
// required stroke is smaller; with diagonal pushing it's larger. Step 5's
// placement layer will replace this with a real geometric check.
const tc::PusherSpec* select_pusher(const tc::PalletizeParams& p,
                                    std::span<const tc::PusherSpec> catalog) {
    const auto required_stroke = std::max(p.pallet.width, p.pallet.length);

    const tc::PusherSpec* best = nullptr;
    for (const auto& pusher : catalog) {
        if (pusher.payload_max < p.item.mass) {
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
//   2. for a Palletize task, pick the cheapest feasible pusher
//   3. if none is feasible, return INFEASIBLE with zero metrics
//   4. otherwise compute placeholder cycle_time (one push per box) and
//      energy_per_cycle, return FULL with the chosen pusher as candidate
StrategyResult PusherStrategy::evaluate(const tc::Task& task) const {
    if (!applies_to(task)) {
        throw std::logic_error("PusherStrategy::evaluate called on inapplicable task");
    }
    const auto& p = std::get<tc::PalletizeParams>(task.params);

    const auto* pusher = select_pusher(p, catalog_);
    if (pusher == nullptr) {
        return StrategyResult{
            .feasibility = Feasibility::INFEASIBLE,
            .strategy_name = std::string{name()},
            .equipment = std::nullopt,
            .energy_per_cycle = 0.0 * si::joule,
            .cycle_time = 0.0 * si::second,
        };
    }

    // PLACEHOLDER (step 4): cycle_time = cycle_time_per_push × box_count
    // assumes one push per box with no overlap between in-feed and pusher
    // motion. Real model needs the throughput pipeline coupling in-feed
    // rate, pusher cycle, and pallet pattern.
    const auto cycle_time = pusher->cycle_time_per_push * static_cast<double>(p.box_count);
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
    };
}

} // namespace tinycell::solver
