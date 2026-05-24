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
// belt against an end stop. When the row is full, the belt stops and
// the pusher transfers the whole row sideways onto the pallet in one
// stroke. The pallet belt then advances perpendicular to the push by
// one box, so the next row has space. One push moves one ROW, not one
// box.
//
//   row direction      = the dimension boxes line up along (perpendicular
//                        to push)
//   push direction     = the dimension the pusher traverses
//   advance direction  = perpendicular to push; accumulated by belt index
//
// The pusher's stroke is sized to traverse the pallet's smaller
// dimension (taken as the row direction). The dimension of each box
// along the row direction depends on the box's ALIGNMENT — see
// alignment-aware layout below.

// RowLayout captures all per-alignment metrics the catalog selection
// and cycle-time math need. The fundamental question for a pusher
// pushing rectangles is "which way is the box oriented when it arrives
// on the in-feed belt?" — different alignments produce different
// boxes-per-row, different stroke requirements, different row payloads.
struct RowLayout {
    int boxes_per_row;
    int num_rows;
    tc::Mass row_payload;
};

// layout_for_alignment: compute row metrics assuming the item arrives
// in the given alignment.
//   * alignment 0 (canonical) = item's `width` along the local frame's
//                               row direction (X axis by our convention).
//   * alignment 1             = item rotated 90° from canonical
//                               (`length` along the row direction).
// For continuous-symmetry items (cylinder), width and length are equal
// and alignment is moot — the helper returns the same metrics for any
// alignment.
RowLayout layout_for_alignment(const tc::PalletizeParams& p, int alignment) {
    // Convention: alignment 0 puts the item's declared `width` along
    // the row direction. alignment 1 swaps it for `length`. Higher
    // alignment indices (asymmetric items at non-cardinal grids) reduce
    // to one of these two by symmetry in MVP — extend when a strategy
    // actually exercises more than two cardinal alignments.
    const auto along_row = (alignment == 0)
        ? p.item.physical.width
        : p.item.physical.length;

    const auto row_dim_m =
        std::min(p.pallet.physical.width, p.pallet.physical.length)
            .numerical_value_in(si::metre);
    const auto along_row_m = along_row.numerical_value_in(si::metre);

    RowLayout layout;
    layout.boxes_per_row =
        static_cast<int>(std::floor(row_dim_m / along_row_m));
    if (layout.boxes_per_row <= 0) {
        layout.num_rows = 0;
        layout.row_payload = 0.0 * si::kilogram;
        return layout;
    }
    layout.num_rows = (p.box_count + layout.boxes_per_row - 1) / layout.boxes_per_row;
    layout.row_payload =
        p.item.physical.mass * static_cast<double>(layout.boxes_per_row);
    return layout;
}

// pick_required_alignment: enumerate the alignments the item could
// arrive in and pick the favourable one (more boxes per row → fewer
// total push cycles). For continuous-symmetry items, the alignment is
// moot — return 0 unconditionally; the predicate's continuous-symmetry
// short-circuit makes the choice irrelevant downstream. For square-
// symmetric items the two layout computations produce identical
// results; we still return 0 to keep the canonical convention.
//
// PLACEHOLDER (step 4): "favourable" is currently more-boxes-per-row.
// A real cost model would weigh row-count against stroke distance,
// energy per push, and row payload against catalog availability.
int pick_required_alignment(const tc::PalletizeParams& p) {
    const auto a0 = layout_for_alignment(p, 0);
    const auto a1 = layout_for_alignment(p, 1);
    return (a1.boxes_per_row > a0.boxes_per_row) ? 1 : 0;
}

// select_pusher():
//   1. for each pusher in the catalog
//   2.   reject if payload_max < row_payload (the stroke moves a full
//        row at once — all of those box masses ride on the pusher face
//        simultaneously)
//   3.   reject if stroke < required_stroke (the pusher must traverse
//        one pallet row in a single extension)
//   4. among remaining pushers, return the one with the lowest list_price_eur
// Returns nullptr if no pusher satisfies both filters.
const tc::PusherSpec* select_pusher(const tc::PalletizeParams& p,
                                    const RowLayout& layout,
                                    std::span<const tc::PusherSpec> catalog) {
    const auto required_stroke =
        std::min(p.pallet.physical.width, p.pallet.physical.length);

    const tc::PusherSpec* best = nullptr;
    for (const auto& pusher : catalog) {
        if (pusher.payload_max.value() < layout.row_payload) {
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
//     known, AND physically held at the specific alignment the row math
//     was sized for. Pushers read the CONTROL axis of orientation
//     (not Knowledge): the pusher's stroke is open-loop, so a camera-
//     only observation isn't sufficient — the item could rotate on the
//     belt between observation and contact. Production cells achieve
//     this with side-guides / pre-fixturing that physically pin the
//     alignment.
//
//     Required alignment is whichever class `pick_required_alignment`
//     selected based on the row math. For a rectangle (Discrete(180))
//     this picks the alignment that maximises boxes-per-row. For
//     cylinder/square the choice collapses (one effective alignment).
//
//   * effect — after palletizing, the item rests on the pallet with
//     position known AND knowledge+control both pinned to the
//     canonical alignment 0 (the pusher's stroke places it
//     deterministically against the pallet's row grid).
//
// "Item is pushable" / shape-regularity guards are STATIC item
// properties, not state predicates (data-model.md §2.1) — they live as
// INFEASIBLE branches inside evaluate (currently the `boxes_per_row
// == 0` short-circuit on the chosen alignment). A `pushable` flag on
// BoxSpec would let pushable-INFEASIBLE be reported pre-catalog like
// the pattern guard, but BoxSpec carries no such flag yet.
// PLACEHOLDER (step 4): add `pushable: bool` to BoxSpec and short-
// circuit non-pushable items here before catalog lookup.
RequiresStateFn pusher_requires_state(tc::RotationalSymmetry item_symmetry,
                                      int required_alignment) {
    return [item_symmetry, required_alignment](const tc::ItemState& s) {
        if (!s.position_known) return false;
        if (s.on_carrier != tc::OnCarrier::Belt) return false;
        return core::control_matches_alignment(
            item_symmetry, s.orientation.control, required_alignment);
    };
}

EffectFn pusher_palletize_effect() {
    return [](const tc::ItemState& s) {
        tc::ItemState next = s;
        next.position_known = true;
        next.on_carrier = tc::OnCarrier::Pallet;
        // Pusher-placed: item is BOTH observed (Knowledge) and
        // physically held (Control) at the canonical alignment 0 by
        // the pallet's row grid.
        next.orientation.knowledge = tc::knowledge::known(0);
        next.orientation.control = tc::control::constrained(0);
        return next;
    };
}

} // namespace

PusherStrategy::PusherStrategy(std::span<const tc::PusherSpec> catalog) : catalog_(catalog) {}

std::string_view PusherStrategy::name() const { return "PusherStrategy"; }

// applies_to(): the pusher covers any task kind the strategy class is
// willing to solve. Today that's Palletize only; PushOff and short-
// distance Transport are the obvious additions when those task kinds
// come online.
bool PusherStrategy::applies_to(const tc::Task& task) const {
    return task.kind() == tc::TaskKind::Palletize;
}

// evaluate(): produces a candidate proposal for the given task.
//   1. precondition check — task must be one we apply to
//   2. pick the required alignment (which class the row math wants);
//      compute the row layout for that alignment
//   3. if boxes_per_row is 0 (box too wide for the pallet row at the
//      chosen alignment), no pusher can serve it — INFEASIBLE before
//      catalog lookup
//   4. pick the cheapest pusher whose stroke covers one row and whose
//      payload covers the row's combined mass
//   5. if none is feasible, return INFEASIBLE with zero metrics
//   6. otherwise compute cycle_time (one push per row) and
//      energy_per_cycle, return FULL with the chosen pusher as
//      candidate; the emitted predicate captures the required alignment
//      so the propagator gates on it.
StrategyResult PusherStrategy::evaluate(const tc::Task& task) const {
    if (!applies_to(task)) {
        throw std::logic_error("PusherStrategy::evaluate called on inapplicable task");
    }
    const auto& p = std::get<tc::PalletizeParams>(task.params);

    const int required_alignment = pick_required_alignment(p);
    const auto layout = layout_for_alignment(p, required_alignment);

    if (layout.boxes_per_row <= 0) {
        // Box too wide for even one to fit in a pallet row at the
        // chosen alignment — no pusher configuration can serve this
        // task. Equivalent to an INFEASIBLE pattern guard
        // (data-model.md §2.1) and reported the same way.
        return StrategyResult{
            .feasibility = Feasibility::INFEASIBLE,
            .strategy_name = std::string{name()},
            .equipment = std::nullopt,
            .energy_per_cycle = 0.0 * si::joule,
            .cycle_time = 0.0 * si::second,
            .requires_state = pusher_requires_state(
                p.item.physical.symmetry, required_alignment),
            .effect = pusher_palletize_effect(),
        };
    }

    const auto* pusher = select_pusher(p, layout, catalog_);
    if (pusher == nullptr) {
        return StrategyResult{
            .feasibility = Feasibility::INFEASIBLE,
            .strategy_name = std::string{name()},
            .equipment = std::nullopt,
            .energy_per_cycle = 0.0 * si::joule,
            .cycle_time = 0.0 * si::second,
            .requires_state = pusher_requires_state(
                p.item.physical.symmetry, required_alignment),
            .effect = pusher_palletize_effect(),
        };
    }

    // PLACEHOLDER (step 4): cycle_time = cycle_time_per_push × num_rows
    // counts only push events; in-feed accumulation between pushes is
    // assumed to overlap perfectly. Real throughput model in step 4
    // will couple in-feed rate, pusher cycle, and pallet advance.
    const auto cycle_time =
        pusher->cycle_time_per_push * static_cast<double>(layout.num_rows);
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
        .requires_state = pusher_requires_state(
            p.item.physical.symmetry, required_alignment),
        .effect = pusher_palletize_effect(),
    };
}

} // namespace tinycell::solver
