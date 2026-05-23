#pragma once

// Item domain types — the static physical properties an item type carries
// (ItemPhysical), the planner's dynamic knowledge about ONE item flowing
// through the workflow (ItemKnowledge), and the small symmetry + orientation-
// knowledge types they rely on.
//
// **Architectural principle (decisions.md "Item properties drive strategy
// gating, not item type").** Strategies reason about item PROPERTIES via
// shared helpers; they never switch on item TYPES. Adding a new shape
// becomes a different set of property values, not a new code path. The
// helper layer (distinct_orientations, orientation_resolved, future
// graspable_by / stable_on / …) is the seam against nested if-trees.
//
// **Concept split — four things, not one.**
//   * ItemPhysical — STATIC properties of an item TYPE (box model, pallet
//     model). Owned by the spec.
//   * ItemKnowledge — DYNAMIC planner-tracked facts about ONE item flowing
//     through the workflow. Mutated by each strategy's effect.
//   * Sensor strategy capability — what a strategy can READ off an item
//     (camera resolves to item.symmetry; laser resolves one position axis).
//   * Actor strategy requirement + effect — what a strategy NEEDS and how
//     it changes the knowledge (arm needs orientation resolved; pusher adds
//     a belt-carrier requirement; fixture forces orientation to a snap).
//
// **Future instance distinction.** Today the workflow threads `ItemKnowledge`
// directly; specs (`BoxSpec`, `PalletSpec`) describe the item TYPE. When
// the cell grows to track individual physical instances, the instance type
// will wrap an `ItemKnowledge` plus a spec reference — `BoxInstance{
// const BoxSpec* spec; ItemKnowledge knowledge; }`. The current shapes are
// designed so that wrap-into-instance is mechanical: `ItemKnowledge` is
// already decoupled from spec types.

#include <limits>
#include <stdexcept>
#include <tinycell/units.hpp>
#include <variant>

namespace tinycell::core {

// ---------------------------------------------------------------------------
// RotationalSymmetry — STATIC property of an item type.
// ---------------------------------------------------------------------------

// RotationalSymmetry describes how the item maps to itself under rotation
// about its vertical axis (2D footprint case). Three states:
//   * Continuous       — every rotation maps to self (cylinder).
//   * Discrete{period} — the smallest rotation that maps the item to itself
//                        is `period_deg` degrees:
//                            rectangle  = 180
//                            square     =  90
//                            triangle   = 120  (equilateral, rotation only)
//                            hexagon    =  60  (regular, rotation only)
//                        Construct via `symmetry::discrete(period)`.
//   * Asymmetric       — only the identity rotation (360°) maps to self.
//
// Reflection symmetries are NOT modelled — chirality is moot for boxes and
// pallets, and mixing rotation + reflection muddies the operational rule
// "if knowledge resolves orientation, the item is width-consistent for a
// pusher." Add when a chiral item actually appears.
struct RotationalSymmetry {
    struct Continuous {};
    struct Discrete { int period_deg; };
    struct Asymmetric {};
    using Kind = std::variant<Continuous, Discrete, Asymmetric>;
    Kind kind = Asymmetric{};
};

// Factories — the only sanctioned way to construct each variant. `discrete`
// validates that the period is a sensible rotation in degrees; the other
// two are trivial but exposed for parallel call-site readability.
namespace symmetry {

inline RotationalSymmetry continuous() {
    return RotationalSymmetry{RotationalSymmetry::Continuous{}};
}

inline RotationalSymmetry asymmetric() {
    return RotationalSymmetry{RotationalSymmetry::Asymmetric{}};
}

inline RotationalSymmetry discrete(int period_deg) {
    if (period_deg <= 0 || period_deg >= 360) {
        throw std::invalid_argument(
            "tinycell::core::symmetry::discrete: period_deg must be in (0, 360)");
    }
    return RotationalSymmetry{RotationalSymmetry::Discrete{period_deg}};
}

} // namespace symmetry

// ---------------------------------------------------------------------------
// OrientationKnowledge — DYNAMIC planner state.
// ---------------------------------------------------------------------------

// OrientationKnowledge tracks how finely the planner knows the item's
// orientation in the item's current local frame. Three states:
//   * Unknown        — no orientation information (continuous range of
//                      possible angles).
//   * Snapped{step}  — knowledge has reduced orientation to one of
//                      360/step discrete snap positions, but it's not yet
//                      known which one. Produced by passive measurement
//                      (camera) or forced placement (fixture).
//   * Exact          — orientation pinned to one specific angle.
//
// Construct Snapped via `orientation::snapped(step)`.
struct OrientationKnowledge {
    struct Unknown {};
    struct Snapped { int step_deg; };
    struct Exact {};
    using Kind = std::variant<Unknown, Snapped, Exact>;
    Kind kind = Unknown{};
};

namespace orientation {

inline OrientationKnowledge unknown() {
    return OrientationKnowledge{OrientationKnowledge::Unknown{}};
}

inline OrientationKnowledge exact() {
    return OrientationKnowledge{OrientationKnowledge::Exact{}};
}

inline OrientationKnowledge snapped(int step_deg) {
    if (step_deg <= 0 || step_deg >= 360) {
        throw std::invalid_argument(
            "tinycell::core::orientation::snapped: step_deg must be in (0, 360)");
    }
    return OrientationKnowledge{OrientationKnowledge::Snapped{step_deg}};
}

} // namespace orientation

// ---------------------------------------------------------------------------
// Property-aware helpers — the seam against type-switching if-trees.
// ---------------------------------------------------------------------------

// distinct_orientations: how many distinguishable physical orientations the
// item could be in, given the planner's current knowledge and the item's
// intrinsic symmetry.
//
//   * Item Continuous           → 1 (all rotations equivalent).
//   * Knowledge Exact           → 1 (one specific angle).
//   * Knowledge Unknown         → numeric_limits<int>::max() (continuous
//                                  range; even after symmetry, a continuous
//                                  range remains continuous — practically
//                                  "unbounded").
//   * Snapped(K) + Discrete(G)  → G / K. Assumes K | G; non-divisible
//                                  combinations are unusual in practice and
//                                  are not normalised here.
//   * Snapped(K) + Asymmetric   → 360 / K.
//
// Strategy predicates compose this helper rather than switching on
// RotationalSymmetry::Kind directly. The arm and pusher gates both reduce
// to `distinct_orientations(sym, k) == 1`.
inline int distinct_orientations(const RotationalSymmetry& sym,
                                 const OrientationKnowledge& k) {
    if (std::holds_alternative<RotationalSymmetry::Continuous>(sym.kind)) {
        return 1;
    }
    if (std::holds_alternative<OrientationKnowledge::Exact>(k.kind)) {
        return 1;
    }
    if (std::holds_alternative<OrientationKnowledge::Unknown>(k.kind)) {
        return std::numeric_limits<int>::max();
    }

    const int step =
        std::get<OrientationKnowledge::Snapped>(k.kind).step_deg;
    const int period =
        std::holds_alternative<RotationalSymmetry::Asymmetric>(sym.kind)
            ? 360
            : std::get<RotationalSymmetry::Discrete>(sym.kind).period_deg;
    return period / step;
}

// orientation_resolved: convenience predicate equivalent to
// distinct_orientations(sym, k) == 1. The arm and pusher gates both reduce
// to this check. Keep `distinct_orientations` exposed for future strategies
// that want a richer count (e.g., "≤ N classes is acceptable").
inline bool orientation_resolved(const RotationalSymmetry& sym,
                                 const OrientationKnowledge& k) {
    return distinct_orientations(sym, k) == 1;
}

// ---------------------------------------------------------------------------
// ItemPhysical — STATIC properties shared by every item type.
// ---------------------------------------------------------------------------

// ItemPhysical aggregates the physical fields every item type in the cell
// (Box, Pallet, and any future item types) carries.
//
// Composition, not inheritance: Box and Pallet remain distinct types that
// each contain an ItemPhysical, so PalletizeParams can express "the
// BoxSpec being placed" vs "the PalletSpec receiving it" without
// accidentally swapping the two roles at call sites. Growth: when a new
// item property is needed by some future strategy (surface, rigidity,
// chirality, …), it lands here once and everywhere that already had an
// ItemPhysical picks it up.
struct ItemPhysical {
    Length width;
    Length length;
    Length height;
    Mass mass;
    RotationalSymmetry symmetry;
};

// ---------------------------------------------------------------------------
// ItemKnowledge — DYNAMIC planner state.
// ---------------------------------------------------------------------------

// OnCarrier — which carrier class the item is currently resting on. Named
// by carrier CLASS, not by a specific equipment instance, because the
// propagation pass reasons about flow categories (a pusher needs items on
// a belt; an arm doesn't care which belt). The catalog binding identifies
// the instance.
enum class OnCarrier { Pallet, Belt, Free, Fixture };

// ItemKnowledge bundles the planner's tracked facts about ONE item flowing
// through the workflow. Mixed-content struct:
//   * pose-precision fields (position_known, orientation) are planner
//     BELIEFS about where the item is and how it's oriented;
//   * on_carrier is a planner-TRACKED-FACT (we placed it there; if it
//     falls off, the tracked fact diverges from reality — that's a
//     robustness issue, not a modelling one).
// The "Knowledge" name is the closer fit, with the understanding that
// the carrier field is part of what the planner *knows* rather than
// *infers*. A future ItemPlacement / PoseKnowledge split is a clean
// refactor when the mixing actually causes friction.
struct ItemKnowledge {
    bool position_known = false;
    OrientationKnowledge orientation;
    OnCarrier on_carrier = OnCarrier::Free;
};

} // namespace tinycell::core
