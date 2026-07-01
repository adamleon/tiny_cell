#pragma once

// Item domain types — static physical properties of an item type
// (ItemPhysical), the planner's dynamic state about ONE item in flight
// (ItemState), and the symmetry + orientation types they rely on.
//
// **Architectural principle: properties, not types.** Strategies switch
// on item PROPERTIES via shared helpers (distinct_alignments,
// orientation_known, orientation_pinned, future graspable_by / …),
// never on item type tags. Adding a new shape becomes a different set
// of property values, not a new code path. See decisions.md.
//
// **Concept split — four things, often conflated:**
//   * ItemPhysical    — STATIC physical properties of an item TYPE
//                       (BoxSpec, PalletSpec). Owned by the spec.
//   * ItemState       — DYNAMIC planner-tracked facts about ONE item
//                       in flight: pose precision/control + carrier
//                       placement. Mutated by each strategy's effect.
//   * Sensor capability — what a strategy can OBSERVE about an item
//                       (camera writes ItemState.orientation.knowledge).
//   * Actor requirement + effect — what an actor needs to operate and
//                       how it changes state (arm needs orientation
//                       knowledge; pusher needs orientation control).
//
// **Knowledge vs Control — two axes of orientation.** The pair captures
// two operationally distinct facts. A camera changes Knowledge (passive
// observation). A fixture changes Control (physically pins the item).
// They can diverge: a rectangle observed by a camera (Knowledge known)
// can still rotate on the belt before the next station consumes it
// (Control free). The orientation predicates each strategy emits read
// only the axis its failure mode actually depends on:
//   * Arm-style strategies plan their motion from the planner's belief
//     and have end-effector compliance to absorb small drift — they
//     read Knowledge.
//   * Pusher-style strategies' strokes are open-loop — once contact
//     starts, no compensation — they require the item to be PHYSICALLY
//     held in the right alignment, reading Control.
//
// **Alignment** — a labelled symmetry-equivalence class of orientations.
// A rectangle has 2 alignments (long-axis-along-X / long-axis-along-Y);
// a square has 1 (all cardinal orientations equivalent under 4-fold
// symmetry); a cylinder has 1 (continuous symmetry makes every rotation
// equivalent). The alignment count comes from item symmetry × the snap
// grid; for MVP we assume a 90° cardinal snap grid (typical 4-jaw
// fixture). See distinct_alignments() and data-model.md §4.
//
// **Future:** an ItemInstance type will eventually carry an ItemState
// plus a spec reference. ItemState stays decoupled from spec types so
// instance-wrapping is mechanical when it arrives.

#include <stdexcept>
#include <tinycell/units.hpp>
#include <variant>
#include <vector>

namespace tinycell::core {

// ---------------------------------------------------------------------------
// RotationalSymmetry — STATIC property of an item type.
// ---------------------------------------------------------------------------

// RotationalSymmetry describes how the item maps to itself under rotation
// about its vertical axis (2D footprint case):
//   * Continuous       — every rotation maps to self (cylinder).
//   * Discrete{period} — smallest self-mapping rotation is period_deg:
//                          rectangle = 180; square = 90;
//                          triangle  = 120; hexagon = 60.
//                        Construct via symmetry::discrete(period).
//   * Asymmetric       — only the identity rotation (360°) maps to self.
//
// Reflection symmetries are NOT modelled — chirality is moot for boxes
// and pallets; add only when a chiral item appears.
struct RotationalSymmetry {
    struct Continuous {};
    struct Discrete { int period_deg; };
    struct Asymmetric {};
    using Kind = std::variant<Continuous, Discrete, Asymmetric>;
    Kind kind = Asymmetric{};
};

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
// distinct_alignments — symmetry quotient over a snap grid.
// ---------------------------------------------------------------------------

// distinct_alignments: how many symmetry-distinct orientations the item
// has at a given snap grid (default 90° = the typical 4-jaw cardinal
// grid). Counts equivalence classes of snap positions under the item's
// rotational symmetry group. Examples at 90° snap:
//   * Continuous → 1 (cylinder; all rotations equivalent)
//   * Square (Discrete(90)) → 1 (all 4 cardinals collapse)
//   * Rectangle (Discrete(180)) → 2 ({0°,180°} and {90°,270°})
//   * Hexagon (Discrete(60)) → 2 ({0°,180°} and {90°,270°})
//   * Triangle (Discrete(120)) → 4 (no cardinal pair shares an orbit)
//   * Asymmetric → 4 (no symmetry collapse)
//
// Used by strategies that enumerate alignments (e.g., PusherStrategy
// comparing layouts) and by orientation::known / control::constrained
// to validate the alignment index against the item's possible set.
inline int distinct_alignments(const RotationalSymmetry& sym, int snap_step_deg = 90) {
    if (std::holds_alternative<RotationalSymmetry::Continuous>(sym.kind)) {
        return 1;
    }
    if (snap_step_deg <= 0 || snap_step_deg >= 360 || 360 % snap_step_deg != 0) {
        throw std::invalid_argument(
            "tinycell::core::distinct_alignments: snap_step_deg must divide 360");
    }
    const int n_snaps = 360 / snap_step_deg;
    const int period =
        std::holds_alternative<RotationalSymmetry::Asymmetric>(sym.kind)
            ? 360
            : std::get<RotationalSymmetry::Discrete>(sym.kind).period_deg;
    std::vector<bool> seen(static_cast<std::size_t>(n_snaps), false);
    int orbits = 0;
    for (int i = 0; i < n_snaps; ++i) {
        if (seen[static_cast<std::size_t>(i)]) continue;
        ++orbits;
        for (int j = 0; j < n_snaps; ++j) {
            const int diff = (((j - i) * snap_step_deg) % 360 + 360) % 360;
            if (diff % period == 0) {
                seen[static_cast<std::size_t>(j)] = true;
            }
        }
    }
    return orbits;
}

// ---------------------------------------------------------------------------
// OrientationKnowledge — what the planner BELIEVES the item's alignment is.
// ---------------------------------------------------------------------------

// OrientationKnowledge tracks the planner's belief about which alignment
// the item is currently in. Two states for Phase 1:
//   * Unknown      — no observation; could be any alignment.
//   * Known{alignment} — observed (camera, prior knowledge) to be in a
//                        specific alignment, 0-indexed in the item's
//                        symmetry quotient.
//
// Phase 2 will add a partial-narrowing state (the "one of N alignments"
// case a sensor with limited resolving power produces). Not in MVP —
// no current sensor strategy produces partial narrowing.
struct OrientationKnowledge {
    struct Unknown {};
    struct Known { int alignment; };
    using Kind = std::variant<Unknown, Known>;
    Kind kind = Unknown{};
};

namespace knowledge {

inline OrientationKnowledge unknown() {
    return OrientationKnowledge{OrientationKnowledge::Unknown{}};
}

inline OrientationKnowledge known(int alignment) {
    if (alignment < 0) {
        throw std::invalid_argument(
            "tinycell::core::knowledge::known: alignment must be non-negative");
    }
    return OrientationKnowledge{OrientationKnowledge::Known{alignment}};
}

} // namespace knowledge

// ---------------------------------------------------------------------------
// OrientationControl — what's PHYSICALLY pinning the item's alignment.
// ---------------------------------------------------------------------------

// OrientationControl tracks whether the item is mechanically held in a
// specific alignment, independent of what the planner has observed. Two
// states for Phase 1:
//   * Free                — no physical constraint; the item can rotate.
//   * Constrained{alignment} — fixture / side-guides / snap mechanism
//                             physically pins the item to one alignment.
//
// Phase 2 will add a multi-alignment Constrained-set state (a generous
// fixture that allows two alignments without picking one). Not in MVP —
// fixtures we model pin one alignment.
struct OrientationControl {
    struct Free {};
    struct Constrained { int alignment; };
    using Kind = std::variant<Free, Constrained>;
    Kind kind = Free{};
};

namespace control {

inline OrientationControl free() {
    return OrientationControl{OrientationControl::Free{}};
}

inline OrientationControl constrained(int alignment) {
    if (alignment < 0) {
        throw std::invalid_argument(
            "tinycell::core::control::constrained: alignment must be non-negative");
    }
    return OrientationControl{OrientationControl::Constrained{alignment}};
}

} // namespace control

// ---------------------------------------------------------------------------
// ItemOrientation — the orientation axis of ItemState.
// ---------------------------------------------------------------------------

// ItemOrientation pairs the planner's belief (knowledge) with the
// physical constraint (control). Both axes are independent and tracked
// separately because different operations write each:
//   * A camera writes knowledge (and may, by inference from item
//     geometry + carrier context, leave control alone).
//   * A fixture writes control (and may, by inference from the
//     fixture's snap geometry vs the item's symmetry, also collapse
//     knowledge).
//   * A placement effect (palletize) writes BOTH — the act of placing
//     the item leaves it both observed-to-be and physically-at the
//     placement alignment.
// Strategy predicates read whichever axis their failure mode requires.
struct ItemOrientation {
    OrientationKnowledge knowledge;
    OrientationControl   control;
};

// ---------------------------------------------------------------------------
// Property-aware helpers — the seam against type-switching if-trees.
// ---------------------------------------------------------------------------

// orientation_known: does the planner's belief narrow the alignment to
// a single specific value? True trivially for Continuous symmetry (one
// alignment exists); true when knowledge is Known{…}; false on Unknown.
inline bool orientation_known(const RotationalSymmetry& sym,
                              const OrientationKnowledge& k) {
    if (std::holds_alternative<RotationalSymmetry::Continuous>(sym.kind)) {
        return true;
    }
    return std::holds_alternative<OrientationKnowledge::Known>(k.kind);
}

// orientation_pinned: is the item physically held in a single specific
// alignment? True trivially for Continuous symmetry (no class distinction
// to enforce); true when control is Constrained{…}; false on Free.
inline bool orientation_pinned(const RotationalSymmetry& sym,
                               const OrientationControl& c) {
    if (std::holds_alternative<RotationalSymmetry::Continuous>(sym.kind)) {
        return true;
    }
    return std::holds_alternative<OrientationControl::Constrained>(c.kind);
}

// knowledge_matches_alignment: does the planner's belief identify the
// specific required alignment? Continuous symmetry passes any
// requirement (no class distinction). Otherwise the knowledge must be
// Known{required_alignment}.
inline bool knowledge_matches_alignment(const RotationalSymmetry& sym,
                                        const OrientationKnowledge& k,
                                        int required_alignment) {
    if (std::holds_alternative<RotationalSymmetry::Continuous>(sym.kind)) {
        return true;
    }
    if (auto* known = std::get_if<OrientationKnowledge::Known>(&k.kind)) {
        return known->alignment == required_alignment;
    }
    return false;
}

// control_matches_alignment: is the item physically held at the specific
// required alignment? Continuous symmetry passes any requirement.
// Otherwise control must be Constrained{required_alignment}.
inline bool control_matches_alignment(const RotationalSymmetry& sym,
                                      const OrientationControl& c,
                                      int required_alignment) {
    if (std::holds_alternative<RotationalSymmetry::Continuous>(sym.kind)) {
        return true;
    }
    if (auto* con = std::get_if<OrientationControl::Constrained>(&c.kind)) {
        return con->alignment == required_alignment;
    }
    return false;
}

// ---------------------------------------------------------------------------
// ItemPhysical — STATIC properties shared by every item type.
// ---------------------------------------------------------------------------

// ItemPhysical aggregates the physical fields every item type in the
// cell (Box, Pallet, and any future item types) carries. Composed by
// BoxSpec and PalletSpec — they remain distinct types so
// PalletizeParams can express "the BoxSpec being placed" vs "the
// PalletSpec receiving it" without accidentally swapping the roles.
struct ItemPhysical {
    Length width;
    Length length;
    Length height;
    Mass mass;
    RotationalSymmetry symmetry;
};

// ---------------------------------------------------------------------------
// ItemState — DYNAMIC planner state.
// ---------------------------------------------------------------------------

// OnCarrier — which carrier class the item is currently resting on.
// Named by carrier CLASS, not by a specific equipment instance — the
// propagation pass reasons about flow categories (a pusher needs items
// on a belt; an arm doesn't care which belt). `Free` here means "not
// on any specific carrier" (distinct from OrientationControl::Free,
// which means "not rotationally constrained"). The two never appear
// in the same scope ambiguously thanks to enum qualification.
enum class OnCarrier { Pallet, Belt, Free, Fixture };

// ItemState bundles the planner's tracked facts about ONE item flowing
// through the workflow. Mixed-content struct:
//   * position_known — coarse pose-precision flag for translation.
//     Phase 2 extends to per-DOF knowledge/control for x and y.
//   * orientation    — knowledge + control axes (this file).
//   * on_carrier     — which carrier class the planner is tracking.
struct ItemState {
    bool position_known = false;
    ItemOrientation orientation;
    OnCarrier on_carrier = OnCarrier::Free;
};

} // namespace tinycell::core
