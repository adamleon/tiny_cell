#pragma once

// Item domain types — the dynamic state of an item as it flows through the
// workflow. Distinct from physical-spec types like BoxSpec (a *kind* of item):
// this file holds the state that changes during the workflow (where the item
// is, what is known about its pose), independent of the item's physical
// type. When more item kinds appear (bottle, cylinder, …), their physical
// specs live in their own headers; ItemState applies to all of them.
//
// Mutated by each chosen strategy's `effect`; checked against each strategy's
// `requires_state` at the node where that strategy sits (data-model.md §4).

namespace tinycell::core {

// OnCarrier — what the item is currently resting on. Names the *carrier
// class*, not a specific equipment instance, because the propagation pass
// reasons about flow categories (a pusher needs items on a belt; an arm
// doesn't care which belt). The catalog binding identifies the instance.
enum class OnCarrier { Pallet, Belt, Free, Fixture };

// OrientationKind — how precisely the item's rotation is currently
// resolved (data-model.md §4):
//   * Unknown    — no orientation information
//   * Continuous — any angle is acceptable; the item is rotationally
//                  symmetric for the purposes of downstream tasks
//   * Discrete   — one of `discrete_n` equally spaced positions (e.g. a
//                  square box, n=4)
//   * Exact      — full pose known to instrument precision
//
// Modelled as kind + discrete_n rather than std::variant<…> because the
// arity is small, fixed, and rarely matched on. Symmetry-aware matching
// (data-model.md §4.1) — comparing what a task *needs* resolved against
// what symmetry collapses — is deferred (PLACEHOLDER below).
enum class OrientationKind { Unknown, Continuous, Discrete, Exact };

// OrientationResolution — the orientation field of ItemState. `discrete_n`
// is meaningful only when `kind == Discrete`; ignored otherwise.
struct OrientationResolution {
    OrientationKind kind = OrientationKind::Unknown;
    int discrete_n = 0;
};

// ItemState — dynamic flow state of an item. Strategies declare a
// `requires_state` predicate and an `effect` (ItemState → ItemState); the
// state propagation pass walks the workflow, applies each effect after a
// task, and checks the next task's requires_state against the resulting
// state (data-model.md §4).
//
// PLACEHOLDER (step 4): symmetry-aware orientation matching (§4.1) needs
// ItemSymmetry on BoxSpec — deferred so step 3 doesn't balloon scope.
// `orientation_resolved_to` is carried here so the field doesn't appear
// later, but step-3 state propagation does NOT match on it; strategies
// emit a default OrientationResolution and the propagator ignores it.
struct ItemState {
    bool position_known = false;
    OrientationResolution orientation_resolved_to{};
    OnCarrier on_carrier = OnCarrier::Free;
};

} // namespace tinycell::core
