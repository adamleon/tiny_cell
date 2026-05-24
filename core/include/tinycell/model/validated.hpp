#pragma once

// Validated value-type wrappers — layer on mp-units quantities with range
// invariants enforced at construction. See architecture.md §8 for the
// design and the decisions.md "Validated value types staged in at step 4"
// entry for why this layer first arrives at Layer 2 (the first point
// where solver code can emit a value that fails a range invariant). The
// current set covers fields Layer 2 touches; additional wrappers are
// added as new solver code consumes each invariant.
//
// Pattern: one wrapper per CONCEPT, not one per unit-quantity. An arm's
// payload capacity and an item's mass are both in kilograms but model
// different things — Payload is a capacity (the maximum it can lift);
// item mass stays as core::Mass. Same-unit fields with different
// invariants (today both "> 0", tomorrow perhaps tighter) deserve
// distinct wrappers so the invariant lives on the concept, not the unit.
//
// Construction throws std::invalid_argument on invariant violation. At
// the IO boundary the catalog loader's read_* helpers reject bad data
// before this constructor sees it (typed ParseError per engineering.md
// §3); the wrapper's throw is the belt-and-braces second line of defence
// against a solver bug. Solver-produced values reaching this layer with
// invalid content means the solver has a bug — fail loud, don't clamp.
//
// Reads return the underlying mp-units quantity via .value() — arithmetic
// and comparison happen on the raw quantity at the call site, where the
// "leaving the validated zone" boundary is visible.

#include <stdexcept>
#include <tinycell/units.hpp>

namespace tinycell::core {

class Payload {
    Mass value_;

public:
    explicit Payload(Mass v) : value_(v) {
        if (v.numerical_value_in(si::kilogram) <= 0.0) {
            throw std::invalid_argument(
                "tinycell::core::Payload must be > 0 kg");
        }
    }
    Mass value() const noexcept { return value_; }
};

class ReachRadius {
    Length value_;

public:
    explicit ReachRadius(Length v) : value_(v) {
        if (v.numerical_value_in(si::metre) <= 0.0) {
            throw std::invalid_argument(
                "tinycell::core::ReachRadius must be > 0 m");
        }
    }
    Length value() const noexcept { return value_; }
};

class CycleTimePerItem {
    Duration value_;

public:
    explicit CycleTimePerItem(Duration v) : value_(v) {
        if (v.numerical_value_in(si::second) <= 0.0) {
            throw std::invalid_argument(
                "tinycell::core::CycleTimePerItem must be > 0 s");
        }
    }
    Duration value() const noexcept { return value_; }
};

} // namespace tinycell::core
