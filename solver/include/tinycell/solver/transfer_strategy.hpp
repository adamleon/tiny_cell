#pragma once

// TransferStrategy — the magical fulfiller of Transport tasks. No
// equipment, no capex, no energy: it simply records that items can
// move from the source port to the sink port. The Layer-3 placer
// reads the connected ports' poses + tolerances to score transfer
// length and check direction compatibility; the strategy itself
// contributes nothing geometric beyond the FULL feasibility verdict.
//
// **Why "magical" is the right MVP shape:** modelling belts as
// equipment with length-dependent capex/energy/footprint is a step-1-
// /step-4-style data + strategy addition — separate Segment catalog
// entries, length-aware motion model, allocator awareness of belt
// sharing across collinear transports. None of that is needed for
// Layer 3 to start producing a placement that respects connectivity
// and direction constraints. When real belts arrive, BeltStrategy
// (a future class) ALSO fulfils Transport tasks; the OR-tree picks
// whichever is cheaper, just like ArmStrategy vs PusherStrategy for
// Palletize today. TransferStrategy stays as the fallback when no
// real equipment is needed (e.g. zero-distance transports between
// co-located stations).
//
// Stateless — holds no catalog. One instance is shared across all
// Transport tasks in a workflow.

#include <tinycell/solver/strategy.hpp>

namespace tinycell::solver {

class TransferStrategy : public Strategy {
public:
    std::string_view name() const override;
    bool applies_to(const core::Task& task) const override;
    StrategyResult evaluate(const core::Task& task) const override;
};

} // namespace tinycell::solver
