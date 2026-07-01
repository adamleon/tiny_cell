#pragma once

// AnchorStrategy — the trivial fulfiller of Anchor tasks. An Anchor
// is a workflow node pinned to a world pose (a feeder belt's start,
// a dispatch zone); it has no equipment, no cost, no energy. The
// strategy emits a single PortConstraint at the anchor's world pose
// with strict tolerance — Layer-3 treats Anchor stations as
// constants the placer cannot move.
//
// Stateless. One instance shared across all Anchor tasks in a
// workflow, like TransferStrategy.

#include <tinycell/solver/strategy.hpp>

namespace tinycell::solver {

class AnchorStrategy : public Strategy {
public:
    std::string_view name() const override;
    bool applies_to(const core::Task& task) const override;
    StrategyResult evaluate(const core::Task& task) const override;
};

} // namespace tinycell::solver
