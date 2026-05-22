#pragma once

// PusherStrategy — engineering knowledge for using a linear pusher to solve
// tasks of various kinds. Today it handles Palletize (push boxes from an
// in-feed onto a pallet, one box per stroke); future kinds a pusher
// legitimately applies to include PushOff and Transport-over-short-distance
// (decisions.md — one strategy per equipment type, not per (type × task)).
//
// Holds a non-owning view of the pusher catalog. The catalog must outlive
// the strategy. Same span semantics as ArmStrategy.

#include <span>
#include <tinycell/model/pusher.hpp>
#include <tinycell/solver/strategy.hpp>

namespace tinycell::solver {

class PusherStrategy : public Strategy {
public:
    explicit PusherStrategy(std::span<const core::PusherSpec> catalog);

    std::string_view name() const override;
    bool applies_to(const core::Task& task) const override;
    StrategyResult evaluate(const core::Task& task) const override;

private:
    std::span<const core::PusherSpec> catalog_;
};

} // namespace tinycell::solver
