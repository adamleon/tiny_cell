#pragma once

// PusherStrategy — engineering knowledge for using a linear pusher to solve
// tasks of various kinds. Today it handles Palletize using the ROW-PER-STROKE
// model (items accumulate on an in-feed belt against an end stop until a full
// row is queued, then one stroke transfers the whole row sideways onto the
// pallet; the pallet belt advances perpendicular by one box width per row).
// NOT one-box-per-stroke — that earlier model made pushers indistinguishable
// from arms and was replaced; see decisions.md "PusherStrategy models
// row-per-stroke palletizing, not box-per-stroke" and pusher_strategy.cpp for
// the durable spec. Future kinds a pusher legitimately applies to include
// PushOff and Transport-over-short-distance (decisions.md — one strategy per
// equipment type, not per (type × task)).
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
