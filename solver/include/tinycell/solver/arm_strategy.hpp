#pragma once

// ArmStrategy — engineering knowledge for using a robotic arm to solve tasks
// of various kinds. Today it handles Palletize; Transport, Grip, Assemble
// will be added on this same class (one strategy per equipment type, not
// per (type × task) — decisions.md).
//
// Holds a non-owning view of the arm catalog. The catalog must outlive the
// strategy. std::span is a C++20 view = pointer + length, no ownership;
// passing the catalog by span makes it explicit that the strategy doesn't
// copy or mutate it.

#include <span>
#include <tinycell/model/arm.hpp>
#include <tinycell/solver/strategy.hpp>

namespace tinycell::solver {

class ArmStrategy : public Strategy {
public:
    explicit ArmStrategy(std::span<const core::ArmSpec> catalog);

    std::string_view name() const override;
    bool applies_to(const core::Task& task) const override;
    StrategyResult evaluate(const core::Task& task) const override;

private:
    std::span<const core::ArmSpec> catalog_;
};

} // namespace tinycell::solver
