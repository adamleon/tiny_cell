#pragma once

#include <span>
#include <tinycell/model/arm.hpp>
#include <tinycell/solver/strategy.hpp>

namespace tinycell::solver {

// ArmStrategy: a robotic arm + (implicit) gripper. Covers Palletize today;
// Transport, Grip, Assemble will follow as those task kinds are added — the
// strategy is named for the equipment type, not the task (decisions.md).
//
// Takes a non-owning view of the arm catalog (std::span is a C++20 view —
// pointer + size, no ownership). The catalog must outlive the strategy.
class ArmStrategy : public Strategy {
public:
    explicit ArmStrategy(std::span<const core::ArmEntry> catalog);

    std::string_view name() const override;
    bool applies_to(const core::Task& task) const override;
    StrategyResult evaluate(const core::Task& task) const override;

private:
    std::span<const core::ArmEntry> catalog_;
};

} // namespace tinycell::solver
