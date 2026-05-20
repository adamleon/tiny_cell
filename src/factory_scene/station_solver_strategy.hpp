#pragma once
#include <vector>
#include "cost_model.hpp"
#include "robot_arm_catalog.hpp"
#include "station_solver_types.hpp"

// Station-solver strategy interface — Phase 1.
//
// A `Strategy` proposes one or more `Proposal`s for a given `Task`. Each
// proposal carries equipment + cost + any unresolved sub-tasks. The solver
// (future Phase 2) iterates over strategies, dispatches proposals, and
// composes complete solutions via search.
//
// `StrategyContext` is the shared bundle every strategy receives — catalog +
// external (per-analysis) economic parameters. Strategy-specific defaults
// (e.g., palletizer station-mechanism cost) live on the concrete strategy
// class as constructor-injected configuration, not in the shared context.

namespace factory::station_solver {

// Shared dependencies passed to every Strategy::propose() call.
// Caller owns the catalog; the context holds a reference.
struct StrategyContext {
    const std::vector<robot_arm_catalog::RobotArmSpec>& catalog;
    cost::ExternalParams                                external_params;
};

class Strategy {
public:
    virtual ~Strategy() = default;
    virtual const char* name() const = 0;
    virtual bool can_solve(TaskKind kind) const = 0;
    virtual std::vector<Proposal> propose(const Task& task,
                                          const StrategyContext& ctx) const = 0;
};

}  // namespace factory::station_solver
