#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <tinycell/model/task.hpp>
#include <tinycell/units.hpp>

namespace tinycell::solver {

enum class Feasibility { FULL, PARTIAL, INFEASIBLE };

// EquipmentRef names a specific catalog entry (its id). A strategy proposes
// this as a *candidate* binding, not a commitment — allocation (Layer 2) is
// what decides whether one physical instance is reused across tasks.
// data-model.md §2 — "candidate binding, not commitment".
struct EquipmentRef {
    std::string catalog_id;
};

struct StrategyResult {
    Feasibility feasibility;
    std::string strategy_name;
    std::optional<EquipmentRef> equipment;
    core::Energy energy_per_cycle;
    core::Duration cycle_time;
    // Deferred until step 2+: poses, requires_state, effect, preconditions, partial_info.
};

// Strategy interface — pure-virtual, named after the equipment type only
// (ArmStrategy, PusherStrategy, ...), per decisions.md.
//
// `applies_to(task)` is capability matching (data-model.md §2 — "matched by
// capability, not name"); a single strategy may cover multiple TaskKinds.
class Strategy {
public:
    virtual ~Strategy() = default;

    virtual std::string_view name() const = 0;
    virtual bool applies_to(const core::Task& task) const = 0;
    virtual StrategyResult evaluate(const core::Task& task) const = 0;
};

} // namespace tinycell::solver
