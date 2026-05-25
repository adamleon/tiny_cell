#pragma once

// Port system — names the I/O channels of a task and the geometric
// realization a strategy proposes for them. Layered:
//
//   Task        declares LOGICAL ports (names + direction) via
//               task_ports(); the kind alone determines the set.
//   Strategy    emits PortConstraints in its StrategyResult; each
//               PortConstraint matches a LogicalPort by name and
//               supplies its concrete pose in the bound instance's
//               station frame + direction tolerance.
//   Transport   tasks (data-model.md, Phase T of step 5) connect a
//               source task's output port to a sink task's input
//               port; the Layer-3 placer scores transfer length and
//               checks direction tolerances at the connected ends.
//
// PortConstraint lives on StrategyResult (T.3); this header carries
// only the task-side logical declaration + the helper that derives
// it from a Task. Strategies refine — they don't redeclare ports the
// task didn't name.

#include <string>
#include <tinycell/model/task.hpp>
#include <vector>

namespace tinycell::core {

// Direction of item flow at a port: items either ENTER the task here
// (Input) or LEAVE the task here (Output). A Palletize task has both:
// item_in (boxes arrive), pallet_in (empty pallet arrives), pallet_out
// (full pallet leaves).
enum class PortDirection { Input, Output };

// LogicalPort — the task's named I/O channel. Geometry-free: the
// strategy that binds to the task supplies the concrete pose +
// direction tolerance via a PortConstraint with the same name.
struct LogicalPort {
    std::string name;
    PortDirection direction;
};

// Derive the logical port set for a task from its kind. Each task
// kind has a fixed port set (this is what makes ports "task-level" —
// they're a property of the goal shape, not of the equipment chosen
// to achieve it). Adding a new TaskKind requires a new case here.
//
// Palletize:
//   item_in     — boxes arrive (input)
//   pallet_in   — empty pallet arrives (input)
//   pallet_out  — full pallet leaves (output)
// The user's working note: pallet_in and pallet_out are typically at
// the same physical location with the same direction (a long belt
// passing through the station). The strategy emits both
// PortConstraints at the same pose; the placer treats them as two
// edges that happen to land in the same place — no merge logic
// required at MVP.
inline std::vector<LogicalPort> task_ports(const Task& t) {
    switch (t.kind()) {
    case TaskKind::Palletize:
        return {
            {"item_in",    PortDirection::Input},
            {"pallet_in",  PortDirection::Input},
            {"pallet_out", PortDirection::Output},
        };
    }
    // Unreachable if every TaskKind has a case above; if a new kind
    // is added without updating this switch, the compile-time
    // [[unreachable]] case below trips at runtime in debug.
    return {};
}

} // namespace tinycell::core
