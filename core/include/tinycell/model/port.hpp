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

#include <optional>
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

// PortConstraint — a strategy's concrete realization of a logical
// port for the specific binding it proposes. Lives on
// StrategyResult.port_constraints; the Layer-3 placer (Phase D) reads
// it to position Transport endpoints and check direction tolerances.
//
// Coordinates are in the BOUND INSTANCE'S STATION FRAME (the instance
// sits at some pose in that frame; for the one-instance-per-station
// pattern current at MVP, that frame's origin is the instance's
// origin). theta names the direction items flow through this port at
// the moment they cross it. direction_tolerance bounds how much the
// connected transport's direction may deviate from theta — 0 means
// strict alignment (e.g. pusher stroke is fixed), pi/2 means any
// direction within ±90° works (e.g. an arm can pick from anywhere
// within its reach).
// Reach annulus (step-6 M1). A port is one of two kinds, distinguished
// by whether reach_max is set:
//   * POINT (reach_max == nullopt) — welded at (x, y) in the station
//     frame, exactly as before M1. Every pre-M1 emitter is a Point by
//     default-init: it sets x/y/theta/tolerance and leaves the optionals
//     unset. Pusher and pallet ports stay Points.
//   * ANNULUS (reach_max set) — the port may be PLACED ANYWHERE in the
//     ring [reach_min, reach_max] around the station origin; the Layer-3
//     placer treats its (x, y) as a variable constrained to that band
//     (an arm's item_in port can pick from anywhere in the reach
//     envelope). (x, y) then carries the SEED/initial port position, not
//     a fixed weld. reach_min unset is treated as 0 (a disc, not a true
//     annulus) when reach_max is given.
// This deliberately does NOT introduce a Point/Annulus/Polygon variant
// taxonomy: the single-palletizer scenario needs exactly one variable-
// port kind, so the optional-annulus form is the crudest thing that
// works (roadmap.md M1 "concepts under discussion"; CLAUDE.md
// crudest-concrete-first). Promote to a variant only when a second
// region shape (e.g. a polygon intersection) earns it.
struct PortConstraint {
    std::string port_name;        // matches a LogicalPort.name on the task
    Length x;
    Length y;
    Angle theta;                  // port "flow direction" in the station frame
    Angle direction_tolerance;    // 0 = strict; pi/2 = any-direction-within-reach
    std::optional<Length> reach_min;  // annulus inner radius; unset => 0 when reach_max set
    std::optional<Length> reach_max;  // set => ANNULUS (variable); unset => POINT (welded at x,y)
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
    case TaskKind::Transport:
        // Transport is an EDGE between two other tasks' ports - it
        // doesn't expose its own logical ports. Its TransportParams
        // already names the source/sink endpoints by reference.
        return {};
    case TaskKind::Anchor:
        // Anchor declares ONE port named "port" with the role from
        // its AnchorParams (Output for a feeder, Input for a
        // dispatch). The Transport that connects to this anchor
        // references it by source/sink port name "port".
        return {{"port", std::get<AnchorParams>(t.params).role}};
    }
    // Unreachable if every TaskKind has a case above; if a new kind
    // is added without updating this switch, the compile-time
    // [[unreachable]] case below trips at runtime in debug.
    return {};
}

} // namespace tinycell::core
