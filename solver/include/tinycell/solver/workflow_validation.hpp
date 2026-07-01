#pragma once

// Pre-solve workflow validation (step-6 M3). Catches port-mismatch and
// unconnected-input failure modes BEFORE enumeration/placement, so a
// malformed workflow fails early with an actionable diagnostic instead of
// silently producing a layout with missing material edges (roadmap.md
// step-6 M3; decisions.md "Concrete-vs-abstract interpretation correction"
// — validate_workflow lands in M3, not deferred to the GUI).
//
// Scope: WORKFLOW TOPOLOGY only — that every task's logical ports
// (core::task_ports) are wired consistently by the workflow's Transport
// tasks. It does NOT touch geometry, equipment, or feasibility (those are
// the enumerator / allocator / placer's job). It runs on the raw Task
// list, before any strategy is consulted.
//
// Checks (the failure modes M3 must catch — roadmap.md "port-mismatch /
// unconnected-input"):
//   * duplicate task ids — Error (references become ambiguous).
//   * a Transport whose source and sink are the SAME task — Error (a
//     self-loop is nonsensical and, uncaught, would mark a task's own
//     input as fed by its own output, masking a genuine unfed input).
//   * every Transport's source/sink names an existing task — Error.
//   * source port exists on the source task and is an OUTPUT; sink port
//     exists on the sink task and is an INPUT — Error (a back-to-front or
//     mis-named edge; port.hpp documents the polarity rule).
//   * every INPUT logical port on every task is fed by >=1 Transport —
//     Error (an unfed input means nothing arrives there: the canonical
//     "pallet_in with no empty-pallet source" gap the HANDOFF flagged).
//   * every OUTPUT logical port on every task is drained by >=1 Transport
//     — Warning (produced material with nowhere to go; the cell still
//     runs — e.g. a feeder whose output is unused, or a pallet_out with
//     no dispatch).
//
// Severity split follows engineering.md §3: this REPORTS authored-workflow
// problems (workflows are C++ literals in demos at MVP, not yet io/-loaded
// — an io/-layer parse check is the eventual home once a JSON workflow
// loader lands). It never clamps or fabricates the missing edge. The demo
// prints the issues and refuses to solve when any Error is present.

#include <cstddef>
#include <span>
#include <string>
#include <tinycell/model/task.hpp>
#include <vector>

namespace tinycell::solver {

// Error blocks the solve (the workflow cannot produce a usable layout);
// Warning is advisory (the layout is producible but something is likely
// missing).
enum class WorkflowIssueSeverity { Error, Warning };

// One diagnostic. `task_id` names the task the issue concerns ("" for a
// cross-cutting issue like a duplicate-id report). `message` is the
// human-readable explanation, already including the relevant ids so the
// caller can print it verbatim.
struct WorkflowIssue {
    WorkflowIssueSeverity severity;
    std::string task_id;
    std::string message;
};

// Result of validate_workflow(). `issues` is in discovery order
// (duplicate-id and transport-reference checks first, then per-port
// connectivity). `ok()` is the gate the caller checks before solving.
struct WorkflowValidation {
    std::vector<WorkflowIssue> issues;

    bool ok() const;                 // no Error-severity issue present
    std::size_t error_count() const;
    std::size_t warning_count() const;
};

// Validate one workflow's port wiring. Pure: reads only the Task list and
// core::task_ports(); no catalogs, no solver state.
WorkflowValidation validate_workflow(std::span<const core::Task> workflow);

} // namespace tinycell::solver
