#pragma once

// Workflow loader — parses a workflow JSON file into the vector<Task> the
// solver consumes (validate_workflow / enumerate take std::span<const Task>).
// Mirrors the per-category catalog loaders: validate at the parse boundary,
// reject authored bad data with a source-locating ParseError, never fabricate
// (engineering.md §3; CLAUDE.md io/ row). Added for the post-MVP sales demo so
// scenarios are authored as data (assets/workflow/*.json) instead of C++
// literals in demos/solve_workflow.
//
// Schema — a top-level object with a "workflow" array of task objects. Every
// task has "id", "kind" (palletize | anchor | transport), and
// "target_ct_per_item_s"; the rest is kind-specific and mirrors the make_*
// helpers in demos/solve_workflow/main.cpp:
//
//   palletize:  item_id, box_count, and item/pallet objects each with
//               width_m,length_m,height_m,mass_kg and a symmetry object
//               ({kind: continuous|asymmetric} or {kind: discrete, period_deg})
//   anchor:     name, role ("input"|"output"), world_x_m, world_y_m,
//               world_theta_deg
//   transport:  source_task_id, source_port_name, sink_task_id, sink_port_name
//
// Units follow the catalog convention: lengths metres (_m), masses kilograms
// (_kg), cycle time seconds (_s); angles are DEGREES (_deg) and converted to
// radians here at the boundary (units.hpp: degree→radian conversion lives at
// io/, never in core/ or solver/).

#include <filesystem>
#include <tinycell/model/task.hpp>
#include <vector>

namespace tinycell::io {

// Loads a workflow from JSON at `path`. Validates shape + range invariants at
// parse time and throws ParseError with a source-locating message on any
// violation (missing/mis-typed field, unknown task kind, unknown symmetry kind
// or anchor role, non-positive dimension/count). Tasks are returned in file
// order (the solver does not depend on order, but stable order keeps diffs and
// diagnostics readable).
std::vector<core::Task> load_workflow(const std::filesystem::path& path);

} // namespace tinycell::io
