#pragma once

// Workflow-diagram SVG (post-MVP sales demo, Phase 1b). Renders a workflow
// (vector<Task>) as a left-to-right DAG: source/feed anchors on the left,
// palletize cells in the middle, dispatch anchors on the right; Transport
// tasks become the directed edges, labelled "source_port -> sink_port".
//
// This visualizes the INPUT workflow — the export the demo shows in-app and
// attaches to emails/docs (ask #2), NOT solver-output geometry. Stdlib-only
// like the geometry Writer in svg.hpp, but a separate emitter: a task DAG is
// abstract diagram space (screen pixels, Y-down), not the meter / Y-flip space
// svg::Writer serves. Authorized as the input-side exception in the svg/ row of
// CLAUDE.md §0.

#include <filesystem>
#include <string>
#include <tinycell/model/task.hpp>
#include <vector>

namespace tinycell::svg {

// Renders `workflow` to a self-contained SVG document string.
std::string render_workflow_svg(const std::vector<core::Task>& workflow);

// Same, written to `path`. Throws std::runtime_error if the file can't be
// opened (this is a demo/debug artifact, not authored input — no ParseError).
void write_workflow_svg(const std::vector<core::Task>& workflow,
                        const std::filesystem::path& path);

} // namespace tinycell::svg
