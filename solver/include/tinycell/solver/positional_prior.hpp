#pragma once

// Workflow positional prior (solver.md "Workflow positional prior").
// Produces a nominal (x, y) per workflow task that the Layer-3 placer
// uses two ways — never as a hard constraint:
//   * Seed for NLP placement (non-convex; init guess matters a lot).
//   * Soft objective Σ ‖actual − nominal‖² with a small weight,
//     breaking symmetry, tiebreaking, making layouts legible.
//
// Phase C of step 5. **Minimum-viable shape for the current workflow
// model**: workflow is std::vector<Task> with no explicit predecessor
// edges yet (data-model.md §1 says "AND-OR graph, built lazily" but
// the in-code form is still a flat sequence). The prior interpolates
// tasks uniformly between input and output anchors along the I→O
// axis. When workflows with explicit edges arrive (parallel siblings,
// merges, splits), this signature grows extra inputs and the body
// gains the spring-model logic solver.md prescribes:
//   * sequential tasks interpolate along I→O axis by topological depth;
//   * parallel siblings offset perpendicular, spaced by footprint;
//   * merges/splits pull toward predecessor/successor centroids.
// No consumer needs that logic yet, and adding it would be
// pre-design against a workflow shape that doesn't exist
// (feedback_concrete_over_abstract).

#include <span>
#include <string>
#include <tinycell/geometry.hpp>
#include <tinycell/model/task.hpp>
#include <vector>

namespace tinycell::solver {

// Per-task nominal position. Coordinates are in the same frame the
// caller supplies anchors in — typically the world frame, since the
// Layer-3 NLP solves for poses in world coords.
struct NominalLayout {
    struct Entry {
        std::string task_id;
        core::Vec2 nominal;
    };
    std::vector<Entry> entries;
};

// Compute the positional prior. `input_anchor` and `output_anchor`
// are the pinned endpoints of the cell's material flow — typically a
// feeder belt and a dispatch zone in real installations. Tasks land
// evenly spaced between them; the first task gets t = 1/(N+1) of the
// way from input to output, the last gets t = N/(N+1), so neither
// end sits on top of an anchor.
//
// Empty workflow → empty layout.
NominalLayout positional_prior(std::span<const core::Task> workflow,
                               core::Vec2 input_anchor,
                               core::Vec2 output_anchor);

} // namespace tinycell::solver
