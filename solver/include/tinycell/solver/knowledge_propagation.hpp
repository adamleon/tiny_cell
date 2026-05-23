#pragma once

// Item-knowledge propagation — BATCH VALIDATOR for a finished candidate
// solution. Walks a linear workflow in declaration order, checks each
// chosen strategy's `requires_knowledge` against the running knowledge,
// applies its `effect`, and returns either a full per-task trajectory
// (success) or the first violation (failure). The knowledge-flow rules
// themselves are in data-model.md §4.
//
// ROLE — read before adding callers. This file is the *batch validator*,
// not the primitive a live solver consumes. The reusable knowledge-flow
// primitive lives on `StrategyResult`: the `requires_knowledge` predicate
// and `effect` function (strategy.hpp). A knowledge-aware solver threads
// knowledge inline through its own per-task loop, using those two fields
// directly — `knowledge` is *input to strategy selection* at each task,
// not something to validate after the whole candidate is built. So
// `propagate_knowledge` is for the cases where something OTHER than a
// knowledge-aware solver produced the candidate: late-stage validation of
// an imported / relaxed / heuristically-built solution, the demo and
// tests for this layer, and a clean diagnostic surface for the user. See
// decisions.md "State-flow primitive … is load-bearing; the batch walker
// … is a validator-only role." Will revisit when step 4 lands a
// knowledge-aware consumer and we know whether the validator role still
// has callers.
//
// No recovery-task spawning at step 3 — a violation FAILs the pass.
// Recovery belongs with the AND-OR walker, which doesn't exist yet; the
// brute-force enumerator is a flat per-task loop and step 3 stays
// compatible with that shape.
//
// Out of scope: parallel branches, DAG edges, PARTIAL feasibility. Add
// when a demo first needs them.

#include <cstddef>
#include <span>
#include <string>
#include <tinycell/model/item.hpp>
#include <tinycell/model/task.hpp>
#include <tinycell/solver/strategy.hpp>
#include <variant>
#include <vector>

namespace tinycell::solver {

// PropagationSuccess — the workflow walked end-to-end without a
// requires_knowledge violation. `trajectory[i]` is the inbound knowledge
// to task i (the knowledge its requires_knowledge was checked against).
// `trajectory.back()` is the final outbound knowledge after the last
// task's effect. Size is always `workflow.size() + 1`.
struct PropagationSuccess {
    std::vector<core::ItemKnowledge> trajectory;
};

// PropagationFailure — one task's requires_knowledge returned false.
// Identifies the failing task's index in the workflow, the strategy name
// whose predicate rejected, and the inbound knowledge that failed. The
// caller can diff `inbound_knowledge` against what the strategy needs to
// diagnose the gap. No partial trajectory is returned — the value of the
// failure surface is "this is where it broke," not "here's how far we
// got."
struct PropagationFailure {
    std::size_t task_index;
    std::string strategy_name;
    core::ItemKnowledge inbound_knowledge;
};

using PropagationResult = std::variant<PropagationSuccess, PropagationFailure>;

// propagate_knowledge(): walk the workflow linearly; check each chosen
// strategy's requires_knowledge against the running knowledge; apply its
// effect to produce the knowledge the next task observes. Returns the
// first violation as PropagationFailure, or the full trajectory as
// PropagationSuccess.
//
//   * workflow — tasks in execution order.
//   * chosen   — one StrategyResult per task, in workflow order. Typically
//                the per-task winners from the brute-force enumerator,
//                hoisted from `TaskEnumeration::proposals[winner_index]`.
//                Must satisfy `chosen.size() == workflow.size()`.
//                Each StrategyResult MUST carry non-empty
//                requires_knowledge + effect — an empty std::function
//                here is a defect in the caller's selection logic
//                (engineering.md §3), not a user-visible failure mode,
//                and the propagator throws on it rather than fabricate
//                identity behavior.
//   * initial  — user-declared initial item knowledge (data-model.md §4).
//
// Throws std::invalid_argument when workflow.size() != chosen.size().
// Throws std::logic_error when a chosen result has empty
// requires_knowledge or effect, or when chosen[i] is null.
PropagationResult propagate_knowledge(std::span<const core::Task> workflow,
                                      std::span<const StrategyResult* const> chosen,
                                      const core::ItemKnowledge& initial);

} // namespace tinycell::solver
