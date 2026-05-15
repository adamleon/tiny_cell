#pragma once
#include <string>
#include <vector>
#include "pose_component.hpp"

// Station-solver data types — the shared vocabulary used by strategies, the
// solver search loop, and downstream consumers. v0 covers the type ontology
// only; the Strategy interface and the search algorithm land in following
// steps.
//
// Architecture summary (for context — see docs/STATION_SOLVER.md when
// written): a station is an abstract task ("palletize N items/hr") realised
// by a *tree* of strategy choices. Each strategy returns Proposals — partial
// solutions carrying accumulated equipment, accumulated cost, and (often) a
// list of unresolved sub-tasks the solver must further dispatch. A terminal
// Proposal (no remaining tasks) is a complete sub-tree. The cheapest
// complete tree that satisfies the root task is the chosen design.
//
// This is, structurally, AND-OR graph search: each Task is an OR node
// (multiple strategies can solve it); each strategy's `remaining` list is
// an AND node (all conditions must be resolved for the Proposal to be
// valid). Branch-and-bound search lives in the (future) solver loop.

namespace factory::station_solver {

// ── Task ontology ───────────────────────────────────────────────────────────
//
// Small starting set. Expand as new strategies need new task kinds. Existing
// strategies don't care about kinds they don't produce or consume.

enum class TaskKind {
    Palletize,           // root task: produce stacked pallets at a given rate
    TransportItem,       // move an item from A to B
    DetectItemPresence,  // sense when an item is at a given location
    GripItem,            // physically grasp an item for transport
    QueueItems,          // accumulate items in a queue prior to batch transfer
    IndexPallet,         // step the pallet by a fixed distance (between rows / layers)
};

// All Task parameters in one struct. Only fields relevant to `kind` are
// populated; the rest stay at zero/default. "By-key" representation chosen
// for v0 over a tagged-union — simpler to extend, no variant boilerplate.
// If the field count grows unwieldy we'll split per-kind later.
struct Task {
    TaskKind kind = TaskKind::Palletize;

    // Shared across kinds
    float throughput_items_per_minute = 0.f;

    // Palletize
    int   pallet_length_mm    = 0;
    int   pallet_width_mm     = 0;
    int   pallet_max_stack_mm = 0;
    int   box_length_mm       = 0;
    int   box_width_mm        = 0;
    int   box_height_mm       = 0;
    float box_mass_kg         = 0.f;

    // TransportItem / DetectItemPresence / GripItem
    Vec3  source_position{0.f};
    Vec3  destination_position{0.f};
    float item_mass_kg        = 0.f;

    // QueueItems
    int   queue_capacity      = 0;

    // IndexPallet
    int   index_step_mm       = 0;
};

// ── Proposal data ───────────────────────────────────────────────────────────

// One piece of equipment in a proposed solution. Strings keep this
// representation strategy-agnostic — the solver only needs to sum cost and
// (later) check footprints. Per-equipment-type details stay inside the
// strategy that emitted the proposal.
struct ProposalEquipment {
    std::string archetype_name;     // "robot_arm", "belt", "pusher", "sensor", "station_frame"
    std::string model_or_variant;   // "KR10_R1100_2", "generic_2m_belt", "pneumatic_500mm_pusher", ...
    float       capex_eur = 0.f;
    float       power_w   = 0.f;
};

// What a strategy returns for a Task. Carries accumulated equipment + cost
// + a list of unresolved sub-tasks. A *terminal* Proposal has
// `remaining.empty()` — it's a complete sub-tree. The solver only commits to
// a Proposal when (a) it's terminal and (b) it's the cheapest complete tree
// among all open partial solutions.
//
// Cost semantics:
//   annual_cost_eur     — sum of `cost::estimate` results for own equipment,
//                         plus any costs already resolved from sub-trees
//   annual_cost_lb_eur  — own cost + minimum-possible-remaining cost; used by
//                         branch-and-bound to prune. For terminal proposals,
//                         this equals annual_cost_eur.
struct Proposal {
    TaskKind                       solves          = TaskKind::Palletize;
    float                          annual_cost_eur    = 0.f;
    float                          annual_cost_lb_eur = 0.f;
    std::vector<ProposalEquipment> equipment;
    std::vector<Task>              remaining;
    std::string                    strategy_name;
};

inline bool is_terminal(const Proposal& p) { return p.remaining.empty(); }

}  // namespace factory::station_solver
