#pragma once

// Task domain types — the OR-nodes of the AND-OR graph the solver builds
// (data-model.md §1). A Task names a *goal*, never the equipment used to
// solve it (decisions.md — strategy-class reuse). Today only the Palletize
// kind exists; new kinds are added by appending an alternative to TaskParams
// and a case to Task::kind().

#include <stdexcept>
#include <string>
#include <tinycell/model/box.hpp>
#include <tinycell/model/pallet.hpp>
#include <tinycell/model/validated.hpp>
#include <tinycell/units.hpp>
#include <variant>

namespace tinycell::core {

// PalletizeParams — "place box_count boxes of `item` onto a `pallet`". The
// stacking pattern (grid only at step 1) is derived from box + pallet
// dimensions; it isn't a parameter so different equipment can choose
// different patterns from the same task.
struct PalletizeParams {
    std::string item_id;
    BoxSpec item;
    PalletSpec pallet;
    int box_count;
};

using TaskParams = std::variant<PalletizeParams>;

// Discriminator mirroring TaskParams' alternatives. Add a value here AND a
// case to Task::kind() in lockstep when a new task kind is introduced.
enum class TaskKind { Palletize };

// Task — one goal the solver must produce a strategy for. `params` carries
// the kind-specific data; `kind()` reports the discriminator (useful for
// switch statements in strategies that handle multiple kinds).
//
// `target_ct_per_item` is the per-item cycle-time target a strategy must
// meet to return FULL feasibility (a strategy slower than this returns
// PARTIAL, with the residual throughput emitted as a child task — see
// decisions.md "Per-task throughput target"). It is a workflow-phase
// input: the customer specifies a whole-cell rate (e.g. "100 pallets/h"),
// the workflow translator converts that to per-task per-item seconds
// (e.g. 24 boxes × 100 pallets/h = 2400 boxes/h → 1.5 s/box) and writes
// it here. The solver itself never allocates cycle time across stations;
// it only consumes this target and explores the OR-tree to meet it.
struct Task {
    std::string id;
    TaskParams params;
    CycleTimePerItem target_ct_per_item;

    TaskKind kind() const {
        if (std::holds_alternative<PalletizeParams>(params)) {
            return TaskKind::Palletize;
        }
        // Reachable only if TaskParams gets a new alternative without a
        // matching case here — guard against the mistake.
        throw std::logic_error("tinycell::core::Task::kind(): unknown variant alternative");
    }
};

} // namespace tinycell::core
