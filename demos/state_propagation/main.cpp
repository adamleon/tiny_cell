// demo_state_propagation — item-state propagation pass (roadmap.md step 3).
//
// Demonstrates: walking a workflow forward through ItemState, checking each
// chosen strategy's requires_state against the running state and applying
// its effect afterward (data-model.md §4). Three scenarios show the three
// outcomes the pass can produce:
//
//   * Scenario A (FAIL at start)   — initial state lacks what the first
//     task's strategy needs; the pass FAILs at task 0 with a clear
//     diagnostic.
//   * Scenario B (clean run)       — initial state satisfies the chosen
//     strategy; the pass returns a full state trajectory.
//   * Scenario C (FAIL mid-flow)   — a multi-task workflow where the
//     PRIOR task's effect breaks the next task's precondition. This is
//     the case that field-by-field initial-state checking cannot catch —
//     it requires the propagator.
//
// Success looks like: three labelled scenario blocks, each printing the
// initial state, the strategy chosen per task, and either a SUCCESS
// trajectory (one ItemState per node + a final outbound) or a FAIL line
// naming the violating task and the state at that node.

#include <filesystem>
#include <iostream>
#include <mp-units/systems/si.h>
#include <span>
#include <string>
#include <tinycell/io/catalog_loader.hpp>
#include <tinycell/model/item.hpp>
#include <tinycell/model/task.hpp>
#include <tinycell/solver/arm_strategy.hpp>
#include <tinycell/solver/pusher_strategy.hpp>
#include <tinycell/solver/state_propagation.hpp>
#include <variant>
#include <vector>

namespace {

using namespace mp_units;
namespace tc = tinycell::core;
namespace ts = tinycell::solver;

const char* on_carrier_str(tc::OnCarrier c) {
    switch (c) {
        case tc::OnCarrier::Pallet:  return "PALLET";
        case tc::OnCarrier::Belt:    return "BELT";
        case tc::OnCarrier::Free:    return "FREE";
        case tc::OnCarrier::Fixture: return "FIXTURE";
    }
    return "?";
}

void print_state(const tc::ItemState& s) {
    std::cout << "{ position_known="
              << (s.position_known ? "true " : "false")
              << " on_carrier=" << on_carrier_str(s.on_carrier) << " }";
}

// Build a Palletize task — small box, standard EUR pallet. Task params are
// not what the propagation demo exercises (strategies match by capability,
// not task shape); same shape across all three scenarios keeps the
// printout focused on the state flow.
tc::Task make_palletize(const std::string& id) {
    return tc::Task{
        .id = id,
        .params = tc::PalletizeParams{
            .item_id = "box",
            .item = tc::BoxSpec{.width = 0.3 * si::metre,
                                .length = 0.4 * si::metre,
                                .height = 0.2 * si::metre,
                                .mass = 5.0 * si::kilogram},
            .pallet = tc::PalletSpec{.width = 1.2 * si::metre,
                                     .length = 0.8 * si::metre},
            .box_count = 24,
        },
    };
}

void print_outcome(const ts::PropagationResult& result,
                   std::span<const tc::Task> workflow) {
    if (const auto* success = std::get_if<ts::PropagationSuccess>(&result)) {
        std::cout << "  SUCCESS — trajectory of "
                  << success->trajectory.size() << " state(s):\n";
        for (std::size_t i = 0; i < success->trajectory.size(); ++i) {
            std::cout << "    state[" << i << "] = ";
            print_state(success->trajectory[i]);
            if (i < workflow.size()) {
                std::cout << "  in to '" << workflow[i].id << "'";
            } else {
                std::cout << "  final outbound";
            }
            std::cout << "\n";
        }
    } else {
        const auto& failure = std::get<ts::PropagationFailure>(result);
        std::cout << "  FAIL — task[" << failure.task_index << "] '"
                  << workflow[failure.task_index].id
                  << "' rejected by " << failure.strategy_name << "\n";
        std::cout << "         inbound state was ";
        print_state(failure.inbound_state);
        std::cout << "\n";
    }
}

// run_scenario(): wire the workflow + chosen strategies + initial state
// through propagate_state and print the outcome. The caller picks the
// winners explicitly per scenario (not through the brute-force enumerator)
// so the demo controls which contract gets exercised in each block.
void run_scenario(const std::string& label,
                  std::span<const tc::Task> workflow,
                  std::span<const ts::StrategyResult* const> chosen,
                  const tc::ItemState& initial) {
    std::cout << label << "\n";
    std::cout << "  initial state = ";
    print_state(initial);
    std::cout << "\n";
    for (std::size_t i = 0; i < workflow.size(); ++i) {
        std::cout << "  task[" << i << "] '" << workflow[i].id
                  << "' winner = " << chosen[i]->strategy_name << "\n";
    }
    auto result = ts::propagate_state(workflow, chosen, initial);
    print_outcome(result, workflow);
    std::cout << "\n";
}

} // namespace

int main() {
    const auto repo = std::filesystem::path(TINYCELL_REPO_ROOT);
    auto arms = tinycell::io::load_arm_catalog(
        repo / "assets" / "arm" / "kuka" / "catalog.json");
    auto pushers = tinycell::io::load_pusher_catalog(
        repo / "assets" / "pusher" / "generic" / "catalog.json");

    ts::ArmStrategy arm_strategy(arms);
    ts::PusherStrategy pusher_strategy(pushers);

    // Each scenario builds its workflow + per-task chosen StrategyResult,
    // then feeds them through propagate_state. The StrategyResults are
    // produced by directly evaluating the picked strategy on each task —
    // identical to what brute-force enumerator winners would carry, but
    // explicit at the call site for demo readability.

    // Scenario A — initial state has position_known=false. ArmStrategy
    // requires position_known, so the pass FAILs immediately.
    {
        const std::vector<tc::Task> workflow{make_palletize("palletize_a")};
        const auto r0 = arm_strategy.evaluate(workflow[0]);
        const std::vector<const ts::StrategyResult*> chosen{&r0};
        const tc::ItemState initial{
            .position_known = false,
            .orientation_resolved_to = {},
            .on_carrier = tc::OnCarrier::Belt,
        };
        run_scenario("Scenario A — FAIL at start: arm needs position_known",
                     workflow, chosen, initial);
    }

    // Scenario B — initial state has position_known=true AND on_carrier=BELT.
    // PusherStrategy requires both; the pass succeeds and the trajectory
    // shows the pusher's effect (BELT → PALLET).
    {
        const std::vector<tc::Task> workflow{make_palletize("palletize_b")};
        const auto r0 = pusher_strategy.evaluate(workflow[0]);
        const std::vector<const ts::StrategyResult*> chosen{&r0};
        const tc::ItemState initial{
            .position_known = true,
            .orientation_resolved_to = {},
            .on_carrier = tc::OnCarrier::Belt,
        };
        run_scenario("Scenario B — clean propagation through PusherStrategy",
                     workflow, chosen, initial);
    }

    // Scenario C — two-task workflow. Task 0 winner is ArmStrategy (carrier-
    // agnostic). Its effect leaves the item on PALLET. Task 1 winner is
    // PusherStrategy, which needs on_carrier=BELT — and FAILs because the
    // previous task moved the item off the belt. This scenario could not
    // be caught by validating the initial state alone; the propagator
    // catches it because it carries state forward.
    {
        const std::vector<tc::Task> workflow{
            make_palletize("palletize_c0"),
            make_palletize("palletize_c1"),
        };
        const auto r0 = arm_strategy.evaluate(workflow[0]);
        const auto r1 = pusher_strategy.evaluate(workflow[1]);
        const std::vector<const ts::StrategyResult*> chosen{&r0, &r1};
        const tc::ItemState initial{
            .position_known = true,
            .orientation_resolved_to = {},
            .on_carrier = tc::OnCarrier::Belt,
        };
        run_scenario("Scenario C — FAIL mid-flow: prior task's effect "
                     "breaks the next task's carrier requirement",
                     workflow, chosen, initial);
    }

    return 0;
}
