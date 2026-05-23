// demo_knowledge_propagation — item-knowledge propagation pass (roadmap.md
// step 3).
//
// Demonstrates: walking a workflow forward through ItemKnowledge, checking
// each chosen strategy's requires_knowledge against the running knowledge
// and applying its effect afterward (data-model.md §4). Four scenarios
// exercise the three outcomes the pass can produce AND the property-based
// gating that distinguishes shapes:
//
//   * Scenario A (cylinder + unknown orientation)  — clean run. Cylinder
//     symmetry covers any orientation knowledge state; pusher and arm
//     gate-pass without needing any measurement.
//   * Scenario B (rectangle + unknown orientation) — FAIL at start.
//     Rectangle's 2-fold symmetry leaves the width inconsistent under
//     unresolved orientation; pusher (chosen as winner) rejects.
//   * Scenario C (rectangle + camera-narrowed)    — clean run. Orientation
//     narrowed to `snapped(180)` (camera reads the rectangle to within
//     its own symmetry) makes distinct_orientations = 1 → gate passes.
//   * Scenario D (FAIL mid-flow)                  — multi-task workflow
//     where the prior task's EFFECT (arm places item on pallet) breaks
//     the next task's REQUIRES (pusher needs belt). Cannot be caught by
//     initial-knowledge validation alone; the propagator catches it
//     because it carries knowledge forward.
//
// Success looks like: four labelled scenario blocks. Each prints the
// initial knowledge, the strategy chosen per task, and either a SUCCESS
// trajectory (one ItemKnowledge per node + a final outbound) or a FAIL
// line naming the violating task and the knowledge at that node.

#include <filesystem>
#include <iostream>
#include <mp-units/systems/si.h>
#include <span>
#include <string>
#include <tinycell/io/catalog_loader.hpp>
#include <tinycell/model/item.hpp>
#include <tinycell/model/task.hpp>
#include <tinycell/solver/arm_strategy.hpp>
#include <tinycell/solver/knowledge_propagation.hpp>
#include <tinycell/solver/pusher_strategy.hpp>
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

std::string orientation_str(const tc::OrientationKnowledge& o) {
    if (std::holds_alternative<tc::OrientationKnowledge::Unknown>(o.kind)) {
        return "Unknown";
    }
    if (std::holds_alternative<tc::OrientationKnowledge::Exact>(o.kind)) {
        return "Exact";
    }
    const int step = std::get<tc::OrientationKnowledge::Snapped>(o.kind).step_deg;
    return "Snapped(" + std::to_string(step) + ")";
}

std::string symmetry_str(const tc::RotationalSymmetry& s) {
    if (std::holds_alternative<tc::RotationalSymmetry::Continuous>(s.kind)) {
        return "Continuous";
    }
    if (std::holds_alternative<tc::RotationalSymmetry::Asymmetric>(s.kind)) {
        return "Asymmetric";
    }
    const int period = std::get<tc::RotationalSymmetry::Discrete>(s.kind).period_deg;
    return "Discrete(" + std::to_string(period) + ")";
}

void print_knowledge(const tc::ItemKnowledge& k) {
    std::cout << "{ position_known="
              << (k.position_known ? "true " : "false")
              << " orientation=" << orientation_str(k.orientation)
              << " on_carrier=" << on_carrier_str(k.on_carrier) << " }";
}

// Build a Palletize task with the given box symmetry. Item dimensions are
// kept small so both Arm and Pusher catalogs accept it on payload/reach
// grounds — the focus of these scenarios is the knowledge gate, not the
// catalog selection.
tc::Task make_palletize(const std::string& id, tc::RotationalSymmetry box_symmetry) {
    return tc::Task{
        .id = id,
        .params = tc::PalletizeParams{
            .item_id = "box",
            .item = tc::BoxSpec{
                .physical = tc::ItemPhysical{
                    .width = 0.3 * si::metre,
                    .length = 0.4 * si::metre,
                    .height = 0.2 * si::metre,
                    .mass = 5.0 * si::kilogram,
                    .symmetry = box_symmetry,
                },
            },
            .pallet = tc::PalletSpec{
                .physical = tc::ItemPhysical{
                    .width = 1.2 * si::metre,
                    .length = 0.8 * si::metre,
                    .height = 0.15 * si::metre,
                    .mass = 25.0 * si::kilogram,
                    .symmetry = tc::symmetry::discrete(180),
                },
            },
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
            print_knowledge(success->trajectory[i]);
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
        std::cout << "         inbound knowledge was ";
        print_knowledge(failure.inbound_knowledge);
        std::cout << "\n";
    }
}

void run_scenario(const std::string& label,
                  std::span<const tc::Task> workflow,
                  std::span<const ts::StrategyResult* const> chosen,
                  const tc::ItemKnowledge& initial) {
    std::cout << label << "\n";
    const auto& first_box = std::get<tc::PalletizeParams>(workflow[0].params).item;
    std::cout << "  item symmetry = " << symmetry_str(first_box.physical.symmetry) << "\n";
    std::cout << "  initial knowledge = ";
    print_knowledge(initial);
    std::cout << "\n";
    for (std::size_t i = 0; i < workflow.size(); ++i) {
        std::cout << "  task[" << i << "] '" << workflow[i].id
                  << "' winner = " << chosen[i]->strategy_name << "\n";
    }
    auto result = ts::propagate_knowledge(workflow, chosen, initial);
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

    // Scenario A — Cylinder. Continuous symmetry covers any orientation,
    // so even Unknown knowledge satisfies the orientation gate. Pusher
    // is the chosen winner and runs cleanly.
    {
        const std::vector<tc::Task> workflow{
            make_palletize("palletize_a", tc::symmetry::continuous()),
        };
        const auto r0 = pusher_strategy.evaluate(workflow[0]);
        const std::vector<const ts::StrategyResult*> chosen{&r0};
        const tc::ItemKnowledge initial{
            .position_known = true,
            .orientation = tc::orientation::unknown(),
            .on_carrier = tc::OnCarrier::Belt,
        };
        run_scenario("Scenario A — cylinder + unknown orientation: gate passes via symmetry",
                     workflow, chosen, initial);
    }

    // Scenario B — Rectangle (Discrete(180)). Unknown orientation leaves
    // distinct_orientations unbounded — width is not consistent under
    // unresolved orientation, so the pusher rejects. Demonstrates the
    // orientation gate firing on a SHAPE property, not on a type-name.
    {
        const std::vector<tc::Task> workflow{
            make_palletize("palletize_b", tc::symmetry::discrete(180)),
        };
        const auto r0 = pusher_strategy.evaluate(workflow[0]);
        const std::vector<const ts::StrategyResult*> chosen{&r0};
        const tc::ItemKnowledge initial{
            .position_known = true,
            .orientation = tc::orientation::unknown(),
            .on_carrier = tc::OnCarrier::Belt,
        };
        run_scenario("Scenario B — rectangle + unknown orientation: gate FAILs",
                     workflow, chosen, initial);
    }

    // Scenario C — same rectangle, but with Snapped(180) knowledge — what
    // a camera would produce after reading the rectangle to within its
    // own 2-fold symmetry. distinct_orientations(180, 180) = 1 → gate
    // passes; pusher runs.
    {
        const std::vector<tc::Task> workflow{
            make_palletize("palletize_c", tc::symmetry::discrete(180)),
        };
        const auto r0 = pusher_strategy.evaluate(workflow[0]);
        const std::vector<const ts::StrategyResult*> chosen{&r0};
        const tc::ItemKnowledge initial{
            .position_known = true,
            .orientation = tc::orientation::snapped(180),
            .on_carrier = tc::OnCarrier::Belt,
        };
        run_scenario("Scenario C — rectangle + snapped(180): camera-narrowed, gate passes",
                     workflow, chosen, initial);
    }

    // Scenario D — two-task workflow. Cylinder so the orientation gate is
    // trivially satisfied; the FAIL comes from carrier flow. Task 0 (Arm)
    // places the item on the pallet; task 1 (Pusher) needs the item on a
    // belt and FAILs. The case that justifies the propagator existing:
    // a flaw visible only by carrying knowledge forward.
    {
        const std::vector<tc::Task> workflow{
            make_palletize("palletize_d0", tc::symmetry::continuous()),
            make_palletize("palletize_d1", tc::symmetry::continuous()),
        };
        const auto r0 = arm_strategy.evaluate(workflow[0]);
        const auto r1 = pusher_strategy.evaluate(workflow[1]);
        const std::vector<const ts::StrategyResult*> chosen{&r0, &r1};
        const tc::ItemKnowledge initial{
            .position_known = true,
            .orientation = tc::orientation::unknown(),
            .on_carrier = tc::OnCarrier::Belt,
        };
        run_scenario("Scenario D — FAIL mid-flow: arm leaves item on pallet, pusher needs belt",
                     workflow, chosen, initial);
    }

    return 0;
}
