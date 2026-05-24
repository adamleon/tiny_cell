// demo_state_propagation — item-state propagation pass (roadmap.md
// step 3) demonstrating the Knowledge/Control split.
//
// Demonstrates: walking a workflow forward through ItemState, checking
// each chosen strategy's requires_state against the running state, and
// applying its effect afterward (data-model.md §4). The scenarios are
// designed to exercise the K/C distinction:
//
//   * Scenario A (cylinder + unknown orientation)
//       Continuous symmetry collapses every alignment to one — pusher
//       passes trivially despite unknown knowledge and free control.
//
//   * Scenario B (rectangle + Knowledge=Known(0) + Control=Free)
//       Camera saw the rectangle but it isn't physically held. The arm
//       (reads Knowledge) accepts; the pusher (reads Control) rejects.
//       This is the case that motivates the K/C split — observation
//       alone isn't enough for an open-loop stroke.
//
//   * Scenario C (rectangle + Knowledge=Known(0) + Control=Constrained(0))
//       Camera + fixture matched. Both arm and pusher pass.
//
//   * Scenario D (mid-flow FAIL)
//       Two-task workflow: arm places item on pallet (effect updates
//       carrier to Pallet), then pusher gates on Belt and fails. The
//       case that justifies the propagator existing — visible only by
//       carrying state forward.
//
// Success looks like: four labelled scenario blocks, each printing the
// initial state, the strategy chosen per task, and either a SUCCESS
// trajectory or a FAIL diagnostic.

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

std::string knowledge_str(const tc::OrientationKnowledge& k) {
    if (std::holds_alternative<tc::OrientationKnowledge::Unknown>(k.kind)) {
        return "Unknown";
    }
    const int a = std::get<tc::OrientationKnowledge::Known>(k.kind).alignment;
    return "Known(" + std::to_string(a) + ")";
}

std::string control_str(const tc::OrientationControl& c) {
    if (std::holds_alternative<tc::OrientationControl::Free>(c.kind)) {
        return "Free";
    }
    const int a = std::get<tc::OrientationControl::Constrained>(c.kind).alignment;
    return "Constrained(" + std::to_string(a) + ")";
}

std::string symmetry_str(const tc::RotationalSymmetry& s) {
    if (std::holds_alternative<tc::RotationalSymmetry::Continuous>(s.kind)) {
        return "Continuous";
    }
    if (std::holds_alternative<tc::RotationalSymmetry::Asymmetric>(s.kind)) {
        return "Asymmetric";
    }
    const int p = std::get<tc::RotationalSymmetry::Discrete>(s.kind).period_deg;
    return "Discrete(" + std::to_string(p) + ")";
}

void print_state(const tc::ItemState& s) {
    std::cout << "{ pos=" << (s.position_known ? "known" : "unknown")
              << ", K=" << knowledge_str(s.orientation.knowledge)
              << ", C=" << control_str(s.orientation.control)
              << ", carrier=" << on_carrier_str(s.on_carrier) << " }";
}

// Build a Palletize task with the given box symmetry. Item dimensions
// are kept small so both Arm and Pusher catalogs accept them on
// payload/reach/stroke grounds — the scenarios isolate the state-flow
// gating, not the catalog selection.
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
        .target_ct_per_item = tc::CycleTimePerItem{5.0 * si::second},
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

void run_scenario(const std::string& label,
                  std::span<const tc::Task> workflow,
                  std::span<const ts::StrategyResult* const> chosen,
                  const tc::ItemState& initial) {
    std::cout << label << "\n";
    const auto& first_box = std::get<tc::PalletizeParams>(workflow[0].params).item;
    std::cout << "  item symmetry = " << symmetry_str(first_box.physical.symmetry) << "\n";
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

    // Scenario A — Cylinder. Continuous symmetry collapses everything.
    // Pusher accepts despite Unknown knowledge and Free control:
    // control_matches_alignment short-circuits on Continuous.
    {
        const std::vector<tc::Task> workflow{
            make_palletize("palletize_a", tc::symmetry::continuous()),
        };
        const auto r0 = pusher_strategy.evaluate(workflow[0]);
        const std::vector<const ts::StrategyResult*> chosen{&r0};
        const tc::ItemState initial{
            .position_known = true,
            .orientation = {
                .knowledge = tc::knowledge::unknown(),
                .control   = tc::control::free(),
            },
            .on_carrier = tc::OnCarrier::Belt,
        };
        run_scenario("Scenario A — cylinder: continuous symmetry short-circuits the gate",
                     workflow, chosen, initial);
    }

    // Scenario B — Rectangle with camera observation but no physical
    // hold. Arm (reads Knowledge) passes; pusher (reads Control)
    // rejects. The motivating case for the K/C split.
    {
        const std::vector<tc::Task> workflow{
            make_palletize("palletize_b", tc::symmetry::discrete(180)),
        };
        // We exercise the PUSHER's predicate here — that's the one
        // that demonstrates the K/C distinction.
        const auto r0 = pusher_strategy.evaluate(workflow[0]);
        const std::vector<const ts::StrategyResult*> chosen{&r0};
        const tc::ItemState initial{
            .position_known = true,
            .orientation = {
                .knowledge = tc::knowledge::known(0),
                .control   = tc::control::free(),
            },
            .on_carrier = tc::OnCarrier::Belt,
        };
        run_scenario("Scenario B — rectangle: camera observed (K=Known) but not physically held (C=Free) — pusher rejects",
                     workflow, chosen, initial);
    }

    // Scenario C — Rectangle with both camera observation AND fixture
    // hold matched to the pusher's required alignment. The pusher's
    // required alignment isn't surfaced as a public field on
    // StrategyResult — the predicate captures it opaquely in its
    // closure — so the demo probes by trying both candidate alignments
    // and using whichever one satisfies the predicate. (A future
    // refactor could surface required_alignment explicitly; out of
    // scope for step 3.)
    {
        const std::vector<tc::Task> workflow{
            make_palletize("palletize_c", tc::symmetry::discrete(180)),
        };
        const auto r0 = pusher_strategy.evaluate(workflow[0]);

        const auto probe_with_alignment = [&](int alignment) {
            return tc::ItemState{
                .position_known = true,
                .orientation = {
                    .knowledge = tc::knowledge::known(alignment),
                    .control   = tc::control::constrained(alignment),
                },
                .on_carrier = tc::OnCarrier::Belt,
            };
        };
        int satisfying = 0;
        for (int a : {0, 1}) {
            if (r0.requires_state(probe_with_alignment(a))) {
                satisfying = a;
                break;
            }
        }

        const auto initial = probe_with_alignment(satisfying);
        std::cout << "  (pusher's required alignment for this rectangle: "
                  << satisfying << ")\n";
        const std::vector<const ts::StrategyResult*> chosen{&r0};
        run_scenario("Scenario C — rectangle: camera + fixture matched — both axes pinned — pusher passes",
                     workflow, chosen, initial);
    }

    // Scenario D — two-task workflow. Cylinder so the orientation gate
    // is trivially satisfied throughout; the FAIL comes from carrier
    // flow. Task 0 (Arm) places the item on the pallet; task 1
    // (Pusher) needs the item on a belt and FAILs. Justifies the
    // propagator existing — a flaw visible only by carrying state
    // forward.
    {
        const std::vector<tc::Task> workflow{
            make_palletize("palletize_d0", tc::symmetry::continuous()),
            make_palletize("palletize_d1", tc::symmetry::continuous()),
        };
        const auto r0 = arm_strategy.evaluate(workflow[0]);
        const auto r1 = pusher_strategy.evaluate(workflow[1]);
        const std::vector<const ts::StrategyResult*> chosen{&r0, &r1};
        const tc::ItemState initial{
            .position_known = true,
            .orientation = {
                .knowledge = tc::knowledge::unknown(),
                .control   = tc::control::free(),
            },
            .on_carrier = tc::OnCarrier::Belt,
        };
        run_scenario("Scenario D — FAIL mid-flow: arm places item on pallet, pusher needs belt",
                     workflow, chosen, initial);
    }

    return 0;
}
