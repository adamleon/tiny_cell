// INTENT: Phase C of step 5 — exercise the workflow positional prior
// on the step-4 demo workflow and write a world-coords SVG showing
// the input + output anchors, the I→O axis between them, and a marker
// per task at its nominal position.
//
// Success: writes <build>/demos/positional_prior/output/prior.svg with
//   * input anchor: filled green circle at the left
//   * output anchor: filled blue circle at the right
//   * dashed line connecting them (the I→O axis)
//   * one labelled X per task at its nominal (x, y)
// Exits 0; the layout for the step-4 three-task workflow shows three
// evenly-spaced markers between anchors set 12 m apart.

#include <filesystem>
#include <iostream>
#include <mp-units/systems/si.h>
#include <string>
#include <vector>

#include <tinycell/geometry.hpp>
#include <tinycell/model/task.hpp>
#include <tinycell/solver/positional_prior.hpp>
#include <tinycell/svg.hpp>

namespace {

using namespace mp_units;
using mp_units::si::metre;
namespace tc = tinycell::core;
namespace ts = tinycell::solver;
namespace svg = tinycell::svg;

tc::Task make_palletize(const std::string& id,
                        double item_mass_kg,
                        double pallet_w_m, double pallet_l_m,
                        int box_count,
                        double target_s_per_item) {
    return tc::Task{
        .id = id,
        .params = tc::PalletizeParams{
            .item_id = "box",
            .item = tc::BoxSpec{
                .physical = tc::ItemPhysical{
                    .width = 0.3 * metre,
                    .length = 0.4 * metre,
                    .height = 0.2 * metre,
                    .mass = item_mass_kg * si::kilogram,
                    .symmetry = tc::symmetry::discrete(180),
                },
            },
            .pallet = tc::PalletSpec{
                .physical = tc::ItemPhysical{
                    .width = pallet_w_m * metre,
                    .length = pallet_l_m * metre,
                    .height = 0.15 * metre,
                    .mass = 25.0 * si::kilogram,
                    .symmetry = tc::symmetry::discrete(180),
                },
            },
            .box_count = box_count,
        },
        .target_ct_per_item = tc::CycleTimePerItem{target_s_per_item * si::second},
    };
}

} // namespace

int main() {
    const auto out_dir = std::filesystem::path(TINYCELL_DEMO_OUTPUT_DIR);
    std::error_code ec;
    std::filesystem::create_directories(out_dir, ec);
    if (ec) {
        std::cerr << "Failed to create output directory " << out_dir
                  << ": " << ec.message() << "\n";
        return 1;
    }

    const std::vector<tc::Task> workflow{
        make_palletize("t_pallet_a",     5.0, 1.2, 0.8, 24, 5.0),
        make_palletize("t_pallet_b",     5.0, 1.2, 0.8, 24, 5.0),
        make_palletize("t_pallet_tight", 8.0, 2.0, 2.0, 12, 2.0),
    };

    // Anchors set 12 m apart along world-x — wide enough to spread three
    // palletizer-class stations comfortably without overlapping reaches.
    const tc::Vec2 input_anchor{ 0.0 * metre, 0.0 * metre};
    const tc::Vec2 output_anchor{12.0 * metre, 0.0 * metre};

    const auto layout = ts::positional_prior(workflow, input_anchor, output_anchor);

    // Page sizing: viewBox spans the anchors with a vertical margin so
    // labels above the I→O axis don't crop.
    const double pad = 1.5;
    const double view_min_x = -pad;
    const double view_min_y = -pad;
    const double view_w = 12.0 + 2 * pad;
    const double view_h = 2 * pad;
    const double page_w_mm = 400.0;
    const double page_h_mm = page_w_mm * (view_h / view_w);

    svg::Writer w(page_w_mm, page_h_mm,
                  view_min_x, view_min_y, view_w, view_h);

    // I→O axis (faint dashed line, drawn as two short segments so the
    // viewer can read the orientation even without dash support).
    w.polygon(tc::Polygon{input_anchor, output_anchor},
              "none", "rgba(0,0,0,0.25)", 0.02);

    // Input anchor: green dot.
    w.circle(input_anchor, 0.20 * metre, "rgba(0,160,0,0.80)", "rgba(0,80,0,1.0)", 0.02);
    w.text(tc::Vec2{input_anchor.x - 0.25 * metre, input_anchor.y - 0.40 * metre},
           "input", 0.20);

    // Output anchor: blue dot.
    w.circle(output_anchor, 0.20 * metre, "rgba(0,80,200,0.80)", "rgba(0,40,120,1.0)", 0.02);
    w.text(tc::Vec2{output_anchor.x - 0.30 * metre, output_anchor.y - 0.40 * metre},
           "output", 0.20);

    // One marker + label per task. The marker is an "X" rendered as
    // two short polygon segments crossing at the nominal position.
    for (const auto& entry : layout.entries) {
        const auto& p = entry.nominal;
        constexpr double half_arm_m = 0.15;
        w.polygon(tc::Polygon{
                      tc::Vec2{p.x - half_arm_m * metre, p.y - half_arm_m * metre},
                      tc::Vec2{p.x + half_arm_m * metre, p.y + half_arm_m * metre}},
                  "none", "black", 0.025);
        w.polygon(tc::Polygon{
                      tc::Vec2{p.x - half_arm_m * metre, p.y + half_arm_m * metre},
                      tc::Vec2{p.x + half_arm_m * metre, p.y - half_arm_m * metre}},
                  "none", "black", 0.025);
        w.text(tc::Vec2{p.x - 0.30 * metre, p.y + 0.30 * metre},
               entry.task_id, 0.15);
    }

    const auto svg_path = out_dir / "prior.svg";
    w.write_to_file(svg_path);

    std::cout << "Wrote positional prior for " << layout.entries.size()
              << " task(s) to " << svg_path.string() << "\n";
    std::cout << "  input anchor:  ("
              << input_anchor.x.numerical_value_in(metre) << " m, "
              << input_anchor.y.numerical_value_in(metre) << " m)\n";
    std::cout << "  output anchor: ("
              << output_anchor.x.numerical_value_in(metre) << " m, "
              << output_anchor.y.numerical_value_in(metre) << " m)\n";
    for (const auto& e : layout.entries) {
        std::cout << "  " << e.task_id << " -> ("
                  << e.nominal.x.numerical_value_in(metre) << " m, "
                  << e.nominal.y.numerical_value_in(metre) << " m)\n";
    }
    return 0;
}
