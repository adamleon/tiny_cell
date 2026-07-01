// Tests for the workflow-diagram SVG (Phase 1b) + integration with the loader
// (1a) and the demo scenario asset (1c). Verifies the emitter produces a
// well-formed SVG with the expected nodes, edges and labels, and that the real
// assets/workflow scenario loads and renders (an artifact is written so the SVG
// can also be eyeballed).

#include <gtest/gtest.h>

#include <tinycell/io/workflow_loader.hpp>
#include <tinycell/workflow_svg.hpp>

#include <filesystem>
#include <fstream>
#include <string>

namespace {

namespace io  = tinycell::io;
namespace svg = tinycell::svg;

std::filesystem::path write_temp(const std::string& name, const std::string& content) {
    const auto path = std::filesystem::temp_directory_path() / name;
    std::ofstream(path) << content;
    return path;
}

constexpr const char* kWf = R"({ "workflow": [
  {"id":"feeder","kind":"anchor","target_ct_per_item_s":1.0,"name":"feeder","role":"output",
   "world_x_m":0,"world_y_m":0,"world_theta_deg":0},
  {"id":"cell","kind":"palletize","target_ct_per_item_s":1.5,"item_id":"box","box_count":24,
   "item":{"width_m":0.3,"length_m":0.4,"height_m":0.2,"mass_kg":25,"symmetry":{"kind":"discrete","period_deg":180}},
   "pallet":{"width_m":1.2,"length_m":0.8,"height_m":0.15,"mass_kg":25,"symmetry":{"kind":"asymmetric"}}},
  {"id":"dispatch","kind":"anchor","target_ct_per_item_s":1.0,"name":"dispatch","role":"input",
   "world_x_m":5,"world_y_m":0,"world_theta_deg":0},
  {"id":"feed_boxes","kind":"transport","target_ct_per_item_s":1.5,
   "source_task_id":"feeder","source_port_name":"port","sink_task_id":"cell","sink_port_name":"item_in"},
  {"id":"ship","kind":"transport","target_ct_per_item_s":1.5,
   "source_task_id":"cell","source_port_name":"pallet_out","sink_task_id":"dispatch","sink_port_name":"port"}
]})";

TEST(WorkflowSvg, RendersNodesEdgesAndLabels) {
    const auto        wf = io::load_workflow(write_temp("tc_wfsvg.json", kWf));
    const std::string s  = svg::render_workflow_svg(wf);

    EXPECT_NE(s.find("<svg"), std::string::npos);
    EXPECT_NE(s.find("</svg>"), std::string::npos);
    // node ids present
    EXPECT_NE(s.find(">feeder<"), std::string::npos);
    EXPECT_NE(s.find(">cell<"), std::string::npos);
    EXPECT_NE(s.find(">dispatch<"), std::string::npos);
    // node boxes + edge arrows
    EXPECT_NE(s.find("<rect"), std::string::npos);
    EXPECT_NE(s.find("marker-end"), std::string::npos);
    // edge port label + palletize detail
    EXPECT_NE(s.find("item_in"), std::string::npos);
    EXPECT_NE(s.find("24 x box"), std::string::npos);
}

TEST(WorkflowSvg, DemoScenarioAssetLoadsAndRenders) {
    const std::filesystem::path asset =
            std::filesystem::path(TINYCELL_REPO_ROOT) / "assets" / "workflow" / "two_line_palletizer.json";
    ASSERT_TRUE(std::filesystem::exists(asset)) << asset.string();

    const auto wf = io::load_workflow(asset);
    // 5 anchors + 2 palletize cells + 6 transports
    EXPECT_EQ(wf.size(), 13u);

    const std::string s = svg::render_workflow_svg(wf);
    EXPECT_NE(s.find(">heavy_cell<"), std::string::npos);
    EXPECT_NE(s.find(">light_cell<"), std::string::npos);

    // Write the artifact so it can be inspected / exported.
    const auto out = std::filesystem::temp_directory_path() / "two_line_palletizer_workflow.svg";
    svg::write_workflow_svg(wf, out);
    EXPECT_TRUE(std::filesystem::exists(out));
}

} // namespace
