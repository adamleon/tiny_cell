// Tests for io::load_workflow — the JSON workflow loader (post-MVP demo,
// Phase 1a). Self-contained: each case writes a temp .json and loads it, so no
// fixture assets are needed. Covers the happy path (all three task kinds parse
// with correct values + units) and that authored bad data is REJECTED with a
// ParseError rather than silently fabricated (CLAUDE.md io/ row).

#include <gtest/gtest.h>

#include <tinycell/io/parse_error.hpp>
#include <tinycell/io/workflow_loader.hpp>
#include <tinycell/model/port.hpp>
#include <tinycell/units.hpp>

#include <filesystem>
#include <fstream>
#include <numbers>
#include <string>
#include <variant>

namespace {

namespace tc = tinycell::core;
namespace io = tinycell::io;
using namespace mp_units;

// Writes `content` to a uniquely-named temp .json and returns the path.
std::filesystem::path write_temp_json(const std::string& name, const std::string& content) {
    const auto path = std::filesystem::temp_directory_path() / name;
    std::ofstream(path) << content;
    return path;
}

constexpr const char* kGoodWorkflow = R"({
  "workflow": [
    {
      "id": "feeder", "kind": "anchor", "target_ct_per_item_s": 1.0,
      "name": "feeder", "role": "output",
      "world_x_m": -2.5, "world_y_m": 0.0, "world_theta_deg": 90
    },
    {
      "id": "cell", "kind": "palletize", "target_ct_per_item_s": 1.5,
      "item_id": "box",
      "item":   {"width_m": 0.3, "length_m": 0.4, "height_m": 0.2, "mass_kg": 25.0,
                 "symmetry": {"kind": "discrete", "period_deg": 180}},
      "pallet": {"width_m": 1.2, "length_m": 0.8, "height_m": 0.15, "mass_kg": 25.0,
                 "symmetry": {"kind": "asymmetric"}},
      "box_count": 24
    },
    {
      "id": "box_to_cell", "kind": "transport", "target_ct_per_item_s": 1.5,
      "source_task_id": "feeder", "source_port_name": "port",
      "sink_task_id": "cell", "sink_port_name": "item_in"
    }
  ]
})";

TEST(WorkflowLoader, ParsesAllThreeKinds) {
    const auto wf = io::load_workflow(write_temp_json("tc_wf_good.json", kGoodWorkflow));
    ASSERT_EQ(wf.size(), 3u);

    // Anchor: role mapped, negative world coord kept, degrees -> radians.
    EXPECT_EQ(wf[0].id, "feeder");
    EXPECT_EQ(wf[0].kind(), tc::TaskKind::Anchor);
    const auto& anchor = std::get<tc::AnchorParams>(wf[0].params);
    EXPECT_EQ(anchor.role, tc::PortDirection::Output);
    EXPECT_NEAR(anchor.world_x.numerical_value_in(si::metre), -2.5, 1e-9);
    EXPECT_NEAR(anchor.world_theta.numerical_value_in(si::radian), std::numbers::pi / 2.0, 1e-9);

    // Palletize: nested item/pallet specs + count + cycle time.
    EXPECT_EQ(wf[1].kind(), tc::TaskKind::Palletize);
    const auto& pal = std::get<tc::PalletizeParams>(wf[1].params);
    EXPECT_EQ(pal.item_id, "box");
    EXPECT_EQ(pal.box_count, 24);
    EXPECT_NEAR(pal.item.physical.width.numerical_value_in(si::metre), 0.3, 1e-9);
    EXPECT_NEAR(pal.pallet.physical.width.numerical_value_in(si::metre), 1.2, 1e-9);
    EXPECT_NEAR(wf[1].target_ct_per_item.value().numerical_value_in(si::second), 1.5, 1e-9);

    // Transport: endpoint references preserved verbatim.
    EXPECT_EQ(wf[2].kind(), tc::TaskKind::Transport);
    const auto& tr = std::get<tc::TransportParams>(wf[2].params);
    EXPECT_EQ(tr.source_task_id, "feeder");
    EXPECT_EQ(tr.sink_port_name, "item_in");
}

TEST(WorkflowLoader, RejectsMissingFile) {
    EXPECT_THROW(io::load_workflow("tc_wf_does_not_exist_zzz.json"), io::ParseError);
}

TEST(WorkflowLoader, RejectsMalformedTopLevel) {
    EXPECT_THROW(io::load_workflow(write_temp_json("tc_wf_top.json", R"([1,2,3])")), io::ParseError);
    EXPECT_THROW(io::load_workflow(write_temp_json("tc_wf_nowf.json", R"({"tasks":[]})")), io::ParseError);
}

TEST(WorkflowLoader, RejectsUnknownTaskKind) {
    EXPECT_THROW(io::load_workflow(write_temp_json("tc_wf_kind.json",
                                                   R"({"workflow":[{"id":"x","kind":"weld","target_ct_per_item_s":1.0}]})")),
                 io::ParseError);
}

TEST(WorkflowLoader, RejectsUnknownAnchorRole) {
    EXPECT_THROW(io::load_workflow(write_temp_json("tc_wf_role.json",
                                                   R"({"workflow":[{"id":"a","kind":"anchor","target_ct_per_item_s":1.0,
        "name":"a","role":"sideways","world_x_m":0,"world_y_m":0,"world_theta_deg":0}]})")),
                 io::ParseError);
}

TEST(WorkflowLoader, RejectsNonPositiveDimension) {
    EXPECT_THROW(io::load_workflow(write_temp_json("tc_wf_dim.json",
                                                   R"({"workflow":[{"id":"c","kind":"palletize","target_ct_per_item_s":1.0,"item_id":"b",
        "item":{"width_m":0.0,"length_m":0.4,"height_m":0.2,"mass_kg":1,"symmetry":{"kind":"asymmetric"}},
        "pallet":{"width_m":1.2,"length_m":0.8,"height_m":0.15,"mass_kg":25,"symmetry":{"kind":"asymmetric"}},
        "box_count":24}]})")),
                 io::ParseError);
}

} // namespace
