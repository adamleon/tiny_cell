// Tests for validate_workflow() — the pre-solve topology check that
// catches port-mismatch and unconnected-input failure modes before the
// solver runs (step-6 M3). Each case builds a tiny workflow exercising
// one failure mode (or the clean baseline) and asserts the diagnostics.

#include <gtest/gtest.h>

#include <algorithm>
#include <mp-units/systems/si.h>
#include <string>
#include <vector>

#include <tinycell/model/port.hpp>
#include <tinycell/model/task.hpp>
#include <tinycell/solver/workflow_validation.hpp>

namespace tc = tinycell::core;
namespace ts = tinycell::solver;
using mp_units::si::kilogram;
using mp_units::si::metre;
using mp_units::si::radian;
using mp_units::si::second;

namespace {

tc::Task make_palletize(const std::string& id) {
    return tc::Task{
        .id = id,
        .params = tc::PalletizeParams{
            .item_id = "box",
            .item = tc::BoxSpec{.physical = tc::ItemPhysical{
                                    .width = 0.3 * metre, .length = 0.4 * metre,
                                    .height = 0.2 * metre, .mass = 5.0 * kilogram,
                                    .symmetry = tc::symmetry::discrete(180)}},
            .pallet = tc::PalletSpec{.physical = tc::ItemPhysical{
                                         .width = 1.2 * metre, .length = 0.8 * metre,
                                         .height = 0.15 * metre, .mass = 25.0 * kilogram,
                                         .symmetry = tc::symmetry::discrete(180)}},
            .box_count = 24,
        },
        .target_ct_per_item = tc::CycleTimePerItem{5.0 * second},
    };
}

tc::Task make_anchor(const std::string& id, tc::PortDirection role) {
    return tc::Task{
        .id = id,
        .params = tc::AnchorParams{.name = id,
                                   .role = role,
                                   .world_x = 0.0 * metre,
                                   .world_y = 0.0 * metre,
                                   .world_theta = 0.0 * radian},
        .target_ct_per_item = tc::CycleTimePerItem{1.0 * second},
    };
}

tc::Task make_transport(const std::string& id, const std::string& src_task,
                        const std::string& src_port, const std::string& dst_task,
                        const std::string& dst_port) {
    return tc::Task{
        .id = id,
        .params = tc::TransportParams{.source_task_id = src_task,
                                      .source_port_name = src_port,
                                      .sink_task_id = dst_task,
                                      .sink_port_name = dst_port},
        .target_ct_per_item = tc::CycleTimePerItem{5.0 * second},
    };
}

// A fully-wired single palletizing cell: box feeder + empty-pallet feeder
// + dispatch + one palletize, every logical port connected.
std::vector<tc::Task> clean_cell() {
    return {
        make_anchor("feeder", tc::PortDirection::Output),
        make_anchor("empty_pallets", tc::PortDirection::Output),
        make_anchor("dispatch", tc::PortDirection::Input),
        make_palletize("pal"),
        make_transport("x_box", "feeder", "port", "pal", "item_in"),
        make_transport("x_empty", "empty_pallets", "port", "pal", "pallet_in"),
        make_transport("x_full", "pal", "pallet_out", "dispatch", "port"),
    };
}

bool has_error_mentioning(const ts::WorkflowValidation& v, const std::string& needle) {
    return std::any_of(v.issues.begin(), v.issues.end(), [&](const ts::WorkflowIssue& i) {
        return i.severity == ts::WorkflowIssueSeverity::Error &&
               i.message.find(needle) != std::string::npos;
    });
}

} // namespace

TEST(WorkflowValidation, CleanCellHasNoIssues) {
    const auto wf = clean_cell();
    const auto v = ts::validate_workflow(wf);
    EXPECT_TRUE(v.ok());
    EXPECT_EQ(v.error_count(), 0u);
    EXPECT_EQ(v.warning_count(), 0u);
}

TEST(WorkflowValidation, UnfedInputIsError) {
    // Drop the empty-pallet feed: pal.pallet_in is now an unfed input —
    // the canonical "no empty-pallet source" gap (HANDOFF known issue).
    auto wf = clean_cell();
    wf.erase(std::remove_if(wf.begin(), wf.end(),
                            [](const tc::Task& t) { return t.id == "x_empty"; }),
             wf.end());
    const auto v = ts::validate_workflow(wf);
    EXPECT_FALSE(v.ok());
    EXPECT_TRUE(has_error_mentioning(v, "pal.pallet_in"));
    // The now-unused empty_pallets feeder output is a (drained-nowhere)
    // Warning, not an Error.
    EXPECT_GE(v.warning_count(), 1u);
}

TEST(WorkflowValidation, UndrainedOutputIsWarningNotError) {
    // Drop the full-pallet takeaway: pal.pallet_out is undrained and the
    // dispatch input is unfed. Output side is a Warning; the unfed
    // dispatch.port input is an Error (so ok() is false), but pallet_out
    // itself must not be an Error.
    auto wf = clean_cell();
    wf.erase(std::remove_if(wf.begin(), wf.end(),
                            [](const tc::Task& t) { return t.id == "x_full"; }),
             wf.end());
    const auto v = ts::validate_workflow(wf);
    EXPECT_FALSE(has_error_mentioning(v, "pal.pallet_out"));
    const bool pallet_out_warned =
        std::any_of(v.issues.begin(), v.issues.end(), [](const ts::WorkflowIssue& i) {
            return i.severity == ts::WorkflowIssueSeverity::Warning &&
                   i.message.find("pal.pallet_out") != std::string::npos;
        });
    EXPECT_TRUE(pallet_out_warned);
}

TEST(WorkflowValidation, TransportToUnknownTaskIsError) {
    auto wf = clean_cell();
    wf.push_back(make_transport("x_bad", "feeder", "port", "ghost", "item_in"));
    const auto v = ts::validate_workflow(wf);
    EXPECT_TRUE(has_error_mentioning(v, "ghost"));
}

TEST(WorkflowValidation, TransportToMissingPortIsError) {
    auto wf = clean_cell();
    // pal has no port named "nonexistent".
    wf.push_back(make_transport("x_bad", "feeder", "port", "pal", "nonexistent"));
    const auto v = ts::validate_workflow(wf);
    EXPECT_TRUE(has_error_mentioning(v, "nonexistent"));
}

TEST(WorkflowValidation, BackwardsPolarityIsError) {
    // Source a transport from an INPUT port (pal.item_in) — wired
    // backwards. Must be flagged as a polarity error.
    auto wf = clean_cell();
    wf.push_back(make_transport("x_back", "pal", "item_in", "dispatch", "port"));
    const auto v = ts::validate_workflow(wf);
    EXPECT_TRUE(has_error_mentioning(v, "item_in"));
}

TEST(WorkflowValidation, SelfLoopIsErrorAndDoesNotMaskUnfedInput) {
    // A self-loop (pal.pallet_out -> pal.pallet_in) is nonsensical AND, if
    // counted toward connectivity, would mark pal.pallet_in as fed by its
    // own output. Drop the real empty-pallet feed so pal.pallet_in is
    // genuinely unfed, then add the self-loop: both the self-loop and the
    // still-unfed input must be flagged (the loop must not mask the gap).
    auto wf = clean_cell();
    wf.erase(std::remove_if(wf.begin(), wf.end(),
                            [](const tc::Task& t) { return t.id == "x_empty"; }),
             wf.end());
    wf.push_back(make_transport("x_self", "pal", "pallet_out", "pal", "pallet_in"));
    const auto v = ts::validate_workflow(wf);
    EXPECT_FALSE(v.ok());
    EXPECT_TRUE(has_error_mentioning(v, "to itself"));
    EXPECT_TRUE(has_error_mentioning(v, "pal.pallet_in"));  // not masked
}

TEST(WorkflowValidation, DuplicateTaskIdIsError) {
    auto wf = clean_cell();
    wf.push_back(make_palletize("pal"));  // second "pal"
    const auto v = ts::validate_workflow(wf);
    EXPECT_TRUE(has_error_mentioning(v, "duplicate"));
}

TEST(WorkflowValidation, SharedFeederToTwoLinesIsClean) {
    // One feeder output sourcing two transports (a splitter) is valid —
    // the two-line M3 plant relies on this.
    auto wf = clean_cell();
    wf.push_back(make_palletize("pal2"));
    wf.push_back(make_anchor("empty_pallets2", tc::PortDirection::Output));
    wf.push_back(make_transport("x_box2", "feeder", "port", "pal2", "item_in"));
    wf.push_back(make_transport("x_empty2", "empty_pallets2", "port", "pal2", "pallet_in"));
    wf.push_back(make_transport("x_full2", "pal2", "pallet_out", "dispatch", "port"));
    const auto v = ts::validate_workflow(wf);
    EXPECT_TRUE(v.ok());
    EXPECT_EQ(v.warning_count(), 0u);
}
