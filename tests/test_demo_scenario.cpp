// Guards the demo scenario asset (Phase 1c): the authored two-line palletizer
// workflow must be topologically valid so the demo can actually solve it. Loads
// the real asset (1a) and runs the solver's pre-solve topology pass
// (validate_workflow, M3) — every palletize input fed, no back-to-front edges,
// no dangling outputs. If this fails, the scenario JSON is broken, not the code.

#include <gtest/gtest.h>

#include <tinycell/io/workflow_loader.hpp>
#include <tinycell/solver/workflow_validation.hpp>

#include <filesystem>
#include <sstream>

namespace {

namespace io = tinycell::io;
namespace ts = tinycell::solver;

TEST(DemoScenario, TwoLinePalletizerValidates) {
    const auto asset =
            std::filesystem::path(TINYCELL_REPO_ROOT) / "assets" / "workflow" / "two_line_palletizer.json";
    ASSERT_TRUE(std::filesystem::exists(asset)) << asset.string();

    const auto wf = io::load_workflow(asset);
    const auto v  = ts::validate_workflow(wf);

    // Build a readable dump of any issues for failure output.
    std::ostringstream diag;
    for (const auto& issue : v.issues) {
        diag << "\n  ["
             << (issue.severity == ts::WorkflowIssueSeverity::Error ? "ERROR" : "WARN")
             << "] " << issue.task_id << ": " << issue.message;
    }

    EXPECT_TRUE(v.ok()) << "validate_workflow reported errors:" << diag.str();
    EXPECT_EQ(v.error_count(), 0u) << diag.str();
    // A clean, fully-wired demo scenario should raise no dangling-output
    // warnings either (every feed drained, every cell output dispatched).
    EXPECT_EQ(v.warning_count(), 0u) << "unexpected warnings:" << diag.str();
}

} // namespace
