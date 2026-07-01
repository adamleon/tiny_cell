// Tests for the workflow positional prior. Minimum useful shape: linear
// interpolation between input and output anchors. The richer spring
// model (parallel siblings, merge/split centroids) lands when an
// edge-bearing workflow first needs it; until then these tests pin
// the I→O axis behaviour the placer will seed against.

#include <gtest/gtest.h>
#include <mp-units/systems/si.h>
#include <tinycell/model/task.hpp>
#include <tinycell/solver/positional_prior.hpp>

namespace tc = tinycell::core;
namespace ts = tinycell::solver;
using mp_units::si::metre;
using mp_units::si::second;
using mp_units::si::kilogram;

namespace {

constexpr double kTol = 1e-9;

tc::Task make_task(const std::string& id) {
    return tc::Task{
        .id = id,
        .params = tc::PalletizeParams{
            .item_id = "box",
            .item = tc::BoxSpec{
                .physical = tc::ItemPhysical{
                    .width = 0.3 * metre, .length = 0.4 * metre,
                    .height = 0.2 * metre, .mass = 5.0 * kilogram,
                    .symmetry = tc::symmetry::discrete(180),
                },
            },
            .pallet = tc::PalletSpec{
                .physical = tc::ItemPhysical{
                    .width = 1.2 * metre, .length = 0.8 * metre,
                    .height = 0.15 * metre, .mass = 25.0 * kilogram,
                    .symmetry = tc::symmetry::discrete(180),
                },
            },
            .box_count = 24,
        },
        .target_ct_per_item = tc::CycleTimePerItem{5.0 * second},
    };
}

double x_m(const tc::Vec2& v) { return v.x.numerical_value_in(metre); }
double y_m(const tc::Vec2& v) { return v.y.numerical_value_in(metre); }

} // namespace

TEST(PositionalPrior, EmptyWorkflowProducesEmptyLayout) {
    auto layout = ts::positional_prior({},
        tc::Vec2{0.0 * metre, 0.0 * metre},
        tc::Vec2{10.0 * metre, 0.0 * metre});
    EXPECT_TRUE(layout.entries.empty());
}

TEST(PositionalPrior, SingleTaskLandsAtMidpoint) {
    // One task: t = 1/(1+1) = 0.5 between anchors. Anchors at (0,0)
    // and (10, 0) → task at (5, 0).
    auto wf = std::vector{make_task("t0")};
    auto layout = ts::positional_prior(wf,
        tc::Vec2{0.0 * metre, 0.0 * metre},
        tc::Vec2{10.0 * metre, 0.0 * metre});
    ASSERT_EQ(layout.entries.size(), 1u);
    EXPECT_EQ(layout.entries[0].task_id, "t0");
    EXPECT_NEAR(x_m(layout.entries[0].nominal), 5.0, kTol);
    EXPECT_NEAR(y_m(layout.entries[0].nominal), 0.0, kTol);
}

TEST(PositionalPrior, ThreeTasksEvenlyDistributed) {
    // Three tasks: t = 1/4, 2/4, 3/4 → x = 2.5, 5.0, 7.5 between (0,0) and (10,0).
    auto wf = std::vector{make_task("t0"), make_task("t1"), make_task("t2")};
    auto layout = ts::positional_prior(wf,
        tc::Vec2{0.0 * metre, 0.0 * metre},
        tc::Vec2{10.0 * metre, 0.0 * metre});
    ASSERT_EQ(layout.entries.size(), 3u);
    EXPECT_NEAR(x_m(layout.entries[0].nominal), 2.5, kTol);
    EXPECT_NEAR(x_m(layout.entries[1].nominal), 5.0, kTol);
    EXPECT_NEAR(x_m(layout.entries[2].nominal), 7.5, kTol);
    // y stays on the I→O axis.
    for (const auto& e : layout.entries) {
        EXPECT_NEAR(y_m(e.nominal), 0.0, kTol);
    }
}

TEST(PositionalPrior, AnchorsAlongYAxis) {
    // Anchors at (0, 0) and (0, 10) — flow runs vertically. Three tasks
    // land at y = 2.5, 5.0, 7.5 with x = 0.
    auto wf = std::vector{make_task("a"), make_task("b"), make_task("c")};
    auto layout = ts::positional_prior(wf,
        tc::Vec2{0.0 * metre, 0.0 * metre},
        tc::Vec2{0.0 * metre, 10.0 * metre});
    ASSERT_EQ(layout.entries.size(), 3u);
    EXPECT_NEAR(y_m(layout.entries[0].nominal), 2.5, kTol);
    EXPECT_NEAR(y_m(layout.entries[1].nominal), 5.0, kTol);
    EXPECT_NEAR(y_m(layout.entries[2].nominal), 7.5, kTol);
    for (const auto& e : layout.entries) {
        EXPECT_NEAR(x_m(e.nominal), 0.0, kTol);
    }
}

TEST(PositionalPrior, ReversedAnchorsReverseDirection) {
    // Swapping anchors reverses the interpolation: with anchors (10,0)
    // → (0,0), task 0 lands at x=7.5 (closer to the "input").
    auto wf = std::vector{make_task("t0"), make_task("t1"), make_task("t2")};
    auto layout = ts::positional_prior(wf,
        tc::Vec2{10.0 * metre, 0.0 * metre},
        tc::Vec2{0.0 * metre, 0.0 * metre});
    ASSERT_EQ(layout.entries.size(), 3u);
    EXPECT_NEAR(x_m(layout.entries[0].nominal), 7.5, kTol);
    EXPECT_NEAR(x_m(layout.entries[1].nominal), 5.0, kTol);
    EXPECT_NEAR(x_m(layout.entries[2].nominal), 2.5, kTol);
}

TEST(PositionalPrior, DiagonalAnchorsInterpBothAxes) {
    // Anchors at (0,0) → (10, 4). Single task at t=0.5 → (5, 2).
    auto wf = std::vector{make_task("t0")};
    auto layout = ts::positional_prior(wf,
        tc::Vec2{0.0 * metre, 0.0 * metre},
        tc::Vec2{10.0 * metre, 4.0 * metre});
    ASSERT_EQ(layout.entries.size(), 1u);
    EXPECT_NEAR(x_m(layout.entries[0].nominal), 5.0, kTol);
    EXPECT_NEAR(y_m(layout.entries[0].nominal), 2.0, kTol);
}
