// Phase E tests: solve() actually optimises rather than passing the
// seed through. Each case picks an initial state with a known
// infeasibility or a known suboptimality and asserts that solve()
// moves it the expected direction.
//
// We don't pin exact poses - BOBYQA is a local optimiser, the
// objective is non-convex, and the test should survive small
// changes to weights or tolerances. We assert on the qualitative
// behaviour (decreased objective, hard constraints satisfied,
// stations no longer overlap) instead.

#include <gtest/gtest.h>
#include <cmath>
#include <mp-units/systems/si.h>
#include <tinycell/solver/layout_objective.hpp>
#include <tinycell/solver/layout_problem.hpp>

namespace tc = tinycell::core;
namespace ts = tinycell::solver;
using mp_units::si::metre;
using mp_units::si::radian;

namespace {

tc::Pose2D pose_at(double x, double y) {
    return tc::Pose2D{
        .x = x * metre,
        .y = y * metre,
        .theta = 0.0 * radian,
        .frame = tc::kWorldFrame,
    };
}

ts::StationProblem make_station(const std::string& id,
                                double seed_x, double seed_y, double radius,
                                double nom_x, double nom_y) {
    return ts::StationProblem{
        .id = id,
        .buffered_hull = {},
        .bounding_radius = radius * metre,
        .nominal = tc::Vec2{nom_x * metre, nom_y * metre},
        .initial_pose = pose_at(seed_x, seed_y),
    };
}

double dist(const tc::Pose2D& a, const tc::Pose2D& b) {
    const double dx = (a.x - b.x).numerical_value_in(metre);
    const double dy = (a.y - b.y).numerical_value_in(metre);
    return std::sqrt(dx * dx + dy * dy);
}

} // namespace

TEST(LayoutSolve, EmptyProblemReturnsEmptySolution) {
    ts::LayoutProblem p{
        .stations = {},
        .floor = ts::Floor{0.0 * metre, 10.0 * metre, 0.0 * metre, 10.0 * metre},
        .weights = ts::ObjectiveWeights{},
    };
    auto out = ts::solve(p);
    EXPECT_TRUE(out.station_poses.empty());
    EXPECT_NEAR(out.cost.total, 0.0, 1e-12);
    EXPECT_TRUE(out.hard_constraints_satisfied);
}

TEST(LayoutSolve, FeasibleSeedReturnsNearlyUnchanged) {
    // One station at its nominal, well inside the floor, no neighbours.
    // The optimiser has nothing to improve - should leave the pose
    // essentially as-is and report zero objective.
    ts::LayoutProblem p{
        .stations = {make_station("a", 5.0, 5.0, 1.0, 5.0, 5.0)},
        .floor = ts::Floor{0.0 * metre, 10.0 * metre, 0.0 * metre, 10.0 * metre},
        .weights = ts::ObjectiveWeights{},
    };
    auto out = ts::solve(p);
    ASSERT_EQ(out.station_poses.size(), 1u);
    EXPECT_NEAR(out.cost.total, 0.0, 1e-6);
    EXPECT_TRUE(out.hard_constraints_satisfied);
    EXPECT_LT(dist(out.station_poses[0], p.stations[0].initial_pose), 0.01);
}

TEST(LayoutSolve, MovesOverlappingStationsApart) {
    // Two stations seeded at the same point. Overlap penalty (large
    // weight) should push them apart; prior penalty (small weight)
    // pulls each toward its own nominal. Expected: stations separate
    // and end up roughly between origin and nominal each.
    ts::LayoutProblem p{
        .stations = {
            make_station("a", 5.0, 5.0, 1.0, 4.0, 5.0),   // nominal at (4, 5)
            make_station("b", 5.0, 5.0, 1.0, 6.0, 5.0),   // nominal at (6, 5)
        },
        .floor = ts::Floor{0.0 * metre, 10.0 * metre, 0.0 * metre, 10.0 * metre},
        .weights = ts::ObjectiveWeights{
            .overlap = 1000.0, .floor = 100.0, .positional_prior = 1.0,
        },
    };

    // Sanity check: seed is overlapping (centres coincide).
    const double seed_dist = dist(p.stations[0].initial_pose,
                                  p.stations[1].initial_pose);
    EXPECT_NEAR(seed_dist, 0.0, 1e-9);

    auto out = ts::solve(p);
    ASSERT_EQ(out.station_poses.size(), 2u);

    const double final_dist = dist(out.station_poses[0], out.station_poses[1]);
    EXPECT_GE(final_dist, 2.0 - 0.05)  // at least the sum of radii (allow eps)
        << "stations still overlap: dist = " << final_dist << " m";
    EXPECT_TRUE(out.hard_constraints_satisfied);
}

TEST(LayoutSolve, MovesStationInsideFloor) {
    // Single station seeded fully outside the floor (centre at (-5, 5)
    // with radius 1 → leftmost x = -6, 6 m past x_min=0). Floor weight
    // dominates; the station should end up with its bounding circle
    // entirely inside the floor.
    ts::LayoutProblem p{
        .stations = {make_station("a", -5.0, 5.0, 1.0, -5.0, 5.0)},
        .floor = ts::Floor{0.0 * metre, 10.0 * metre, 0.0 * metre, 10.0 * metre},
        .weights = ts::ObjectiveWeights{
            .overlap = 1.0, .floor = 1000.0, .positional_prior = 1.0,
        },
    };
    auto out = ts::solve(p);
    ASSERT_EQ(out.station_poses.size(), 1u);
    const double final_x = out.station_poses[0].x.numerical_value_in(metre);
    const double fp = ts::floor_penalty(out.station_poses[0],
                                        p.stations[0].bounding_radius, p.floor);
    EXPECT_GE(final_x, 1.0 - 0.05)  // x - r >= x_min → x >= r = 1.0
        << "station still outside floor: x = " << final_x
        << ", floor_penalty = " << fp
        << ", final_obj = " << out.cost.total;
    EXPECT_TRUE(out.hard_constraints_satisfied)
        << "x = " << final_x << ", floor_penalty = " << fp;
}

// ---- Phase F: partial-freeze + warm-start -------------------------------

TEST(LayoutSolve, AllStationsFrozenSkipsOptimisation) {
    // Two stations both frozen at infeasible seeds (overlapping).
    // solve() should NOT move them - it returns the initial poses
    // and reports the infeasibility honestly via the flag.
    ts::LayoutProblem p{
        .stations = {},
        .floor = ts::Floor{0.0 * metre, 10.0 * metre, 0.0 * metre, 10.0 * metre},
        .weights = ts::ObjectiveWeights{},
    };
    auto sa = make_station("a", 4.5, 5.0, 1.0, 4.0, 5.0);
    sa.frozen = true;
    auto sb = make_station("b", 5.5, 5.0, 1.0, 6.0, 5.0);
    sb.frozen = true;
    p.stations = {sa, sb};

    auto out = ts::solve(p);
    ASSERT_EQ(out.station_poses.size(), 2u);
    // Returned poses == initial poses (no optimisation ran).
    EXPECT_NEAR(out.station_poses[0].x.numerical_value_in(metre), 4.5, 1e-12);
    EXPECT_NEAR(out.station_poses[1].x.numerical_value_in(metre), 5.5, 1e-12);
    // Centres 1 m apart, radii sum 2 → still overlapping → infeasible.
    EXPECT_FALSE(out.hard_constraints_satisfied);
}

TEST(LayoutSolve, FrozenStationActsAsObstacle) {
    // One frozen station blocking the variable station's path to its
    // nominal. The frozen station's overlap contribution must push the
    // variable one off-axis.
    auto frozen = make_station("blocker", 5.0, 5.0, 1.0, 5.0, 5.0);
    frozen.frozen = true;
    auto mover = make_station("mover", 8.0, 5.0, 1.0, /*nom*/ 5.0, 5.0);

    ts::LayoutProblem p{
        .stations = {frozen, mover},
        .floor = ts::Floor{0.0 * metre, 20.0 * metre, 0.0 * metre, 10.0 * metre},
        .weights = ts::ObjectiveWeights{
            .overlap = 1000.0, .floor = 100.0, .positional_prior = 1.0,
        },
    };
    auto out = ts::solve(p);
    ASSERT_EQ(out.station_poses.size(), 2u);

    // Frozen station did not move.
    EXPECT_NEAR(out.station_poses[0].x.numerical_value_in(metre), 5.0, 1e-9);
    EXPECT_NEAR(out.station_poses[0].y.numerical_value_in(metre), 5.0, 1e-9);

    // Mover wanted to land at (5, 5) (its nominal) but the frozen
    // station is there with radius 1 + mover's radius 1 = 2. The
    // mover must land approximately tangent to the frozen one - i.e.
    // dist from blocker ≈ 2 m.
    //
    // Note: penalty-method equilibrium leaves a small residual
    // interpenetration (~2 mm here at overlap_weight=1000 versus
    // prior_weight=1) where the prior's pull balances the overlap
    // penalty's gradient. We check the spatial constraint at
    // engineering-scale tolerance (5 mm) rather than asserting
    // hard_constraints_satisfied=true. The strict flag requires
    // sub-mm precision and would need either much higher overlap
    // weight (numerically poor) or a real inequality constraint via
    // NLopt's LN_COBYLA (planned follow-up - converts overlap from
    // a soft penalty to a hard constraint, eliminating the residual).
    const double dx = out.station_poses[1].x.numerical_value_in(metre) - 5.0;
    const double dy = out.station_poses[1].y.numerical_value_in(metre) - 5.0;
    const double dist_from_blocker = std::sqrt(dx * dx + dy * dy);
    EXPECT_GE(dist_from_blocker, 2.0 - 0.005)  // within 5 mm of tangent
        << "mover penetrated frozen blocker beyond engineering tolerance: dist = "
        << dist_from_blocker;
}

TEST(LayoutSolve, FrozenInfeasibilityDoesNotBlockMover) {
    // Frozen station outside the floor (its caller's mistake — the
    // solver doesn't validate frozen stations at entry). The mover
    // should still find a feasible spot; only the frozen one's
    // floor_penalty makes the overall flag false.
    auto bad = make_station("bad_frozen", -10.0, 5.0, 1.0, -10.0, 5.0);
    bad.frozen = true;
    auto mover = make_station("mover", 5.0, 5.0, 1.0, 5.0, 5.0);

    ts::LayoutProblem p{
        .stations = {bad, mover},
        .floor = ts::Floor{0.0 * metre, 20.0 * metre, 0.0 * metre, 10.0 * metre},
        .weights = ts::ObjectiveWeights{},
    };
    auto out = ts::solve(p);

    // Bad station didn't move (frozen).
    EXPECT_NEAR(out.station_poses[0].x.numerical_value_in(metre), -10.0, 1e-9);
    // Mover stayed near its nominal (far from the bad frozen station,
    // no overlap pressure).
    EXPECT_NEAR(out.station_poses[1].x.numerical_value_in(metre), 5.0, 1e-3);
    // Bad station outside floor → infeasible overall.
    EXPECT_FALSE(out.hard_constraints_satisfied);
}

TEST(LayoutSolve, WarmStartFromPreviousSolutionIsStable) {
    // Run the same problem twice: once from the nominal seed, once
    // warm-started from the first solve's output. The second pose
    // should be essentially unchanged (the first solve already found
    // the local optimum; warm-starting from there is a fixed point).
    ts::LayoutProblem p1{
        .stations = {
            make_station("a", 0.0, 5.0, 1.0, 3.0, 5.0),
            make_station("b", 0.5, 5.0, 1.0, 7.0, 5.0),
        },
        .floor = ts::Floor{0.0 * metre, 10.0 * metre, 0.0 * metre, 10.0 * metre},
        .weights = ts::ObjectiveWeights{
            .overlap = 1000.0, .floor = 1000.0, .positional_prior = 1.0,
        },
    };
    auto out1 = ts::solve(p1);
    ASSERT_TRUE(out1.hard_constraints_satisfied);

    // Build a second problem with initial_pose = previous solve's output.
    ts::LayoutProblem p2 = p1;
    for (std::size_t i = 0; i < p2.stations.size(); ++i) {
        p2.stations[i].initial_pose = out1.station_poses[i];
    }
    auto out2 = ts::solve(p2);
    ASSERT_TRUE(out2.hard_constraints_satisfied);

    // Warm-started solve lands within a fraction of a millimetre of
    // the first - confirms initial_pose IS the warm-start hook.
    for (std::size_t i = 0; i < out1.station_poses.size(); ++i) {
        EXPECT_NEAR(out1.station_poses[i].x.numerical_value_in(metre),
                    out2.station_poses[i].x.numerical_value_in(metre), 1e-3);
        EXPECT_NEAR(out1.station_poses[i].y.numerical_value_in(metre),
                    out2.station_poses[i].y.numerical_value_in(metre), 1e-3);
    }
    // And the warm-started objective is no worse than the cold one
    // (might be identical, might be marginally better due to
    // re-running from a near-optimal point).
    EXPECT_LE(out2.cost.total, out1.cost.total + 1e-6);
}

// ---- Step 6 Phase 1: transport-distance term ----------------------------

TEST(LayoutSolve, TransportPullsStationTowardAnchor) {
    // Movable station A nominally at (0, 0). Frozen anchor B pinned at
    // (10, 0). A transport edge connects them at station-origin ports.
    // Without the transport term, A stays at its nominal; with the
    // transport term, A should be pulled measurably toward the anchor
    // along +x.
    auto a = make_station("a", 0.0, 0.0, 0.5, 0.0, 0.0);
    auto b = make_station("anchor", 10.0, 0.0, 0.5, 10.0, 0.0);
    b.frozen = true;

    const tc::Vec2 zero{0.0 * metre, 0.0 * metre};

    // Baseline: no transport. A stays near (0, 0) (prior pulls it
    // there; nothing pulls it elsewhere).
    ts::LayoutProblem p_no_xport{
        .stations = {a, b},
        .transports = {},
        .floor = ts::Floor{-5.0 * metre, 20.0 * metre, -5.0 * metre, 5.0 * metre},
        .weights = ts::ObjectiveWeights{
            .overlap = 1000.0, .floor = 1000.0,
            .positional_prior = 1.0, .transport = 0.0,
        },
    };
    auto out_no_xport = ts::solve(p_no_xport);
    const double x_no_xport = out_no_xport.station_poses[0].x.numerical_value_in(metre);
    EXPECT_NEAR(x_no_xport, 0.0, 0.1);  // sits at nominal

    // With transport (weight = prior weight): A balances prior pull at
    // 0 against transport pull at 10. At equal weights and quadratic
    // penalties on both terms the minimum is the midpoint x = 5.
    ts::LayoutProblem p_with_xport = p_no_xport;
    p_with_xport.transports = {
        ts::TransportConstraint{
            .source_station = 0, .source_port_local = zero,
            .sink_station = 1,   .sink_port_local = zero,
        },
    };
    p_with_xport.weights.transport = 1.0;
    auto out_with_xport = ts::solve(p_with_xport);
    const double x_with_xport = out_with_xport.station_poses[0].x.numerical_value_in(metre);
    EXPECT_NEAR(x_with_xport, 5.0, 0.1);  // midpoint between nominal (0) and anchor (10)
    EXPECT_GT(x_with_xport, x_no_xport + 1.0);  // unambiguously pulled

    // Anchor doesn't move (frozen).
    EXPECT_NEAR(out_with_xport.station_poses[1].x.numerical_value_in(metre), 10.0, 1e-9);
}

// ---- Step 6 M1.4: annulus ports as placer variables ---------------------

TEST(LayoutSolve, PointPortsUnchangedByPlacer) {
    // Regression guard for the M1.4 indexing rework: a transport with
    // POINT endpoints (no reach band) introduces NO port variables, so
    // the variable vector is sized 2*n_stations exactly as before and the
    // returned transports' port-locals come back bit-identical to the
    // problem's. If the port-block indexing ever bled into the station
    // block this would shift or corrupt the pass-through ports.
    auto a = make_station("a", 0.0, 0.0, 0.5, 0.0, 0.0);
    auto b = make_station("anchor", 10.0, 0.0, 0.5, 10.0, 0.0);
    b.frozen = true;
    const tc::Vec2 src_port{0.3 * metre, 0.0 * metre};
    const tc::Vec2 zero{0.0 * metre, 0.0 * metre};

    ts::LayoutProblem p{
        .stations = {a, b},
        .transports = {
            ts::TransportConstraint{
                .source_station = 0, .source_port_local = src_port,
                .sink_station = 1,   .sink_port_local = zero,
            },
        },
        .floor = ts::Floor{-5.0 * metre, 20.0 * metre, -5.0 * metre, 5.0 * metre},
        .weights = ts::ObjectiveWeights{
            .overlap = 1000.0, .floor = 1000.0,
            .positional_prior = 1.0, .transport = 1.0,
        },
    };
    auto out = ts::solve(p);
    ASSERT_EQ(out.transports.size(), 1u);
    // POINT endpoints pass through untouched - no variable was added.
    EXPECT_DOUBLE_EQ(out.transports[0].source_port_local.x.numerical_value_in(metre),
                     0.3);
    EXPECT_DOUBLE_EQ(out.transports[0].source_port_local.y.numerical_value_in(metre),
                     0.0);
    EXPECT_DOUBLE_EQ(out.transports[0].sink_port_local.x.numerical_value_in(metre),
                     0.0);
    EXPECT_TRUE(out.hard_constraints_satisfied);
}

TEST(LayoutSolve, AnnulusPortSlidesTowardAnchorWithinBand) {
    // Mirror of TransportPullsStationTowardAnchor, but the source
    // endpoint (an arm's item_in) is an ANNULUS port with band
    // [0.3, 0.6] m, SEEDED at the inner edge (0.3, 0) pointing toward
    // the anchor. M1.4 makes that port a placer variable: it should
    // slide OUTWARD toward the +x band edge (~0.6) to shorten the
    // transport to the anchor at (10, 0), while staying inside the reach
    // band. A pass-through (non-variable) port would stay pinned at 0.3.
    //
    // Critically, this runs at the DEFAULT annulus weight (1.0): the band
    // is held by the polar r box (a hard, exact constraint), NOT by a heavy
    // soft penalty, so the port lands AT the band edge (r ~= reach_max)
    // rather than past it. (An earlier draft used annulus=1000 to mask a
    // soft-penalty overshoot to 2*reach_max; the hard band makes that
    // override unnecessary.)
    auto a = make_station("arm", 0.0, 0.0, 0.5, 0.0, 0.0);
    auto b = make_station("anchor", 10.0, 0.0, 0.5, 10.0, 0.0);
    b.frozen = true;
    const tc::Vec2 zero{0.0 * metre, 0.0 * metre};

    ts::LayoutProblem p{
        .stations = {a, b},
        .transports = {
            ts::TransportConstraint{
                .source_station = 0,
                .source_port_local = tc::Vec2{0.3 * metre, 0.0 * metre},  // inner-edge seed
                .sink_station = 1,
                .sink_port_local = zero,
                .source_reach_min = 0.3 * metre,
                .source_reach_max = 0.6 * metre,
            },
        },
        .floor = ts::Floor{-5.0 * metre, 20.0 * metre, -5.0 * metre, 5.0 * metre},
        .weights = ts::ObjectiveWeights{
            .annulus = 1.0,  // DEFAULT weight: the hard polar-r box, not this, holds the band
            .overlap = 1000.0, .floor = 1000.0,
            .positional_prior = 1.0, .transport = 1.0,
        },
    };
    auto out = ts::solve(p);
    ASSERT_EQ(out.transports.size(), 1u);

    const double px = out.transports[0].source_port_local.x.numerical_value_in(metre);
    const double py = out.transports[0].source_port_local.y.numerical_value_in(metre);
    const double r = std::sqrt(px * px + py * py);

    // Port slid outward from the 0.3 seed toward the anchor along +x.
    EXPECT_GT(px, 0.45)
        << "annulus port did not slide outward toward the anchor: x = " << px;
    EXPECT_LT(std::abs(py), 0.05)
        << "annulus port drifted off the +x axis: y = " << py;
    // ...and stayed inside the reach band. The polar r box is a HARD bound,
    // so r <= reach_max exactly (modulo fp), even at the default weight.
    EXPECT_GE(r, 0.3 - 1e-6) << "annulus port collapsed inside the band: r = " << r;
    EXPECT_LE(r, 0.6 + 1e-6) << "annulus port left the reach band: r = " << r;
    EXPECT_TRUE(out.hard_constraints_satisfied);

    // The arm station itself is still pulled toward the anchor (the
    // station variable and the port variable cooperate, not compete).
    EXPECT_GT(out.station_poses[0].x.numerical_value_in(metre), 1.0);
    // Anchor stays pinned (frozen).
    EXPECT_NEAR(out.station_poses[1].x.numerical_value_in(metre), 10.0, 1e-9);
}

TEST(LayoutSolve, AnnulusPortVariableWhenStationFrozen) {
    // n_vars == 0 && n_ports > 0: a FROZEN station can still carry a
    // variable annulus port. solve() must NOT take the all-frozen
    // short-circuit; the port slides within its band toward the (frozen)
    // sink even though the station never moves. This exercises the
    // base == 0 port-block indexing (station_var_count == 0) that no other
    // test reaches — the indexing case the handoff flagged as the rework's
    // main risk.
    auto arm = make_station("arm", 0.0, 0.0, 0.5, 0.0, 0.0);
    arm.frozen = true;  // station frozen, but its item_in port is an annulus
    auto anchor = make_station("anchor", 10.0, 0.0, 0.5, 10.0, 0.0);
    anchor.frozen = true;
    const tc::Vec2 zero{0.0 * metre, 0.0 * metre};

    ts::LayoutProblem p{
        .stations = {arm, anchor},
        .transports = {
            ts::TransportConstraint{
                .source_station = 0,
                .source_port_local = tc::Vec2{0.0 * metre, 0.3 * metre},  // seed +y, away from anchor
                .sink_station = 1,
                .sink_port_local = zero,
                .source_reach_min = 0.3 * metre,
                .source_reach_max = 0.6 * metre,
            },
        },
        .floor = ts::Floor{-5.0 * metre, 20.0 * metre, -5.0 * metre, 5.0 * metre},
        .weights = ts::ObjectiveWeights{.transport = 1.0},
    };
    auto out = ts::solve(p);
    ASSERT_EQ(out.station_poses.size(), 2u);
    ASSERT_EQ(out.transports.size(), 1u);

    // Frozen station did not move.
    EXPECT_NEAR(out.station_poses[0].x.numerical_value_in(metre), 0.0, 1e-9);
    EXPECT_NEAR(out.station_poses[0].y.numerical_value_in(metre), 0.0, 1e-9);

    // Port swung from the +y seed around to +x (toward the anchor),
    // staying inside the band — proving it was a live variable on the
    // base == 0 path, not passed through.
    const double px = out.transports[0].source_port_local.x.numerical_value_in(metre);
    const double py = out.transports[0].source_port_local.y.numerical_value_in(metre);
    const double r = std::sqrt(px * px + py * py);
    EXPECT_GT(px, 0.45) << "frozen-station annulus port did not slide toward anchor: x = " << px;
    EXPECT_LT(std::abs(py), 0.05) << "port did not swing onto the +x axis: y = " << py;
    EXPECT_LE(r, 0.6 + 1e-6) << "port left the reach band: r = " << r;
}

TEST(LayoutSolve, DiscAnnulusWithOriginSeedSlidesTowardSink) {
    // Regression for the polar r=0 singularity: a DISC reach band
    // (reach_min unset => 0) with the port SEEDED at the station origin,
    // exactly as ArmStrategy emits item_in. The naive polar seed
    // (r=0, theta=0) is the singularity where BOBYQA has no gradient to
    // move on; solve() must instead seed the port toward the other
    // endpoint at the band midpoint, so it slides out to the band edge
    // toward the (frozen) feeder rather than staying stuck at the origin.
    // (M1.4's other annulus tests all used reach_min=0.3 > 0, which masks
    // this; the solve_workflow demo's real catalog has reach_min=0.)
    auto arm = make_station("arm", 10.0, 0.0, 0.5, 10.0, 0.0);
    auto feeder = make_station("feeder", 0.0, 0.0, 0.5, 0.0, 0.0);
    feeder.frozen = true;
    const tc::Vec2 origin{0.0 * metre, 0.0 * metre};

    ts::LayoutProblem p{
        .stations = {arm, feeder},
        .transports = {
            ts::TransportConstraint{
                .source_station = 1,            // feeder is the (point) source
                .source_port_local = origin,
                .sink_station = 0,              // arm item_in is the annulus sink
                .sink_port_local = origin,      // SEEDED at the arm origin (degenerate)
                .sink_reach_max = 1.6 * metre,  // DISC band [0, 1.6] (reach_min unset)
            },
        },
        .floor = ts::Floor{-5.0 * metre, 20.0 * metre, -5.0 * metre, 5.0 * metre},
        .weights = ts::ObjectiveWeights{.transport = 1.0},
    };
    auto out = ts::solve(p);
    ASSERT_EQ(out.transports.size(), 1u);
    const double px = out.transports[0].sink_port_local.x.numerical_value_in(metre);
    const double py = out.transports[0].sink_port_local.y.numerical_value_in(metre);
    const double r = std::sqrt(px * px + py * py);
    // Not stuck at the origin, pointed toward the feeder at -x, in band.
    EXPECT_GT(r, 0.5) << "disc-band port stuck near the origin singularity: r = " << r;
    EXPECT_LT(px, -0.4) << "disc-band port did not point toward the feeder: x = " << px;
    EXPECT_LE(r, 1.6 + 1e-6) << "disc-band port left its band: r = " << r;
    EXPECT_TRUE(out.hard_constraints_satisfied);
}

TEST(LayoutSolve, BothEndpointsAnnulusLandInTheirOwnBands) {
    // Both ends of ONE transport are annulus ports with DIFFERENT bands.
    // Exercises collect_port_vars ordering (source before sink) and the
    // per-port-pair unpack: each endpoint must land in its OWN band, which
    // fails if the two port pairs are cross-assigned. Both stations frozen
    // so n_vars == 0, n_ports == 2 (two pairs on the base == 0 path).
    auto s0 = make_station("s0", 0.0, 0.0, 0.3, 0.0, 0.0);
    s0.frozen = true;
    auto s1 = make_station("s1", 4.0, 0.0, 0.3, 4.0, 0.0);
    s1.frozen = true;

    ts::LayoutProblem p{
        .stations = {s0, s1},
        .transports = {
            ts::TransportConstraint{
                .source_station = 0,
                .source_port_local = tc::Vec2{0.3 * metre, 0.0 * metre},   // band [0.3, 0.6]
                .sink_station = 1,
                .sink_port_local = tc::Vec2{-0.5 * metre, 0.0 * metre},    // band [0.5, 1.0]
                .source_reach_min = 0.3 * metre,
                .source_reach_max = 0.6 * metre,
                .sink_reach_min = 0.5 * metre,
                .sink_reach_max = 1.0 * metre,
            },
        },
        .floor = ts::Floor{-5.0 * metre, 10.0 * metre, -5.0 * metre, 5.0 * metre},
        .weights = ts::ObjectiveWeights{.transport = 1.0},
    };
    auto out = ts::solve(p);
    ASSERT_EQ(out.transports.size(), 1u);
    const auto& tr = out.transports[0];
    const double s_x = tr.source_port_local.x.numerical_value_in(metre);
    const double s_y = tr.source_port_local.y.numerical_value_in(metre);
    const double k_x = tr.sink_port_local.x.numerical_value_in(metre);
    const double k_y = tr.sink_port_local.y.numerical_value_in(metre);
    const double s_r = std::sqrt(s_x * s_x + s_y * s_y);
    const double k_r = std::sqrt(k_x * k_x + k_y * k_y);

    // Source (on s0) reaches toward s1 at +x, capped at its OWN band (0.6).
    EXPECT_GT(s_x, 0.4) << "source did not reach toward s1: x = " << s_x;
    EXPECT_LE(s_r, 0.6 + 1e-6) << "source left its band: r = " << s_r;
    // Sink (on s1) reaches toward s0 at -x, capped at its OWN larger band (1.0).
    EXPECT_LT(k_x, -0.4) << "sink did not reach toward s0: x = " << k_x;
    EXPECT_LE(k_r, 1.0 + 1e-6) << "sink left its band: r = " << k_r;
    // Discriminator: the sink used its OWN (larger) band, not the source's.
    // A cross-assignment would cap the sink at 0.6 and this would fail.
    EXPECT_GT(k_r, 0.65) << "sink did not use its own larger band (cross-assigned?): r = " << k_r;
}

TEST(LayoutSolve, ObjectiveDecreasesFromInfeasibleSeed) {
    // Property: starting from any infeasible state, the optimised
    // objective is strictly lower than the initial objective.
    ts::LayoutProblem p{
        .stations = {
            make_station("a", 0.0, 5.0, 1.0, 3.0, 5.0),   // overlap floor at left edge
            make_station("b", 0.5, 5.0, 1.0, 7.0, 5.0),   // overlapping a
        },
        .floor = ts::Floor{0.0 * metre, 10.0 * metre, 0.0 * metre, 10.0 * metre},
        .weights = ts::ObjectiveWeights{
            .overlap = 1000.0, .floor = 1000.0, .positional_prior = 1.0,
        },
    };

    // Initial objective from the stub-style direct evaluation.
    const std::vector<tc::Pose2D> initial_poses{
        p.stations[0].initial_pose, p.stations[1].initial_pose,
    };
    const double initial_obj = ts::evaluate_objective(p, initial_poses);
    ASSERT_GT(initial_obj, 0.0);

    auto out = ts::solve(p);
    EXPECT_LT(out.cost.total, initial_obj * 0.5)
        << "solve() should make significant progress on the infeasible seed";
    EXPECT_TRUE(out.hard_constraints_satisfied);
}
