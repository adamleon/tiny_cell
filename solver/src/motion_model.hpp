#pragma once

// Solver-internal motion model — coarse per-cycle estimates of equipment
// cycle time and energy. One arm pick-place-return is one cycle; one
// pusher extend-retract is one cycle. Strategies multiply these per-
// cycle numbers by their task multiplicity (box_count for ArmStrategy,
// num_rows for PusherStrategy) to fill StrategyResult.
//
// Header is solver-internal — lives in src/, not include/ — because
// the only consumers are sibling strategy .cpps. The numbers reach
// external code only through StrategyResult. Promote to
// solver/include/tinycell/solver/ if a future external consumer
// legitimately needs the per-cycle numbers directly; not before.
//
// Coarse-by-design per `decisions.md` "Distance-based motion model".
// Numerical constants are placeholders sized to produce sensible
// orderings on toy problems, NOT realistic absolutes. The seam is the
// durable contract; real calibrated bodies arrive at step 5+ without
// changing call sites. Known model optimism and the deferred safety-
// margin framing live in `decisions.md` "Model-vs-reality safety
// margin deferred."

#include <tinycell/model/arm.hpp>
#include <tinycell/model/pusher.hpp>
#include <tinycell/units.hpp>

namespace tinycell::solver {

// One equipment motion cycle's outputs. Arm cycle = one pick → place →
// return-to-pick (round trip). Pusher cycle = one extend → retract
// stroke (one row transferred).
struct MotionCycle {
    core::Duration cycle_time;
    core::Energy energy_per_cycle;
};

// arm_motion_cycle — round-trip travel over `one_way_distance` plus a
// fixed pick+place overhead, with energy split across active-power and
// idle-power durations via a duty-fraction placeholder. `payload` is
// reserved on the signature so a future per-motion physics body can
// distinguish loaded and unloaded legs; today the body ignores it.
MotionCycle arm_motion_cycle(core::Length one_way_distance,
                             core::Mass payload,
                             const core::ArmSpec& arm);

// pusher_motion_cycle — one extend + retract stroke using the catalog-
// declared `cycle_time_per_push` directly (manufacturer-set; the
// pusher catalog does not expose enough kinematics to compute it from
// stroke + speed). `row_payload` is reserved on the signature for the
// same forward-looking reason as `payload` on `arm_motion_cycle`.
MotionCycle pusher_motion_cycle(core::Mass row_payload,
                                const core::PusherSpec& pusher);

} // namespace tinycell::solver
