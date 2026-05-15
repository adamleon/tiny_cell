#pragma once
#include <cassert>
#include <cmath>

namespace factory::throughput {

// Analytic throughput model: given an archetype's *parameters* and the
// *geometry* it operates in, return the max sustainable output rate in
// items/minute. Pure functions over POD types — no ECS, no global state.
//
// Step 2 scope: picker only. Belt and palletizer station to follow.

// ── Belt ────────────────────────────────────────────────────────────────────
//
// Throughput model (single-file packing along belt direction):
//
//   throughput = 60 × belt_speed / (item_length + safety_gap)   items/min
//
// item_length is the dimension of the carried item *along* the belt direction.
// safety_gap is the minimum bumper between consecutive items — set by sensor
// reaction time, vacuum-pickup clearance, or operational policy. 0 means
// back-to-back packing (matches the current sim's behavior; conservative
// integrators use 100-300 mm).
//
// Width is captured for feasibility (item must fit on belt) but does not
// affect throughput in single-file operation. Belt length affects buffering
// and cost, not throughput.

struct BeltParams {
    float max_speed_mm_s;
    int   width_mm;
};

struct BeltGeometry {
    int item_length_mm;   // along belt direction
    int item_width_mm;    // perpendicular (feasibility only)
    int safety_gap_mm;    // min spacing between items
};

inline float belt(const BeltParams& p, const BeltGeometry& g) {
    assert(std::isfinite(p.max_speed_mm_s) && p.max_speed_mm_s > 0.f);
    assert(p.width_mm        > 0);
    assert(g.item_length_mm  > 0);
    assert(g.item_width_mm   > 0);
    assert(g.safety_gap_mm  >= 0);

    return 60.f * p.max_speed_mm_s / float(g.item_length_mm + g.safety_gap_mm);
}

// ── Palletizer station ──────────────────────────────────────────────────────
//
// A palletizer's throughput is *composite*: it's the picker(s) feeding the
// pallet, divided by the number of boxes that fit on one pallet. We expose
// two pure helpers:
//
//   boxes_per_pallet(geometry) — pure-geometry rectangular packing. This is
//     the theoretical max for axis-aligned single-orientation boxes; real
//     placement patterns (interlocking, rotated layers) can deviate. Callers
//     that have a specific PlacementPattern should pass its capacity()
//     directly into palletizer() instead.
//
//   palletizer(boxes_per_pallet_count, per_picker_box_per_min, num_pickers)
//     — composition: picker throughput summed across pickers, divided by
//     boxes-per-pallet, gives pallets-per-minute output.
//
// What this model assumes / ignores at Step 2:
//   1. Pallet exchange is free (next pallet always ready). Real cells lose
//      seconds per pallet swap. Future refinement: add exchange_time_s.
//   2. Pickers don't interfere with each other when sharing a pallet. Real
//      multi-picker stations lose some throughput to coordination overhead.
//   3. The picker's per-box cycle time is the same for every drop position
//      on the pallet. In reality, deep-stack drops take slightly longer.

struct PalletizerGeometry {
    int pallet_length_mm;
    int pallet_width_mm;
    int pallet_max_stack_mm;
    int box_length_mm;
    int box_width_mm;
    int box_height_mm;
};

inline int boxes_per_pallet(const PalletizerGeometry& g) {
    assert(g.pallet_length_mm    > 0 && g.box_length_mm > 0);
    assert(g.pallet_width_mm     > 0 && g.box_width_mm  > 0);
    assert(g.pallet_max_stack_mm > 0 && g.box_height_mm > 0);
    return (g.pallet_length_mm    / g.box_length_mm)
         * (g.pallet_width_mm     / g.box_width_mm)
         * (g.pallet_max_stack_mm / g.box_height_mm);
}

inline float palletizer(int   boxes_per_pallet_count,
                        float per_picker_box_per_min,
                        int   num_pickers)
{
    assert(boxes_per_pallet_count > 0);
    assert(std::isfinite(per_picker_box_per_min) && per_picker_box_per_min > 0.f);
    assert(num_pickers > 0);
    return float(num_pickers) * per_picker_box_per_min / float(boxes_per_pallet_count);
}

// ── Picker ──────────────────────────────────────────────────────────────────
//
// Cycle model (home pose assumed hovering at pickup):
//
//   cycle = collision_pessimism * ( 2 * pickup_to_drop_mm
//                                   / (max_tcp_speed * effective_tcp_factor)
//                                   + grip_time + release_time )
//
//   throughput = 60 / cycle  (items/minute)
//
// Approximations baked in (see ARCHETYPES.md when written):
//   1. Cartesian distance (straight line) instead of joint-space motion.
//      Slightly pessimistic — joint-space is usually faster.
//   2. effective_tcp_factor (~0.6) folds in accel/decel: TCP only sustains
//      max speed over long moves; short moves are slower in effect.
//   3. collision_pessimism (~1.3) accounts for non-straight collision-avoiding
//      paths (up-over-down instead of straight-line).
//   4. Home pose is hovering above pickup, so home→pickup is treated as zero.
//      A separate placement check decides where home actually goes.
//
// payload_kg is carried for the reachability/feasibility gate elsewhere; it
// does not enter the throughput calculation directly (we don't model
// payload-dependent dynamics).

struct PickerParams {
    float max_tcp_speed_mm_s;
    float max_payload_kg;
    float grip_time_s;
    float release_time_s;
    float effective_tcp_factor = 0.6f;   // duty factor on max TCP speed
    float collision_pessimism  = 1.3f;   // multiplier on cycle time
};

struct PickerGeometry {
    float pickup_to_drop_mm;
};

inline float picker(const PickerParams& p, const PickerGeometry& g) {
    assert(std::isfinite(p.max_tcp_speed_mm_s)   && p.max_tcp_speed_mm_s > 0.f);
    assert(std::isfinite(p.effective_tcp_factor) && p.effective_tcp_factor > 0.f);
    assert(std::isfinite(p.collision_pessimism)  && p.collision_pessimism >= 1.f);
    assert(std::isfinite(p.grip_time_s)          && p.grip_time_s    >= 0.f);
    assert(std::isfinite(p.release_time_s)       && p.release_time_s >= 0.f);
    assert(std::isfinite(g.pickup_to_drop_mm)    && g.pickup_to_drop_mm >= 0.f);

    const float effective_speed = p.max_tcp_speed_mm_s * p.effective_tcp_factor;
    const float move_time       = g.pickup_to_drop_mm / effective_speed;
    const float cycle           = p.collision_pessimism *
                                  (2.f * move_time + p.grip_time_s + p.release_time_s);
    return 60.f / cycle;
}

}  // namespace factory::throughput
