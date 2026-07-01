// INTENT: B.3 of step 5 — exercise the new placement primitives end-to-
// end on the step-4 workflow output. The Layer-3 NLP placer (Phase E)
// doesn't exist yet; this demo HAND-ROLLS a trivial layout — one
// instance per Station, stations laid out along world-x at a fixed
// pitch — so the frame system + Station type + boost_conv + svg
// pipeline has a concrete consumer. The SVG that comes out is the
// visible artifact: a world-coords plan view of the step-4 demo's
// bound instances sitting on a notional floor.
//
// Success: writes one world-coords SVG to
//   <build>/demos/layer_3_placement/output/layout.svg
// containing one footprint per BoundInstance, positioned at its
// station's world pose, with arm reach envelopes overlaid where the
// instance is an ArmStrategy winner. Exits 0 on success; non-zero
// means catalog load, frame construction, or SVG write failed and
// the message says which.
//
// This is a building-block demo: when the real Layer-3 placer lands
// (Phase E), it will replace the hand-rolled pitch heuristic with a
// solved layout, and the same SVG output stays a useful diff target.

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <mp-units/systems/si.h>
#include <stdexcept>
#include <string>
#include <vector>

#include <tinycell/geometry.hpp>
#include <tinycell/io/catalog_loader.hpp>
#include <tinycell/model/arm.hpp>
#include <tinycell/model/pusher.hpp>
#include <tinycell/model/task.hpp>
#include <tinycell/solver/allocator.hpp>
#include <tinycell/solver/arm_strategy.hpp>
#include <tinycell/solver/enumerator.hpp>
#include <tinycell/solver/pusher_strategy.hpp>
#include <tinycell/solver/station.hpp>
#include <tinycell/svg.hpp>

namespace {

using namespace mp_units;
using mp_units::si::metre;
using mp_units::si::radian;
namespace tc = tinycell::core;
namespace ts = tinycell::solver;
namespace io = tinycell::io;
namespace svg = tinycell::svg;

// Workflow identical to the step-4 demo so we exercise the same
// allocator output. Kept local rather than refactored into a shared
// helper — two demos sharing this is not yet a third concrete case
// (concrete-over-abstract).
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

const tc::ArmSpec* find_arm(const std::vector<tc::ArmSpec>& arms,
                            const std::string& id) {
    auto it = std::find_if(arms.begin(), arms.end(),
                           [&](const tc::ArmSpec& a) { return a.id == id; });
    return it == arms.end() ? nullptr : &*it;
}

const tc::PusherSpec* find_pusher(const std::vector<tc::PusherSpec>& pushers,
                                  const std::string& id) {
    auto it = std::find_if(pushers.begin(), pushers.end(),
                           [&](const tc::PusherSpec& p) { return p.id == id; });
    return it == pushers.end() ? nullptr : &*it;
}

// Build one Station per BoundInstance, laid out along world-x at
// `pitch` spacing. Each instance sits at the station's origin
// (pose_in_station = identity) — the simplest hand-rolled placement
// that still exercises the full FrameTable composition path. The
// real Layer-3 placer (Phase E) replaces this with NLopt-solved
// poses; the Station/FrameTable plumbing stays.
struct Layout {
    tc::FrameTable frames;
    std::vector<ts::Station> stations;
};

Layout build_hand_rolled_layout(const ts::AllocationResult& alloc,
                                double pitch_m) {
    Layout layout;
    int idx = 0;
    for (const auto& bi : alloc.instances) {
        const auto station_frame = layout.frames.add(
            tc::kWorldFrame,
            tc::Transform2D{idx * pitch_m * metre, 0.0 * metre, 0.0 * radian});
        ts::Station st{
            .id = "station_" + std::to_string(idx),
            .own_frame = station_frame,
            .instances = {
                ts::InstancePlacement{
                    .bound_instance_id = bi.id,
                    .catalog_id = bi.catalog_id,
                    .strategy_name = bi.strategy_name,
                    .pose_in_station = tc::Pose2D{
                        0.0 * metre, 0.0 * metre, 0.0 * radian, station_frame},
                },
            },
        };
        layout.stations.push_back(std::move(st));
        ++idx;
    }
    return layout;
}

// Bbox helper for sizing the viewBox.
struct Bbox {
    double min_x = 1e9, min_y = 1e9, max_x = -1e9, max_y = -1e9;
    void include(double x, double y) {
        min_x = std::min(min_x, x);
        min_y = std::min(min_y, y);
        max_x = std::max(max_x, x);
        max_y = std::max(max_y, y);
    }
    void include_circle(double cx, double cy, double r) {
        include(cx - r, cy - r);
        include(cx + r, cy + r);
    }
};

void draw_layout_svg(const Layout& layout,
                     const std::vector<tc::ArmSpec>& arms,
                     const std::vector<tc::PusherSpec>& pushers,
                     const std::filesystem::path& out_path) {
    // First pass: compute world bbox so we know the viewBox.
    Bbox b;
    for (const auto& st : layout.stations) {
        const auto t_world = layout.frames.resolve_to_world(st.own_frame);
        for (const auto& inst : st.instances) {
            if (inst.strategy_name == "ArmStrategy") {
                const auto* spec = find_arm(arms, inst.catalog_id);
                if (!spec) continue;
                auto fp = tc::apply(t_world, spec->footprint);
                for (const auto& v : fp) {
                    b.include(v.x.numerical_value_in(metre),
                              v.y.numerical_value_in(metre));
                }
                // Reach circle centered at instance origin (which is at
                // pose_in_station = identity, so the station's own world
                // origin), radius = max reach.
                const double cx = t_world.tx.numerical_value_in(metre);
                const double cy = t_world.ty.numerical_value_in(metre);
                const double r = spec->reach.max_radius.value().numerical_value_in(metre);
                b.include_circle(cx, cy, r);
            } else if (inst.strategy_name == "PusherStrategy") {
                const auto* spec = find_pusher(pushers, inst.catalog_id);
                if (!spec) continue;
                auto fp = tc::apply(t_world, spec->footprint);
                for (const auto& v : fp) {
                    b.include(v.x.numerical_value_in(metre),
                              v.y.numerical_value_in(metre));
                }
            }
        }
    }

    // Margin so labels and reach circles don't kiss the page edge.
    const double span = std::max(b.max_x - b.min_x, b.max_y - b.min_y);
    const double margin = 0.10 * span;
    const double view_min_x = b.min_x - margin;
    const double view_min_y = b.min_y - margin;
    const double view_w = (b.max_x - b.min_x) + 2 * margin;
    const double view_h = (b.max_y - b.min_y) + 2 * margin;
    // 400 mm landscape (or portrait — preserves the data's aspect).
    const double page_w_mm = 400.0;
    const double page_h_mm = page_w_mm * (view_h / view_w);

    svg::Writer w(page_w_mm, page_h_mm,
                  view_min_x, view_min_y, view_w, view_h);

    // World-origin crosshair (light grey), 0.5 m arms — orientation
    // anchor for the reader.
    w.polygon(tc::Polygon{
                  tc::Vec2{-0.5 * metre, 0.0 * metre},
                  tc::Vec2{ 0.5 * metre, 0.0 * metre}},
              "none", "rgba(0,0,0,0.20)", 0.01);
    w.polygon(tc::Polygon{
                  tc::Vec2{0.0 * metre, -0.5 * metre},
                  tc::Vec2{0.0 * metre,  0.5 * metre}},
              "none", "rgba(0,0,0,0.20)", 0.01);

    for (const auto& st : layout.stations) {
        const auto t_world = layout.frames.resolve_to_world(st.own_frame);
        for (const auto& inst : st.instances) {
            if (inst.strategy_name == "ArmStrategy") {
                const auto* spec = find_arm(arms, inst.catalog_id);
                if (!spec) continue;
                // Reach annulus first (underneath the footprint).
                w.ring(tc::Vec2{t_world.tx, t_world.ty},
                       spec->reach.min_radius,
                       spec->reach.max_radius.value());
                auto fp = tc::apply(t_world, spec->footprint);
                w.polygon(fp, "rgba(64,64,64,0.20)", "black", 0.01);
                // Label at the station's world origin, slightly offset.
                w.text(tc::Vec2{t_world.tx + 0.20 * metre,
                                t_world.ty + 0.20 * metre},
                       st.id + ":" + inst.catalog_id, 0.10);
            } else if (inst.strategy_name == "PusherStrategy") {
                const auto* spec = find_pusher(pushers, inst.catalog_id);
                if (!spec) continue;
                auto fp = tc::apply(t_world, spec->footprint);
                w.polygon(fp, "rgba(0,128,0,0.20)", "rgba(0,128,0,0.80)", 0.01);
                w.text(tc::Vec2{t_world.tx + 0.10 * metre,
                                t_world.ty + 0.20 * metre},
                       st.id + ":" + inst.catalog_id, 0.05);
            }
        }
    }

    w.write_to_file(out_path);
}

} // namespace

int main() {
    const auto repo = std::filesystem::path(TINYCELL_REPO_ROOT);
    const auto out_dir = std::filesystem::path(TINYCELL_DEMO_OUTPUT_DIR);
    std::error_code ec;
    std::filesystem::create_directories(out_dir, ec);
    if (ec) {
        std::cerr << "Failed to create output directory " << out_dir
                  << ": " << ec.message() << "\n";
        return 1;
    }

    try {
        const auto arms = io::load_arm_catalog(
            repo / "assets" / "arm" / "kuka" / "catalog.json");
        const auto pushers = io::load_pusher_catalog(
            repo / "assets" / "pusher" / "generic" / "catalog.json");

        ts::ArmStrategy arm_strategy(arms);
        ts::PusherStrategy pusher_strategy(pushers);
        const std::vector<const ts::Strategy*> strategies{
            &arm_strategy, &pusher_strategy};

        const std::vector<tc::Task> workflow{
            make_palletize("t_pallet_a",     5.0, 1.2, 0.8, 24, 5.0),
            make_palletize("t_pallet_b",     5.0, 1.2, 0.8, 24, 5.0),
            make_palletize("t_pallet_tight", 8.0, 2.0, 2.0, 12, 2.0),
        };

        const auto enumeration = ts::enumerate(workflow, strategies);
        const auto allocation = ts::allocate(enumeration);

        // Pitch wide enough to clear even palletizer-class reach (3.2 m
        // max radius from the catalog) plus a bit of breathing room.
        constexpr double kStationPitch_m = 8.0;
        const auto layout = build_hand_rolled_layout(allocation, kStationPitch_m);

        const auto svg_path = out_dir / "layout.svg";
        draw_layout_svg(layout, arms, pushers, svg_path);

        std::cout << "Wrote " << layout.stations.size()
                  << " station(s) to " << svg_path.string() << "\n";
        for (const auto& st : layout.stations) {
            const auto t = layout.frames.resolve_to_world(st.own_frame);
            std::cout << "  " << st.id << " at world ("
                      << t.tx.numerical_value_in(metre) << " m, "
                      << t.ty.numerical_value_in(metre) << " m)";
            for (const auto& inst : st.instances) {
                std::cout << " | " << inst.catalog_id
                          << " (" << inst.strategy_name << ")";
            }
            std::cout << "\n";
        }
    } catch (const std::exception& e) {
        std::cerr << "Layer-3 placement demo failed: " << e.what() << "\n";
        return 2;
    }

    return 0;
}
