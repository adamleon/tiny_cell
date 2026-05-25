// demo_catalog_svg — step-1 SVG backfill (roadmap.md "Supporting tools").
//
// Demonstrates: for each catalog entry (every arm in assets/arm/, every
// pusher in assets/pusher/), emit a per-entry SVG showing its footprint
// polygon in core's local coords, plus — for arms — the min/max reach
// envelope as an annulus. Pure visual oracle: a sanity check on the
// shapes the JSON loader produces, ahead of any solver layer that will
// compose them.
//
// Success: writes (N_arms + N_pushers) SVG files into
//   <build>/demos/catalog_svg/output/
// and prints the count plus the output directory. Each SVG opens cleanly
// in a browser; for an arm, the reach annulus is centred on the origin
// of the entry's local frame and surrounds the base footprint. Any
// non-zero exit means catalog loading or write failed; the message
// names what went wrong.

#include <filesystem>
#include <iostream>
#include <mp-units/systems/si.h>
#include <tinycell/adapters/boost_conv.hpp>
#include <tinycell/io/catalog_loader.hpp>
#include <tinycell/io/parse_error.hpp>
#include <tinycell/svg.hpp>

namespace {

using namespace mp_units;
using mp_units::si::metre;
namespace core = tinycell::core;
namespace svg = tinycell::svg;
namespace io = tinycell::io;
namespace adapters = tinycell::adapters;

// Bounding box of a polygon, in metres. Used to size the SVG viewBox
// so each shape fills its page comfortably with a small margin.
struct Bbox {
    double min_x, min_y, max_x, max_y;
};

Bbox bbox(const core::Polygon& p) {
    Bbox b{1e9, 1e9, -1e9, -1e9};
    for (const auto& v : p) {
        const double x = v.x.numerical_value_in(metre);
        const double y = v.y.numerical_value_in(metre);
        b.min_x = std::min(b.min_x, x);
        b.min_y = std::min(b.min_y, y);
        b.max_x = std::max(b.max_x, x);
        b.max_y = std::max(b.max_y, y);
    }
    return b;
}

// Expand bbox to include a circle of `r` metres around the origin.
// Used for arms — the reach annulus typically extends well past the
// base footprint.
Bbox expand_for_circle(Bbox b, double r) {
    b.min_x = std::min(b.min_x, -r);
    b.min_y = std::min(b.min_y, -r);
    b.max_x = std::max(b.max_x, r);
    b.max_y = std::max(b.max_y, r);
    return b;
}

void write_arm_svg(const core::ArmSpec& arm, const std::filesystem::path& out) {
    const double r_max = arm.reach.max_radius.value().numerical_value_in(metre);
    const double r_min = arm.reach.min_radius.numerical_value_in(metre);
    Bbox b = expand_for_circle(bbox(arm.footprint), r_max);

    // Add 10 % margin on each side for legibility.
    const double margin = 0.1 * std::max(b.max_x - b.min_x, b.max_y - b.min_y);
    const double view_min_x = b.min_x - margin;
    const double view_min_y = b.min_y - margin;
    const double view_width = (b.max_x - b.min_x) + 2 * margin;
    const double view_height = (b.max_y - b.min_y) + 2 * margin;

    // 200 mm wide page; height preserves the viewBox aspect ratio.
    const double page_width_mm = 200.0;
    const double page_height_mm = page_width_mm * (view_height / view_width);

    svg::Writer w(page_width_mm, page_height_mm,
                  view_min_x, view_min_y, view_width, view_height);

    // Reach annulus underneath the footprint (filled, semi-transparent).
    w.ring(core::Vec2{0.0 * metre, 0.0 * metre},
           r_min * metre, r_max * metre);
    // Footprint on top, opaque outline.
    w.polygon(arm.footprint, "rgba(64,64,64,0.20)", "black", 0.005);
    // Label at the bottom-left of the viewBox.
    w.text(core::Vec2{(view_min_x + 0.02 * view_width) * metre,
                      (view_min_y + 0.04 * view_height) * metre},
           arm.id, 0.04 * view_width / 10.0);

    w.write_to_file(out);
}

void write_pusher_svg(const core::PusherSpec& pusher, const std::filesystem::path& out) {
    Bbox b = bbox(pusher.footprint);
    const double margin = 0.1 * std::max(b.max_x - b.min_x, b.max_y - b.min_y);
    const double view_min_x = b.min_x - margin;
    const double view_min_y = b.min_y - margin;
    const double view_width = (b.max_x - b.min_x) + 2 * margin;
    const double view_height = (b.max_y - b.min_y) + 2 * margin;
    const double page_width_mm = 200.0;
    const double page_height_mm = page_width_mm * (view_height / view_width);

    svg::Writer w(page_width_mm, page_height_mm,
                  view_min_x, view_min_y, view_width, view_height);

    // Footprint and its convex hull (the broad-phase shape Layer-3
    // collision checks will use). Drawing both gives a visual diff
    // between the catalog's authored polygon and the hull the solver
    // will see — for a convex catalog footprint these are identical
    // and the hull just hugs the polygon.
    auto hull = adapters::convex_hull(pusher.footprint);
    w.polygon(hull, "rgba(0,128,0,0.10)", "rgba(0,128,0,0.60)", 0.005);
    w.polygon(pusher.footprint, "rgba(64,64,64,0.20)", "black", 0.005);
    w.text(core::Vec2{(view_min_x + 0.02 * view_width) * metre,
                      (view_min_y + 0.04 * view_height) * metre},
           pusher.id, 0.04 * view_width / 10.0);

    w.write_to_file(out);
}

} // namespace

int main() {
    const std::filesystem::path repo_root{TINYCELL_REPO_ROOT};
    const std::filesystem::path out_dir =
        std::filesystem::path{TINYCELL_DEMO_OUTPUT_DIR};
    std::error_code ec;
    std::filesystem::create_directories(out_dir, ec);
    if (ec) {
        std::cerr << "Failed to create output directory " << out_dir << ": " << ec.message() << "\n";
        return 1;
    }

    int written = 0;
    try {
        const auto arms = io::load_arm_catalog(repo_root / "assets" / "arm" / "kuka" / "catalog.json");
        for (const auto& arm : arms) {
            write_arm_svg(arm, out_dir / (arm.id + ".svg"));
            ++written;
        }
    } catch (const io::ParseError& e) {
        std::cerr << "Arm catalog load failed: " << e.what() << "\n";
        return 2;
    }

    try {
        const auto pushers = io::load_pusher_catalog(repo_root / "assets" / "pusher" / "generic" / "catalog.json");
        for (const auto& pusher : pushers) {
            write_pusher_svg(pusher, out_dir / (pusher.id + ".svg"));
            ++written;
        }
    } catch (const io::ParseError& e) {
        std::cerr << "Pusher catalog load failed: " << e.what() << "\n";
        return 3;
    }

    std::cout << "Wrote " << written << " SVG files to " << out_dir.string() << "\n";
    return 0;
}
