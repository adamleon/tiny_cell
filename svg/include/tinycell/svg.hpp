#pragma once

// Minimal stdlib-only SVG writer for the solver's debug-oracle role
// (roadmap.md "Supporting tools"). Reads core::Polygon / core::Vec2 /
// core::Length directly; emits SVG text. No foreign-lib includes
// (CLAUDE.md §0 svg/ row): pure C++ stdlib output.
//
// Coordinate convention: input coords are core's right-handed Y-up
// (math convention, looking down at the floor); SVG natively uses
// Y-down screen convention. The writer flips Y at emission time, per
// CLAUDE.md §0 ("flip Y at write time only"). Callers pass coords in
// the natural math convention and don't think about screen flipping.

#include <filesystem>
#include <sstream>
#include <string>
#include <string_view>
#include <tinycell/geometry.hpp>
#include <tinycell/units.hpp>

namespace tinycell::svg {

// One SVG document. Coordinates passed to add_* are in meters; the
// writer maps them to SVG user units 1:1 and flips Y. The viewBox is
// inferred from the union of added shapes if not supplied explicitly;
// callers that want a fixed extent (e.g. a known floor) can pass one.
class Writer {
public:
    // Sizing. width_mm/height_mm set the rendered page size; the viewBox
    // is given in meters (matching core::Length internal unit). The
    // viewBox is what gets flipped; one meter in viewBox space = one
    // SVG user unit.
    Writer(double width_mm, double height_mm,
           double view_min_x_m, double view_min_y_m,
           double view_width_m, double view_height_m);

    // Closed polygon (vertices in core's math-Y-up convention; the
    // writer emits them Y-flipped). `stroke_width_m` is in meters,
    // matching the viewBox unit.
    void polygon(const core::Polygon& p,
                 std::string_view fill = "none",
                 std::string_view stroke = "black",
                 double stroke_width_m = 0.005);

    // Circle. Used for arm reach envelopes and point markers.
    void circle(core::Vec2 center, core::Length radius,
                std::string_view fill = "none",
                std::string_view stroke = "black",
                double stroke_width_m = 0.005);

    // Annulus (ring) — outer radius circle minus inner radius hole.
    // Used to draw an arm's reach envelope as the band between
    // min_radius and max_radius.
    void ring(core::Vec2 center, core::Length r_inner, core::Length r_outer,
              std::string_view fill = "rgba(0,128,255,0.10)",
              std::string_view stroke = "rgba(0,128,255,0.50)",
              double stroke_width_m = 0.005);

    // Free-text label, anchored at (x, y) in viewBox coords. Used for
    // catalog ids etc.
    void text(core::Vec2 anchor, std::string_view content,
              double font_size_m = 0.05,
              std::string_view fill = "black");

    // Serialize to string or file. The writer is finalized on call;
    // subsequent add_* calls would not appear in the already-written
    // output (callers should treat write_to_* as the last action).
    std::string to_string() const;
    void write_to_file(const std::filesystem::path& path) const;

private:
    double width_mm_;
    double height_mm_;
    double view_min_x_m_;
    double view_min_y_m_;
    double view_width_m_;
    double view_height_m_;
    std::ostringstream body_;
};

} // namespace tinycell::svg
