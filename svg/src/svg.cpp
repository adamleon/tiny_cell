// SVG writer implementation. Pure stdlib (CLAUDE.md §0 svg/ row).
//
// Y-flip strategy: the SVG viewBox is set to a Y-negative range
// (min_y' = -(view_min_y + view_height), max_y' = -view_min_y), which
// causes math-Y-up coords to land in the right visual position when
// the SVG renderer treats Y as screen-down. Equivalently, every Y
// value is negated on emission. We do the latter — it keeps the
// viewBox numbers literal and easier to debug.

#include <tinycell/svg.hpp>

#include <fstream>
#include <iomanip>
#include <mp-units/systems/si.h>
#include <stdexcept>

namespace tinycell::svg {

namespace {

double in_m(core::Length v) {
    return v.numerical_value_in(mp_units::si::metre);
}

// Render a double with reasonable precision, no trailing zeros. SVG
// doesn't need ten digits for a layout; six covers sub-millimetre
// precision at metre-scale and keeps files diff-friendly. Negative
// zero is normalised to "0" so Y-flipping a y=0 vertex doesn't print
// "-0".
std::string fmt(double x) {
    if (x == 0.0) x = 0.0;  // strip sign of -0.0
    std::ostringstream s;
    s << std::fixed << std::setprecision(6) << x;
    auto out = s.str();
    // Trim trailing zeros and a dangling decimal point.
    if (out.find('.') != std::string::npos) {
        while (!out.empty() && out.back() == '0') out.pop_back();
        if (!out.empty() && out.back() == '.') out.pop_back();
    }
    return out;
}

} // namespace

Writer::Writer(double width_mm, double height_mm,
               double view_min_x_m, double view_min_y_m,
               double view_width_m, double view_height_m)
    : width_mm_(width_mm), height_mm_(height_mm),
      view_min_x_m_(view_min_x_m), view_min_y_m_(view_min_y_m),
      view_width_m_(view_width_m), view_height_m_(view_height_m) {}

void Writer::polygon(const core::Polygon& p,
                     std::string_view fill,
                     std::string_view stroke,
                     double stroke_width_m) {
    if (p.empty()) return;
    body_ << "  <polygon points=\"";
    bool first = true;
    for (const auto& v : p) {
        if (!first) body_ << ' ';
        first = false;
        body_ << fmt(in_m(v.x)) << ',' << fmt(-in_m(v.y));
    }
    body_ << "\" fill=\"" << fill << "\" stroke=\"" << stroke
          << "\" stroke-width=\"" << fmt(stroke_width_m) << "\"/>\n";
}

void Writer::circle(core::Vec2 center, core::Length radius,
                    std::string_view fill,
                    std::string_view stroke,
                    double stroke_width_m) {
    body_ << "  <circle cx=\"" << fmt(in_m(center.x))
          << "\" cy=\"" << fmt(-in_m(center.y))
          << "\" r=\"" << fmt(in_m(radius))
          << "\" fill=\"" << fill << "\" stroke=\"" << stroke
          << "\" stroke-width=\"" << fmt(stroke_width_m) << "\"/>\n";
}

void Writer::ring(core::Vec2 center, core::Length r_inner, core::Length r_outer,
                  std::string_view fill,
                  std::string_view stroke,
                  double stroke_width_m) {
    // Annulus as a single path: outer circle CW, inner circle CCW;
    // even-odd fill leaves the hole in the middle. Two arcs per
    // circle keep the SVG path short.
    const double cx = in_m(center.x);
    const double cy = -in_m(center.y);
    const double r_i = in_m(r_inner);
    const double r_o = in_m(r_outer);
    body_ << "  <path fill-rule=\"evenodd\" d=\""
          << "M " << fmt(cx - r_o) << ' ' << fmt(cy)
          << " A " << fmt(r_o) << ' ' << fmt(r_o) << " 0 1 0 " << fmt(cx + r_o) << ' ' << fmt(cy)
          << " A " << fmt(r_o) << ' ' << fmt(r_o) << " 0 1 0 " << fmt(cx - r_o) << ' ' << fmt(cy)
          << " Z";
    if (r_i > 0.0) {
        body_ << " M " << fmt(cx - r_i) << ' ' << fmt(cy)
              << " A " << fmt(r_i) << ' ' << fmt(r_i) << " 0 1 0 " << fmt(cx + r_i) << ' ' << fmt(cy)
              << " A " << fmt(r_i) << ' ' << fmt(r_i) << " 0 1 0 " << fmt(cx - r_i) << ' ' << fmt(cy)
              << " Z";
    }
    body_ << "\" fill=\"" << fill << "\" stroke=\"" << stroke
          << "\" stroke-width=\"" << fmt(stroke_width_m) << "\"/>\n";
}

void Writer::text(core::Vec2 anchor, std::string_view content,
                  double font_size_m,
                  std::string_view fill) {
    body_ << "  <text x=\"" << fmt(in_m(anchor.x))
          << "\" y=\"" << fmt(-in_m(anchor.y))
          << "\" font-size=\"" << fmt(font_size_m)
          << "\" font-family=\"sans-serif\" fill=\"" << fill << "\">"
          << content << "</text>\n";
}

std::string Writer::to_string() const {
    std::ostringstream out;
    // viewBox y range is flipped (negated) to match the Y-flip in shape emission.
    const double view_min_y_flipped = -(view_min_y_m_ + view_height_m_);
    out << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    out << "<svg xmlns=\"http://www.w3.org/2000/svg\""
        << " width=\"" << fmt(width_mm_) << "mm\""
        << " height=\"" << fmt(height_mm_) << "mm\""
        << " viewBox=\"" << fmt(view_min_x_m_) << ' ' << fmt(view_min_y_flipped)
        << ' ' << fmt(view_width_m_) << ' ' << fmt(view_height_m_) << "\""
        << ">\n";
    out << body_.str();
    out << "</svg>\n";
    return out.str();
}

void Writer::write_to_file(const std::filesystem::path& path) const {
    std::ofstream f(path);
    if (!f) throw std::runtime_error("svg::Writer: failed to open " + path.string());
    f << to_string();
}

} // namespace tinycell::svg
