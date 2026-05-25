// Tests for svg/: the Writer emits the expected SVG primitives and
// flips Y on the way out (CLAUDE.md §0 svg row: "flip Y at write time
// only"). The XML isn't parsed — we look for substring landmarks,
// since correctness of an SVG serializer is best verified by inspection
// + a few invariants.

#include <gtest/gtest.h>
#include <mp-units/systems/si.h>
#include <string>
#include <tinycell/geometry.hpp>
#include <tinycell/svg.hpp>

namespace core = tinycell::core;
namespace svg = tinycell::svg;
using mp_units::si::metre;

namespace {

core::Vec2 v(double x, double y) {
    return core::Vec2{x * metre, y * metre};
}

} // namespace

TEST(Svg, WrapsBodyInSvgElement) {
    svg::Writer w(200, 200, -1, -1, 2, 2);
    auto out = w.to_string();
    EXPECT_NE(out.find("<?xml"), std::string::npos);
    EXPECT_NE(out.find("<svg"), std::string::npos);
    EXPECT_NE(out.find("</svg>"), std::string::npos);
}

TEST(Svg, PolygonFlipsY) {
    // Input vertex (0, 1) — Y-positive in core's math convention —
    // should be emitted with a NEGATIVE Y in SVG screen coords.
    svg::Writer w(100, 100, -1, -1, 2, 2);
    w.polygon(core::Polygon{v(0, 0), v(1, 0), v(0, 1)});
    auto out = w.to_string();
    // The Y of the (0,1) input should appear as "0,-1" in the points
    // attribute. The (0,0) point appears as "0,0" and (1,0) as "1,0".
    EXPECT_NE(out.find("0,-1"), std::string::npos);
    EXPECT_NE(out.find("0,0"), std::string::npos);
    EXPECT_NE(out.find("1,0"), std::string::npos);
}

TEST(Svg, CircleEmitsCorrectAttributes) {
    svg::Writer w(100, 100, -1, -1, 2, 2);
    w.circle(v(0, 0.5), 0.25 * metre);
    auto out = w.to_string();
    EXPECT_NE(out.find("<circle"), std::string::npos);
    EXPECT_NE(out.find("cx=\"0\""), std::string::npos);
    EXPECT_NE(out.find("cy=\"-0.5\""), std::string::npos);  // Y flipped
    EXPECT_NE(out.find("r=\"0.25\""), std::string::npos);
}

TEST(Svg, RingEmitsTwoArcsWithEvenOddFill) {
    svg::Writer w(100, 100, -2, -2, 4, 4);
    w.ring(v(0, 0), 0.5 * metre, 1.0 * metre);
    auto out = w.to_string();
    EXPECT_NE(out.find("<path"), std::string::npos);
    EXPECT_NE(out.find("fill-rule=\"evenodd\""), std::string::npos);
    // Two "A " arc commands for the outer ring, two for the inner ring.
    std::size_t arc_count = 0;
    for (std::size_t i = 0; (i = out.find(" A ", i)) != std::string::npos; ++i, ++arc_count) {}
    EXPECT_EQ(arc_count, 4u);
}

TEST(Svg, ViewBoxFlippedSoMathYUpRenders) {
    // viewBox y is flipped: min_y' = -(view_min_y + view_height).
    // For Writer(_, _, -1, -1, 2, 2): -(-1 + 2) = -1. So viewBox should
    // start with "-1 -1 2 2".
    svg::Writer w(100, 100, -1, -1, 2, 2);
    auto out = w.to_string();
    EXPECT_NE(out.find("viewBox=\"-1 -1 2 2\""), std::string::npos);
}
