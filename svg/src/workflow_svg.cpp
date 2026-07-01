// Workflow-diagram SVG emitter (Phase 1b). Pure stdlib (CLAUDE.md svg/ row).
// A crude-but-readable 3-column DAG: feed/source anchors | palletize cells |
// dispatch anchors, with Transport tasks drawn as arrows between the nodes
// they connect. Not a general graph-layout engine — the fixed column layout is
// the crudest thing that reads as a palletizing workflow (crudest-concrete-
// first); a real layered layout is earned only if a scenario defeats this.

#include <tinycell/workflow_svg.hpp>

#include <tinycell/model/port.hpp> // PortDirection (+ task.hpp kinds/params)

#include <algorithm>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <variant>

namespace tinycell::svg {

namespace {

namespace tc = tinycell::core;

// Layout constants, in SVG user units (== px at 1:1).
constexpr double kNodeW = 190.0;
constexpr double kNodeH = 76.0;
constexpr double kColGap = 130.0;
constexpr double kRowGap = 34.0;
constexpr double kMargin = 28.0;

struct Node {
    double x = 0.0; // top-left
    double y = 0.0;
};

// Minimal XML text escaping (task ids / port names are author-controlled).
std::string esc(const std::string& s) {
    std::string o;
    o.reserve(s.size());
    for (char c : s) {
        switch (c) {
        case '&': o += "&amp;"; break;
        case '<': o += "&lt;"; break;
        case '>': o += "&gt;"; break;
        default: o += c;
        }
    }
    return o;
}

// Column index by task kind: 0 = feed/source anchor, 1 = palletize cell,
// 2 = dispatch anchor, -1 = Transport (an edge, not a node).
int column_of(const tc::Task& t) {
    switch (t.kind()) {
    case tc::TaskKind::Anchor:
        return std::get<tc::AnchorParams>(t.params).role == tc::PortDirection::Output ? 0 : 2;
    case tc::TaskKind::Palletize:
        return 1;
    case tc::TaskKind::Transport:
        return -1;
    }
    return -1;
}

std::string kind_label(const tc::Task& t) {
    switch (t.kind()) {
    case tc::TaskKind::Anchor:
        return std::get<tc::AnchorParams>(t.params).role == tc::PortDirection::Output
                       ? "source (feed)"
                       : "sink (dispatch)";
    case tc::TaskKind::Palletize:
        return "palletize";
    case tc::TaskKind::Transport:
        return "transport";
    }
    return "";
}

std::string detail_label(const tc::Task& t) {
    if (t.kind() == tc::TaskKind::Palletize) {
        const auto& p = std::get<tc::PalletizeParams>(t.params);
        return std::to_string(p.box_count) + " x " + p.item_id;
    }
    return "";
}

} // namespace

std::string render_workflow_svg(const std::vector<core::Task>& workflow) {
    // 1) Place each non-transport task into its column, stacked by arrival order.
    std::map<std::string, Node> nodes;
    int rows[3] = {0, 0, 0};
    for (const auto& t : workflow) {
        const int col = column_of(t);
        if (col < 0) continue; // Transports are edges
        const int row = rows[col]++;
        nodes[t.id] = Node{
                .x = kMargin + col * (kNodeW + kColGap),
                .y = kMargin + row * (kNodeH + kRowGap),
        };
    }
    const int    maxRows = std::max({rows[0], rows[1], rows[2], 1});
    const double W = kMargin * 2.0 + 3.0 * kNodeW + 2.0 * kColGap;
    const double H = kMargin * 2.0 + maxRows * kNodeH + (maxRows - 1) * kRowGap;

    std::ostringstream s;
    s << "<?xml version=\"1.0\" encoding=\"UTF-8\"?>\n";
    s << "<svg xmlns=\"http://www.w3.org/2000/svg\" width=\"" << W << "\" height=\"" << H
      << "\" viewBox=\"0 0 " << W << ' ' << H << "\" font-family=\"sans-serif\">\n";
    s << "  <defs><marker id=\"arrow\" markerWidth=\"9\" markerHeight=\"9\" refX=\"8\" refY=\"3\" "
         "orient=\"auto\" markerUnits=\"strokeWidth\">"
         "<path d=\"M0,0 L8,3 L0,6 Z\" fill=\"#4a5568\"/></marker></defs>\n";

    // 2) Edges first (Transports), so node boxes draw on top of the lines.
    for (const auto& t : workflow) {
        if (t.kind() != tc::TaskKind::Transport) continue;
        const auto& tr  = std::get<tc::TransportParams>(t.params);
        const auto  src = nodes.find(tr.source_task_id);
        const auto  dst = nodes.find(tr.sink_task_id);
        if (src == nodes.end() || dst == nodes.end()) continue; // dangling ref (validate_workflow reports these)
        const double x1 = src->second.x + kNodeW, y1 = src->second.y + kNodeH / 2.0;
        const double x2 = dst->second.x, y2 = dst->second.y + kNodeH / 2.0;
        s << "  <line x1=\"" << x1 << "\" y1=\"" << y1 << "\" x2=\"" << x2 << "\" y2=\"" << y2
          << "\" stroke=\"#4a5568\" stroke-width=\"1.6\" marker-end=\"url(#arrow)\"/>\n";
        s << "  <text x=\"" << (x1 + x2) / 2.0 << "\" y=\"" << (y1 + y2) / 2.0 - 4.0
          << "\" font-size=\"11\" fill=\"#4a5568\" text-anchor=\"middle\">"
          << esc(tr.source_port_name) << " &#8594; " << esc(tr.sink_port_name) << "</text>\n";
    }

    // 3) Nodes: cells warm, anchors cool.
    for (const auto& t : workflow) {
        const int col = column_of(t);
        if (col < 0) continue;
        const Node& n = nodes.at(t.id);
        const char* fill   = col == 1 ? "#fff3e0" : "#e8f0fe";
        const char* stroke = col == 1 ? "#f59e0b" : "#4a90d9";
        s << "  <rect x=\"" << n.x << "\" y=\"" << n.y << "\" width=\"" << kNodeW << "\" height=\"" << kNodeH
          << "\" rx=\"8\" fill=\"" << fill << "\" stroke=\"" << stroke << "\" stroke-width=\"1.6\"/>\n";
        const double cx = n.x + kNodeW / 2.0;
        s << "  <text x=\"" << cx << "\" y=\"" << n.y + 26.0
          << "\" font-size=\"14\" font-weight=\"bold\" text-anchor=\"middle\" fill=\"#1a202c\">"
          << esc(t.id) << "</text>\n";
        s << "  <text x=\"" << cx << "\" y=\"" << n.y + 45.0
          << "\" font-size=\"11\" text-anchor=\"middle\" fill=\"#4a5568\">" << esc(kind_label(t)) << "</text>\n";
        if (const std::string d = detail_label(t); !d.empty()) {
            s << "  <text x=\"" << cx << "\" y=\"" << n.y + 62.0
              << "\" font-size=\"11\" text-anchor=\"middle\" fill=\"#4a5568\">" << esc(d) << "</text>\n";
        }
    }

    s << "</svg>\n";
    return s.str();
}

void write_workflow_svg(const std::vector<core::Task>& workflow, const std::filesystem::path& path) {
    std::ofstream f(path);
    if (!f) {
        throw std::runtime_error("svg::write_workflow_svg: failed to open " + path.string());
    }
    f << render_workflow_svg(workflow);
}

} // namespace tinycell::svg
