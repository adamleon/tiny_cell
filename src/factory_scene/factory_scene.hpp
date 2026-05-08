#pragma once
#include <cmath>
#include <entt/entt.hpp>
#include <memory>
#include <unordered_map>
#include <vector>
#include "components.hpp"
#include "placement_pattern.hpp"
#include "solver/solver.hpp"
#include "station_components.hpp"

namespace factory {

static constexpr solver::EntityId kSceneEntityId = 1u;

class FactoryScene {
    entt::registry                                     registry_;
    std::unordered_map<solver::EntityId, entt::entity> id_map_;
    solver::SolverOutput                               current_layout_;
    solver::EntityId                                   next_id_ = 2;
    std::vector<solver::EntityId>                      node_order_;

    entt::entity get_or_create(solver::EntityId id) {
        auto [it, inserted] = id_map_.emplace(id, entt::null);
        if (inserted) it->second = registry_.create();
        return it->second;
    }

public:
    FactoryScene() {
        auto scene_ent  = registry_.create();
        id_map_.emplace(kSceneEntityId, scene_ent);
        auto& pose      = registry_.emplace<PoseComponent>(scene_ent);
        pose.parent     = scene_ent;
    }

    entt::entity place_node(float x_mm, float y_mm) {
        auto id = next_id_++;
        node_order_.push_back(id);
        auto e        = get_or_create(id);
        auto& pose    = registry_.emplace_or_replace<PoseComponent>(e);
        pose.position = Vec3{x_mm, y_mm, 0.f};
        pose.parent   = id_map_.at(kSceneEntityId);
        registry_.emplace_or_replace<NodeComponent>(e);
        return e;
    }

    entt::entity declare_opening(int width_mm) {
        auto id    = next_id_++;
        auto e     = get_or_create(id);
        auto& oc   = registry_.emplace_or_replace<DeclaredOpeningComponent>(e);
        oc.set_width_mm(width_mm);
        return e;
    }

    entt::entity declare_opening_anchored(int edge_index, int position_mm, int width_mm) {
        auto id  = next_id_++;
        auto e   = get_or_create(id);
        auto& oc = registry_.emplace_or_replace<DeclaredOpeningComponent>(e);
        oc.set_width_mm(width_mm);
        oc.set_desired_position_mm(position_mm);
        oc.set_hint_edge_index(edge_index);
        return e;
    }

    void solve(const LookupTable& table, const std::string& catalog_path) {
        solver::SolverInput in;
        in.catalog_path = catalog_path;

        for (auto sid : node_order_) {
            const auto& pose = registry_.get<PoseComponent>(id_map_.at(sid));
            in.nodes.push_back({sid, pose.position.x, pose.position.y});
        }
        for (const auto& [sid, e] : id_map_) {
            if (sid == kSceneEntityId) continue;
            if (auto* oc = registry_.try_get<DeclaredOpeningComponent>(e)) {
                oc->set_parent_edge(entt::null);
                if (oc->hint_edge_index() >= 0 && oc->desired_position_mm().has_value()) {
                    in.anchored_openings.push_back({
                        sid, oc->width_mm(),
                        oc->hint_edge_index(), *oc->desired_position_mm()
                    });
                } else {
                    in.unallocated_openings.push_back({sid, oc->width_mm()});
                }
            }
        }

        apply(solver::solve(in, table));

        for (const auto& [id, _] : id_map_)
            if (id < solver::kNewEntity) next_id_ = std::max(next_id_, id + 1);
    }

    void apply(const solver::SolverOutput& output) {
        current_layout_    = output;
        auto scene_ent     = id_map_.at(kSceneEntityId);

        for (const auto& n : output.nodes) {
            auto e  = get_or_create(n.entity_id);
            registry_.emplace_or_replace<NodeComponent>(e).set_type(n.type);

            auto& pose    = registry_.emplace_or_replace<PoseComponent>(e);
            pose.position = Vec3{n.x_mm, n.z_mm, 0.f};
            pose.parent   = scene_ent;
        }

        for (const auto& edge : output.edges) {
            auto e  = get_or_create(edge.entity_id);
            auto& ec = registry_.emplace_or_replace<EdgeComponent>(e);
            ec.set_node_a(get_or_create(edge.node_a_id));
            ec.set_node_b(get_or_create(edge.node_b_id));
            ec.set_spans_mm(edge.spans_mm);
            ec.set_catalog_ref(edge.catalog_ref);

            const auto& pa = registry_.get<PoseComponent>(ec.node_a());
            const auto& pb = registry_.get<PoseComponent>(ec.node_b());
            Vec3  dir      = pb.position - pa.position;
            float len      = glm::length(dir);
            Vec3  dnorm    = (len > 0.f) ? dir / len : Vec3{1.f, 0.f, 0.f};
            float yaw      = std::atan2(dnorm.y, dnorm.x);

            auto& ep       = registry_.emplace_or_replace<PoseComponent>(e);
            ep.position    = (pa.position + pb.position) * 0.5f;
            ep.orientation = glm::angleAxis(yaw, Vec3{0.f, 0.f, 1.f});
            ep.parent      = scene_ent;

            for (const auto& op : edge.openings) {
                auto oe  = get_or_create(op.entity_id);
                auto& oc = registry_.emplace_or_replace<DeclaredOpeningComponent>(oe);
                oc.set_parent_edge(e);
                oc.set_width_mm(op.width_mm);

                float local_x       = static_cast<float>(op.position_mm) - len * 0.5f;
                auto& op_pose       = registry_.emplace_or_replace<PoseComponent>(oe);
                op_pose.position    = Vec3{local_x, 0.f, 0.f};
                op_pose.orientation = Quat{1.f, 0.f, 0.f, 0.f};
                op_pose.parent      = e;
            }
        }
    }

    int edge_polygon_index(entt::entity edge_e) const {
        const auto* ec = registry_.try_get<EdgeComponent>(edge_e);
        if (!ec) return -1;
        int n = static_cast<int>(node_order_.size());
        for (int i = 0; i < n; ++i) {
            auto it_a = id_map_.find(node_order_[i]);
            auto it_b = id_map_.find(node_order_[(i + 1) % n]);
            if (it_a == id_map_.end() || it_b == id_map_.end()) continue;
            if ((ec->node_a() == it_a->second && ec->node_b() == it_b->second) ||
                (ec->node_a() == it_b->second && ec->node_b() == it_a->second))
                return i;
        }
        return -1;
    }

    entt::entity find(solver::EntityId id) const {
        auto it = id_map_.find(id);
        return it != id_map_.end() ? it->second : entt::null;
    }

    std::size_t entity_count() const { return id_map_.size(); }

    entt::entity root_entity() const { return id_map_.at(kSceneEntityId); }

    const solver::SolverOutput& current_layout() const { return current_layout_; }
    entt::registry&             registry()              { return registry_; }
    const entt::registry&       registry()        const { return registry_; }

    // ── Transport / workflow ──────────────────────────────────────────────────

    entt::entity add_belt(int width_mm, int length_mm, int surface_height_mm,
                          float speed_mm_s, Vec3 dir) {
        auto e   = registry_.create();
        auto& bc = registry_.emplace<ConveyorBeltComponent>(e);
        bc.set_width_mm(width_mm);
        bc.set_length_mm(length_mm);
        bc.set_surface_height_mm(surface_height_mm);
        bc.set_belt_speed_mm_s(speed_mm_s);
        bc.set_dir(dir);
        registry_.emplace<TransportComponent>(e);
        return e;
    }

    int items_on(entt::entity transport) const {
        int n = 0;
        registry_.view<ItemOnTransportComponent>().each(
            [&](const ItemOnTransportComponent& it) {
                if (it.transport() == transport) ++n;
            });
        return n;
    }

    // Add a port at world `position` with local +x oriented along `forward`
    // (yaw quaternion in the X-Y plane). Defaults to world +x. Sensors that
    // reference this port inherit this orientation through PoseComponent's
    // parent chain.
    entt::entity add_port(const std::string& name, Vec3 position,
                          Vec3 forward = Vec3{1.f, 0.f, 0.f}) {
        auto e   = registry_.create();
        auto& pc = registry_.emplace<PortComponent>(e);
        pc.set_name(name);
        auto& pose = registry_.emplace<PoseComponent>(e);
        pose.position = position;
        if (glm::dot(forward, forward) > 0.f) {
            float yaw = std::atan2(forward.y, forward.x);
            pose.orientation = glm::angleAxis(yaw, Vec3{0.f, 0.f, 1.f});
        }
        pose.parent = root_entity();
        return e;
    }

    void connect_belt(entt::entity belt, entt::entity entry_port, entt::entity exit_port) {
        auto& bc = registry_.get<ConveyorBeltComponent>(belt);
        bc.set_entry_port(entry_port);
        bc.set_exit_port(exit_port);
        // Default: exit_port gates belt motion (downstream-blocked behaviour).
        bc.add_gate_port(exit_port);
    }

    void set_port_transport(entt::entity port, entt::entity transport) {
        registry_.get<PortComponent>(port).set_transport(transport);
    }

    // ── Sensors ───────────────────────────────────────────────────────────────

    // Laser sensor: a 1D detection line at the sensor's world position,
    // parented to `port` so its orientation tracks the port's forward
    // direction. The sensor itself carries no extents — the item's bounding
    // box (from ItemPrototypeComponent) decides whether it crosses the
    // line. `local_offset` shifts the laser along the port's local axes
    // (typically `{x, 0, 0}` to move it forward / back along the belt).
    entt::entity add_laser_sensor(entt::entity port,
                                  Vec3         local_offset = Vec3{0.f}) {
        auto e = registry_.create();
        registry_.emplace<SensorComponent>(e);
        registry_.emplace<LaserSensorComponent>(e);
        auto& pose = registry_.emplace<PoseComponent>(e);
        pose.position = local_offset;
        pose.parent   = port;
        registry_.get<PortComponent>(port).add_sensor(e);
        return e;
    }

    // Virtual sensor: presence in a port's sensor list, but no detection
    // volume — `blocked` is written by orchestration code.
    entt::entity add_virtual_sensor(entt::entity port) {
        auto e = registry_.create();
        registry_.emplace<SensorComponent>(e);
        auto& pose = registry_.emplace<PoseComponent>(e);
        pose.parent = port;
        registry_.get<PortComponent>(port).add_sensor(e);
        return e;
    }

    // ── Pickers and tap ports ────────────────────────────────────────────────

    entt::entity add_picker(Vec3 home_pose, float speed_mm_s) {
        auto e = registry_.create();
        registry_.emplace<TransportComponent>(e);
        auto& pt = registry_.emplace<PickerTransportComponent>(e);
        pt.set_home_pose(home_pose);
        pt.set_speed_mm_s(speed_mm_s);
        auto& pose = registry_.emplace<PoseComponent>(e);
        pose.position = home_pose;
        pose.parent   = root_entity();
        return e;
    }

    // Whimsical placeholder for transports we haven't fully designed yet.
    // Same dispatch contract as add_picker — the station treats both the
    // same way; only the visible motion differs.
    entt::entity add_magic_transport(Vec3 home_pose, float leg_duration_s = 1.5f) {
        auto e = registry_.create();
        registry_.emplace<TransportComponent>(e);
        auto& mt = registry_.emplace<MagicTransportComponent>(e);
        mt.set_home_pose(home_pose);
        mt.set_leg_duration_s(leg_duration_s);
        mt.set_leg_origin(home_pose);
        auto& pose = registry_.emplace<PoseComponent>(e);
        pose.position = home_pose;
        pose.parent   = root_entity();
        return e;
    }

    // Add a port at distance `position_mm` along the belt's direction from
    // the entry port. The new port is parented to the belt's entry port so
    // its world transform follows the belt's orientation automatically.
    entt::entity add_tap_port(entt::entity belt, const std::string& name, int position_mm) {
        auto& bc      = registry_.get<ConveyorBeltComponent>(belt);
        auto entry_e  = bc.entry_port();
        assert(entry_e != entt::null && "connect_belt must be called before add_tap_port");

        auto e   = registry_.create();
        auto& pc = registry_.emplace<PortComponent>(e);
        pc.set_name(name);
        auto& pose = registry_.emplace<PoseComponent>(e);
        // Local +x of the entry port aligns with belt direction by construction
        // (add_port set its yaw from the forward vector, which connect_belt
        // implicitly assumes matches the belt's dir).
        pose.position = Vec3{static_cast<float>(position_mm), 0.f, 0.f};
        pose.parent   = entry_e;
        bc.add_tap_port(e);
        return e;
    }

    // Promote a port (typically a tap port) to a gate port — its blocked
    // sensors will now freeze the belt.
    void make_gate_port(entt::entity belt, entt::entity port) {
        registry_.get<ConveyorBeltComponent>(belt).add_gate_port(port);
    }

    // For "claim-and-fill" stations (palletizer, drill, anything where the
    // workpiece stays put and is later released back into flow), the gating
    // and detection roles need separate ports at the same belt position:
    //
    //   - The gate port carries the station-driven virtual sensor that
    //     halts the belt while the station has work to do. It is in the
    //     belt's gate_ports.
    //   - The detect port carries a laser the station polls to find the
    //     newly arrived workpiece. It is NOT in gate_ports — otherwise
    //     after the station releases the workpiece the laser would still
    //     be tripped by the workpiece's body sitting at the tap, and the
    //     belt could never move it forward.
    //
    // Returns both, plus the virtual sensor handle the station writes to.
    struct ClaimStationTaps {
        entt::entity detect_port;
        entt::entity gate_port;
        entt::entity virtual_sensor;
    };
    ClaimStationTaps add_claim_station_taps(entt::entity       belt,
                                            int                position_mm,
                                            const std::string& name_prefix = "tap")
    {
        ClaimStationTaps out;
        out.gate_port      = add_tap_port(belt, name_prefix + "_gate",   position_mm);
        make_gate_port(belt, out.gate_port);
        out.virtual_sensor = add_virtual_sensor(out.gate_port);

        out.detect_port    = add_tap_port(belt, name_prefix + "_detect", position_mm);
        add_laser_sensor(out.detect_port);
        return out;
    }

    // ── Items ────────────────────────────────────────────────────────────────

    entt::entity add_prototype(int length_mm, int width_mm, int height_mm, uint32_t color_hex) {
        auto e  = registry_.create();
        auto& p = registry_.emplace<ItemPrototypeComponent>(e);
        p.set_length_mm(length_mm);
        p.set_width_mm(width_mm);
        p.set_height_mm(height_mm);
        p.set_color_hex(color_hex);
        return e;
    }

    entt::entity add_source(float rate_per_hour, entt::entity prototype, entt::entity out_port) {
        auto e   = registry_.create();
        auto& sc = registry_.emplace<SourceComponent>(e);
        sc.set_rate_per_hour(rate_per_hour);
        sc.set_prototype(prototype);
        sc.set_out_port(out_port);
        return e;
    }

    entt::entity add_sink(entt::entity exit_port) {
        auto e   = registry_.create();
        registry_.emplace<SinkComponent>(e).set_in_port(exit_port);
        return e;
    }

    // ── Declarative workflow API ─────────────────────────────────────────────
    //
    // Higher-level than the add_belt / add_port / add_source primitives: the
    // user describes the *flow graph* (which sources feed which station,
    // which station feeds which sink) and solve_workflow() figures out the
    // belt/port/sensor layout. The lower-level primitives stay available
    // for power users and the workflow solver itself.
    //
    // v0 of the workflow solver only handles the palletizer pattern (one
    // pallet source + one box source + one palletizer station + one sink).
    // The layout it produces matches the manually-wired demo: a 6 m pallet
    // belt with a mid-tap, a 2.8 m box belt at z=800, lasers at every port,
    // a magic transport at home. Future iterations: cell-bounds-aware
    // placement, multi-station, other station archetypes.

    entt::entity declare_source(float rate_per_hour, entt::entity prototype) {
        auto e = registry_.create();
        auto& sc = registry_.emplace<SourceComponent>(e);
        sc.set_rate_per_hour(rate_per_hour);
        sc.set_prototype(prototype);
        // out_port is left null — solve_workflow() fills it in.
        return e;
    }

    entt::entity declare_sink() {
        auto e = registry_.create();
        registry_.emplace<SinkComponent>(e);
        // in_port left null — solve_workflow() fills it in.
        return e;
    }

    entt::entity declare_palletizer_station(entt::entity pallet_proto,
                                            entt::entity box_proto)
    {
        auto e = registry_.create();
        registry_.emplace<StationComponent>(e);
        auto& palc = registry_.emplace<PalletizeComponent>(e);
        palc.set_pattern(std::make_shared<GridPattern>());
        if (const auto* p = registry_.try_get<ItemPrototypeComponent>(pallet_proto)) {
            palc.set_pallet_dimensions(p->length_mm(), p->width_mm(),
                                       p->height_mm(), 1500);
        }
        registry_.emplace<PalletizerInputs>(e, pallet_proto, box_proto);
        return e;
    }

    void declare_flow(entt::entity from, entt::entity to) {
        flows_.push_back({from, to});
    }

    // Build belts, ports, sensors, picker, and wire everything to the
    // declared sources / sinks / station.
    void solve_workflow();

private:
    struct WorkflowFlow {
        entt::entity from;
        entt::entity to;
    };
    std::vector<WorkflowFlow> flows_;
};

inline void FactoryScene::solve_workflow() {
    auto& reg = registry_;

    // ── Find the palletizer station and its declared input prototypes ───
    // (v0: assume a single palletizer.)
    entt::entity palletizer   = entt::null;
    entt::entity pallet_proto = entt::null;
    entt::entity box_proto    = entt::null;
    {
        auto view = reg.view<PalletizerInputs>();
        auto it   = view.begin();
        if (it == view.end()) return;
        palletizer            = *it;
        const auto& inputs    = view.get<PalletizerInputs>(palletizer);
        pallet_proto          = inputs.pallet_proto;
        box_proto             = inputs.box_proto;
    }

    // ── Match flows to the actual entities ──────────────────────────────
    entt::entity pallet_source = entt::null;
    entt::entity box_source    = entt::null;
    entt::entity pallet_sink   = entt::null;
    for (const auto& f : flows_) {
        if (f.to == palletizer) {
            if (const auto* sc = reg.try_get<SourceComponent>(f.from)) {
                if (sc->prototype() == pallet_proto) pallet_source = f.from;
                else if (sc->prototype() == box_proto) box_source  = f.from;
            }
        }
        if (f.from == palletizer && reg.any_of<SinkComponent>(f.to)) {
            pallet_sink = f.to;
        }
    }

    // ── Lay out the pallet belt: source → tap → sink, +y direction ──────
    auto pallet_belt = add_belt(800, 6000, 0, 1000.f, {0.f, 1.f, 0.f});
    auto pal_entry   = add_port("pal_entry", {0.f, -3000.f, 0.f}, {0.f, 1.f, 0.f});
    auto pal_exit    = add_port("pal_exit",  {0.f,  3000.f, 0.f}, {0.f, 1.f, 0.f});
    connect_belt(pallet_belt, pal_entry, pal_exit);
    set_port_transport(pal_entry, pallet_belt);

    auto pal_tap = add_claim_station_taps(pallet_belt, 3000, "pal_tap");
    add_laser_sensor(pal_entry);
    add_laser_sensor(pal_exit);

    // ── Lay out the box belt: source → station, +x at z=800 ─────────────
    auto box_belt  = add_belt(300, 2800, 800, 1000.f, {1.f, 0.f, 0.f});
    auto box_entry = add_port("box_entry", {-2400.f, 0.f, 800.f}, {1.f, 0.f, 0.f});
    auto box_exit  = add_port("box_exit",  {  400.f, 0.f, 800.f}, {1.f, 0.f, 0.f});
    connect_belt(box_belt, box_entry, box_exit);
    set_port_transport(box_entry, box_belt);

    add_laser_sensor(box_entry);
    add_laser_sensor(box_exit);
    auto box_exit_virt = add_virtual_sensor(box_exit);

    // ── Wire sources / sinks to the appropriate ports ───────────────────
    if (pallet_source != entt::null)
        reg.get<SourceComponent>(pallet_source).set_out_port(pal_entry);
    if (box_source != entt::null)
        reg.get<SourceComponent>(box_source).set_out_port(box_entry);
    if (pallet_sink != entt::null)
        reg.get<SinkComponent>(pallet_sink).set_in_port(pal_exit);

    // ── Add the picker and finish wiring the station ────────────────────
    auto picker = add_magic_transport(Vec3{-1000.f, 0.f, 1500.f}, 1.5f);

    auto& sc = reg.get<StationComponent>(palletizer);
    sc.set_arrival_port(box_exit);
    sc.set_arrival_virtual_sensor(box_exit_virt);
    sc.add_picker(picker);

    auto& palc = reg.get<PalletizeComponent>(palletizer);
    palc.set_pallet_arrival_port(pal_tap.detect_port);
    palc.set_pallet_tap_virtual_sensor(pal_tap.virtual_sensor);
}

}  // namespace factory
