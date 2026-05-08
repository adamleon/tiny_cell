#pragma once
#include <cmath>
#include <entt/entt.hpp>
#include <unordered_map>
#include "components.hpp"
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

    // Physical sensor: parented to `port` with `local_offset` and a detection
    // volume. The scan system updates `blocked` from item poses each tick.
    entt::entity add_physical_sensor(entt::entity port,
                                     int length_mm, int width_mm, int height_mm,
                                     Vec3 local_offset = Vec3{0.f}) {
        auto e = registry_.create();
        registry_.emplace<SensorComponent>(e);
        registry_.emplace<DetectionVolumeComponent>(e)
                 .set_dimensions(length_mm, width_mm, height_mm);
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
};

}  // namespace factory
