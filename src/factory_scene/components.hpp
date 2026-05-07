#pragma once
#include <cassert>
#include <cmath>
#include <cstdint>
#include <entt/entt.hpp>
#include <optional>
#include <string>
#include <vector>
#include "solver/types.hpp"
#include "pose_component.hpp"

namespace factory {

// ── Validation helpers ────────────────────────────────────────────────────────

namespace detail {
    inline float finite_non_neg(float v)  { assert(std::isfinite(v) && v >= 0.f);               return v; }
    inline float unit_interval(float v)   { assert(std::isfinite(v) && v >= 0.f && v <= 1.f);   return v; }
    inline int   positive(int v)          { assert(v > 0);                                        return v; }
    inline int   non_neg(int v)           { assert(v >= 0);                                       return v; }
    inline entt::entity valid_entity(entt::entity e) { assert(e != entt::null); return e; }
}

// ── Fence layout ──────────────────────────────────────────────────────────────

struct NodeComponent {
    solver::NodeType type() const { return type_; }
    void set_type(solver::NodeType t) { type_ = t; }

private:
    solver::NodeType type_ = solver::NodeType::Post;
};

struct EdgeComponent {
    entt::entity                         node_a()      const { return node_a_; }
    entt::entity                         node_b()      const { return node_b_; }
    const std::vector<std::vector<int>>& spans_mm()    const { return spans_mm_; }
    const std::string&                   catalog_ref() const { return catalog_ref_; }

    void set_node_a(entt::entity e)                    { node_a_      = e; }
    void set_node_b(entt::entity e)                    { node_b_      = e; }
    void set_spans_mm(std::vector<std::vector<int>> s) { spans_mm_    = std::move(s); }
    void set_catalog_ref(std::string r)                { catalog_ref_ = std::move(r); }

private:
    entt::entity                  node_a_     = entt::null;
    entt::entity                  node_b_     = entt::null;
    std::vector<std::vector<int>> spans_mm_;
    std::string                   catalog_ref_;
};

struct DeclaredOpeningComponent {
    entt::entity              parent_edge()         const { return parent_edge_; }
    const std::optional<int>& desired_position_mm() const { return desired_position_mm_; }
    int                       width_mm()            const { return width_mm_; }
    float                     mobility()            const { return mobility_; }
    int                       hint_edge_index()     const { return hint_edge_index_; }

    void set_parent_edge(entt::entity e)               { parent_edge_         = e; }
    void set_desired_position_mm(std::optional<int> p) { desired_position_mm_ = p; }
    void set_width_mm(int mm)                          { width_mm_            = detail::positive(mm); }
    void set_mobility(float m)                         { mobility_            = detail::unit_interval(m); }
    void set_hint_edge_index(int i)                    { assert(i >= -1); hint_edge_index_ = i; }

private:
    entt::entity       parent_edge_         = entt::null;
    std::optional<int> desired_position_mm_;
    int                width_mm_            = 0;
    float              mobility_            = 0.f;
    int                hint_edge_index_     = -1;
};

// ── Transport / workflow ──────────────────────────────────────────────────────

struct TransportComponent {
    bool running()  const { return running_; }
    int  capacity() const { return capacity_; }

    void set_running(bool r) { running_  = r; }
    void set_capacity(int c) { capacity_ = detail::non_neg(c); }

private:
    bool running_  = true;
    int  capacity_ = 0;
};

struct ConveyorBeltComponent {
    const std::string& catalog_ref()          const { return catalog_ref_; }
    int                width_mm()             const { return width_mm_; }
    int                length_mm()            const { return length_mm_; }
    int                surface_height_mm()    const { return surface_height_mm_; }
    int                opening_clearance_mm() const { return opening_clearance_mm_; }
    float              belt_speed_mm_s()      const { return belt_speed_mm_s_; }
    Vec3               dir()                  const { return dir_; }
    entt::entity       entry_port()           const { return entry_port_; }
    entt::entity       exit_port()            const { return exit_port_; }
    bool               capacity_blocked()     const { return capacity_blocked_; }

    void set_catalog_ref(std::string r)         { catalog_ref_          = std::move(r); }
    void set_width_mm(int mm)                   { width_mm_             = detail::positive(mm); }
    void set_length_mm(int mm)                  { length_mm_            = detail::positive(mm); }
    void set_surface_height_mm(int mm)          { surface_height_mm_    = detail::non_neg(mm); }
    void set_opening_clearance_mm(int mm)       { opening_clearance_mm_ = detail::non_neg(mm); }
    void set_belt_speed_mm_s(float v)           { belt_speed_mm_s_      = detail::finite_non_neg(v); }
    void set_dir(Vec3 d)                        { dir_                  = d; }
    void set_entry_port(entt::entity e)         { entry_port_           = e; }
    void set_exit_port(entt::entity e)          { exit_port_            = e; }
    void set_capacity_blocked(bool b)           { capacity_blocked_     = b; }

private:
    std::string  catalog_ref_          = "generic/flat-belt";
    int          width_mm_             = 200;
    int          length_mm_            = 2000;
    int          surface_height_mm_    = 800;
    int          opening_clearance_mm_ = 50;
    float        belt_speed_mm_s_      = 200.f;
    Vec3         dir_                  = {1.f, 0.f, 0.f};
    entt::entity entry_port_           = entt::null;
    entt::entity exit_port_            = entt::null;
    bool         capacity_blocked_     = false;
};

// ── Ports ─────────────────────────────────────────────────────────────────────

struct PortComponent {
    const std::string& name()        const { return name_; }
    entt::entity       transport()   const { return transport_; }
    float              min_gap_mm()  const { return min_gap_mm_; }

    void set_name(std::string n)       { name_       = std::move(n); }
    void set_transport(entt::entity e) { transport_  = e; }
    void set_min_gap_mm(float v)       { min_gap_mm_ = detail::finite_non_neg(v); }

private:
    std::string  name_;
    entt::entity transport_  = entt::null;
    float        min_gap_mm_ = 0.f;
};

// ── Source / Sink ─────────────────────────────────────────────────────────────

struct SourceComponent {
    float        rate_per_hour() const { return rate_per_hour_; }
    entt::entity prototype()     const { return prototype_; }
    entt::entity out_port()      const { return out_port_; }
    float        spawn_debt()    const { return spawn_debt_; }

    void set_rate_per_hour(float r)    { rate_per_hour_ = detail::finite_non_neg(r); }
    void set_prototype(entt::entity e) { prototype_     = detail::valid_entity(e); }
    void set_out_port(entt::entity e)  { out_port_      = detail::valid_entity(e); }
    void add_spawn_debt(float d)       { assert(std::isfinite(d) && d >= 0.f); spawn_debt_ += d; }
    void consume_spawn()               { spawn_debt_ -= 1.f; }

private:
    float        rate_per_hour_ = 0.f;
    entt::entity prototype_     = entt::null;
    entt::entity out_port_      = entt::null;
    float        spawn_debt_    = 0.f;
};

struct SinkComponent {
    entt::entity in_port()  const { return in_port_; }
    int          received() const { return received_; }

    void set_in_port(entt::entity e) { in_port_ = detail::valid_entity(e); }
    void increment_received()        { ++received_; }

private:
    entt::entity in_port_  = entt::null;
    int          received_ = 0;
};

// ── Item prototypes and instances ─────────────────────────────────────────────

struct ItemPrototypeComponent {
    int      length_mm() const { return length_mm_; }
    int      width_mm()  const { return width_mm_; }
    int      height_mm() const { return height_mm_; }
    uint32_t color_hex() const { return color_hex_; }

    void set_length_mm(int mm)     { length_mm_ = detail::positive(mm); }
    void set_width_mm(int mm)      { width_mm_  = detail::positive(mm); }
    void set_height_mm(int mm)     { height_mm_ = detail::positive(mm); }
    void set_color_hex(uint32_t c) { color_hex_ = c; }

private:
    int      length_mm_ = 300;
    int      width_mm_  = 300;
    int      height_mm_ = 300;
    uint32_t color_hex_ = 0x8B4513u;
};

struct SpawnedItemComponent {
    entt::entity prototype() const { return prototype_; }
    void set_prototype(entt::entity e) { prototype_ = detail::valid_entity(e); }

private:
    entt::entity prototype_ = entt::null;
};

struct ItemOnTransportComponent {
    entt::entity transport() const { return transport_; }
    void set_transport(entt::entity e) { transport_ = e; }

private:
    entt::entity transport_ = entt::null;
};

// ── Pallet ────────────────────────────────────────────────────────────────────

struct PalletComponent {
    int                              length_mm()          const { return length_mm_; }
    int                              width_mm()           const { return width_mm_; }
    int                              height_mm()          const { return height_mm_; }
    int                              max_stack_height_mm() const { return max_stack_height_mm_; }
    const std::vector<entt::entity>& items()              const { return items_; }

    void set_length_mm(int mm)          { length_mm_          = detail::positive(mm); }
    void set_width_mm(int mm)           { width_mm_           = detail::positive(mm); }
    void set_height_mm(int mm)          { height_mm_          = detail::positive(mm); }
    void set_max_stack_height_mm(int mm){ max_stack_height_mm_ = detail::positive(mm); }
    void add_item(entt::entity e)       { items_.push_back(e); }

private:
    int                      length_mm_          = 1200;
    int                      width_mm_           = 800;
    int                      height_mm_          = 145;
    int                      max_stack_height_mm_ = 1500;
    std::vector<entt::entity> items_;
};

}  // namespace factory
