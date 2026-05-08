#pragma once
#include <memory>
#include <vector>
#include <entt/entt.hpp>
#include "placement_pattern.hpp"
#include "components.hpp"

namespace factory {

// Station: pure dispatcher. Reads sensors, assigns idle pickers to incoming
// items, writes virtual sensors. Does NOT own items.
struct StationComponent {
    entt::entity                     arrival_port()           const { return arrival_port_; }
    entt::entity                     arrival_virtual_sensor() const { return arrival_virtual_sensor_; }
    const std::vector<entt::entity>& pickers()                const { return pickers_; }

    void set_arrival_port(entt::entity e)           { arrival_port_           = e; }
    void set_arrival_virtual_sensor(entt::entity e) { arrival_virtual_sensor_ = e; }
    void add_picker(entt::entity e)                 { pickers_.push_back(detail::valid_entity(e)); }

private:
    entt::entity              arrival_port_           = entt::null;
    entt::entity              arrival_virtual_sensor_ = entt::null;
    std::vector<entt::entity> pickers_;
};

// Palletize: type-specific addition to a station. Combined with
// StationComponent on the same entity. The station has a pallet to fill iff
// `current_pallet != null` — no separate state enum needed.
struct PalletizeComponent {
    entt::entity      pallet_arrival_port()       const { return pallet_arrival_port_; }
    entt::entity      pallet_tap_virtual_sensor() const { return pallet_tap_virtual_sensor_; }
    entt::entity      current_pallet()            const { return current_pallet_; }
    PlacementPattern* pattern()                   const { return pattern_.get(); }
    int               pallet_length_mm()          const { return pallet_length_mm_; }
    int               pallet_width_mm()           const { return pallet_width_mm_; }
    int               pallet_height_mm()          const { return pallet_height_mm_; }
    int               pallet_max_stack_mm()       const { return pallet_max_stack_mm_; }

    void set_pallet_arrival_port(entt::entity e)       { pallet_arrival_port_       = e; }
    void set_pallet_tap_virtual_sensor(entt::entity e) { pallet_tap_virtual_sensor_ = e; }
    void set_current_pallet(entt::entity e)            { current_pallet_            = e; }
    void set_pattern(std::shared_ptr<PlacementPattern> p) { pattern_ = std::move(p); }

    void set_pallet_dimensions(int length, int width, int height, int max_stack) {
        pallet_length_mm_    = detail::positive(length);
        pallet_width_mm_     = detail::positive(width);
        pallet_height_mm_    = detail::positive(height);
        pallet_max_stack_mm_ = detail::positive(max_stack);
    }

private:
    entt::entity                      pallet_arrival_port_       = entt::null;
    entt::entity                      pallet_tap_virtual_sensor_ = entt::null;
    entt::entity                      current_pallet_            = entt::null;
    std::shared_ptr<PlacementPattern> pattern_;
    int                               pallet_length_mm_          = 1200;
    int                               pallet_width_mm_           = 800;
    int                               pallet_height_mm_          = 145;
    int                               pallet_max_stack_mm_       = 1500;
};

}  // namespace factory
