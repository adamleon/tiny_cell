#pragma once
#include <memory>
#include <entt/entt.hpp>
#include "mechanisms.hpp"
#include "placement_pattern.hpp"
#include "components.hpp"

namespace factory {

enum class StationState   { Idle, Processing };
enum class PalletizeState { WaitingForPallet, HasPallet };

struct StationComponent {
    entt::entity   arrival_port()         const { return arrival_port_; }
    entt::entity   controlled_transport() const { return controlled_transport_; }
    entt::entity   held_item()            const { return held_item_; }
    StationState   state()                const { return state_; }
    IMechanism*    mechanism()            const { return mechanism_.get(); }

    void set_arrival_port(entt::entity e)         { arrival_port_         = e; }
    void set_controlled_transport(entt::entity e) { controlled_transport_ = e; }
    void set_mechanism(std::shared_ptr<IMechanism> m) { mechanism_ = std::move(m); }

    void capture(entt::entity item) {
        held_item_ = item;
        state_     = StationState::Processing;
    }

    void release() {
        held_item_ = entt::null;
        state_     = StationState::Idle;
        if (mechanism_) mechanism_->reset();
    }

private:
    entt::entity                 arrival_port_         = entt::null;
    entt::entity                 controlled_transport_ = entt::null;
    entt::entity                 held_item_            = entt::null;
    StationState                 state_                = StationState::Idle;
    std::shared_ptr<IMechanism>  mechanism_;
};

struct PalletizeComponent {
    entt::entity                       pallet_arrival_port()    const { return pallet_arrival_port_; }
    entt::entity                       pallet_output_transport() const { return pallet_output_transport_; }
    entt::entity                       pallet_segment()         const { return pallet_segment_; }
    entt::entity                       current_pallet()         const { return current_pallet_; }
    PlacementPattern*                  pattern()                const { return pattern_.get(); }
    int                                pallet_length_mm()       const { return pallet_length_mm_; }
    int                                pallet_width_mm()        const { return pallet_width_mm_; }
    int                                pallet_height_mm()       const { return pallet_height_mm_; }
    int                                pallet_max_stack_mm()    const { return pallet_max_stack_mm_; }
    PalletizeState                     state()                  const { return state_; }

    void set_pallet_arrival_port(entt::entity e)     { pallet_arrival_port_    = e; }
    void set_pallet_output_transport(entt::entity e) { pallet_output_transport_ = e; }
    void set_pallet_segment(entt::entity e)          { pallet_segment_         = e; }
    void set_current_pallet(entt::entity e)          { current_pallet_         = e; }
    void set_pattern(std::shared_ptr<PlacementPattern> p) { pattern_ = std::move(p); }
    void set_state(PalletizeState s)                 { state_ = s; }

    void set_pallet_dimensions(int length, int width, int height, int max_stack) {
        pallet_length_mm_    = detail::positive(length);
        pallet_width_mm_     = detail::positive(width);
        pallet_height_mm_    = detail::positive(height);
        pallet_max_stack_mm_ = detail::positive(max_stack);
    }

private:
    entt::entity                      pallet_arrival_port_     = entt::null;
    entt::entity                      pallet_output_transport_ = entt::null;
    entt::entity                      pallet_segment_          = entt::null;
    entt::entity                      current_pallet_          = entt::null;
    std::shared_ptr<PlacementPattern> pattern_;
    int                               pallet_length_mm_        = 1200;
    int                               pallet_width_mm_         = 800;
    int                               pallet_height_mm_        = 145;
    int                               pallet_max_stack_mm_     = 1500;
    PalletizeState                    state_                   = PalletizeState::WaitingForPallet;
};

}  // namespace factory
