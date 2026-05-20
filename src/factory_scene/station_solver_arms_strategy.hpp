#pragma once
#include <cmath>
#include <string>
#include <vector>

#include "cost_options.hpp"
#include "station_solver_strategy.hpp"

// ArmsStrategy — Phase 1.
//
// Wraps the existing `cost::palletizer_options(...)` enumerator behind the
// new `Strategy` interface. Each `PalletizerOption` (one `(num_pickers,
// arm_model)` combination) becomes one terminal `Proposal` with the matching
// equipment list and annual cost.
//
// This is a thin adapter — no new math, no new search. It validates that the
// interface fits the existing implementation and gives us a real strategy
// to feed the upcoming solver loop in Phase 2.
//
// Tunables for the palletizer station mechanism (frame, dispenser, claim
// logic) and the per-arm task heuristics live on the strategy as
// constructor-injected configuration so callers can override without
// changing the shared StrategyContext.

namespace factory::station_solver {

class ArmsStrategy final : public Strategy {
public:
    // Defaults match the realistic palletizing scenario the cost-model
    // tests already cover: ~€15k frame, 150 W idle, €600/yr maintenance,
    // 14 yr lifetime, up to 3 pickers.
    struct Config {
        float station_mech_capex_eur                = 15000.f;
        float station_mech_power_w                  = 150.f;
        float station_mech_maintenance_annual_eur   = 600.f;
        float station_mech_lifetime_years           = 14.f;
        int   max_pickers                           = 3;
        // gravity_factor for the per-arm cycle (0.5 = cyclic A↔B return)
        float per_arm_gravity_factor                = 0.5f;
    };

    explicit ArmsStrategy(Config cfg = {}) : cfg_(cfg) {}

    const char* name() const override { return "ArmsStrategy"; }

    bool can_solve(TaskKind kind) const override {
        return kind == TaskKind::Palletize;
    }

    std::vector<Proposal> propose(const Task& task,
                                  const StrategyContext& ctx) const override
    {
        if (!can_solve(task.kind))            return {};
        if (task.throughput_items_per_minute <= 0.f) return {};
        if (task.pallet_length_mm <= 0 || task.box_length_mm <= 0) return {};

        cost::PalletizerContext pctx = build_palletizer_context(task);
        auto options = cost::palletizer_options(
            task.throughput_items_per_minute,    // pallets per minute
            ctx.catalog, pctx, ctx.external_params);

        std::vector<Proposal> out;
        out.reserve(options.size());
        for (const auto& opt : options) {
            out.push_back(option_to_proposal(opt));
        }
        return out;
    }

private:
    Config cfg_;

    // Map task geometry onto the existing PalletizerContext. Heuristic
    // defaults for per-arm task fields that aren't carried on Task: the
    // pickup→drop distance is approximated as the pallet half-diagonal;
    // vertical lift as half the max stack; operating_distance left zero
    // (cost::estimate falls back to the arm's reach midrange).
    cost::PalletizerContext build_palletizer_context(const Task& task) const {
        cost::PalletizerContext p{};
        p.geometry.pallet_length_mm    = task.pallet_length_mm;
        p.geometry.pallet_width_mm     = task.pallet_width_mm;
        p.geometry.pallet_max_stack_mm = task.pallet_max_stack_mm;
        p.geometry.box_length_mm       = task.box_length_mm;
        p.geometry.box_width_mm        = task.box_width_mm;
        p.geometry.box_height_mm       = task.box_height_mm;

        const float half_l = 0.5f * static_cast<float>(task.pallet_length_mm) * 0.001f;
        const float half_w = 0.5f * static_cast<float>(task.pallet_width_mm)  * 0.001f;
        p.per_arm_task.distance_m            = std::sqrt(half_l * half_l + half_w * half_w);
        p.per_arm_task.payload_mass_kg       = task.box_mass_kg;
        p.per_arm_task.cycle_time_s          = 0.f;  // overwritten by palletizer_options
        p.per_arm_task.vertical_lift_m       = 0.5f * static_cast<float>(task.pallet_max_stack_mm) * 0.001f;
        p.per_arm_task.gravity_factor        = cfg_.per_arm_gravity_factor;
        p.per_arm_task.operating_distance_m  = 0.f;  // resolved to arm midrange

        p.station_mech_capex_eur                = cfg_.station_mech_capex_eur;
        p.station_mech_power_w                  = cfg_.station_mech_power_w;
        p.station_mech_maintenance_annual_eur   = cfg_.station_mech_maintenance_annual_eur;
        p.station_mech_lifetime_years           = cfg_.station_mech_lifetime_years;
        p.max_pickers                           = cfg_.max_pickers;
        return p;
    }

    Proposal option_to_proposal(const cost::PalletizerOption& opt) const {
        Proposal p{};
        p.solves              = TaskKind::Palletize;
        p.strategy_name       = std::string("ArmsStrategy(N=")
                              + std::to_string(opt.num_pickers) + ", "
                              + opt.arm.name + ")";
        p.annual_cost_eur     = opt.annual_total_cost_eur;
        p.annual_cost_lb_eur  = opt.annual_total_cost_eur;   // terminal

        // N × identical arms
        for (int i = 0; i < opt.num_pickers; ++i) {
            ProposalEquipment arm{};
            arm.archetype_name   = "robot_arm";
            arm.model_or_variant = opt.arm.name;
            arm.capex_eur        = opt.arm.price_purchase_eur;
            arm.power_w          = opt.arm.power_idle_w;
            p.equipment.push_back(arm);
        }
        // Station mechanism (frame, dispenser, controllers — one per station)
        ProposalEquipment frame{};
        frame.archetype_name   = "station_frame";
        frame.model_or_variant = "generic_palletizer_frame";
        frame.capex_eur        = cfg_.station_mech_capex_eur;
        frame.power_w          = cfg_.station_mech_power_w;
        p.equipment.push_back(frame);
        return p;
    }
};

}  // namespace factory::station_solver
