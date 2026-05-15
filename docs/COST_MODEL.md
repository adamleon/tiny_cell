# Cost model — robot arms

A per-arm annual lifecycle cost estimator. Given a `RobotArmSpec` (commercial
+ mechanical data from the catalog), a `TaskParams` (move payload from A to
B in a cycle time), and `ExternalParams` (electricity price, operating
hours), returns an `annual_cost_eur` for ranking candidate arms.

Source: [`src/factory_scene/cost_model.hpp`](../src/factory_scene/cost_model.hpp).
Catalog: [`assets/robots/kuka/catalog.json`](../assets/robots/kuka/catalog.json) — covers Agilus, Cybertech, and Quantec PA families.

Absolute accuracy is not a goal; relative ranking between candidates is. The
model is intentionally simple — every coefficient is a single scalar we can
tune from validation data later.

## Annual cost

```
AnnualCost = price_purchase / lifetime_years          (annualised capex)
           + maintenance_cost_annual                  (catalog or 4% of price)
           + annual_kwh × electricity_price           (energy)
```

Note: only **kWh × €/kWh** for the energy half. Demand charges, peak tariffs,
heat offsets, regenerative-power export — all out of scope.

## Per-cycle energy

```
E_cycle = E_kinetic + E_gravity_lift + E_gravity_hold + E_friction + E_standby
```

```
m_eff           = payload_mass + α · mass_robot
v_peak          = 1.5 · distance / cycle_time
reach_util      = operating_distance / reach_max
config_penalty  = 1 + β · reach_util²

E_kinetic       = 0.5 · m_eff · v_peak² · (1 − η_regen) / η_drive · config_penalty
E_gravity_lift  = max(0, payload_mass · g · vertical_lift) · gravity_factor
E_gravity_hold  = payload_mass · g · operating_distance · cycle_time · 0.5 / η_drive
E_friction      = k_friction · m_eff · distance / η_drive
E_standby       = power_idle · cycle_time
```

Annual energy:

```
cycles_per_year = operating_hours_per_year · 3600 / cycle_time
annual_kwh      = E_cycle · cycles_per_year / 3.6e6
```

## Constants

Fixed in the model — not user-tunable per the task spec.

| Symbol     | Name in code                  | Value | Meaning                                         |
|------------|-------------------------------|-------|-------------------------------------------------|
| α          | `kEffectiveMassFraction`      | 0.3   | Fraction of robot mass treated as moving inertia |
| β          | `kReachPenaltyCoeff`          | 1.8   | Reach-utilisation penalty coefficient            |
| k_friction | `kFrictionPerKgM`             | 3.0   | Friction loss, J/(m·kg)                          |
| η_drive    | `kDrivetrainEfficiency`       | 0.8   | Drivetrain (gear + motor) efficiency             |
| η_regen    | `kRegenEfficiencyOff/On`      | 0.05 / 0.30 | Regenerative-braking efficiency (off / on)  |
| g          | `kGravity`                    | 9.81  | m/s²                                             |
| —          | `kPeakSpeedFactor`            | 1.5   | v_peak = factor × distance / cycle_time          |
| —          | `kAccelEstimateFactor`        | 4.5   | required_accel ≈ factor × distance / cycle_time² |

## Feasibility filter

Before computing cost, eliminate infeasible candidates:

| Constraint              | Source field                           |
|-------------------------|----------------------------------------|
| payload ≤ payload_max   | `RobotArmSpec.payload_max_kg`          |
| operating_distance ≤ reach_max | `RobotArmSpec.reach_max_m`     |
| operating_distance ≥ reach_min (if `reach_min_m > 0`) | `RobotArmSpec.reach_min_m` |
| v_peak ≤ speed_max      | `RobotArmSpec.speed_max_m_s`           |
| required_accel ≤ acceleration_max (if `> 0`) | `RobotArmSpec.acceleration_max_m_s2` |

Infeasible candidates return `{feasible = false, annual_cost_eur = +∞, infeasible_reason = ...}`.

## Catalog schema

JSON catalog at `assets/robots/<brand>/catalog.json`. Top-level key is
`robots`, an array of entries:

```jsonc
{
  "name": "KR6_R700_2",
  "urdf": "urdf/kr6_r700_2.urdf",        // path relative to catalog file
  "price_purchase_eur": 32000.0,
  "mass_robot_kg": 52.0,
  "payload_max_kg": 6.0,
  "reach_max_m": 0.706,
  "reach_min_m": 0.0,                     // optional, default 0
  "speed_max_m_s": 2.0,
  "power_idle_w": 250.0,                  // optional, resolved from controller_class
  "power_peak_w": 1500.0,                 // informational; not used by base model
  "acceleration_max_m_s2": 12.0,          // 0 / null → no acceleration constraint
  "maintenance_cost_annual_eur": null,    // null → 4% of price_purchase_eur
  "lifetime_years": null,                 // null → 14
  "regen_capable": false,
  "controller_class": "medium"            // small | medium | large → 150 / 250 / 400 W idle
}
```

Defaults are resolved at load time so downstream code sees concrete values
only.

## Approximations baked in

1. **Kinetic energy is lumped, not joint-by-joint.** `m_eff = payload + α · robot_mass` is a crude rigid-body proxy.
2. **No payload-dependent dynamics.** Heavier payloads add to kinetic / friction / gravity terms but don't shift the speed envelope.
3. **No motion planning.** Distance is straight-line A→B; real paths are slightly longer (and this part of the model already has the `config_penalty` knob to absorb that).
4. **No duty-cycle modulation.** The model assumes the arm is moving on every cycle for the full operating hours.
5. **Energy components are additive.** Real motor losses are non-linear with velocity and load; we treat them independently.

These can be replaced incrementally without changing the function signature.

## Validation strategy

The per-cycle constants (α, β, k_friction, etc.) are calibration knobs. The
validation campaign (planned, not yet built) will:
1. Pick 3–5 real robot installations with measured annual energy use.
2. Run the model with their nominal task parameters.
3. Tune the constants to minimise total error across all five.
4. Report bounded error — biased direction yet to be confirmed.

For now, the constants are spec defaults; numerical values from the model
should be read as relative rankings, not exact yearly cost predictions.
