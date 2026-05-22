# Data Model

All type and schema contracts. **No algorithms here** — see `solver.md` for how these are used. Terms are defined in `README.md` glossary.

---

## 1. Task

Atomic unit of work, coarse granularity. Compound tasks (e.g. palletize N items) are **not** expanded into N subtasks — a task owns its internal repetition; cycle time = `unit_time × count`, and shared equipment must account for total occupancy time.

Tasks form an AND-OR graph, built **lazily** (expand on demand, never the full graph upfront).

```
Task {
  id,
  kind,            // e.g. DetectPose | Transport | Palletize | Grip | ...
  params,          // item, pallet, from/to, count, etc. — kind-specific
}
```

## 2. Strategy

Matched to tasks by capability, not name:

```
Strategy.applies_to(task) -> bool
```

A strategy may apply to multiple task *kinds* (e.g. a pusher applies to both "push off belt" and "palletize"). Do **not** wire strategies to task names — capability matching is what enables strategy-class reuse.

```
Strategy.evaluate(task, context) -> StrategyResult {
  feasibility:    FULL | PARTIAL | INFEASIBLE,
  equipment:      EquipmentInstance | null,   // a SPECIFIC catalog model — a CANDIDATE (see glossary)
  shareable:      bool,                        // can this instance serve other tasks?
  energy_per_cycle: joules,                    // primary objective (units: §6)
  cycle_time:     seconds,                      // this task's contribution
  poses:          { pickup?, dropoff?, anchor?, segment? },
  requires_state: predicate(ItemState),        // STATE precondition (§4)
  effect:         ItemState -> ItemState,      // how this equipment mutates item state (§4)
  preconditions:  [Task],                       // STRUCTURAL preconditions — AND children
  partial_info:   { achievable_ct, target_ct } | null,
}
```

- **Capex is NOT on the strategy.** It lives on the catalog entry, resolved at binding time (same strategy + same model = same price regardless of task). See `decisions.md`.
- **Energy IS on the strategy** — it depends on the motion the task requires.

### 2.1 Guards vs. preconditions

| | Guard | Precondition (Task) |
|---|---|---|
| Nature | Pass/fail predicate | Solvable sub-goal |
| Solvable? | No | Yes — by adding equipment |
| On failure | Return INFEASIBLE immediately, spawn nothing | Spawn child Task, recurse |
| Example | "item fits pallet pattern"; "pattern is simple grid"; "item is pushable" | "item must be gripped"; "item pose must be known" |

Distinguishing these saves the solver from trying to spawn equipment to fix unfixable geometry.

## 3. Equipment Catalog [MVP]

Fixed, user-provided library of real equipment with real limitations. The author supplies most entries; **implementations must not invent specs** — a missing entry is an input error, not something to fabricate.

```
CatalogEntry {
  id, model_name, vendor,
  category:        ARM | GRIPPER | CONVEYOR | PUSHER | CAMERA | FIXTURE | ...,
  footprint:       polygon (2D),
  reach_envelope:  radius or polygon (arms),
  payload:         kg,
  max_speed:       per relevant axis,
  repeatability:   mm,
  power_draw:      W (or an energy-per-motion model),
  standby_power:   W,             // idle/holding draw (§6)
  list_price:      currency,
}
```

**Two geometry classes** (matters for placement & cost):
- **Point/footprint** (arm, gripper, camera, pusher, fixture): anchors to a pose; footprint & cost fixed at binding.
- **Segment** (conveyor): connects from-pose → to-pose; length/footprint/capex/energy are **layout-dependent**, re-evaluated each optimizer iteration.

### 3.1 Station footprint cache

A station's footprint is the **accumulated** footprint of its equipment (equipment poses are relative to the station frame). Hull and union are **one unified object** (shared invalidation — they describe the same footprint at two fidelities, so they must never drift apart), but the **union is built lazily**: most collision checks are resolved by the hull broad-phase and never need it (see `solver-v2.md` "Geometry flattening & caching", broad/narrow phase).

```
StationFootprint {
  hull_local:   Polygon,             // convex hull, STATION frame — always built when dirty_local cleared (broad-phase)
  union_local:  optional<Polygon>,   // true non-convex union, STATION frame — built lazily on first narrow-phase need
  // both buffered outward by governing standards clearance at build time
  world_hull:   Polygon,             // hull_local → world; cached, invalidated when station moves
  world_union:  optional<Polygon>,   // union_local → world; lazy, only if union_local exists and narrow-phase reached
  dirty_local:  bool,                // equipment moved within station → rebuild hull_local, drop union_local
  dirty_world:  bool,                // station moved → re-transform to world only
}
```

- **One object, one invalidation trigger per level** — hull and union never cached separately, so they cannot drift.
- **Union is `optional`, populated on demand:** a check asks for the hull first; only if hulls overlap does it request the union, which builds + caches it until the next `dirty_local`. Stations that never get into a close call this iteration never pay union cost.
- `hull_local`/`world_hull` rebuilt/re-transformed eagerly; `union_local`/`world_union` dropped on `dirty_local` and rebuilt only when next needed.
- **Hull has independent consumers** beyond collision — a cheap "how much floor does this station want" proxy for Layer 2 coarse spatial reasoning and positional-prior spacing — so it stays a named field, not hidden behind a generic footprint accessor.
- `world_*` recomputed only when the station's frame-to-world transform changes — one transform on one polygon, not a re-flatten of equipment.
- Clearance buffer baked in at `*_local` build time (not per collision check).
- **[deferred / see architecture]** `world_hull`/`world_union` may be unified with a general frame-world-pose cache rather than maintained as a second independent world cache — resolve before implementing to avoid two parallel caches that must stay in sync.

## 4. Item State [MVP]

A small, fixed vector propagated forward through the workflow DAG, mutated by each chosen strategy's `effect`. Checked against each strategy's `requires_state` at the node where that strategy sits.

```
ItemState {
  position_known:          bool,
  orientation_resolved_to: CONTINUOUS | DISCRETE(n) | EXACT | UNKNOWN,
  on_carrier:              PALLET | BELT | FREE | FIXTURE,
}
```

**Propagation:** start from user-declared initial state at input; apply each strategy's `effect` in flow order; check `requires_state` against state at that node. A violated state precondition spawns a recovery task **at that point** (not earlier — where the state was already satisfied, recovery equipment would be wasted).

Some equipment exists purely to mutate state (a fixture forcing orientation, a camera establishing pose). Valid strategy whose `effect` is the whole point.

**[deferred]** This is a fixed small vector, not a general predicate/effect planner. Generalize only if a real case demands it.

### 4.1 Orientation as symmetry-aware precision [MVP]

```
ItemSymmetry {
  z_rotation: CONTINUOUS | DISCRETE(n) | NONE,
  flippable:  bool,
}
```

**Satisfaction rule:** an orientation requirement is met when *what the task needs resolved* ⊇ *what symmetry collapses*. Reduce required precision by symmetry before checking against available precision.
- Bottle (CONTINUOUS) + any pick → orientation requirement vanishes; no orientation sensing.
- Square box (DISCRETE 4) + grid palletize → need pose only mod 90°; coarse sensor suffices.
- Asymmetric (NONE) + oriented placement → full pose; pay for vision or forcing fixture.

**[MVP scope]** z-rotation symmetry + flippable only. Full SO(3) symmetry **[deferred]**.

## 5. Output format [MVP]

The deliverable:

```
LayoutSolution {
  equipment:  [ { instance_id, catalog_id, pose:(x,y,θ) | segment:(from,to) } ],
  transfers:  [ { source_equipment, sink_equipment, transfer_method } ],   // explicit graph for validation/simulation
  seed,                                                                      // reproducibility
  metrics:    { energy_kwh, capex, cell_cycle_time, ... },
}
```

**[deferred]:** 3D models, full simulation, BOM/costing report, GUI visualization.

## 6. Energy units

**Internal unit is joules** (an amount), not watts (a rate). `energy_per_cycle` is energy per one cycle. Total cell energy has two terms:

```
energy = Σ_tasks (energy_per_cycle × cycles)        // active motion
       + Σ_instances (standby_power × idle_time)     // idle/holding draw
```

The standby term penalizes underutilized equipment — aligns with the sharing objective. **Convert to kWh only at the cost calc & UI boundary** (1 kWh = 3.6 MJ); joules compose cleanly internally and avoid unit-mixing bugs. Same quantity, display-only conversion.

## 7. Standards rules schema

Defined in `standards.md` (structured rules file the solver reads; hard constraints in Layer 3).
