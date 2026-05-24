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

Concretely: name strategies after the *equipment type only*, not `(equipment × task)`. Use `ArmStrategy` (one class covering `Palletize`, `Transport`, `Assemble`), not `ArmPalletizeStrategy` + `ArmTransportStrategy` + `ArmAssembleStrategy`. Shape tasks equipment-agnostically (`Palletize(item, pallet, count)`, not `PalletizeWithArm`) so the OR-node fan-out has multiple strategies to choose between. See `decisions.md`.

```
Strategy.evaluate(task, context) -> StrategyResult {
  feasibility:    FULL | PARTIAL | INFEASIBLE,
  equipment:      EquipmentInstance | null,   // a SPECIFIC catalog model — a CANDIDATE (see glossary)
  shareable:      bool,                        // can this instance serve other tasks?
  energy_per_cycle: joules,                    // primary objective (units: §6)
  cycle_time:     seconds,                      // this task's contribution
  poses:          { pickup?, dropoff?, anchor?, segment? },
  requires_state:     predicate(ItemState),         // STATE precondition (§4)
  effect:             ItemState -> ItemState,       // how this equipment mutates planner state (§4)
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

A station's footprint is the **accumulated** footprint of its equipment (equipment poses are relative to the station frame). Hull and union are **one unified object** (shared invalidation — they describe the same footprint at two fidelities, so they must never drift apart), but the **union is built lazily**: most collision checks are resolved by the hull broad-phase and never need it (see `solver.md` "Geometry flattening & caching", broad/narrow phase).

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

## 4. Items and state [MVP]

**Principle: properties, not types.** Strategies reason about item *properties* (symmetry, dimensions, future surface / rigidity / chirality) via shared helpers — never via item-type switches. Adding a new shape becomes "a different set of property values," not "a new code path." The helper layer (`distinct_alignments`, `orientation_known`, `orientation_pinned`, `control_matches_alignment`, future `graspable_by` / `stable_on` / …) is the seam against the nested if-trees that an item-type switch invites. See `decisions.md` "Item properties drive strategy gating, not item type."

Four conceptually distinct things, often conflated:

| | Owned by | Mutates? |
|---|---|---|
| **ItemPhysical** — static physical-spec fields (dimensions, mass, symmetry; future surface/rigidity/chirality) | `core/model/<item>.hpp` (BoxSpec, PalletSpec, …) | Never |
| **ItemState** — planner-tracked facts about ONE item in the workflow (pose precision/control, carrier placement) | Threaded through workflow | Each task |
| **Sensor capability** — what a strategy can OBSERVE about an item (camera updates Knowledge axis; laser updates one position axis) | The sensor strategy class | Static per strategy |
| **Actor requirement + effect** — what an actor needs to operate and how it changes state (arm reads Knowledge; pusher reads Control; both update both axes on placement) | The actor strategy class | Static per strategy |

### 4.1 ItemPhysical and the spec types [MVP]

Every item type (Box, Pallet, future Tote / Bottle / …) carries the same shape of physical properties:

```
ItemPhysical {
  width, length, height: Length,
  mass:                  Mass,
  symmetry:              RotationalSymmetry,
}

BoxSpec    { physical: ItemPhysical }
PalletSpec { physical: ItemPhysical }
```

`BoxSpec` and `PalletSpec` remain distinct types (composition, not inheritance / not aliases) so a task like `PalletizeParams{ BoxSpec item; PalletSpec pallet; }` can't accidentally swap the roles. New shared physical fields land on `ItemPhysical` once; new item-type-specific fields land on the wrapper.

**Future:** when the workflow shifts from threading specs to threading individual instances, instances will be `{ const <Spec>* spec; ItemState state; }` wrappers. Today's shapes are designed so this becomes a mechanical wrap.

### 4.2 RotationalSymmetry [MVP]

Static property on `ItemPhysical`. Three states, variant-of-tagged-types so invariants hold by construction:

```
RotationalSymmetry =
  | Continuous                      // cylinder; any rotation maps to self
  | Discrete { period_deg: int }    // smallest self-mapping rotation
  | Asymmetric                      // only the identity (360°) maps to self
```

`period_deg ∈ (0, 360)`, rotation only (reflections not modelled; chirality is moot for boxes/pallets, and mixing reflection in muddies the alignment-counting):

| Item | Symmetry | Alignments at 90° cardinal grid |
|---|---|---|
| Cylinder | `Continuous` | 1 (trivially) |
| Hexagon | `Discrete(60)` | 2 ({0°,180°} / {90°,270°}) |
| Square / 4-fold | `Discrete(90)` | 1 (all cardinals equivalent) |
| Equilateral triangle | `Discrete(120)` | 4 (no cardinal pairs share an orbit) |
| Rectangle / 2-fold | `Discrete(180)` | 2 ({0°,180°} long-along-X / {90°,270°} long-along-Y) |
| Asymmetric block | `Asymmetric` | 4 (all cardinals distinct) |

An **alignment** is one of the symmetry-equivalence classes of orientations at the snap grid the cell uses. For Phase 1 we assume the 90° cardinal grid; the snap step is a parameter to `distinct_alignments` for future fixtures that use a different geometry.

### 4.3 ItemState — the dynamic planner state [MVP]

```
ItemState {
  position_known: bool,
  orientation: {
    knowledge: OrientationKnowledge,
    control:   OrientationControl,
  },
  on_carrier: OnCarrier { Pallet | Belt | Free | Fixture },
}

OrientationKnowledge =
  | Unknown                          // no observation
  | Known { alignment: int }         // observed (e.g., camera) to be in this alignment

OrientationControl =
  | Free                             // no physical constraint
  | Constrained { alignment: int }   // physically pinned (e.g., fixture, side-guides)
```

**Knowledge vs Control — two independent axes.** A camera reads alignment passively without changing the physical state — it writes `knowledge`, not `control`. A fixture physically pins the item to a specific alignment — it writes `control` (and by inference may also collapse `knowledge`). They can diverge: a rectangle observed by a camera (Knowledge = Known(0)) can still rotate on the belt before the next station consumes it (Control = Free). The next station's failure mode dictates which axis it reads:

- **Arm-style strategies** plan their motion from the planner's belief and have end-effector compliance to absorb minor drift — they read `knowledge`.
- **Pusher-style strategies** have open-loop strokes (no compensation once contact starts) — they require the item to be physically held in the right alignment, reading `control`.

This is the load-bearing distinction. Camera + arm: fine, the arm uses the belief. Camera + pusher: NOT fine — the item could rotate between observation and contact. Camera + fixture + pusher: fine, the fixture provides the physical hold.

### 4.4 The property → strategy helper layer [MVP]

Strategy predicates compose these helpers rather than switching on item-type or on `RotationalSymmetry::Kind`:

```
distinct_alignments(symmetry, snap_step_deg = 90) -> int
  Continuous symmetry      -> 1
  Discrete(period) at K°   -> count of cardinal-orbit equivalence classes
  Asymmetric at K°         -> 360 / K (no symmetry collapse)

orientation_known(symmetry, knowledge) -> bool
  Continuous symmetry      -> true (trivially)
  knowledge is Known       -> true
  knowledge is Unknown     -> false

orientation_pinned(symmetry, control) -> bool
  Continuous symmetry      -> true (no class distinction)
  control is Constrained   -> true
  control is Free          -> false

knowledge_matches_alignment(symmetry, knowledge, required) -> bool
  Continuous symmetry              -> true
  knowledge is Known{required}     -> true
  otherwise                        -> false

control_matches_alignment(symmetry, control, required) -> bool
  Continuous symmetry                 -> true
  control is Constrained{required}    -> true
  otherwise                           -> false
```

This is the seam. New strategy classes call these helpers; new symmetries (or future K/C states like partial-narrowing `Hypotheses{set}` or multi-alignment `Constrained{set}`) update the helpers in one place; no strategy code switches on item type.

**Phase 2 helpers (when needed):**
- `camera_reads(ItemPhysical) -> OrientationKnowledge` — when a CameraStrategy lands.
- `fixture_forces(FixtureSpec, RotationalSymmetry) -> OrientationControl` — when a FixtureStrategy lands.
- `graspable_by(ItemPhysical, GripperSpec) -> bool` — when grippers diversify.
- `stable_on(ItemPhysical, ItemPhysical) -> bool` — when palletize patterns get geometry-aware.

See `roadmap.md` for the staging.

### 4.5 State propagation [MVP]

Start from a user-declared initial `ItemState`. For each task in workflow order: check the chosen strategy's `requires_state` against the running state; if it passes, apply the strategy's `effect` to produce the next task's inbound state. A violation FAILs the pass at that task — no recovery-task spawning at MVP (recovery belongs with the AND-OR walker, which doesn't exist yet).

The propagator (`solver/state_propagation.{hpp,cpp}`) plays the **batch validator** role for finished candidates; the *primitive* a live solver consumes is `requires_state` + `effect` on each `StrategyResult`, threaded inline through whichever search loop is running. See `decisions.md` "State-flow primitive … is load-bearing; the batch walker … is a validator-only role."

**[deferred]** Full predicate/effect planner; recovery-task spawning; probabilistic / confidence-weighted knowledge; multi-alignment `Hypotheses` / `Constrained-set` variants (for partial-narrowing sensors and generous fixtures); per-DOF position knowledge/control (currently `position_known: bool`); explicit frame composition for orientation across frames.

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
