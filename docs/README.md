# Robot Cell Layout Solver — Documentation

This is the documentation root. It defines shared terms and points to every other doc. **Read this first to orient, then follow the pointers.** Terms are defined here *once*; other docs reference them rather than redefining (avoid drift).

## What this software is

Given a user-specified **workflow** (raw material → tasks → finished product), a **factory-floor blueprint** (bounded region with obstacles and material I/O), a **fixed equipment catalog**, and **cycle-time requirements**, the software produces a **cell layout**: equipment instances, their poses, and the explicit item-transfer connections between them — minimizing cost (energy primary, capex secondary) subject to feasibility, geometry, and industrial-standard compliance.

The solution need not be globally optimal — approximately optimal is acceptable. Runs are seed-controlled and reproducible.

## Document map

| Doc | Contents | Status |
|---|---|---|
| `README-v2.md` | This file: glossary, map | — |
| `architecture-v2.md` | System-level module breakdown, boundaries, data flow, frames, geometry/units, ECS | Written |
| `data-model-v2.md` | All type/schema contracts (Task, Strategy, ItemState, catalog, footprint cache, output) | Written |
| `solver-v2.md` | The solver algorithm: three layers + LNS optimizer + positional prior + geometry flattening & caching | Written |
| `interaction-v2.md` | Graded interactive editing ; maps onto footprint cache levels | Written |
| `standards.md` | Compliance standards + rules-file schema | **STUB — needs your input** |
| `roadmap.md` | Build order, MVP milestone definition, open questions | Written |
| `decisions-v2.md` | Decision log: each resolved choice + one-line why | Written |

## How to use these docs (for an implementation session)

Load only what the task needs:
- Catalog parser → `data-model-v2.md` + `roadmap.md`
- AND-OR expansion → `solver-v2.md` + `data-model-v2.md`
- Standards into placement → `standards.md` + `solver-v2.md` (Layer 3)
- Interactive editor (later) → `interaction-v2.md` + `data-model-v2.md`

## Scope tags

Sections are tagged inline:
- **[MVP]** — in the first milestone
- **[v2]** / **[deferred]** — explicitly later

MVP is a *subset and sequence* of this design, not a separate design. There is no separate "MVP spec" — scope lives inline, and the *sequence* lives in `roadmap.md`.

---

## Glossary (single source of truth)

**Task** — Atomic unit of work at coarse granularity (e.g. `DetectPose(item)`, `Transport(item, from, to)`, `Palletize(item, pallet, count)`). Compound tasks own their internal repetition; cycle time = `unit_time × count`. Tasks form an **AND-OR graph**.

**AND-OR graph** — OR-nodes: a task solvable by several strategies (solver keeps the best feasible). AND-nodes: a strategy spawns several preconditions, all of which must be feasible. Built lazily.

**Strategy** — Encodes engineering knowledge for solving one *kind* of task. Matched to tasks by **capability** (`applies_to(task)`), not by name. May apply to multiple task types (enabling strategy-class reuse).

**Guard** — A pass/fail predicate inside a strategy that *nothing can fix* (e.g. "item geometry fits pallet pattern"). On failure → return INFEASIBLE immediately, spawn nothing.

**Precondition** — A *solvable* sub-goal a strategy emits, itself a Task (e.g. "item must be gripped"). On need → spawn a child Task and recurse. **Structural** preconditions always spawn equipment; **state** preconditions spawn recovery only if propagated ItemState doesn't already satisfy them.

**Candidate binding** — A strategy proposes a *specific* real equipment model (to compute reach/energy/feasibility) but as a *candidate*, not a commitment. Deciding to share one physical instance across tasks is the allocation layer's job, not the strategy's — committing inside the strategy would destroy cross-station sharing.

**ItemState** — A small, fixed, forward-propagated vector of facts about an item (position_known, orientation_resolved_to, on_carrier, …) that equipment `effect`s mutate as the item flows through the workflow. A strategy's `requires_state` is checked against the state *as it is at that node*.

**ItemSymmetry** — Describes how much orientation must be known: z_rotation ∈ {CONTINUOUS, DISCRETE(n), NONE} + flippable. A task's orientation requirement is satisfied when what it needs resolved ⊇ what symmetry already collapses (a bottle needs no rotation known; a square box only mod 90°).

**Sharing (two levels)** — *Strategy-class reuse*: one strategy applies to multiple task types. *Instance reuse*: one physical unit serves multiple tasks via candidate binding (e.g. an underutilized arm covering an adjacent station's transport task).

**Point vs. segment equipment** — Point/footprint equipment (arm, gripper, camera, pusher, fixture) anchors to a pose; cost fixed at binding. Segment equipment (conveyor) connects two poses; length/footprint/cost are **layout-dependent**, re-evaluated each optimizer iteration.

**Frame / FrameId** — A nested reference frame for poses, with its own id space (World = FrameId 1, `null` = undefined) — **not** an ECS entity id. A `Pose2D` is expressed *in* a frame; resolving across frames composes rigid **transforms** (not triplet arithmetic). See `architecture-v2.md` §4.

**FactoryCell** — The domain name for the live world state (held in the EnTT registry). Distinct from the threepp render `Scene` and from EnTT's `registry` — three different things, three names.
