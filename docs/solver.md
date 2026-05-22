# Solver Algorithm

How the layout is computed. Types/schemas are in `data-model.md` (referenced, not repeated). Terms in `README.md` glossary.

This is a coupled **HTN planning** problem and **continuous-space layout/packing** problem. The coupling — a strategy is feasible only if its equipment physically fits, and placement cost depends on which strategies were chosen — is the central difficulty.

> **Boundary note (see `architecture.md` §3):** the solver is a **pure module operating on its own internal structs**, not on the EnTT registry. It explores thousands of candidate configurations per run and discards most; mutating live world state per candidate would thrash the renderer and couple search to ECS identity. The solver returns a `LayoutSolution`; the `sync/` step writes it into the registry. Frame math uses solver-internal `FrameId`s (see `architecture.md` §4).

---

## Architecture: three layers + outer metaheuristic

Three layers with feedback, driven by an outer Large-Neighborhood-Search loop.

### Layer 1 — Task decomposition (AND-OR / HTN) [MVP]

Lazily expand the task graph. For each task, gather strategies whose `applies_to` matches; each OR-branch evaluates, applying **guards** (reject → INFEASIBLE) and spawning **structural + state preconditions** (AND children). Produces candidate `(task → strategy)` assignments with candidate equipment requirements.

Worked example — `Palletize(item, pallet, count)`:

```
OR ── RobotArmStrategy
│       preconditions: [ Grip(item), DetectPose(item) ]
│       equipment: arm (candidate); energy: high
│
└── PusherStrategy
        guards: item.pushable ; pattern is simple grid (no interlock)
        preconditions: [ IndexPalletPosition(pallet, item_width), AlignItemOnBelt(item) ]
        equipment: pusher + indexing conveyor; energy: low
```

Pusher is often cheaper per-task but pushes complexity into preconditions (indexing conveyor, alignment). Whether it wins is for the solver to compute.

### Layer 2 — Resource allocation & sharing [MVP]

Treat equipment as **resources with a time budget and spatial reach**, not something owned by a subtree. Bind candidate requirements to physical catalog instances, deciding where one instance serves multiple tasks. Generalized assignment / bin-packing: pack tasks onto instances subject to per-instance cycle-time capacity and reachability, minimizing instance count (hence capex).

**Cross-station sharing emerges here** — an arm placed for Station 2, with spare cycle-time budget and Station 1 within reach, wins `Transport(S1→S2)` at near-zero marginal capex.

Two sharing levels (see glossary): strategy-class reuse (matching) and instance reuse (binding).

> ⚠ Naïve AND-OR cost accounting (AND = Σ children, OR = min) **overcounts** when siblings share an instance. Compute cost against *bound instances* after allocation, not summed over the tree.

### Layer 3 — Continuous layout [MVP]

Variables: `(x, y, θ)` per point-equipment instance + conveyor segment routing. Geometry is **2D** (footprints + reach circles), not 3D kinematics **[deferred]**.

**Hard constraints:** no footprint overlap; no overlap with blueprint obstacles; within floor bounds; reach feasibility (assigned task points within reach envelopes); safety clearances from `standards.md`.

**Objective:** minimize energy cost (primary) + capex + conveyor/transfer length, plus soft terms:
- **Reach-band preference:** penalty for pickup/dropoff outside 30–70 % of reach (soft — the hard reach constraint is the envelope itself).
- **Workflow positional prior** (below).

**Method:** NLP with a good initial guess (IPOPT / SLSQP for a few dozen rigid bodies); overlap as a smooth penalty during search. If infeasible, propagate back: invalidate the Layer 2 binding, possibly the Layer 1 strategy choice, record the conflicting combination.

### Outer loop — Large Neighborhood Search + simulated annealing [MVP]

1. **Construct:** greedy AND-OR expansion (best standalone strategy per task), quick capacity-packing assignment, constructive placement seeded by the positional prior.
2. **Repair to feasibility:** locally relax (rotate, swap worst-fitting strategy) until feasible; record conflicts that resist repair.
3. **Destroy-and-repair:** pick a neighborhood (2–3 adjacent stations, or all tasks using one equipment type), rip out their strategy/assignment/placement, re-solve that subset to near-optimality (small enough for exact MILP/CP-SAT if desired). Accept if better.
4. **Simulated-annealing acceptance:** accept worse solutions with probability falling as a "temperature" cools on a schedule — escapes local optima early, refines late. (Term from metallurgy: slow cooling settles a system into a low-energy state.)

Layout-dependent costs (conveyor segments) are re-evaluated each iteration.

**Solve timing (order of magnitude, 5–15 task cell):** Layer 1 ms; Layer 2 ms greedy / ~1 s exact; Layer 3 tens-of-ms to a few seconds (the slow layer). LNS runs Layers 2–3 hundreds–thousands of times → **full cold solve = seconds to a few minutes.** Fine for batch; too slow for interactive drag (see `interaction.md`).

---

## Geometry flattening & caching

The solver must **not** resolve poses to world every evaluation. Flattening is done lazily at the **coarsest stable unit** and cached, with two independent invalidation triggers. This rests on a key separation: there are **two independent placement problems**, and conflating them is what makes flattening seem expensive.

| Problem | Coordinates | Changes when… | Invalidates |
|---|---|---|---|
| **Intra-station** | equipment poses relative to **station frame** | equipment reorganized *within* a station | that station's accumulated footprint |
| **Inter-station** | station poses relative to **world** | a station moves on the floor | only that station's world footprint |

These are independent. Reorganizing equipment inside Station A changes A's internal layout and accumulated footprint but not where any other station sits. Moving Station A changes nothing about its internals (equipment-to-station-frame poses untouched, footprint shape unchanged) — only A's frame-to-world transform. That independence is what keeps the solver fast.

### Two-level flatten/cache hierarchy

```
equipment footprint (catalog, equipment's own frame)
  → [intra-station solve] → ACCUMULATED STATION FOOTPRINT (in STATION frame)
        cached; invalidated only when equipment moves within the station
  → [station moves]       → STATION FOOTPRINT IN WORLD
        cached per station; invalidated only when THAT station moves
```

World transforms happen **only at the top level, only for the station that moved, only on its single accumulated polygon.** A station move is *one polygon-transform*, never a re-flatten of its contents — this is the entire payoff of frame nesting (`architecture.md` §4).

### Where each layer flattens

- **Layer 1 (HTN):** no flattening. Footprints exist only as rough catalog *envelope estimates* for feasibility sanity ("does this plausibly fit a station at all"). No frames resolved.
- **Layer 2 (allocation/sharing):** one spatial check — reachability for sharing. Uses the **relative transform between two station frames** (cheaper, more stable than absolute world poses), and only a **coarse station-relative distance** — Layer 2 decides whether sharing is *plausible*; Layer 3 confirms geometrically. Do not drag world flattening into the allocation loop.
- **Layer 3 intra-station:** flatten equipment to the **station frame** (already there — no transform), build the cached accumulated footprint.
- **Layer 3 inter-station:** flatten **station footprints** to world — one transform per station, cached, invalidated only when that station moves. Equipment internals are *not* re-flattened.

### Accumulated footprint: hull + union (broad/narrow phase)

**One unified `StationFootprint` object** (`data-model-v2.md` §3.1) holding both fidelities — they describe the same footprint and share invalidation, so they are never cached separately (preventing drift). Within it:
- **Convex hull** — always present; fast-reject broad phase. Hulls don't overlap → stations definitely don't collide → done. Covers the common "clearly no collision" case with one cheap test. Also serves as a cheap "floor wanted" proxy for Layer 2 / positional-prior spacing, so it has consumers beyond collision.
- **True union** (non-convex) — **lazy/optional**; built only on the first narrow-phase need (when hulls *do* overlap — might be a real collision, might be a nestable concavity, e.g. an L-shaped station's notch against a neighbor) and cached until the next intra-station change. Packs tighter but costlier; stations never in a close call this iteration never pay union cost.

Neither fidelity alone suffices: hull-only over-rejects valid tight packings (wastes floor); union-only makes every broad-phase check pay narrow-phase cost (slow in the LNS inner loop). Both available, union deferred.

### Clearance baked into the cached footprint

Standards clearances (`standards.md`) inflate footprints into keep-out buffers, which are part of what must not overlap between stations. **Buffer the hull/union outward by the governing clearance once, at intra-station solve time**, and store the buffered polygon. The inter-station check is then plain polygon-overlap with no per-check buffering (buffering is expensive — do it once when the footprint is built, not every collision check).

### LNS move types map to cache levels

Each destroy-and-repair move is usually *either*:
- **Reorganize equipment in station X** → intra-station: invalidate X's accumulated footprint, rebuild it, re-check X against neighbors via world footprints.
- **Move station X** → inter-station: invalidate only X's world footprint (one transform), re-check against neighbors; X's internals untouched.

A move almost never invalidates everything — it touches one cache level for one station.

### Collision repair: move-then-reorganize (cheap before expensive)

When moving Station A collides A's footprint with B:
1. **First-resort (cheap, inter-station):** nudge A's frame along the free direction until footprints separate. One transform recompute, no internal change. Common case.
2. **Second-resort (expensive, intra-station):** if no nearby free position exists (tight floor), reorganize A's internal equipment to shrink/reshape its accumulated footprint in the contested direction. Invalidates A's accumulated footprint, re-runs the intra-station solve.

Try the cheap nudge first; fall to internal reorganization only when placement alone can't resolve it. The accumulated-footprint abstraction is what lets you slide one polygon instead of re-reasoning about equipment.

---

## Feasibility

A station is feasible **iff** its root task's chosen strategy is feasible AND every precondition Task recursively has a feasible strategy AND all guards pass AND all state requirements are satisfiable. Straight recursive evaluation over the AND-OR tree.

---

## Workflow positional prior [MVP]

The workflow topology implies approximate positions: input/output are fixed anchors, tasks fall between them, parallel tasks side-by-side (reflects real material-flow design — monotonic flow, U/I-shaped lines).

Compute a **nominal position** per task via force-directed graph embedding (spring model) with input/output pinned: sequential tasks interpolate along the input→output axis by topological depth; parallel siblings offset perpendicular, spaced by footprint estimates; merges/splits pull toward predecessor/successor centroids.

Use **two ways, never as a hard constraint:**
- **Seed** for Layer 3 placement (NLP convergence is highly sensitive to initial guess on non-convex problems).
- **Soft objective** `Σ ‖actual − nominal‖²`, small weight — breaks symmetry, tiebreaks, makes layouts legible to human reviewers.

Partly redundant with transport-cost minimization (which clusters connected tasks) but adds what transport cost can't: flow *direction* (input→output monotonicity) and clean parallel-sibling handling. The real optimum sometimes violates the prior (a shared arm rationally sits between two non-adjacent stations) — guides, never dictates.

---

## Partial solutions [MVP, provisional]

A PARTIAL strategy violates cycle time. Current rule: lexicographic — first maximize fully-solved task count, then minimize cost. **Provisional, expected to change.** Known miss: a partial solution for task A might free a resource enabling a full solution for task B; strict lexicographic can miss this. Accepted for MVP (see `roadmap.md` open questions).
