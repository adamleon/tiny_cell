# Roadmap

**Plan, not spec.** This sequences the build and defines milestones; it points into the design docs rather than restating them. Design docs describe the *end state* and don't change as you build; this file describes the *path* and gets checked off.

---

## Build order

1. ✅ **Equipment catalog + strategy library** (`data-model.md` §2–3). Done. Rigorously type what each strategy consumes (`requires_state`, guards, structural preconditions) and produces (`effect`, equipment, energy). The solver is generic; this domain knowledge determines whether output is usable. *Notes:* catalog structs use raw mp-units quantity types in step 1; range invariants are enforced in the JSON loader at the `io/` boundary. Validated value-type wrappers (`architecture.md` §8) are staged in at step 4 — see `decisions.md`. **PARTIAL feasibility + precondition spawning are skipped here** and staged to step 4 along with the analytic throughput model (also `decisions.md`); step 1 strategies return only `FULL` / `INFEASIBLE`.
2. ✅ **Brute-force enumerator** over tiny problems (3–5 tasks) to validate the strategy library produces sensible plans before any metaheuristic can hide bad engineering. Done. *Note:* the enumerator loads each catalog via its per-category loader (`decisions.md` — per-category dispatch); strategies match tasks by capability and consume only their own category.
3. ✅ **Item-state propagation** pass over the workflow DAG (`data-model.md` §4). Done. Introduces the four-way concept split (item-physical / item-state / sensor-capability / actor-requirement+effect), the property-based gating principle (strategies switch on item PROPERTIES via shared helpers, not on item TYPES — see `decisions.md`), the Knowledge/Control split on orientation (camera writes K; fixture writes C; arm reads K; pusher reads C), and the minimum types that make it work: `RotationalSymmetry` on `ItemPhysical`, `OrientationKnowledge` + `OrientationControl` on `ItemState`, `distinct_alignments` / `orientation_known` / `orientation_pinned` / `*_matches_alignment` helpers, Arm + Pusher predicates wired to gate on the appropriate axis. Position knowledge stays binary at this step. The shape of `ItemPhysical` extracted now (composed by `BoxSpec` and `PalletSpec`); future item types follow the same wrap pattern.
4. ✅ **Layer 2 allocation** — assignment/packing with sharing (`solver.md` Layer 2). Done. Includes validated value-type wrappers (`architecture.md` §8) for fields Layer 2 produces/mutates; the analytic per-archetype throughput models (`belt`, `picker`, `palletizer`-style functions giving items/min for a given parameter set); PARTIAL handling in strategies that can serve a task partially (`evaluate` emits `feasibility=PARTIAL` with `partial_info` and a residual sub-task in `preconditions`, enabling the "n× small equipment chained" branch of the OR-tree).
5. ✅ **Layer 3 placement** — 2D NLP with positional-prior seeding (`solver.md` Layer 3 + positional prior). Done. Includes the transport-and-ports work (`decisions.md` "Layer-3 algorithm" entry et al.) that landed mid-step-5: Anchor + Transport task kinds, PortConstraint on StrategyResult, TransferStrategy (magical). The placer is NLopt/BOBYQA behind a library-neutral `solve()` seam, with hard floor bounds, soft overlap penalty, and positional-prior soft term; partial-freeze + warm-start expressed on `StationProblem`. End-to-end demo in `demos/solve_workflow/`. See "Step-5 deferrals" below for what's still soft/missing.
6. **Inner-problem realism — port regions, belt geometry, realistic palletizer scenario.** Originally scoped as LNS + simulated annealing; re-scoped 2026-05-27 after building LNS end-to-end and finding that with the existing inner problem (one-instance-per-station at origin, port positions at `(0,0)`, magical transports with no geometry) there were no genuine local optima to escape and SA never fired. LNS scaffold parked on `feature/step-6-lns` (decisions.md "MVP re-scoped"). The new step-6 milestones, each on its own feature branch:
   - **M1 — Port regions** (`feature/port-regions`). Goal: ports become placer variables; equipment-specific reachability and rigidity drive where they can land.

     *Concepts under discussion for M1 (not committed; subject to change before code lands):*
     - `PortConstraint` carries one `PortRegion` (variant: `Point` for pushers, `Annulus` for arms, `Polygon` for intersections). Multiple `PortConstraint` records with the same `port_name` stack as independent region penalties so the equilibrium is the intersection without precomputing a polygon.
     - Soft preferred-radius term on annulus regions (arms drift toward midpoint of reach band).
     - `ArmStrategy` emits `item_in` as Annulus + preferred_radius and `pallet_in` / `pallet_out` as Point regions at a templated station-frame offset (~60% of reach along +x). Template is hardcoded inside `ArmStrategy` for now; promoting to a real `StationTemplate` type happens in M4.
     - `PusherStrategy` emits its port as a Point at the stroke endpoint offset, perpendicular direction. The left-or-right ambiguity (items can arrive from either side of the stroke axis) is handled by station rotation, not by a discrete port-variant — the placer is free to rotate the station around its origin, so spinning 180° flips which "side" the belt comes from without changing the model. If discrete left/right variants ever need to be enumerated (e.g. for catalog entries with intrinsically-handed strokes), that's an enumerator-layer concern, not the placer's.
     - `pallet_in` and `pallet_out` for the palletizer co-locate naturally: both emit as Points at the same station-frame offset with the same direction (the long belt passes through the station). No "linked port" abstraction needed — they're two independent port records that happen to share their region and direction, so the optimizer puts them in the same place.
     - Inter-station placer gains per-port (x, y) variables for non-Point regions; Point ports stay welded to their offset.

   - **M2 — Belt geometry.** ✅ **Shipped on `dev`** (not a feature branch — M1/M2 landed directly on `dev`). Transports become real placed objects with footprint and length, via a POST-SOLVE routing pass (`solver/belt_routing`): `BeltSpec` catalog + `io::load_belt_catalog`, `PlacedBelt` (sibling to `BoundInstance`), `route_belts()` picks the cheapest catalog belt covering the resolved distance and flags belt-vs-station collision (convex SAT; belt-vs-belt skipped). The `BeltStrategy` OR-tree competitor was deliberately NOT built — belt length is only known post-placement and magic transport is free (see `decisions.md` "Belt geometry: PlacedBelt sibling + post-solve sequential routing"). What actually shipped vs. the original concepts below:

     *Concepts under discussion for M2 (now mostly committed — see decisions.md; kept here for the rationale trail):*
     - `BeltSpec` catalog (width / length range / speed / throughput / list_price) + JSON loader.
     - `BeltStrategy` replacing the magical `TransferStrategy` — picks a catalog belt whose length range covers the source-sink distance.
     - `PlacedBelt` as a sibling type to `BoundInstance` (start_pose + end_pose + width + catalog ref) rather than a variant arm — point and linear placed objects differ enough in their optimisation variables that unifying them would force pervasive variant-handling. Reconsider if a third placed-object kind shows up.
     - Sequential placement: solve stations + ports first (M1 placer), then route straight-line belts of catalog length between resolved port positions. Joint placement (one NLP over stations + belt routes together) is more correct but deferred until a real failure case (a belt forced to pass through a station) appears.
     - Belt-vs-station collision IS checked; belt-vs-belt explicitly skipped — belts can cross, real cells run them at different heights or accept proximity. Forward-compatible: a `route_height` field on `PlacedBelt` could turn belt-vs-belt into a height-aware check later.
     - Forward-compatible data-model slot for equipment attached to a belt at parameter t (cameras / sensors on conveyors); no consumer yet at MVP.

   - **M3 — Realistic palletizer scenario.** ✅ **Shipped on `feature/real-palletizer`.** `demos/solve_workflow` is now a TWO-LINE robotic palletizing plant: one shared box feeder → a heavy cell (25 kg → KUKA KR30) + a light cell (5 kg → KR6), each with its own empty-pallet supply + a shared dispatch; six belts route clear (0 collisions), two distinct arms (no allocator collapse, no PARTIAL/orphan), `hard_constraints_satisfied`. Net-new `validate_workflow()` (`solver/workflow_validation`) is a report-only topology pass catching port-mismatch / unconnected-input gaps (the demo refuses to solve on any Error). Domain-credible (NOT certified) per-category clearances (~500 mm around robots) replace the 0.2 m placeholder, labelled as engineering placeholders in-code. See `decisions.md` "Realistic palletizer scenario …". **Go/no-go: PASS** — the SVG reads as a recognisable two-line palletizing plant, so the MVP is essentially met and M4 stays optional (it would only be forced by a scenario that needs multi-equipment stations).

   - **M4 — Multi-instance stations + intra-station packing.** ✅ **Landed on `dev` (M4.1 + M4.2 + M4.3).** Builds the second placement problem the architecture has always anticipated (`solver.md` "Geometry flattening & caching"). **M4.1 (shipped):** a palletizer station is now a MULTI-EQUIPMENT cell — the arm PLUS a real **pallet-zone footprint** (the physical pallet build position, a footprint the cell reserves as floor, not just a port at an offset). The station's collision footprint is the **accumulated convex hull** of its equipment in the station frame, buffered by clearance, driving inter-station placement + drawn in the SVG. The intra-station layout is TEMPLATED (fixed offsets — real palletizer cells are largely templated, so no intra-station NLP). Built **caller-side via the boost adapter** and passed to the placer as a plain polygon (`solver/` depends on no foreign lib — CLAUDE.md §1), so this is demo-level wiring over the tested `convex_hull`/`buffer_outward` primitives, not a new solver module. **M4.2 (shipped):** the inter-station overlap term gains a **convex-polygon NARROW PHASE** — `overlap_penalty_poly` (pure-core SAT minimum-translation-distance, same family as belt-vs-station collision) on the accumulated world hulls, with the bounding circle kept as the cheap broad-phase reject + the fallback for radius-only callers. Elongated cells now pack to **footprint contact** instead of being held apart by their conservative bounding circles (the demo's two cells pack at ~½ the prior spacing, still feasible and collision-free); the demo scenario is tightened so the footprint is the binding constraint, with a stiff overlap weight keeping the soft-penalty residual sub-mm. **M4.3 capstone (shipped):** the intra-station sub-problem becomes REAL — a **dual-pallet palletizing cell** (one KR120 PA arm serving TWO pallet build positions; the two identical Palletize tasks are SHARED onto one arm, so the cell physically holds the arm + two pallet zones). A net-new pure-solver `layout_palletizer_cell()` (`solver/station_template`) is the **`StationTemplate`** in its crudest form: it places the pallet slots on a reach-radius arc and **CHECKS FEASIBILITY** (every pallet within reach + non-colliding), reporting a diagnostic instead of fabricating an unreachable layout — "if the templated approach is enough, the intra-station NLP shrinks to a feasibility check." The demo turns each slot into a zone footprint (hull/buffer caller-side per §1), remaps each task's `pallet_in`/`pallet_out` to its assigned slot, and refuses to draw an infeasible cell. The feasibility check also surfaces a realism truth — only PA-class arms (≳2.1 m reach) can actually cover a full EUR pallet. **Deferred (earned when a scenario needs it):** a **free intra-station NLP** (over slot positions / rotated zones — beyond the fixed template); the lazy **non-convex union** + dirty-flag **`StationFootprint` cache** (data-model.md §3.1 — for cells that NEST into concavities the convex hull over-covers); a **hard `LN_COBYLA` overlap constraint** (when tight inter-station layouts make the soft residual load-bearing); genuinely heterogeneous **multi-instance stations**. See `decisions.md` "Multi-equipment station footprint (M4.1)" / "Convex-polygon narrow-phase overlap (M4.2)" / "Intra-station templated layout (M4.3)".

## Item-model staging (lives on the build steps above)

The item-physical / item-state model (`data-model.md` §4) grows as new strategies arrive. Concrete properties added when a real consumer needs them — never speculatively (`feedback_concrete_over_abstract`).

- **Phase 1 — step 3 (now).** `ItemPhysical` with `width / length / height / mass / RotationalSymmetry`. `ItemState` with `position_known: bool / ItemOrientation{ knowledge, control } / OnCarrier`. Knowledge/Control single-alignment variants (`Unknown | Known{alignment}` and `Free | Constrained{alignment}`). Helpers `distinct_alignments`, `orientation_known`, `orientation_pinned`, `knowledge_matches_alignment`, `control_matches_alignment`. Arm predicate gates on Knowledge; pusher predicate gates on Control; the K/C split surfaces the difference between "we observed the alignment" (camera) and "the item is physically held in the alignment" (fixture / side-guides).
- **Phase 2 — when the first sensor or shape-aware actor strategy lands** (likely post-MVP since the MVP roadmap steps 4–6 don't introduce new equipment classes — they're optimizer machinery on existing strategies). Each addition is a property + a helper + the affected strategy:
  - **CameraStrategy** → `camera_reads(ItemPhysical) -> OrientationKnowledge` helper. Camera writes the Knowledge axis only — passive observation, doesn't pin the item physically. For discrete items it returns `Known{observed_alignment}`; for continuous items it returns `Unknown` (a featureless cylinder can't be resolved).
  - **FixtureStrategy** → `fixture_forces(FixtureSpec, RotationalSymmetry) -> OrientationControl` helper. Fixture writes the Control axis. For discrete items it returns `Constrained{snap_alignment}` based on the fixture's geometry. May also collapse Knowledge by inference (if the fixture geometry only admits one alignment, Knowledge follows).
  - **Multi-alignment Knowledge/Control variants** — extend `OrientationKnowledge` with a partial-narrowing `Hypotheses{set}` (camera with limited resolving power) and `OrientationControl` with a multi-alignment `Constrained{set}` (a generous fixture that admits two alignments without picking one). Single-alignment variants are Phase 1; the set form arrives with the first consumer that needs it.
  - **Per-DOF position knowledge/control** — upgrade `position_known: bool` to a per-DOF structure parallel to orientation: each of x and y gets its own Knowledge (`Unknown | Known`) and Control (`Free | Constrained`) variant. Earned by a laser / encoder / edge-sensor strategy that resolves one axis but not the other. The orientation pair is already a per-DOF instance — θ with its own K/C — so the pattern is ready to replicate.
  - **Different gripper classes** → `SurfaceProperties` on `ItemPhysical` + `graspable_by(ItemPhysical, GripperSpec) -> bool` helper.
  - **Stack-stability-aware palletizing** → `stable_on(ItemPhysical, ItemPhysical) -> bool` helper, possibly with `Rigidity` on `ItemPhysical`.
  - **Pallet manipulation** (forklift / transport) → no new property needed (pallet already has symmetry via `ItemPhysical`); the new strategy reads existing fields.
- **Phase 3 — much later, if at all.**
  - Probabilistic / confidence-weighted knowledge (camera reading with < 100% confidence). MVP is binary.
  - Knowledge of orientation in *which frame* — explicit when frame composition lands (`architecture.md` §4).
  - Composite items (a stack on a pallet as one item). Likely never; the workflow tracks each item separately.
  - Chirality / reflection symmetries — needed only if a chiral item appears.

The point of the staging is that each item-property addition is local: one property on `ItemPhysical` (or one new variant arm on K/C), one helper, one or more strategy predicates updated. The architectural seam (property-based gating, `decisions.md`) prevents the alternative — every new property forcing every strategy to add a switch-arm.

## Supporting tools

Built incrementally alongside the build steps, not separate milestones.
Add capability when the layer it serves first needs it.

- **`svg/` — 2D vector export of solver output.** Debug oracle for every
  solver layer that carries geometry. Pure C++ stdlib (no new
  dependencies); browser/phone viewable; diff-friendly, so SVGs can
  serve as CI regression artifacts.
  - Step 1: each catalog entry's footprint + (for arms) reach envelope —
    visual sanity check on `load_*_catalog`.
  - Step 4: station-local accumulated footprints (hull + union per
    `data-model.md` §3.1) — verify intra-station packing before
    inter-station placement runs.
  - Step 5: the headline use — full `LayoutSolution` in world coords
    (equipment polygons, transfer arrows, clearance buffers, floor
    bounds).
  - Step 6: same as step 5; before/after pairs make LNS regressions
    visible by diff.

  Distinct from `render/` (threepp, post-MVP, interactive 3D, reads from
  the EnTT registry) — `svg/` reads solver-internal structs + catalogs
  directly and emits text. The two coexist after MVP: SVG for batch /
  CI / diffs, threepp for interactive editing.

## MVP milestone — "done" means ✅ **MET** (steps 1–6 complete)

**All criteria below are satisfied** — the system produces a recognizable robotic palletizing cell, with real belt geometry, domain-credible clearances, reach-annulus + point port regions, and (step-6 M4) multi-equipment cells with an intra-station layout. 19/19 `ctest` green on `dev`. **The project is now post-MVP — see `HANDOFF.md` for the candidate next legs** (interactive `render`/`gui` via the `sync`/`ecs` registry boundary, the parked LNS outer loop, the deferred geometric-realism increments). None scaffolded; open one crudest-concrete-first.

**Re-defined 2026-05-27** (decisions.md "MVP re-scoped"). The original definition was algorithm-coverage-oriented (all three layers + LNS). The new definition is scenario-oriented: the system produces a recognizable layout for one real palletizing cell.

- Catalog + strategy library loadable and validated.
- Given a workflow + blueprint + catalog + cycle-time target, produces a `LayoutSolution` (`data-model.md` §5): equipment list, poses, explicit transfer graph **with real belt geometry** (start/end pose + width + footprint participating in collision).
- Hard standards constraints enforced in placement (`standards.md` — domain-credible clearance numbers land in step-6 M3 even if not certified).
- Port regions on equipment drive feasibility: an arm's port is anywhere within its reach annulus, a pusher's is at a fixed point, multiple equipment intersect their region constraints on shared ports.
- One concrete scenario passes: feeder → arm-palletizer → dispatch with realistic numbers, output SVG inspectable as a plausible factory cell.
- Seed-reproducible.
- **Not** in MVP: 3D, full simulation, BOM/costing report, GUI, interactive editing (`interaction.md`), LNS outer loop (parked on `feature/step-6-lns`), Layer-2 instance sharing across tasks (code shipped, scenario-inactive), PARTIAL chain handling (code shipped, catalog choice avoids triggering it), JSON workflow loader (trivial to add later; workflows are C++ literals in demos for now), stack pattern / palletizing pattern (lives outside the solver — workflow-input or GUI-side).

## Open questions (resolve as you reach them)

- **Energy model granularity:** per-motion physics, or flat `power_draw × occupancy_time` for MVP? (Latter likely enough initially.)
- ~~**Cycle-time allocation:**~~ **Resolved** (decisions.md "Per-task throughput target"): moved out of the solver to the workflow-translator phase that converts customer rates (pallets/h) into per-task `target_ct_per_item`. The solver consumes these as input via the PARTIAL mechanism.
- **Conflict learning:** should resisted Layer-3 conflicts feed a no-good list to prune Layer 1/2 search, or is plain LNS re-sampling enough? Deferred with LNS.
- **Partial-solution scoring:** lexicographic is provisional (`solver.md`). Revisit when partial cases appear in practice — the MVP scenario picks catalog entries that don't trigger PARTIAL.
- **Belt placement: sequential vs joint with stations.** Sequential at MVP (decisions.md "Belt geometry: sequential placement at MVP"). The failure case — station positions force a belt to pass through another station — doesn't appear in single-palletizer topology but will appear in richer scenarios. Joint placement (stations + belt routes as one NLP) is the eventual correct answer; earn it from a real failure.
- **Station template abstraction.** Hardcoded inside `ArmStrategy` at MVP for the palletizer (pallet ports as fixed Points at ~60% reach along +x). When multi-equipment palletizers land (arm + separate pallet-zone equipment with its own footprint), the implicit template gains structural identity and becomes a real type.

## Step-5 deferrals — status as of 2026-05-27

Re-classified after the MVP re-scope. Some closed by step 6 Phase 1; the rest mapped to the new step-6 milestones (M1–M4) or kept as known limitations.

- **Bounding-circle collision uses footprint, not reach envelope.** Still open. One-line fix when convenient; not blocking M1.
- **~~Transports don't drive the objective.~~** **Closed by step 6 Phase 1** (decisions.md "Transport-distance term"). Cherry-picked to `dev`.
- **Port direction tolerances unenforced by the placer.** Folded into M1 scope — `PortConstraint` carries direction + tolerance; the placer's port-region penalties can grow a direction-alignment soft term as needed once M1 ports are variables. Hard constraints via `LN_COBYLA` remain a future swap.
- **Overlap residual feasibility.** Unchanged — sub-mm interpenetration is engineering-tolerant for MVP; `LN_COBYLA` upgrade still deferred.
- **`StationFootprint` cache.** Folded into M4 (intra-station packing). When multi-equipment stations land, the cache becomes the natural data structure for the accumulated station footprint that inter-station collision reads.
- **Intra-station equipment layout.** Now M4 of step 6 — triggered by a scenario that requires multi-equipment stations.
- **Workflow validation.** **Shipped in step-6 M3** — `validate_workflow()` (`solver/workflow_validation`) is a report-only topology pass catching port-mismatch / unconnected-input / duplicate-id / polarity errors before the solve (decisions.md "Realistic palletizer scenario …"). Was promoted out of "deferred to GUI" when the MVP re-scope needed missing edges caught early, not silent.

## Known limitations / deferred

- Coarse arm motion model (straight-line at `max_speed`) is structurally optimistic vs. catalog-declared pusher cycle times, skewing OR-tree comparisons toward arms. Overall safety margin on `target_ct_per_item` (~1.3×) and arm-specific derate both deferred — framing in `decisions.md` "Model-vs-reality safety margin deferred". Revisit at first visibly-wrong choice from the step-4 demo or when calibration data lands.
- Station replication when no single-station solution exists is deferred to the Layer-2 allocator (commit 7+) — framing in `decisions.md` "Station-splitting mechanism deferred". Commit-9 tests exercise PARTIAL chaining only on toy problems with one-station solutions.
- Strict lexicographic partial-handling can miss A-frees-resource-for-B cases. Accepted for MVP.
- Item knowledge is a fixed small vector, not a general planner. Generalize only on demand.
- Symmetry limited to z-rotation + flippable. Full SO(3) deferred.
- 2D geometry, no IK. 3D deferred.
- Approximate optimum only — LNS/annealing gives no global guarantee (acceptable by requirement).
