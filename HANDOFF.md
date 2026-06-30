# HANDOFF — the MVP is complete; choosing the post-MVP leg

**Transient orientation note for a fresh context.** Delete or rewrite this
file when the next leg is chosen and underway. Permanent docs: `CLAUDE.md`
(rules + anti-stray gate), `docs/roadmap.md` (build order + milestones),
`docs/decisions.md` (settled choices), `docs/architecture.md` /
`data-model.md` / `interaction.md` (the END-STATE design — a description, not
a backlog).

To resume: open Claude Code in this repo, point it at this file, say which
leg below you want to open.

---

## Where we are: MVP DONE (steps 1–6), `dev` == `origin/dev`, 19/19 green

The whole planned build is finished. The system takes a workflow + catalog +
cycle-time target and produces a geometrically plausible `LayoutSolution`
(equipment, poses, ports, real belt geometry), enforces domain-credible
clearances, and the headline demo (`demos/solve_workflow`) renders a
recognizable robotic palletizing cell to SVG.

- **Steps 1–5** (solver core): catalog/strategy library → brute-force
  enumerator → item-state propagation → Layer-2 allocation (sharing) →
  Layer-3 NLP placement (NLopt/BOBYQA behind a library-neutral `solve()`).
- **Step-6 M1** — reach-annulus ports are Layer-3 placer variables (polar
  `(r, θ)`, `r` hard-bounded to the reach band). `solver/layout_problem`.
- **Step-6 M2** — transports routed as real placed belts post-solve
  (`BeltSpec` catalog + `solver/belt_routing.route_belts()`); belt-vs-station
  collision flagged (pure-core SAT). No `BeltStrategy` in the OR-tree (magic
  transport is free, so it'd always win — `decisions.md`).
- **Step-6 M3** — the MVP go/no-go: a realistic palletizer scenario +
  `validate_workflow()` (report-only topology pass) + domain-credible
  (~500 mm, NOT certified) clearances. **PASSED.**
- **Step-6 M4** — multi-instance stations + intra-station packing:
  - **M4.1** a station is a multi-equipment CELL (arm + pallet-zone
    footprint); collision footprint = accumulated convex hull, buffered.
  - **M4.2** inter-station overlap gains a convex-polygon NARROW PHASE
    (`overlap_penalty_poly`, pure-core SAT) so elongated cells pack to
    footprint contact instead of being over-spaced by bounding circles.
  - **M4.3** the intra-station sub-problem made real: a dual-pallet cell
    (one KR120 PA arm serving two pallet positions via task-sharing), laid
    out + feasibility-checked by `solver/station_template`
    (`layout_palletizer_cell` — the crudest `StationTemplate`).

The MVP "done" checklist in `docs/roadmap.md` reads fully met.

---

## The phase boundary: pick ONE post-MVP leg

These are NOT a backlog to grind through — open the one the user wants, build
its crudest concrete step, let the interface emerge (CLAUDE.md anti-stray
gate still binds). The first three carry an *interactive 3D* end goal; the
last two stay batch/solver.

1. **`ecs/` + `sync/` — the registry boundary (foundation for interactive
   work).** The solver is a pure module over its own structs; the END-STATE
   design (`architecture.md` §11, `data-model.md` §4.4, `interaction.md`) has
   the solver's `LayoutSolution` SYNC into an EnTT `ecs/` registry of
   validated components, on a durable FrameId↔entity map, diffed on stable
   identity. This is the prerequisite for `render/` (reads the registry) and
   `gui/` (edits via the sync round-trip). Crudest first step: a one-shot
   full sync of one `LayoutSolution` into a minimal registry — not the tier
   system or delta sync yet. Rules: CLAUDE.md §0 `sync/`/`ecs/` rows, §8, §11.

2. **`render/` — interactive 3D (threepp).** Read-only over the registry,
   draws the cell in 3D; ALL axis/handedness conversion confined to
   `render/threepp_conv.hpp` (core & threepp are both right-handed; the map
   is a +90° rotation about shared X — §7, with a round-trip identity test).
   Needs leg 1 first (it reads the registry). Distinct from `svg/`, which
   already gives batch 2D debug output and stays.

3. **`gui/` — interactive editing (ImGui).** Drag/edit drives a re-solve via
   the sync round-trip, classifying the re-solve TIER (0 free move → 3 full,
   PROPOSED) *before* running it; never auto-applies a Tier-3 result
   (`interaction.md`, CLAUDE.md §4/§13). Needs legs 1–2.

4. **LNS outer loop — resurrect from `feature/step-6-lns`.** Built + parked
   because single-station destroy + smooth NLP inner had no local optima to
   escape. Earns its keep when destroy operators broaden (>1 station) or the
   objective goes non-smooth — e.g. after the deferred M4 increments below.
   `decisions.md` "MVP re-scoped".

5. **Deferred M4 / geometric-realism increments** (each earned by a concrete
   scenario, never speculative): a **free intra-station NLP** over slot
   positions / rotated zones (beyond the fixed template); the lazy
   **non-convex union** + dirty-flag **`StationFootprint` cache**
   (`data-model.md` §3.1 — for cells that NEST into concavities the convex
   hull over-covers; its world/local cache unification is an open design Q);
   a **hard `LN_COBYLA` overlap** constraint (when a tight layout's
   soft-penalty residual becomes load-bearing — the stiff overlap weight
   `20000` in the demo is the tell); heterogeneous **multi-instance
   stations**. Also low-risk and handy: a **JSON workflow loader** (workflows
   are C++ literals in demos today; trivial to add — `roadmap.md`).

Open decisions a leg may need to resolve first: optimizer library for Layer 2
(seam still abstract); the `sync/` world/local footprint-cache unification;
real standards clearance numbers (a safety-engineer input, `standards.md`).

---

## Orientation — where things live

- **Solver pipeline:** `solver/src/{enumerator,allocator}.cpp` (Layer 1–2),
  `solver/src/{arm,pusher,transfer,anchor}_strategy.cpp`,
  `solver/src/layout_problem.cpp` (`solve()`, NLopt) + `layout_objective.cpp`
  (penalties incl. the M4.2 narrow-phase), `solver/src/belt_routing.cpp`,
  `solver/src/station_template.cpp` (M4.3 intra-station),
  `solver/src/workflow_validation.cpp` (M3).
- **Core model:** `core/include/tinycell/model/{task,port,arm,pusher,belt,
  box,pallet}.hpp`; geometry/frames in `core/include/tinycell/geometry.hpp`.
- **Catalogs:** `assets/{arm/kuka,pusher/generic,belt/generic}/catalog.json`.
- **Adapters (only home for foreign libs):**
  `adapters/boost_conv` (`convex_hull`, `buffer_outward`).
- **SVG debug oracle:** `svg/` (stdlib-only).
- **Headline demo:** `demos/solve_workflow/main.cpp` (the dual-pallet cell).
- **Tests:** `tests/` (19 ctest targets).
- **Parked LNS:** branch `feature/step-6-lns`.

---

## Build / test loop

Windows, Visual Studio 2022 generator, vcpkg. `build/default` is already
configured, so a plain build needs nothing special:
```
cmake --build build/default --config Debug --parallel
ctest --test-dir build/default --build-config Debug --output-on-failure
```
`cl` does NOT need to be on PATH — MSBuild drives the compiler. 19/19 pass.
Run the demo:
`build/default/demos/solve_workflow/Debug/demo_solve_workflow.exe`.
If you wipe `build/` and reconfigure: set `VCPKG_ROOT` and run
`cmake --preset default` from a VS Build Tools dev shell first.
Gotcha: in PowerShell don't `2>&1` native exes (git/cmake) — stderr gets
wrapped as a fatal error; check `$LASTEXITCODE` instead. Occasional `.obj`
"permission denied" on parallel rebuild is a transient lock — just rebuild.

---

## Anti-stray reminders for the new phase (read `CLAUDE.md` first)

- **Open ONE leg.** The END-STATE design docs describe all of `sync`/`ecs`/
  `render`/`gui`/LNS in loving detail — that is a description, not a license
  to build them all. Build the crudest concrete step of the chosen leg.
- **Crudest-concrete-first** still governs (it's how the `StationTemplate`
  arrived). Don't build the general form before a consumer exists.
- **Foreign libs only in their adapter home** — threepp/ImGui/EnTT enter via
  `render/`/`gui/`/`ecs/`/`adapters/`, never `core/` or `solver/` (§1).
- **Verify, don't assert** — build + `ctest` before claiming green.
- Each substantive change has been adversarially reviewed before landing;
  keep that bar.
