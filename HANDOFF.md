# HANDOFF — starting step-6 M3 (realistic palletizer scenario)

**Transient working note for a fresh context.** Delete this file when M3 lands.
It captures where M1+M2 left things and what M3 needs, so a new session can
start cold. Permanent docs: `CLAUDE.md` (rules + anti-stray session contract),
`docs/roadmap.md` (step-6 M1–M4), `docs/decisions.md` (settled choices).

To resume: open Claude Code in this repo, point it at this file, say
"continue step-6 M3".

---

## Where we are (done)

- Branch `dev`. **M1 (port regions) and M2 (belt geometry) COMPLETE.** `ctest`
  is **17/17 green**. The tree is committed but **4 commits ahead of
  `origin/dev` — confirm whether it was pushed**:
  `a10dd35` M1.4, `d44aa46` M1.5, `a6a2870` M2-core, `551f95d` M2-demo.
- **M1** — a transport endpoint that is a reach-annulus port (`item_in` on an
  arm) is a Layer-3 placer variable, optimised in polar `(r, θ)` with `r`
  hard-bounded to `[reach_min, reach_max]`. `solver/src/layout_problem.cpp`
  (the `solve()` variable layout + port-block bounds + polar seeding),
  `layout_objective`, `core/model/port.hpp` (the optional `reach_min/reach_max`
  fields). See `decisions.md` "Annulus reach-band enforced as a HARD polar".
- **M2** — transports are routed as real placed belts **after** `solve()`:
  `route_belts()` (`solver/src/belt_routing.cpp`) builds a `PlacedBelt` per
  transport (cheapest catalog belt covering the resolved distance) and flags
  belt-vs-station collision (convex SAT; belt-vs-belt skipped). `BeltSpec`
  catalog in `core/model/belt.hpp` + `io::load_belt_catalog` +
  `assets/belt/generic/catalog.json`. **A `BeltStrategy` OR-tree competitor was
  deliberately NOT built** — belt length is only known post-placement and magic
  `TransferStrategy` is free, so it would always lose; see `decisions.md`
  "Belt geometry: PlacedBelt sibling + post-solve sequential routing".
- The capstone demo `demos/solve_workflow/main.cpp` runs end to end: enumerate
  → allocate → positional_prior → build LayoutProblem → solve → route_belts →
  SVG, and prints port + belt diagnostics.

---

## NEXT: M3 — realistic palletizer scenario  (the MVP go/no-go)

Goal (`roadmap.md` step-6 M3): produce a **geometrically plausible layout for
ONE real palletizing cell that a person would recognise as such**. The MVP
"done" evaluation point. Pieces:

1. **A realistic scenario + clearances.** A KUKA-class arm + the generic belt
   catalog (both already exist). The current clearance is a labelled
   placeholder — `constexpr double kClearance_m = 0.2;` at
   `demos/solve_workflow/main.cpp:589` — and `standards.md` is a stub. M3 lands
   **domain-credible (not certified) numbers** (~500 mm around robots),
   labelled as such. Do NOT invent certified values (CLAUDE.md §5).
2. **`validate_workflow()`** — net-new (grep: it exists only in docs). A pass
   that catches port-mismatch / unconnected-input failure modes early, e.g. a
   `Palletize` task whose `pallet_in` has no feeding transport. It's the
   natural home for the gaps listed below.
3. **2D spacing.** The current scenario crams every station onto the `y=0`
   line, so the layout isn't cell-like and belts collide (see below). A real
   cell needs the stations spread (L-shape / two rows) so belts route clear —
   this is the core "looks real" test.
4. **Go/no-go.** Does it produce a plausible cell? If yes, MVP is essentially
   met and M4 stays optional; if a gap forces multi-equipment stations, M4.

---

## Known issues M3 should resolve or consciously park

- **Cramped collinear layout → belt collisions.** All 3 stations land near
  `y=0` between feeder(0) and dispatch(20), so 4 of 6 routed belts pass through
  an intervening station (M2 flags them; it does NOT repair — joint placement
  is deferred). A 2D-spaced scenario is the proper test and the main thing that
  makes the layout look real.
- **Arm-2-idle allocation quirk.** `allocate()` produces a 3rd instance (an arm
  with a reach ring) that no transport touches, while the two identical light
  pallets collapse onto one shared pusher. Pre-existing, NOT introduced by
  M1/M2. Unknown whether it's an enumeration/allocation bug or expected sharing
  behaviour — diagnose before relying on the scenario. Trace
  `solver/src/allocator.cpp` + `build_layout_problem` task→station mapping.
- **No empty-pallet input.** The demo workflow wires `feeder→item_in` and
  `pallet_out→dispatch` but nothing feeds `pallet_in` (empty pallets), even
  though the model supports it (`pallet_in` is a declared logical port in
  `core/model/port.hpp` and both arm/pusher strategies emit a PortConstraint
  for it). A realistic cell should add an empty-pallet source; `validate_workflow()`
  is the natural place to flag the unconnected port.

---

## Orientation — where things live

- Scenario + demo: `demos/solve_workflow/main.cpp` (`build_layout_problem`, the
  `workflow` vector ~line 530, `kClearance_m`, `route_belts`, `draw_layout_svg`).
- Strategies: `solver/src/{arm,pusher,transfer,anchor}_strategy.cpp`.
- Enumerate / allocate: `solver/src/{enumerator,allocator}.cpp`
  (+ `solver/include/.../allocator.hpp` for `BoundInstance`/`TransportEdge`).
- Placement: `solver/src/layout_problem.cpp` (`solve()`), `layout_objective.cpp`.
- Belts: `solver/src/belt_routing.cpp`, `core/model/belt.hpp`, io belt loader.
- Catalogs: `assets/{arm/kuka, pusher/generic, belt/generic}/catalog.json`.
- Ports / tasks / specs: `core/model/{port,task,arm,pusher,belt}.hpp`.
- Clearance placeholder: demo `:589`; `docs/standards.md` (stub).

---

## Build / test loop

Windows, Visual Studio 2022 generator, vcpkg. The `build/default` dir is
already configured, so a plain build needs nothing special:
```
cmake --build build/default --config Debug --parallel
ctest --test-dir build/default --build-config Debug --output-on-failure
```
`cl` does NOT need to be on PATH — MSBuild drives the compiler. 17/17 should
pass. Run the demo:
`build/default/demos/solve_workflow/Debug/demo_solve_workflow.exe` (writes
`build/default/demos/solve_workflow/output/layout.svg` + prints diagnostics).
If you wipe `build/` and reconfigure from scratch, set `VCPKG_ROOT` and run
`cmake --preset default` from a VS Build Tools dev shell first.
Gotcha: in PowerShell don't `2>&1` native exes (git/cmake) — stderr gets
wrapped as a fatal error; check `$LASTEXITCODE` instead.

---

## Anti-stray reminders (read `CLAUDE.md` "Session contract" first)

- MVP source set is `{core, solver, io, svg, adapters}` (+ `tests`, `demos`).
  `sync/ ecs/ render/ gui/` do NOT exist and are post-MVP — do not scaffold them.
- Crudest-concrete-first. Before any code, state the `roadmap.md` step (M3) and
  that the file is in the MVP set.
- M3 does NOT need: a `BeltStrategy` OR-tree competitor (deferred), the
  `StationTemplate` abstraction (M4), intra-station packing (M4), the LNS outer
  loop (parked), or joint belt placement (deferred). Don't build them.
- Verify, don't assert: build + `ctest` before claiming anything green.
- `roadmap.md` is the only thing that authorizes new work; the other design
  docs describe the END STATE, not a backlog.
