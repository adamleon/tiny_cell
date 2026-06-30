# HANDOFF — step-6 M1 in progress (cross-machine continuity note)

**Transient working note.** Delete this file when M1 is complete. It captures the
live state and the next two steps so work can resume on another computer. The
permanent docs are `CLAUDE.md` (rules + anti-stray session contract), `docs/roadmap.md`
(M1–M4), and `docs/decisions.md` (settled choices).

---

## Where we are

- Branch: `dev` (pushed to `origin/dev`). M1.1–M1.3 are **committed**; **M1.4 is
  implemented and green in the working tree but NOT yet committed** (awaiting the
  go-ahead to commit).
- **16/16 test binaries green** (`ctest`), including 3 new M1.4 cases in
  `tests/test_layout_solve.cpp` (default-weight band hold, frozen-station port,
  both-ends-annulus ordering).
- Milestone: **step-6 M1 "port regions"** — ports become placer variables.
  Crudest concrete form only: optional reach-annulus fields on `PortConstraint`,
  **NOT** a Point/Annulus/Polygon variant taxonomy (that's deferred).

### M1 commits landed (this session)
- `M1.1` — `PortConstraint` gains optional `reach_min`/`reach_max`
  (`core/include/tinycell/model/port.hpp`). Unset = welded POINT; set = ANNULUS.
- `M1.2` — `ArmStrategy` emits `item_in` as a reach-annulus port (band = chosen
  arm's reach envelope) and `pallet_in`/`pallet_out` as co-located fixed POINTS at
  ~60% of reach along +x (`solver/src/arm_strategy.cpp`). 60% template is hardcoded
  in the strategy — promoting it to a `StationTemplate` type is **M4**, do not do it now.
- `M1.3` — soft `annulus_penalty` on the objective (`solver/src/layout_objective.cpp`,
  declared in `layout_objective.hpp`); reach band carried onto each
  `solver::TransportConstraint` endpoint; new `ObjectiveWeights.annulus` +
  `ObjectiveBreakdown.annulus` summed into `total`. Soft term only (NOT in
  `hard_constraints_satisfied`). It's a constant until M1.4 makes the port variable.

Plus 3 non-M1 commits: anti-stray session contract in `CLAUDE.md`; doc fix reconciling
the allocator "exact validator" claim (it's greedy-only) + the stale pusher
"one box per stroke" comment.

---

## DONE (working tree, uncommitted): M1.4 — annulus port is a placer variable

Implemented in `solver/src/layout_problem.cpp` (+ `layout_objective.{hpp,cpp}` overloads,
`layout_problem.hpp` `LayoutSolution::transports` field) and `tests/test_layout_solve.cpp`.
Variable vector is `[ station pairs: 2*n_variable_stations ) [ port pairs: 2*n_ports )`;
`poses_from_vars` reads only the station block (unchanged). Each annulus endpoint is two
variables in **polar `(r, θ)`** form, `r` hard-bounded to `[reach_min, reach_max]`.

**Design correction vs the original plan (#4):** the plan said the band would be held by
the M1.3 soft `annulus_penalty` with the box "just a sanity clamp." An adversarial review
(ran a probe through `solve()`) showed that's false — at the *default* weights the port
pinned to the cartesian box edge at **~2× the arm's reach** and was reported feasible. A
finite-weight soft penalty can't hold a hard radial band against the transport pull. Fix:
polar parametrisation makes the band an **exact, weight-independent** box on `r`, and keeps
the objective smooth so BOBYQA can swing the port around the ring (a cartesian radial
*projection*, tried first, stalled BOBYQA on the swing — see git history of this session).
The soft penalty is kept as a redundant backstop; `reach_min > reach_max` is rejected.
Logged in `decisions.md` "Annulus reach-band enforced as a HARD polar (r, θ) box".

## NEXT: M1.5 — demo + SVG wiring (closes M1)

`demos/solve_workflow/main.cpp`:
- `build_layout_problem`'s transport loop (~lines 327–351) currently copies only
  `{pc->x, pc->y}` into `TransportConstraint`; also copy `reach_min`/`reach_max` so the
  placer sees the band.
- `draw_layout_svg` (~lines 391–409): draw the solved port position
  (station pose ∘ optimized port-local) as a marker so the SVG visibly shows `item_in`
  landing off-origin inside its reach ring (ring already drawn ~lines 435–437).
- svg/ stays stdlib-only, read-only — no foreign includes (CLAUDE.md §0).

---

## Build / test loop on the new machine

Needs vcpkg + MSVC (the preset hard-requires `VCPKG_ROOT`). On this machine it was:
vcpkg at `C:\Users\adamk\vcpkg`, MSVC from **VS 2022 Build Tools**. On the new machine,
bootstrap vcpkg and set `VCPKG_ROOT`, then from a **VS Build Tools dev shell** (so `cl`
is on PATH):
```
cmake --preset default        # first run is slow: vcpkg builds boost-geometry + nlopt
cmake --build build/default --parallel
ctest --test-dir build/default --output-on-failure
```
Gotchas seen here:
- **Smart App Control**: if `ctest` reports `BAD_COMMAND` / "Process not started" on
  freshly-built test exes (not assertion failures), that's Windows Smart App Control
  blocking unsigned binaries — NOT a code bug. Disable SAC or build under WSL2 (GCC/Clang
  is the project's conformance oracle).
- **PowerShell**: don't `2>&1` native exes (git/cmake) or set `ErrorActionPreference=Stop`
  around them — stderr gets wrapped as a fatal error. Use git from a bash shell; check
  `$LASTEXITCODE` for cmake/ctest.

---

## Anti-stray reminders (read `CLAUDE.md` "Session contract" first)

- MVP source set is `{core, solver, io, svg, adapters}`. `sync/ ecs/ render/ gui/` do
  NOT exist and are post-MVP — do not scaffold them.
- M1 does NOT touch: `StationTemplate` (M4), belt geometry (M2), intra-station packing /
  StationFootprint cache (M4), the LNS outer loop (parked), Layer-2 sharing / PARTIAL
  chaining (shipped, scenario-inactive). No CP-SAT/IPOPT scaffolding. No invented
  clearance numbers (`standards.md` is a stub; `kClearance_m = 0.2` in the demo is a
  labelled placeholder).
- `roadmap.md` is the only thing that authorizes new work; the other design docs are
  END STATE, not a backlog.

To resume: open Claude Code in this repo on the new machine, point it at this file, and
say "continue step-6 M1.4".
