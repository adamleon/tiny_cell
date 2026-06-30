# CLAUDE.md — FactoryCell working rules

Checkable constraints for code *content*. Companion layers: `architecture.md` is the **reasoning** layer (the *why* behind these rules); `engineering.md` is the **infrastructure** layer (toolchain, dependencies, error-handling mechanism, tests — the *how it compiles, fails, and verifies*). Read this every task; open `architecture.md` for the why, `engineering.md` when standing up the build, adding a dependency, deciding how something fails, or writing a test. Bare section refs (§N) point into `architecture.md`; refs into other docs are named.

---

## Session contract — read first (anti-stray gate)

*This project's recurring failure mode is **straying**: sessions redesign the solver, build abstractions with no consumer, or implement post-MVP modules — instead of doing the next concrete step. The design docs are the attractor; they describe a finished system in loving detail and read like a backlog. They are not. Hold the line below before doing anything else.*

**Where we are** (update this line when a milestone lands): Steps 1–5 done. Step-6 M1+M2 on `dev`. Step-6 **M3 (realistic palletizer scenario) COMPLETE on `feature/real-palletizer`** (18/18 green) — the MVP go/no-go **PASSED**. `demos/solve_workflow` is now a TWO-LINE robotic palletizing plant: one shared box feeder → a heavy cell (25 kg → KUKA KR30) + a light cell (5 kg → KR6), each with its own empty-pallet supply + a shared dispatch; six belts route clear (0 collisions), two distinct arms (no allocator collapse, no PARTIAL/orphan), `hard_constraints_satisfied`. Net-new **`validate_workflow()`** (`solver/workflow_validation`) is a report-only topology pass (duplicate-id / unknown-task / port-polarity / unfed-input Errors, undrained-output Warning) that the demo gates the solve on. Domain-credible (NOT certified) per-category clearances (~500 mm around robots) replace the 0.2 m placeholder, labelled in-code. (M1: annulus transport endpoints are polar `(r, θ)` placer variables, `r` hard-bounded to the reach band. M2: post-solve `route_belts()` builds a `PlacedBelt` per edge from `BeltSpec` catalog, belt-vs-station collision flagged.) **A `BeltStrategy` OR-tree competitor was deliberately NOT built** (`decisions.md` "Belt geometry …"). **Next: merge `feature/real-palletizer` → `dev` (PR), then M4 is OPTIONAL** — multi-instance stations + intra-station packing, only forced by a scenario that needs multi-equipment stations (`roadmap.md` M4; `decisions.md` "Realistic palletizer scenario …").

**Before writing any code, state two things explicitly:**
1. The single `roadmap.md` build-order step or step-6 milestone (M1/M2/M3/M4) this change serves.
2. That the file you're about to touch is in the MVP source set **{`core`, `solver`, `io`, `svg`, `adapters`}** (+ `tests`, `demos`).
If you can't state both, stop — you're about to stray.

- **`roadmap.md` is the only thing that authorizes new work.** `architecture.md`, `data-model.md`, and `interaction.md` describe the **END STATE**, never a task list. Their finished detail for `sync/`, `ecs/`, `render/`, `gui/`, the LNS outer loop, the tier system, and the StationFootprint cache does **NOT** mean build them now. None of those four dirs exist on disk — if you're about to create one, **STOP** (§6).
- **Crudest-concrete-first.** An unclear interface is not license to defer the concrete work. Build the smallest concrete thing that works for one real scenario and let the interface emerge; hardcode a template inside the strategy before inventing a `StationTemplate` (that's M4). (`decisions.md` "Concrete-vs-abstract interpretation correction".)
- **Do not redesign the solver** (Layer-1 OR-tree → Layer-2 alloc → Layer-3 NLP → LNS) or swap the optimizer paradigm. If a better search/planner formulation tempts you, extract only the local refinement that has a named consumer and stop. (`decisions.md` "Means-ends … not adopted".)
- **Do not build the general form before a consumer exists** (per-DOF position, vector-`StrategyResult`, variant loader, `StationTemplate`). Every such refactor is documented as "mechanical at that point" — defer it.
- **Verify, don't assert.** Build and run `ctest` before claiming anything passes — never report green from reading the code. Build loop: vcpkg at `%VCPKG_ROOT%`; configure with `cmake --preset default` from a "Developer PowerShell for VS 2022 Build Tools" shell (so `cl` is on PATH), then `cmake --build` and `ctest`. (`engineering.md`.)
- **Trust code over prose.** `grep` before relying on a "we have X" doc claim — some entries describe intent that never shipped (e.g. the Layer-2 exact validator). When code and a doc disagree, the code is the truth; fix the doc or build the thing, but never assume the safety net.

---

## 0. Before writing in a directory — the module index

Obey the row for the module you are editing. This is the first thing to check.

| Working in… | MUST | MUST NOT |
|---|---|---|
| `core/` (geometry, units, model) | `#include` only stdlib + mp-units; pure value types; right-handed Z-up meters | include EnTT, threepp, glm, Boost, ImGui; store foreign types; strip units |
| `solver/` | operate on internal lean structs; flatten at coarsest stable unit + cache (§5.3); footprint cache is solver-internal, not ECS (§3); dirty-flag via single accessor (§5.5); seed re-solve with existing FrameIds (§4.5); optimizer behind neutral seam (§9) | touch the EnTT registry; build validated components; call `express()` per object/pair in a kernel; mutate pose/equipment without dirtying the cache; scaffold against CP-SAT/IPOPT APIs |
| `ecs/` (components, systems) | validated components via accessors; layer invariants in value types (§8) | expose public mutable fields on invariant-carrying components; bypass invariants via raw pool iteration |
| `sync/` | own the durable FrameId↔entity map (§4.5); diff on stable identity; order frames→poses→transfers→metrics (§11); validate + assert here (§8) | destroy-and-recreate persistent entities; delete outside delta scope; auto-apply Tier-3 results (§13) |
| `io/` | parse JSON → `core/model`; validate at parse boundary, reject authored bad data with a message (§8) | invent catalog specs for missing entries (`data-model.md` §3) |
| `render/` | read-only on the registry; all axis/handedness conversion in `threepp_conv.hpp` only (§7) | write to the registry; perform frame math; inline a second copy of the basis-change matrix |
| `svg/` (debug viz, MVP) | stdlib-only output; read solver-internal structs + catalogs directly; one named artifact per call (one catalog entry, one station footprint, one `LayoutSolution`); coords come straight from `core/` (right-handed, Z up — flip Y at write time only) | include EnTT, threepp, glm, Boost; mutate any solver state; invent or interpret geometric data the source doesn't carry; visualize anything not produced by a solver layer |
| `gui/` | drive drag/edit via the sync round-trip; classify re-solve tier *before* running it (`interaction.md`) | mutate solver internals directly; apply a background Tier-3 result without explicit user action |
| `adapters/` (`glm_conv`, `boost_conv`, `threepp_conv`) | the ONLY place foreign libs are `#include`d; convert at boundaries; strip/reattach units explicitly (§6) | leak a foreign type back into `core/` storage; convert per-element mid-kernel (§5.1) |

---

## 1. Hard prohibitions (these produce silently-wrong output or break the design)

- **No foreign-lib include outside its home.** If `#include <glm/...>`, `<boost/...>`, `<entt/...>`, or `<threepp/...>` appears outside `adapters/`, `ecs/`, `render/`, `gui/`, or an optimizer seam — stop and relocate it. (§2, §5)
- **`core/` and `solver/` depend on nothing external.** Not EnTT, not threepp, not glm, not Boost. (§2)
- **Never `express()` (resolve-to-world) per object or per pair inside a collision/clearance/NLP kernel.** Kernels read cached world data only. This is the #1 performance-cliff bug. (§5.1, §5.3)
- **Never mutate a frame transform or a station's equipment set without dirtying the cache.** Route every such mutation through the single accessor that sets the dirty flag. Raw mutation that bypasses it is forbidden. (§5.5)
- **Never store a foreign type as component/entity data** (no `boost::geometry::polygon`, no `glm::mat4` in storage). Convert at the call site only. (§5.2)
- **Never scaffold against CP-SAT or IPOPT APIs.** Neither optimizer is chosen. Build only the neutral solver-internal problem rep + abstract `solve()` seam. (§9)
- **Never auto-apply a Tier-3 (full) re-solve result.** It is a *proposed* solution; apply only on explicit user accept. (§13)
- **Never clamp a bad value.** Three distinct situations, three mechanisms (`engineering.md` §3 is authoritative): authored bad data → reject at `io/` with a diagnostic (typed `ParseError`); missing catalog/standards entry → reject as input error, never fabricate; solver-produced value failing a validated-component invariant → assert/throw at `sync/` (it's a defect). Never clamp/default/fabricate past any of them. (§8, `engineering.md` §3)
- **Never renumber surviving FrameIds on re-solve.** Reuse existing IDs for persisting frames; mint new IDs only for new frames. (§4.5)

---

## 2. Correctness rules (get these exactly right)

- **Frame resolution composes transforms, never adds triplets.** A parent's rotation rotates the child's offset. `(xa+xb, ya+yb, θa+θb)` is wrong. Regression test: sensor at `(belt_length/2, 0, 0)` in a belt frame rotated 90° must land along the belt's *rotated* axis. (§4.3)
- **FrameId is its own id space, never `entt::entity`.** A frame is a table row; the solver creates/discards candidate frames freely. (§4.1)
- **Pose carries a FrameId** `{ value(x,y,θ), FrameId frame }`; the frame is a FrameId, never an entity id. (§4.2)
- **Two-level footprint cache (not unified with a frame cache).** Local footprints (`hull_local`, lazy `union_local`) built + clearance-buffered once per intra-station change (`dirty_local`); world copies (`world_hull`/`world_union`) are one transform of local, recomputed only on station-move (`dirty_world`). Hull and union are one object, never cached separately. (§4.3, §5.4, `data-model.md` §3.1)
- **Render axis/handedness conversion is one named constant in `threepp_conv.hpp`.** Core and threepp are both right-handed; the map is a +90° rotation about shared X (det = +1, no reflection). Round-trip `Pose2D → threepp → Pose2D` must be identity within epsilon. (§7)
- **Units and frames are orthogonal guards.** Units catch dimensional errors at compile time; frames catch spatial-reference errors at runtime. Keep both. Strip/reattach units only in adapters. (§6)
- **Sync ordering is a hard requirement:** frames/FrameRefs → poses → transfer graph → metrics. A pose referencing a not-yet-created frame is a dangling reference. (§11)
- **Sync is atomic:** apply fully or leave the registry untouched. MVP uses staged-then-commit. (§11)

---

## 3. Performance rules

- **Convert at kernel boundaries, once in / once out — never per-element mid-loop.** An NLP iteration is a kernel entry: rebuild the live flat data at iteration top, then the inner loop reads it flat. (§5.1)
- **Flatten at the coarsest stable unit.** Equipment is never individually world-resolved during inter-station search; a station move is one transform on its cached accumulated footprint. (§5.3, §4.6)
- **Buffer + union footprints once at cache-build time**, not per collision check. The per-check path is plain polygon-overlap on already-buffered, already-unioned polygons. (§5.4)
- **Two-phase collision:** convex-hull broad-phase reject first; true non-convex union narrow-phase only when hulls overlap. (`solver.md`)

---

## 4. Tier → sync scope (interactive editing)

Classify the tier *before* running the re-solve. Sync scope is determined by tier.

| Tier | Sync mode | May touch |
|---|---|---|
| 0 free move | no sync | nothing (provisional pose; flags only) |
| 1 local geometry | delta | poses inside active radius; no deletions |
| 2 partial structural | delta | create/update/delete confined to affected segment |
| 3 full re-solve | full, but PROPOSED | nothing until user accepts; never auto-apply |

**A delta sync never deletes outside its scope** — absence outside scope means "not re-solved," not "removed." Only full sync treats absence as deletion. (§11, §13)

---

## 5. Open decisions — do not lock in code

- **Optimizer library (Layer 2):** unchosen. Keep the seam abstract. (§9. Layer 3 settled: NLopt/BOBYQA via vcpkg — `decisions.md` "Layer-3 NLP backend".)
- **Standards clearance numbers:** `standards.md` is a stub. Do **not** invent numeric clearances — they come from the standard or a safety engineer. Build the cache/geometry pipeline with a placeholder buffer; wire real values later without structural change. Step-6 M3 lands domain-credible (not certified) numbers as part of the realistic-palletizer scenario. (`standards.md`)
- **Sync transactional rollback:** deferred; MVP staged-then-commit. (§11)
- **Belt placement: sequential vs joint:** sequential at MVP (route straight-line belts between resolved port positions after station placement); joint deferred until a real failure case appears. `decisions.md` "Belt geometry: sequential placement at MVP".
- **`StationTemplate` abstraction:** deferred — palletizer's templated-ish pallet ports are hardcoded inside `ArmStrategy` for MVP. Promote when multi-equipment stations land. `decisions.md` "Pallet ports as templated Point regions".
- **LNS outer loop:** **parked on `feature/step-6-lns`.** Built + evaluated; doesn't earn its keep on single-station destroy + smooth NLP inner. Resurrects when destroy operators broaden or objective becomes non-smooth. `decisions.md` "MVP re-scoped".

Already settled (do not re-litigate): cycle-time allocation moved to the workflow-translator phase, not the solver (`decisions.md` "Per-task throughput target"). Dependency manager vcpkg, validated by the Layer-3 spike (`decisions.md` "Layer-3 NLP backend"). Infra decisions (`engineering.md` §5): C++20 fixed (no modules / no `import std;`); CMake 3.28+; MSVC 194+ / GCC 13+ / Clang 17+ with Windows in CI day-one and GCC/Clang as conformance oracle; GoogleTest; typed `ParseError` at `io/` + assert/throw at `sync/`; mp-units contract checking ON in debug *and* release, pinned explicitly; `std::format` with fmtlib as MSVC fallback.

---

## 6. MVP scope — what to build vs. skip

MVP is **solver + `core/` only**, batch, no GUI (`roadmap.md`). Build order: catalog/strategy library → brute-force validator → item-state propagation → Layer 2 allocation (sharing inactive in first scenario) → Layer 3 placement (with transport-distance term) → **port regions → belt geometry → realistic palletizer scenario**. LNS is **parked** after evaluation showed the inner problem was the limiting factor; see `decisions.md` "MVP re-scoped (2026-05-27)".

- **Do NOT scaffold `sync/`, `ecs/`, `render/`, or `gui/` yet** — they are post-MVP. The §11 sync contract, §4.4 ECS mapping, §7 render conversion, and the right half of the module index (§0) describe the eventual system, not the MVP. Their rules still bind *when* those modules are built; they are simply not in the first milestone.
- **`svg/` IS in MVP scope** — debug oracle for any solver layer that produces geometry (`roadmap.md` "Supporting tools"). Build incrementally per layer that needs it, not as a separate milestone. Distinct from `render/`: SVG is stdlib-only batch output for diff-friendly inspection; `render/` is interactive 3D via threepp and arrives post-MVP. The two coexist after MVP.
- `core/` (geometry, units, model) is the foundation everything else needs and has zero external dependencies — start there, it has no build-standup blocker.
- **Windows is day-one even though the GUI is not.** `core/` and `solver/` must build and pass tests on MSVC from the first commit; deferring threepp/ImGui defers the GUI, *not* Windows. MVP dependency surface is small — mp-units, Boost.Geometry, glm — and builds cleanly under either dependency manager. (`engineering.md` §2.2, §6)
- **Build/error decisions block the first line of `core/`** and are already settled (§5 above, `engineering.md`) precisely so they don't stall you: C++20, CMake 3.28+, the compiler set, and the typed-exception / assert-throw error split must be in place before `core/` compiles. Mechanical style is enforced by committed `.clang-format`/`.clang-tidy`, not prose (`engineering.md` §2.5).
- Steps 1–5 done (catalog → brute-force → item-state → Layer 2 → Layer 3 with transport-distance term + decomposed cost). Step 6 in flight as port-regions → belt-geometry → realistic-palletizer per `roadmap.md`. The brute-force enumerator (step 2) **is** the strategy-library test harness, not a throwaway (`engineering.md` §4.2).
