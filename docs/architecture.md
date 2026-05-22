# System Architecture

Module boundaries, data flow, and the cross-cutting systems (frames, geometry, units, ECS) of the FactoryCell application. Schemas live in `data-model.md`; the solver algorithm in `solver.md`; terms in `README.md`. This doc is the **how it's all wired** layer.

---

## 1. Stack & top-level shape

**C++ throughout.** One interactive application from day one (the chosen stack — EnTT, threepp, ImGui — is a C++ GUI app, so there is no "headless CLI first, UI later" phase; interactivity is implied, see `interaction.md`).

| Concern | Choice |
|---|---|
| Language | C++ |
| World/entity state | **EnTT** registry — the live application state, domain-named **FactoryCell** |
| Renderer | **threepp** (three.js port) |
| GUI / workflow node-graph | **ImGui** |
| Units | **mp-units**, wired into the geometry abstraction |
| Geometry algorithms | **Boost.Geometry** (for now; confined to an adapter — swappable) |
| Render-facing math | **glm** (confined to an adapter) |
| Layer 2 optimizer | optimizer-pluggable — candidate OR-Tools CP-SAT, **NOT chosen; do not scaffold against its API** (§9) |
| Layer 3 optimizer | optimizer-pluggable — candidate IPOPT, **NOT chosen; do not scaffold against its API** (§9) |

## 2. Module breakdown & boundaries

```
core/              ← pure: no external libs, no ECS, no rendering
  geometry/          Pose2D, Vec2, Polygon, Transform2D, FrameId + frame resolution
  units/             constrained quantity types (mp-units + invariants)
  model/             domain types (mirror data-model.md schemas)
solver/            ← pure module: operates on its OWN internal structs; objects in, LayoutSolution out
  layer1_htn/        AND-OR decomposition
  layer2_alloc/      assignment/sharing  (optimizer seam)
  layer3_layout/     continuous placement (optimizer seam)
  lns/               outer metaheuristic
  frames/            solver-internal FrameRegistry (plain table)
ecs/               ← EnTT integration: components, systems
  components/        validated components (accessors, not public fields)
  systems/
sync/              ← THE boundary: LayoutSolution (solver structs) <-> EnTT registry
io/                ← parse JSON -> core/model objects; LayoutSolution -> files
render/            ← threepp; reads registry; owns Z-up<->Y-up transform
gui/               ← ImGui; workflow node-graph editor; drives drag/edit
adapters/          ← the ONLY places foreign libs appear
  glm_conv, boost_conv, threepp_conv
app/               ← wires it together; main loop
```

**The load-bearing rule:** `core/` and `solver/` depend on **nothing external** — not EnTT, not threepp, not glm, not Boost. Foreign libraries appear *only* in `adapters/`, `render/`, `gui/`, and the optimizer seams inside `solver/`. This is what makes libraries swappable and keeps the numerical core testable in isolation.

## 3. The solver / ECS boundary (critical)

The solver explores thousands of candidate configurations per run (LNS), discarding most. It **must not** operate on the EnTT registry — mutating live world state per discarded candidate would thrash the renderer and couple search to ECS identity.

- Solver works on **its own lightweight internal structs** (cheap to copy/snapshot for destroy-and-repair).
- Solver produces a `LayoutSolution` (`data-model.md` §5).
- The **`sync/` step** writes that solution into the EnTT registry; renderer and GUI observe the registry.
- The **footprint cache** (local hull + lazy union per station, plus cheap world copies — `data-model.md` §3.1, architecture §5.4) is **solver-internal state**, living on the solver's own structs alongside the FrameRegistry — *not* EnTT components. It is rebuilt constantly during LNS on candidates that are discarded; surfacing it as registry components would thrash the registry for the same reason candidate poses must not. Only final footprints, if needed for rendering/debug, cross the boundary via sync.

This preserves the "pure module, objects in/out" principle: the *adapter* (sync) translates between solver structs and ECS components. The same boundary serves graded re-solve — a drag reads current poses from the registry, builds a solver problem with most things frozen, solves on internal structs, syncs the delta back (`interaction.md`).

## 4. Frame system (nested, data-driven, own id space)

Replaces ECS/scene-graph parent-child with explicit frames, so resolution stays in *our* unit-typed, Z-up, convention-controlled core rather than in EnTT's or threepp's parenting (which know nothing of mp-units or our axes).

### 4.1 Frames have their own identity — NOT entity ids

```
Frame { FrameId id; FrameId parent; Pose2D pose_in_parent; }
```

- `FrameId` is its own id space. World = **FrameId 1**, parent = null, pose = identity. `null` = undefined.
- **Not** an `entt::entity`. Reason: the solver creates/discards candidate frames freely (a frame is a table row, not an entity); and a candidate frame may exist before any entity is committed. Coupling frames to entity ids would drag the solver into ECS identity space.
- A single entity can both *be placed in* a frame and *define* a frame (a belt has pose-in-station **and** owns the belt frame its sensor lives in). Separate id spaces remove that collision.

### 4.2 Pose carries a FrameId

`Pose2D` stores `{ value (unit-typed x,y,θ), FrameId frame }`. This was the right instinct (frame-in-pose). The fix vs. the original idea: the frame is a `FrameId`, never an `entt::entity`, and `FrameId` is not the entity's id.

### 4.3 Resolution composes TRANSFORMS, not triplets  ⚠ #1 correctness rule

Resolving a nested pose to world walks parent links, composing rigid transforms. Composition is **transform multiply**, NOT component-wise `(xa+xb, ya+yb, θa+θb)`. A parent's rotation rotates the child's offset.

> **Regression test (write this):** sensor at `(belt_length/2, 0, 0)` in a belt frame rotated 90° in its station must land displaced along the belt's *rotated* axis in world, not along world-X. Triplet addition fails this; transform composition passes.

`express(pose, target_frame)` = resolve both pose and target to world, then `inverse(world_of(target)) ∘ world_of(pose)`. Callers always say "give me this pose in frame X" and never assume hop count — so the signature is stable whether resolution is one hop or many.

Safety: depth-guard / cycle-check in resolution (a malformed parent chain must assert, not infinite-loop).
Performance: lazy walk-to-world is correct and fine for MVP. The station footprint cache (§5.4, `data-model.md` §3.1) stores world footprints (`world_hull`/`world_union`) as cached polygons, invalidated by `dirty_world` when the station moves. This is **not** unified with a general frame-world-pose cache, and that is deliberate: the expensive cached artifact is the accumulated, hulled, unioned, clearance-buffered *local* footprint (keyed to intra-station change, `dirty_local`); the world copy is one cheap transform of it (keyed to station-move, `dirty_world`). The two triggers map to two genuinely independent events (`solver.md`, "two independent placement problems"), so they are two real invalidation causes, not redundant bookkeeping. A general frame-world-pose cache would not subsume the footprint cache — it stores a different (and cheaper) thing.

### 4.4 In the ECS

An entity that defines a frame carries a `FrameRef { FrameId own_frame; }`; its placement is a `Pose2D` carrying the parent `FrameId`. The sync step maps solver `FrameId`s onto ECS entities/components. Frame *math* lives in `core/geometry` and is identical solver-side and ECS-side — always on `FrameId` + `Pose2D`, never on `entt::entity`.

### 4.5 FrameId lifecycle — stable across solves ⚠ re-solve correctness rule

The solver↔ECS FrameId mapping is a **stable bijection persisted across solves**, not regenerated per run. A re-solve **reuses the existing FrameId** for any frame that persists (same belt, same station), and mints a new FrameId **only** for genuinely new frames. It must NOT renumber surviving frames.

Why this is load-bearing: graded re-solve (`interaction.md`, Tiers 1–2) writes back only a **delta** — sync diffs the new `LayoutSolution` against the registry and updates what changed. Diffing requires identity continuity: sync recognizes "belt frame moved" only if the belt frame carries the same FrameId in both solutions. If a re-solve assigned IDs by internal iteration order (the trap), surviving frames would renumber, sync could not distinguish *moved* from *destroyed-and-recreated*, and every drag would tear down and rebuild the whole cell — defeating the entire tiered design.

- Persistence lives at the **sync boundary**: sync owns the durable FrameId↔entity map. The batch solver's *internal* candidate frames (created/discarded during LNS) are ephemeral and need no stable IDs; stability is required only for frames that survive into a committed `LayoutSolution` and get synced.
- On re-solve, the solver is seeded with the existing frames (their IDs and poses) as warm-start input — so it operates on the same IDs rather than inventing a fresh table. New frames take fresh IDs from a monotonic counter that never reuses a retired ID within a session.
- Deleted frames: a frame present in the registry but absent from a new committed solution is a deletion (see sync contract §11) — its FrameId is retired, not recycled.

### 4.6 Why nesting pays off, concretely

Frame nesting exists precisely so that moving a station is *one transform applied to one cached polygon* (its accumulated world footprint, §5.4), never a re-resolution of its equipment. Equipment poses live in the station frame and are **not world-resolved during inter-station search**; only the station frame's pose-in-world changes when a station moves. This is the payoff that the solver's two independent placement problems (intra-station vs. inter-station, `solver.md`) are built on.

## 5. Geometry abstraction (swappable libs)

Reason: threepp, glm, Boost, (Eigen) each have their own geometry/transform types; the underlying lib may change and conversions are frequent. Abstract the **stored/passed types**, never the **algorithms**.

- **Own types** (`core/geometry`): `Pose2D`, `Vec2`, `Polygon` (list of unit-typed `Vec2` in a frame), `Transform2D`. These `#include` nothing external.
- **Do NOT abstract** Boost's algorithms or glm's math — convert to them at the call site, compute in native types, convert back.
- Each foreign lib touched by **exactly one** adapter file (`glm_conv.hpp`, `boost_conv.hpp`, `threepp_conv.hpp`). Swapping Boost→CGAL = rewrite `boost_conv.hpp`, nothing else. If `#include <glm/...>` appears outside an adapter, the abstraction has failed.

### 5.1  ⚠ #2 performance rule: convert at boundaries, never mid-kernel

The abstraction is canonical storage + boundary type. Inside a hot kernel (NLP solve, per-iteration overlap checks) convert **once** to the kernel's native type, compute entirely native, convert **once** out. Never convert per-element inside a loop. Violating this turns the abstraction into a per-conversion tax.

### 5.2 Foreign types never stored

A `Polygon` on an entity is *our* type. Never store a `boost::geometry::polygon` or a `glm::mat4` as component/entity data — that re-couples storage to the lib and kills swappability. Convert at the algorithm call site only.

### 5.3 Flatten at the coarsest stable unit, never per-pair ⚠ resolves §4.3 ↔ §5.1

Poses are **stored** frame-relative (a sensor at `(belt_length/2, 0, 0)` in its belt frame — authored once, never edited as the belt moves). Geometry kernels evaluate in world. The bridge is **not** a naïve "resolve every object to world every evaluation" — it is flattening at the **coarsest stable unit and caching**, because there are two independent placement problems (`solver.md`): equipment-relative-to-station (changes only when a station's internals reorganize) and station-relative-to-world (changes only when a station moves). Conflating them is what makes flattening look expensive.

The rule: kernels read **cached** world-frame footprints; they never call `express()` per object or per pair. A station move recomputes **one** transform on **one** accumulated polygon, not a re-flatten of its equipment (§4.6). Equipment is never individually world-resolved during inter-station search.

This reconciles the two earlier rules: §4.3's transform-composition is correct *for the flatten/cache-build step*; §5.1's "no per-element conversion in a hot kernel" holds because the collision kernel touches only cached world polygons. The sensor "moving with the belt" is automatic — its world position follows from the belt/station frame's transform, recomputed when (and only when) that frame moves.

**What Claude Code must NOT generate:** an overlap/clearance/NLP loop that calls `express()` (or any resolve-to-world) per object or per collision pair. That is the §5.1 violation this rule exists to prevent — a performance cliff invisible on small cells. The cache (§5.4) is the required structure, not an optimization to add later.

### 5.4 Station footprint cache: two-level, hull + lazy union

The cache has two invalidation levels mapping to the solver's two independent placement problems (`solver.md`), and stores world copies rather than deriving them — see §4.3 for why this is not unified with a frame-world-pose cache. Schema is `data-model.md` §3.1; the architectural rules:

- **Local footprints** (`hull_local` always, `union_local` lazy) are the expensive artifact: accumulated from equipment in the **station frame**, hulled/unioned, and clearance-buffered outward — all **once at intra-station solve time** (`dirty_local`). The convex hull is the broad-phase fast-reject and also a "floor wanted" proxy for Layer 2 / positional-prior; the true non-convex union is built **only** on first narrow-phase need (hulls overlap) and cached until the next `dirty_local`.
- **World footprints** (`world_hull`, `world_union`) are the cheap artifact: each is one transform of its local counterpart, recomputed **only** when the station's frame-to-world transform changes (`dirty_world`) — one polygon-transform, never a re-flatten of equipment.
- **Hull and union are one object, never cached separately** — they describe the same footprint at two fidelities and share invalidation, so they cannot drift.

Storage types: all cached footprints are core `Polygon` (own type). Hull, union, and clearance-buffer are Boost.Geometry algorithms invoked at the call site via `boost_conv` (convert in, compute, convert out) — never stored as Boost types. Buffering and union run **once at build time**, never per collision check (§5.1); the per-check path is plain polygon-overlap on already-buffered, already-unioned polygons.

### 5.5 ⚠ Cache invalidation must be structural, not by convention

The cache is correct only if a dirty-flag is *never* missed. A station moved without setting `dirty_world`, or equipment changed within a station without setting `dirty_local`, yields a silently-wrong collision check — a plausible but invalid layout, the worst failure class (§8). There are exactly **two** distinct events, each with **one** trigger (§5.4):

- **Station moved** (frame-to-world transform changed) → set `dirty_world`; re-transform `*_local` → `world_*` (one polygon-transform, internals untouched).
- **Equipment reorganized within a station** → set `dirty_local`; rebuild `hull_local`, drop `union_local`, re-buffer.

Enforcement (the load-bearing part):

- The **only** code path that mutates a station's frame-to-world transform must also set `dirty_world`; the **only** path that mutates intra-station equipment must set `dirty_local`. Route both through a **single accessor that sets the flag** — never ask callers to remember.
- Raw mutation of a transform or equipment set bypassing that accessor is **forbidden** — same discipline as §8 validated components: the invariant lives in the type, not in caller diligence.

This is deliberately the §8 principle applied to a cache: structural enforcement, not convention. If Claude Code exposes a setter that mutates pose/equipment without dirtying the cache, that is the bug this rule exists to prevent.

## 6. Units (mp-units, wired into geometry)

`Pose2D` x,y are unit-typed length (Z-up meters); θ is unit-typed angle. Core computes in units throughout. Units catch dimensional errors **at compile time** (the joules-vs-watts class of bug — see `decisions.md`).

- Foreign libs (glm, Boost, threepp) are **unitless**. Adapters are the **only** place units are stripped (out) and reattached (in) — explicit and auditable. A stray unitless number cannot propagate through core undetected.
- Units (compile-time, dimensional) and frames (runtime, spatial reference) are **orthogonal guards** — a pose can be dimensionally correct but in the wrong frame, or right frame wrong unit. Keep both; they catch different bugs.

## 7. Coordinate frames & handedness (render boundary)

- **Core/solver: Z-up, meters.** **threepp: Y-up.** Common X axis.
- Mapping Z-up→Y-up with shared X is a rotation about X (Z→Y, Y→−Z). **Handedness: threepp is right-handed (confirmed — it is a three.js port and three.js is right-handed). Core is also right-handed (Z-up, right-handed). Both spaces are right-handed, so the mapping is a pure +90° rotation about the shared X axis with no reflection / no determinant flip.** The locked basis-change matrix (core Z-up → threepp Y-up), applied to a column vector `[x, y, z]ᵀ`:
  ```
  R_core→threepp =  [ 1   0   0 ]      x_threepp =  x_core
                    [ 0   0   1 ]  →   y_threepp =  z_core
                    [ 0  -1   0 ]      z_threepp = -y_core
  ```
  Inverse (threepp Y-up → core Z-up) is its transpose (`x, -z, y`). This is the ONLY axis transform in the system; define it as a single named constant in `threepp_conv.hpp` (e.g. `kCoreToThreepp`) with this comment, and never inline a second copy. det = +1 (a rotation, not a reflection) — if a future change makes it −1 something is mirrored.
- The axis/handedness conversion lives **only** in the render adapter (`threepp_conv.hpp`), as a named constant with a comment, applied at the same boundary that converts `Pose2D`→threepp transform. Core is Z-up everywhere.
- **Test (write this):** round-trip identity `Pose2D → threepp → Pose2D ≈ identity` within epsilon. Render-boundary axis bugs are invisible until something renders mirrored/backwards.

## 8. Validated components & value types

Components carrying invariants are **not** plain public-field PODs. A length must be > 0; a payload, reach radius, etc. must be valid by construction. Garbage geometry produces silently-wrong layouts — the worst failure mode — so invalid values must be **unconstructable**, not caught later.

- **Push invariants into value types, not component setters.** `PositiveLength`, `Payload`, `ReachRadius` — layered on mp-units (which enforces dimension, not range; the wrapper adds the range/positivity invariant). Each invariant written **once per concept**, not per field. Components then just hold these types.
- **Construction-time enforcement + immutability** wherever data isn't mutated (catalog specs, item dimensions) — no setter, no per-write cost, impossible to later corrupt. Reserve setters for genuinely mutable fields (poses, task bindings).
- **Failure mode: throw/assert, never silently clamp.** Clamping turns a detectable bug into an invisible wrong answer. Authored/parse-boundary bad data → reject with a message; solver-produced bad value hitting a validated boundary setter → assert (it means a solver bug, fail loud). The concrete mechanism per situation (typed `ParseError` at `io/` vs. assert/throw at `sync/`, and why C++20's lack of `std::expected` makes the exception the right call) is specified in `engineering.md` §3.
- **Validated components use accessor methods, not public fields, and are not accessed via raw EnTT pool iteration** (which would bypass invariants). Reserve raw-data access for genuinely-POD components (tags, flags).
- **This does not tax the solver:** the solver works on its own lean internal structs; validation happens at the **sync boundary** (boundary frequency), not in the LNS inner loop.

## 9. Optimizer seams (open, low-cost)

Layer 2 (CP-SAT candidate) and Layer 3 (IPOPT candidate) optimizers sit **inside** `solver/` and consume the solver's internal problem representation — they never touch core stored types, so leaving the choice open has no architectural cost. The solver converts frame-resolved geometry into optimizer form (CP-SAT vars / NLP callbacks) at an internal seam.

**[do not scaffold — Claude Code]** Neither optimizer is chosen. Do **not** write code against a specific optimizer's API (no `ortools::sat::CpModelBuilder`, no IPOPT `TNLP` subclass) until the choice is validated and recorded here. The seam Claude Code may build now is the **solver-internal problem representation** (the neutral struct describing vars/constraints/objective) plus an abstract `solve()` interface the optimizer implements behind it. Baking one optimizer's call shape into the seam now is the specific mistake to avoid — it shapes the seam wrong and couples `solver/` to a library that isn't picked.

**[build note]** IPOPT is the gnarliest dependency to stand up (linear-algebra backend, often MUMPS/Fortran). Budget build-system time when reaching Layer 3, or trial a lighter NLP first on a small problem.

## 10. Data flow (one solve)

```
JSON (workflow, blueprint, catalog)
  → io/ parse → core/model objects (unit-typed, validated)
  → solver/ (internal structs + FrameRegistry + footprint cache; Layers 1→2→3 under LNS — L3 builds/refreshes station footprint caches as part of placement, §5.4)
  → LayoutSolution
  → sync/ → EnTT registry (validated components, FrameRefs, Poses)
  → render/ (threepp, Z-up→Y-up) + gui/ (ImGui)
[interactive edit] gui drag → sync builds frozen solver problem → solver → sync delta back  (interaction.md)
```

## 11. The `sync/` contract (the boundary, specified)

`sync/` is the only module that writes solver output into the EnTT registry. §3 gives the *rationale*; this is the *contract* Claude Code must implement against. It runs in two modes:

**Full sync** (after a cold solve / Tier-3 accept): the new `LayoutSolution` is authoritative. Reconcile the registry to it.
**Delta sync** (after Tier-1/2 re-solve): only a scoped subset changed; touch only that subset (see §13).

Reconciliation rules (both modes operate by diffing solution vs. registry on stable identity — `instance_id` for equipment, `FrameId` for frames, per §4.5):
- **Present in solution, absent in registry → create.** Mint entity, attach validated components, attach `FrameRef` if it defines a frame.
- **Present in both → update in place.** Reuse the entity; write changed poses/fields. Do NOT destroy-and-recreate (it would churn renderer state and break the stable bijection).
- **Absent in solution, present in registry → delete** (full sync only; a delta sync never deletes outside its scope — absence outside scope means "not re-solved", not "removed"). Retire the FrameId (§4.5).

**Ordering (hard requirement):** within a sync, create/update **frame-defining entities and their `FrameRef`s before** any `Pose2D` that references those `FrameId`s. A pose referencing a not-yet-created frame is a dangling reference. Order: (1) frames/FrameRefs, (2) poses, (3) transfer graph, (4) metrics.

**Validation fires here, at boundary frequency (§8).** Solver-produced values cross into validated components *only* through sync; this is where `PositiveLength` etc. are constructed. A solver-produced value that fails a validated-component invariant is a **solver bug → assert/throw, do not clamp** (§8). Authored data was already validated at the `io/` boundary, so failures here implicate the solver.

**Atomicity:** a sync either applies fully or leaves the registry untouched — never half-applied (a partial write renders a malformed cell). For MVP a simple staged-then-commit (build the change set, validate all, then apply) is sufficient; full transactional rollback is **[deferred]**.

**Direction:** sync is the round-trip boundary, not one-way. The drag path *reads* current poses from the registry to build the frozen solver problem, then *writes* the delta back (§10, last line). Both directions live in `sync/`.

## 12. Module work-rules index (for Claude Code)

When generating or editing code, obey the rule for the module you are in. This re-projects the rules above by *location* — it is the index Claude Code should consult before writing in a directory.

| Working in… | MUST | MUST NOT |
|---|---|---|
| `core/` (geometry, units, model) | `#include` only stdlib + mp-units; pure value types; right-handed Z-up meters | include EnTT, threepp, glm, Boost, ImGui; store foreign types; strip units |
| `solver/` | operate on internal lean structs; flatten at coarsest stable unit + cache (§5.3); footprint cache is solver-internal, not ECS (§3); dirty-flag via single accessor (§5.5); seed re-solve with existing FrameIds (§4.5); optimizer behind neutral seam (§9) | touch the EnTT registry; build validated components; call `express()` per object/pair in a kernel; mutate pose/equipment without dirtying the cache; scaffold against CP-SAT/IPOPT APIs |
| `ecs/` (components, systems) | validated components via accessors; layer invariants in value types (§8) | expose public mutable fields on invariant-carrying components; bypass invariants via raw pool iteration |
| `sync/` | own the durable FrameId↔entity map (§4.5); diff on stable identity; order frames→poses→transfers→metrics (§11); validate + assert here (§8) | destroy-and-recreate persistent entities; delete outside delta scope; auto-apply Tier-3 results (§13) |
| `io/` | parse JSON → `core/model`; validate at parse boundary, reject authored bad data with a message (§8) | invent catalog specs for missing entries (`data-model.md` §3) |
| `render/` | read-only on the registry; all axis/handedness conversion in `threepp_conv.hpp` only (§7) | write to the registry; perform frame math; inline a second copy of the basis-change matrix |
| `gui/` | drive drag/edit via the sync round-trip; classify re-solve tier *before* running it (`interaction.md`) | mutate solver internals directly; apply a background Tier-3 result without explicit user action |
| `adapters/` (`glm_conv`, `boost_conv`, `threepp_conv`) | the ONLY place foreign libs are `#include`d; convert at boundaries; strip/reattach units explicitly (§6) | leak a foreign type back into `core/` storage; convert per-element mid-kernel (§5.1) |

Litmus (self-check): if `#include <glm/...>`, `<boost/...>`, `<entt/...>`, or `<threepp/...>` appears outside `adapters/`, `ecs/`, `render/`, `gui/`, or an optimizer seam, the abstraction has failed — stop and relocate it.

## 13. Re-solve scope ↔ sync scope

The `interaction.md` tiers map directly onto sync mode and scope. Claude Code should treat the tier as determining what sync is allowed to touch:

| Tier | Solver scope | Sync mode | Sync may touch |
|---|---|---|---|
| 0 — free move | none (live constraint check only) | **no sync** | nothing — the dragged pose is provisional until released; only overlap/reach flags update |
| 1 — local geometry | warm-start Layer 3, frozen outside radius | **delta** | poses of objects inside the active radius; nothing else, no deletions |
| 2 — partial structural | partial L1 re-expand + L2 rebind + local L3 on affected segment | **delta** | create/update/delete confined to the affected segment's equipment + frames; rest of cell untouched |
| 3 — full re-solve | full LNS, async background | **full — but PROPOSED** | nothing until the user accepts; on accept, full sync. **Never auto-apply** (`interaction.md`) |

The load-bearing pairing: a **delta sync must never delete outside its scope.** Outside the re-solved region, absence from the (partial) solution means "not considered this run," not "removed." Only a full sync treats absence as deletion (§11). Tier-3's result is a *proposed* full solution held aside; sync applies it only on explicit accept, and silently rearranging the cell after a single drag is prohibited.

## 14. Open / deferred

- Optimizer library choices (Layer 2/3) — open, pending validation; **do not scaffold against either API (§9)**.
- threepp handedness — **resolved (§7): right-handed, basis-change matrix locked.**
- Footprint world-cache vs. frame-world-pose cache — **resolved: kept separate (two-level cache).** World footprints (`world_hull`/`world_union`) are stored and `dirty_world`-invalidated, not derived from a general frame cache. The two triggers (`dirty_local` intra-station, `dirty_world` station-move) map to two genuinely independent events (§4.3, §5.4); a frame-world-pose cache would store a different, cheaper thing and would not subsume the footprint cache. Architecture and `data-model.md` §3.1 now agree.
- Sync transactional rollback — deferred (§11); MVP uses staged-then-commit.
- Boost→CGAL swap — supported by design, not anticipated for MVP.
