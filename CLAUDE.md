# CLAUDE.md — FactoryCell working rules

Checkable constraints for code *content*. Companion layers: `architecture.md` is the **reasoning** layer (the *why* behind these rules); `engineering.md` is the **infrastructure** layer (toolchain, dependencies, error-handling mechanism, tests — the *how it compiles, fails, and verifies*). Read this every task; open `architecture.md` for the why, `engineering.md` when standing up the build, adding a dependency, deciding how something fails, or writing a test. Bare section refs (§N) point into `architecture.md`; refs into other docs are named.

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

- **Optimizer libraries (Layer 2/3):** unchosen. Keep the seam abstract. (§9)
- **Cycle-time allocation across stations:** unresolved and **gates Layer 2** (`roadmap.md`) — how a whole-cell cycle-time target splits across stations for capacity packing. Do not build Layer 2 until decided.
- **Dependency manager (packaged libs):** vcpkg vs. Conan — [OPEN], leaning vcpkg; the real tiebreaker is OR-Tools/IPOPT-on-Windows build pain, settled by a proof-of-concept spike at Layer 3, not before. threepp is FetchContent regardless. Does not block `core/`. (`engineering.md` §2.3)
- **Sync transactional rollback:** deferred; MVP staged-then-commit. (§11)
- **Standards clearance numbers:** `standards.md` is a stub. Do **not** invent numeric clearances — they come from the standard or a safety engineer. Build the cache/geometry pipeline with a placeholder buffer; wire real values later without structural change. (`standards.md`)

Infra decisions already settled (do not re-litigate, see `engineering.md` §5): C++20 fixed (no modules / no `import std;`); CMake 3.28+; MSVC 194+ / GCC 13+ / Clang 17+ with Windows in CI day-one and GCC/Clang as conformance oracle; GoogleTest; typed `ParseError` at `io/` + assert/throw at `sync/`; mp-units contract checking ON in debug *and* release, pinned explicitly; `std::format` with fmtlib as MSVC fallback.

---

## 6. MVP scope — what to build vs. skip

MVP is **solver + `core/` only**, batch, no GUI (`roadmap.md`). Build order: catalog/strategy library → brute-force validator → item-state propagation → Layer 2 → Layer 3 → LNS.

- **Do NOT scaffold `sync/`, `ecs/`, `render/`, or `gui/` yet** — they are post-MVP. The §11 sync contract, §4.4 ECS mapping, §7 render conversion, and the right half of the module index (§0) describe the eventual system, not the MVP. Their rules still bind *when* those modules are built; they are simply not in the first milestone.
- **`svg/` IS in MVP scope** — debug oracle for any solver layer that produces geometry (`roadmap.md` "Supporting tools"). Build incrementally per layer that needs it, not as a separate milestone. Distinct from `render/`: SVG is stdlib-only batch output for diff-friendly inspection; `render/` is interactive 3D via threepp and arrives post-MVP. The two coexist after MVP.
- `core/` (geometry, units, model) is the foundation everything else needs and has zero external dependencies — start there, it has no build-standup blocker.
- **Windows is day-one even though the GUI is not.** `core/` and `solver/` must build and pass tests on MSVC from the first commit; deferring threepp/ImGui defers the GUI, *not* Windows. MVP dependency surface is small — mp-units, Boost.Geometry, glm — and builds cleanly under either dependency manager. (`engineering.md` §2.2, §6)
- **Build/error decisions block the first line of `core/`** and are already settled (§5 above, `engineering.md`) precisely so they don't stall you: C++20, CMake 3.28+, the compiler set, and the typed-exception / assert-throw error split must be in place before `core/` compiles. Mechanical style is enforced by committed `.clang-format`/`.clang-tidy`, not prose (`engineering.md` §2.5).
- Steps 1–3 (catalog → brute-force → item-state) are fully ready. Layer 2 has the cycle-time gate above; Layers 3/LNS depend on the optimizer choice (§9). The brute-force enumerator (step 2) **is** the strategy-library test harness, not a throwaway (`engineering.md` §4.2).
