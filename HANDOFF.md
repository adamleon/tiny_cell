# HANDOFF — POST-MVP LEG: the sales demo

**Transient orientation for a fresh context.** The MVP is complete; the chosen
post-MVP leg is a **sales demo**. This file is the demo's living plan — update
the status line and phase checkboxes as they land; rewrite it if the leg
changes. Permanent docs unchanged: `CLAUDE.md` (rules + anti-stray gate),
`docs/roadmap.md` (MVP build order), `docs/decisions.md` (settled choices),
`docs/architecture.md` / `data-model.md` / `interaction.md` (END-STATE design —
a description, not a backlog).

---

## The goal

A **video/GIF sales demo** to pitch a prospective industrial partner who uses
**KUKA.Sim**, showing that tiny_cell *could* replace/beat it *if they bring the
industrial accuracy*. Ballpark accuracy is fine; the demo may be
fabricated/tailored. **Pitch narrative = generative layout + feasibility
insight, NOT photoreal simulation** (we don't out-render KUKA.Sim; we win on
"it designs the cell for you and shows you where things can move").

Three asks:
1. **Visually appealing animated 3D** — cute Tiny Glade style; robot stacks
   pallets, belts move items; a **"spawn/assemble" animation** on Generate
   (entities assemble in sequence).
2. **Workflow → cell** — a fresh, camera-tuned scenario; the workflow is loaded
   **from JSON** and shown as an exportable **SVG** diagram (for emails/docs).
   Not interactive.
3. **Basic interaction** — click a robot → highlight the floor region where it
   can be relocated **without the solution changing**, found by sweeping its
   pose and re-running the **cheap solver layers** until its rank changes (would
   force swapping it out). Covers the "new robot type" angle.

## Locked decisions (2026-07-01)

- **Standalone tiny_cell app**, **pure fullscreen threepp Vulkan
  (deferred-hybrid) renderer** + a minimal ImGui overlay for Generate/pick. NOT
  built on `utsyn`; NO threepp fork; no ImGui-in-Vulkan framebuffer trick.
- **Real KUKA URDF** (KR-series, e.g. KR120 PA) sourced for the arm.
- **Minimal UI**: viewport + Generate + pick.
- **`ecs`/`sync` EnTT registry deferred** — `render/` translates a solver
  `LayoutSolution` (+ `AllocationResult`/`PlacedBelt` side-tables) → threepp
  directly. The registry earns its place only if the demo grows a real
  edit-and-re-lay-out-in-place interaction (the "swap → recompute highlight"
  interaction does NOT cross that line). So the §0 `render/` row's "read-only on
  the registry" is, for the demo, "read-only on solver output".
- **`physics_simulator` archived**; PhysX out of scope. `utsyn` (a sibling
  threepp-Vulkan rviz2 alternative) is proof the backend runs on the RTX 3090 +
  the CMake recipe source, NOT a design to copy.

## Why the demo is not straying

The demo is the single **consumer** that authorizes each module, opened one at a
time crudest-first (CLAUDE.md anti-stray gate). It touches `render/` (new), then
`io` (workflow loader), `svg` (workflow diagram), and a minimal `gui`/pick — each
pulled in only when a phase concretely needs it, never speculatively.

---

## The phased plan (crudest-first; de-risk the long poles early)

- [x] **Phase 0 — Vulkan render spike** *(DONE — verified 2026-07-01)*. Standalone
  exe (`render/src/spike.cpp` → target `tinycell_render_spike`) builds threepp's
  Vulkan backend inside tiny_cell's vcpkg+FetchContent tree and renders a lit box
  on a floor with an orbit camera. **Verified:** configure green; build+link green;
  ran on the RTX 3090 (`--frames 60 --out spike.png`) → correct path-traced image
  (warm-lit box, floor, contact shadow). The vcpkg+FetchContent+Vulkan-SDK
  coexistence risk is retired. Benign NVIDIA swapchain-semaphore validation
  warnings at runtime (same as utsyn). Build/run loop below.
- [x] **Phase 1 — scenario authoring** *(DONE — verified 2026-07-01)*.
  **1a** `io::load_workflow(json)` → `vector<Task>` (mirrors the catalog loaders
  + `ParseError`; `io/{include,src}/.../workflow_loader.*`), tested
  (`tests/test_workflow_loader.cpp`). **1b** `svg::render_workflow_svg` /
  `write_workflow_svg` — a stdlib-only left-to-right workflow DAG
  (`svg/{include/tinycell/workflow_svg.hpp,src/workflow_svg.cpp}`), tested
  (`tests/test_workflow_svg.cpp`); svg/ charter row in CLAUDE.md §0 extended to
  authorize the input-workflow diagram. **1c** the fresh camera-tuned scenario
  `assets/workflow/two_line_palletizer.json` (2 feeders + 2 pallet supplies → 2
  cells → shared dispatch, 13 tasks), loads + renders green. Built/tested in a
  `build-p1` tree (tests ON, Vulkan OFF, vcpkg reuse workaround); 4/4 workflow
  tests pass. KNOWN NIT: the fixed 3-column layout lets pallet-supply→cell
  diagonals cross feed edges and overlaps a couple of edge labels — readable,
  refine to a layered layout only if a scenario needs it.
- [ ] **Phase 2 — solver → 3D scene, static.** `render/` reads a `LayoutSolution`
  and builds a threepp scene (floor, equipment, belts, pallets, KUKA arm via
  `URDFLoader`). Axis/handedness in ONE `render/threepp_conv.hpp` constant
  (+ round-trip identity test, §7).
- [ ] **Phase 3 — art direction (Tiny Glade look).** Materials, warm lighting,
  soft shadows, tone mapping, palette, framing. The Vulkan PT gives real GI/AO/
  soft shadows live.
- [ ] **Phase 4 — animation.** Spawn/assemble tween; canned KUKA joint keyframes
  (`setJointValue`); items on belts; pallet builds up. Presentation-layer only
  (render owns the tween clock; the solved scene stays the static ground truth).
- [ ] **Phase 5 — pick + feasibility-region highlight** *(the differentiator)*.
  threepp raycast → selected equipment; **needs a new cheap solver entry point**
  (`evaluate_candidate_at_pose(equipment, pose) → {feasible, score}` reusing the
  Layer-1/2 evaluators, NOT the Layer-3 NLP); sweep a floor grid, mark keep-rank
  cells, draw a glowing region; "swap KR120→KR6, region shrinks".
- [ ] **Phase 6 — capture the GIF.** Compose the reel on the RTX 3090.

**Dependencies:** Phase 0 gates 2–5; Phase 1 is fully parallel; 3/4/5 overlap
after 2. **Parallel asset track:** source the KUKA KR-series URDF+meshes
(kuka_experimental / vendor STEP→mesh).

**Open risk to confirm early:** the Phase-5 cheap-sweep entry point — verify the
solver can cheaply re-rank one equipment at a candidate pose without a full solve.

---

## Build / test loop

**Core build unchanged** (Vulkan is OFF by default): from a VS dev shell with
`VCPKG_ROOT` set, `cmake --preset default` → `cmake --build build/default
--config Debug` → `ctest --test-dir build/default`. 19/19 green.

**Vulkan render module** (opt-in, separate `build-vk` tree so the core build is
untouched). Needs the Vulkan SDK. threepp is pulled by FetchContent; point it at
the local clone to skip the download:
```
$env:VCPKG_ROOT='D:\development\vcpkg'
$env:VULKAN_SDK='C:\VulkanSDK\1.4.350.0'
cmake -S D:\development\tiny_cell -B D:\development\tiny_cell\build-vk `
  -G "Visual Studio 17 2022" -A x64 `
  -DCMAKE_TOOLCHAIN_FILE=D:/development/vcpkg/scripts/buildsystems/vcpkg.cmake `
  -DVCPKG_TARGET_TRIPLET=x64-windows `
  -DTINYCELL_WITH_VULKAN=ON `
  -DFETCHCONTENT_SOURCE_DIR_THREEPP=D:/development/threepp   # optional: reuse local clone
cmake --build D:\development\tiny_cell\build-vk --config Debug --target tinycell_render_spike --parallel
build-vk\Debug\tinycell_render_spike.exe                    # interactive
build-vk\Debug\tinycell_render_spike.exe --frames 90 --out spike.png   # headless smoke → PNG
```
Notes / gotchas:
- The local vcpkg is a **blobless clone missing the pinned baseline blob**, so a
  clean manifest-mode install fails (`versions/baseline.json ... not in
  <baseline>`). Fix either by `git fetch` in `D:\development\vcpkg`, or (what the
  spike build used) reuse the already-installed packages:
  `-DVCPKG_MANIFEST_MODE=OFF -DVCPKG_INSTALLED_DIR=D:/development/tiny_cell/build/default/vcpkg_installed`.
- threepp embeds its SPIR-V shaders as C++ headers at build time (no runtime
  shader files); static threepp needs no DLL copy.
- MSBuild drives `cl` via the VS generator — no dev shell needed, just the two
  env vars. In PowerShell don't `2>&1` native exes; check `$LASTEXITCODE`. Kill a
  running `tinycell_render_spike.exe` before rebuilding (it locks the exe).

---

## Orientation — where things live

- **New:** `render/` (`CMakeLists.txt`, `src/spike.cpp`). Root `CMakeLists.txt`
  gates it behind `option(TINYCELL_WITH_VULKAN ... OFF)` + a FetchContent block
  (VMA + threepp) at the end.
- **Solver output the demo consumes:** `solver/include/tinycell/solver/
  layout_problem.hpp` (`LayoutSolution`), `allocator.hpp` (`AllocationResult`/
  identity), `belt_routing.hpp` (`PlacedBelt`); `demos/solve_workflow/main.cpp`
  shows the build-time side tables.
- **Reference (do not copy as design):** `D:\development\utsyn` (threepp-Vulkan
  recipe + RTX-3090 proof), `D:\development\threepp` (local clone, branch
  `feat/vulkan-rendertarget`).
- **Everything MVP:** `docs/roadmap.md` records steps 1–6.
