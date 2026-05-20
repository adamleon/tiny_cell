# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

# TinyCell — Claude Code conventions

## Build & test

First-time configure (Release, with compile_commands.json for clangd):

    cmake -S . -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

Build everything:

    cmake --build build --parallel

Build only the tests (CI-equivalent — no OpenGL/X11 deps needed):

    cmake --build build --target TestCell TestSolver TestFactoryScene TestSim

Run all tests:

    ctest --test-dir build --output-on-failure

Run a single test binary directly. Tests load assets via relative paths, so run from the repo root:

    ./build/Debug/TestSim.exe        # on Windows MSVC
    ./build/TestSim                  # on Linux/Mac

`TestSim` is the largest binary (~130 tests covering sim, throughput models, cost model, options enumerators, solver strategies, depot, demos). The custom harness uses `REQUIRE` / `REQUIRE_EQ` / `REQUIRE_NEAR` macros — no gtest.

Demos are the only application entry points — there is no monolithic app. Build and run by name:

    cmake --build build --target palletizing           # belt-based palletizing
    cmake --build build --target static_palletizing    # depot-based + KUKA URDF + IK
    cmake --build build --target fence_walls           # fence layout demo
    cmake --build build --target htn_sandbox           # throwaway solver experiment

Demos require OpenGL/X11 system libs. CI disables them with `-DTINYCELL_BUILD_DEMOS=OFF`.

## Architecture

Four interlocking layers, plus an experiments folder for spikes.

### 1. Fence solver — `src/solver/`, `src/cell/`

Pure functions over POD types. No ECS, no threepp. Takes a `SolverInput` + a panel `LookupTable` (loaded from `combinations.json`) and returns a `SolverOutput` with stable `EntityId`s. `cell/fence_solver.hpp` owns the JSON loader; `solver/solver.hpp` is the computation. This is the *original* solver in the project — it sizes fence panels and openings around a declared cell perimeter.

### 2. Sim / ECS world — `src/factory_scene/`

`FactoryScene` wraps an `entt::registry` plus a `solver::EntityId ↔ entt::entity` bimap so fence-solver output round-trips into components. The simulation is split into headers:

- `sensor_systems.hpp` — refresh physical-sensor `blocked` flags (laser model)
- `transport_systems.hpp` — belt motion + picker/magic state machines
- `station_systems.hpp` — pure dispatch (claim pallet, assign pickers, write virtual sensors)
- `lifecycle_systems.hpp` — source spawn + sink despawn, returns `LifecycleEvents` for the demo to consume

Tick order is sensor → transport → station → lifecycle. The full transport / port / sensor / station model is in [docs/TRANSPORT_MODEL.md](docs/TRANSPORT_MODEL.md).

Transport archetypes (each is a component on an entity that also carries `TransportComponent`):

- **ConveyorBelt** (`components.hpp::ConveyorBeltComponent`) — straight belt, items advance at `belt_speed_mm_s`.
- **Picker** (`PickerTransportComponent`) — robotic arm; straight-line cartesian motion at `speed_mm_s` between pickup/drop targets via the `PickerState` state machine.
- **Magic** (`MagicTransportComponent`) — placeholder transport with parabolic-arc visual + particle effects. **Used in the belt-based palletizing demo.** Stays in the codebase; the user will decide when (if ever) to remove it.
- **Depot** (`depot_components.hpp::DepotTransportComponent`) — stationary platform; items appear at pattern-determined positions, don't move. Backpressure via laser sensor on the port.

Station archetypes (additional components on a station entity):

- **Palletize** (`station_components.hpp::PalletizeComponent`) — claims a pallet via `pallet_arrival_port`, dispatches pickers to place boxes from `arrival_port`, releases when full + all pickers idle.

The generic `PlacementPattern` (in `placement_pattern.hpp`) operates on a `PlacementSurface` (length / width / height / placed_count) and works for both palletizers and depots. `GridPattern` is the only implementation today.

### 3. Render / demos — `src/factory_scene/render_system.hpp`, `src/demos/`

`render_system.hpp` reads ECS state and writes threepp objects — no simulation logic. Sub-namespaces handle different rendering concerns:

- `render::buildScene` — fence panels + posts
- `render::magic::ParticleSystem` — sparkle effects for active magic transports
- `render::robot::Registry` — loads URDFs lazily via threepp's `URDFLoader`, owns the `threepp::Robot` visuals, runs IK (`factory::ik::DLSSolver`) every frame to make the arm track the picker's TCP. The registry uses a `FlatGeometryLoader` to strip ColladaLoader's Z_UP rotation (ColladaLoader and the URDF's visual-origin handling disagree about up-axis; without the strip, DAE meshes are offset from the kinematic chain).

Demos at `src/demos/` are top-level executables that wire a `FactoryScene` to a threepp window via `common/scene_setup.hpp`. They handle mesh creation for spawned items, particle effects, and (in `static_palletizing`) released-pallet polling.

### 4. Station solver — `src/factory_scene/`, prefixed `cost_*`, `throughput.hpp`, `robot_arm_catalog.hpp`, `station_solver_*`

A multi-layered solver being built up bottom-up. Each layer is pure-function over POD types, no ECS, no threepp.

- **`throughput.hpp`** — analytic max-throughput per archetype: `belt(p, g)`, `picker(p, g)`, `palletizer(boxes_per_pallet, per_picker_rate, num_pickers)`. Each function returns items/min. Validated against the sim (`*_idealised_matches_sim_*` tests).
- **`robot_arm_catalog.hpp`** — `RobotArmSpec` POD + JSON loader. Catalog at `assets/robots/kuka/catalog.json`. Resolves null/optional fields (lifetime → 14 yr, maintenance → 4% of price, idle power → controller_class lookup) at load time.
- **`cost_model.hpp`** — `factory::cost::estimate(spec, task, ext) → CostBreakdown`. Annual lifecycle cost per single robot arm: capex / lifetime + maintenance + energy. Feasibility filter (payload, reach, speed, accel). Constants (α, β, k_friction, η_drive, η_regen, g, peak speed factor, accel estimate factor) live at namespace scope. Full formula + assumptions in [docs/COST_MODEL.md](docs/COST_MODEL.md).
- **`cost_options.hpp`** — enumerators that walk the catalog and return all feasible candidates: `robot_arm_options(...)` and `palletizer_options(...)`. The palletizer enumerator iterates `(num_pickers, arm)` combinations so the trade-off is *solver-visible* (the cost layer surfaces both "1× big arm" and "3× small arm" rather than picking internally).
- **`station_solver_types.hpp`** — shared vocabulary: `TaskKind` enum, `Task` (by-key parameter struct), `ProposalEquipment`, `Proposal`. The architecture is AND-OR graph search: each `Task` is an OR node (multiple strategies can solve it); each strategy's `Proposal.remaining` is an AND node (all sub-tasks must be resolved). Implemented incrementally.
- **`station_solver_strategy.hpp`** — `Strategy` abstract interface + `StrategyContext` (catalog reference + `cost::ExternalParams`).
- **`station_solver_arms_strategy.hpp`** — `ArmsStrategy`: wraps `cost::palletizer_options` behind the `Strategy` interface. Phase 1 of porting the HTN search from the `experiments/htn_sandbox/` spike into the main solver.

The station-solver port from sandbox to main is intentionally phased — each step is reversible and tested. Phase 1 (Strategy interface + ArmsStrategy) is done; Phases 2-5 add the search loop, sub-task expansion, geometry, and outer position search. See the experiments/htn_sandbox/README.md for the architectural pattern being ported.

### Experiments — `experiments/`

Throwaway / exploratory code that doesn't belong in the production tree. Currently just `htn_sandbox/` — a self-contained ~700-line C++ program that demonstrates HTN search with geometric placement, multi-task claims, branch-and-bound, and outer-loop task-position enumeration in a 1-D toy domain. Used as an architectural sketch before porting back to the main solver. Build target: `htn_sandbox`.

## Dependencies

Vendored via CMake FetchContent:

- **threepp** — pinned to a specific commit hash in `CMakeLists.txt`. Bumping it is a deliberate change. Provides `URDFLoader`, `ColladaLoader`, `STLLoader`, `Robot` (with FK via `computeEndEffectorTransform`).
- **EnTT** 3.16 — ECS registry.
- **GLM** 1.0.3 — vec/mat/quat types via the `Vec3` / `Quat` / `Mat4` aliases in `pose_component.hpp`. **Nothing outside `pose_component.hpp` should reference `glm::` directly** — change the typedefs there to swap libraries.
- **nlohmann/json** — vendored inside threepp's `src/external/nlohmann`. Used for fence catalog, robot catalog, HTN sandbox input.

## Component design rules

Every component in `src/factory_scene/components.hpp` (and adjacent `*_components.hpp`) must follow this contract:

1. **All fields are private.** No public data members.
2. **Every field has a getter.** The getter is `const`, returns by value or `const&`, and has the same name as the field minus the trailing underscore.
3. **Every mutable field has a setter.** Prefix `set_`. For fields that accumulate (e.g. `spawn_debt`), use a dedicated mutation method instead.
4. **Setters validate invariants.** Use the `detail::` helpers in `components.hpp`:
   - `detail::finite_non_neg(v)` — floats that must be ≥ 0 and finite
   - `detail::positive(v)` — ints that must be > 0
   - `detail::non_neg(v)` — ints that must be ≥ 0
   - `detail::unit_interval(v)` — floats in [0, 1]
   - `detail::valid_entity(e)` — must not be `entt::null`
   - Assert in debug, compile out in release.
5. **No external validation.** Callers must not re-check invariants that setters already enforce.

### Example

```cpp
struct ConveyorBeltComponent {
    int   length_mm()       const { return length_mm_; }
    float belt_speed_mm_s() const { return belt_speed_mm_s_; }

    void set_length_mm(int mm)        { length_mm_       = detail::positive(mm); }
    void set_belt_speed_mm_s(float v) { belt_speed_mm_s_ = detail::finite_non_neg(v); }

private:
    int   length_mm_       = 2000;
    float belt_speed_mm_s_ = 200.f;
};
```

## Coordinate system

- ECS is Z-up, millimetres: `x` = east, `y` = north, `z` = height.
- Floor plane is the X-Y plane (`z = 0`).
- threepp is Y-up, metres: `threepp(x, y, z) = ECS(x * 0.001, z * 0.001, y * 0.001)`. The render loop applies this swap (see `static_palletizing.cpp` for the canonical conversion).
- Never use `x_mm` / `z_mm` for floor-plane pairs — always `x_mm` / `y_mm`.
- The cost model (`factory::cost`) and throughput model (`factory::throughput`) use SI units (m, kg, s, W, EUR) — callers in ECS-mm land convert at the boundary.

## sim / render separation

- The four sim headers (`sensor_systems.hpp`, `transport_systems.hpp`, `station_systems.hpp`, `lifecycle_systems.hpp`) are pure ECS — no threepp types.
- `render_system.hpp` reads ECS state and writes threepp objects — no simulation logic.
- `lifecycle::step()` returns `LifecycleEvents { spawned, despawned }`; the render loop is responsible for creating and removing meshes (and destroying the corresponding entities).
- `render::robot::Registry` is the only render-side structure that *also* drives behavior (IK each frame), but it does so by reading the ECS picker pose and writing to the threepp Robot — no ECS-side mutation.

## Asset organisation

`assets/` follows the convention `assets/type/brand/(family if many files)/`. The catalog file always lives **at the brand level**:

- `assets/components/fences/axelent_x-guard/catalog.json` + post + panel OBJs
- `assets/robots/kuka/catalog.json` — covers all KUKA families (Agilus + Cybertech + Quantec PA)
- `assets/robots/kuka/agilus/urdf/*.urdf` + `meshes/...` — URDFs are family-scoped because each family has many mesh files

URDF references inside the catalog are relative to the catalog file's directory (e.g., `"agilus/urdf/kr10_r1100_2.urdf"`). Catalog entries can have `urdf: null` for arms whose mesh we don't have yet — the cost model works without one; only the renderer needs URDFs.

## Testing

- Unit tests live in `tests/`. Each file corresponds to a layer (cell, solver, factory_scene, sim).
- No threepp dependency in tests — they link only `EnTT` + `GLM` + the vendored nlohmann/json include path.
- The custom harness uses `REQUIRE` / `REQUIRE_EQ` / `REQUIRE_NEAR` macros — no gtest.
- To test that a setter rejects an invalid value, run the test binary under a death-test harness (or use a dedicated subprocess test). The current harness cannot catch `abort()`.

## Notes for ongoing work

- **Magic transport stays.** `MagicTransportComponent` is an intentional placeholder used by the belt-based palletizing demo. Don't propose removing it, the magic particle effects, or the `agent_*` polymorphism in `station_systems.hpp` that handles both pickers and magic. The user will decide when (if ever) to remove it.
- **Architecture work is concrete-first.** When introducing new abstractions (interfaces, type ontologies, search algorithms), build at least two concrete implementations first and *then* extract the shared shape. Designing the interface before any real consumer exists has burned us; sandbox spikes in `experiments/` are how we validate before committing.
- **Naming**: new robot-arm-related code uses `RobotArm*` (e.g., `RobotArmSpec`). Older code uses `Picker*` (e.g., `PickerTransportComponent`). They refer to overlapping concepts; the rename is pending but not blocking.
