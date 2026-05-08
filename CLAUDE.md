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

    ./build/TestSim

Demos are the only application entry points — there is no monolithic app. Build and run by name:

    cmake --build build --target palletizing
    ./build/palletizing

Demos require OpenGL/X11 system libs. CI disables them with `-DTINYCELL_BUILD_DEMOS=OFF`.

## Architecture

Three layers, bottom-up:

1. **`src/solver/`** — pure functions over POD types. No ECS, no threepp. Takes a `SolverInput` + a panel `LookupTable` (loaded from `combinations.json`) and returns a `SolverOutput` with stable `EntityId`s. `cell/fence_solver.hpp` owns the JSON loader; `solver/solver.hpp` is the computation.

2. **`src/factory_scene/`** — the ECS world (EnTT). `FactoryScene` wraps an `entt::registry` plus a `solver::EntityId ↔ entt::entity` bimap so solver output round-trips into components. The simulation is split into four headers: `sensor_systems.hpp` (refresh physical-sensor `blocked` flags), `transport_systems.hpp` (belt motion + picker state machine), `station_systems.hpp` (pure dispatch — claim pallet, assign pickers, write virtual sensors), `lifecycle_systems.hpp` (source spawn + sink despawn, returns spawned/despawned events for the demo to consume). Tick order is the order above. `render_system.hpp` reflects ECS state into threepp meshes — see *sim / render separation* below. The full transport / port / sensor / station model is in [docs/TRANSPORT_MODEL.md](docs/TRANSPORT_MODEL.md).

3. **`src/demos/`** — top-level executables that wire a `FactoryScene` to a threepp window via `common/scene_setup.hpp`.

Dependencies are vendored via CMake FetchContent: **threepp is pinned to a specific commit hash** (see `CMakeLists.txt`) — bumping it is a deliberate change, not automatic. EnTT 3.16, GLM 1.0.3.

## Component design rules

Every component in `src/factory_scene/components.hpp` must follow this contract:

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
- threepp is Y-up: `threepp(x, y, z) = ECS(x * 0.001, z * 0.001, y * 0.001)`.
- Never use `x_mm` / `z_mm` for floor-plane pairs — always `x_mm` / `y_mm`.

## sim / render separation

- The four sim headers (`sensor_systems.hpp`, `transport_systems.hpp`, `station_systems.hpp`, `lifecycle_systems.hpp`) are pure ECS — no threepp types.
- `render_system.hpp` reads ECS state and writes threepp objects — no simulation logic.
- `lifecycle::step()` returns `LifecycleEvents { spawned, despawned }`; the render loop is responsible for creating and removing meshes (and destroying the corresponding entities).

## Testing

- Unit tests live in `tests/`. Each file corresponds to a layer (cell, solver, factory_scene, sim).
- No threepp dependency in tests.
- The custom harness uses `REQUIRE` / `REQUIRE_EQ` / `REQUIRE_NEAR` macros — no gtest.
- To test that a setter rejects an invalid value, run the test binary under a death-test harness (or use a dedicated subprocess test). The current harness cannot catch `abort()`.
