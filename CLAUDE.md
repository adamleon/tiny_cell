# TinyCell — Claude Code conventions

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

- `sim_systems.hpp` is pure ECS — no threepp types.
- `render_system.hpp` reads ECS state and writes threepp objects — no simulation logic.
- `sim::step()` returns `StepEvents`; the render loop is responsible for creating and removing meshes.

## Testing

- Unit tests live in `tests/`. Each file corresponds to a layer (cell, solver, factory_scene, sim).
- No threepp dependency in tests.
- The custom harness uses `REQUIRE` / `REQUIRE_EQ` / `REQUIRE_NEAR` macros — no gtest.
- To test that a setter rejects an invalid value, run the test binary under a death-test harness (or use a dedicated subprocess test). The current harness cannot catch `abort()`.
