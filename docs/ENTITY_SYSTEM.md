# Entity System

## Overview

The scene is managed as an Entity-Component-System using **EnTT**. The `FactoryScene` wraps an EnTT registry and is the **single source of truth** for all scene state. Every visible or interactive element in the scene is an entity.

This document covers the entity types, components, systems, and the rules for extending the system. For how the layout solver uses entity data, see [LAYOUT_SOLVER.md](LAYOUT_SOLVER.md).

---

## Why EnTT

- Dynamic component composition: not every entity has every component. The renderer iterates only entities with a VisualComponent; the solver reads only entities with layout-relevant components (NodeComponent, EdgeComponent, DeclaredOpeningComponent, etc.).
- Stable `entt::entity` handles with generation counters — safe across insertion and deletion, with built-in detection of stale references.
- Header-only, MIT licensed, widely used in production.
- Views and queries scale cleanly as entity count grows.

---

## Entities

Every visible or interactive element is an entity, referenced by a stable `entt::entity` handle. This includes elements that are *derived* by the system rather than explicitly placed by the user — an Edge entity is created automatically when two Node entities are connected, but it is still a full scene participant with rendering and interaction.

| Entity | Created by | Components |
|---|---|---|
| Node | User / blueprint import | Pose, Node, Visual *(future)*, Interactive *(future)* |
| Edge | System (when two nodes connect) | Pose, Edge, Visual *(future)*, Interactive *(future)* |
| DeclaredOpening | User (may be unallocated — no edge yet) | Pose, DeclaredOpening, Visual *(future)*, Interactive *(future)* |
| WorldFeature | User / blueprint import | Pose, WorldFeature *(future)*, Visual *(future)*, Interactive *(future)* |
| SpanConstraint | User / blueprint import | Pose, SpanConstraint *(future)*, Interactive *(future)* |
| Robot | User / blueprint import | Pose, Robot *(future)*, Visual *(future)*, Interactive *(future)*, Simulation *(future)* |
| ConveyorBelt | User / blueprint import | Pose, ConveyorBelt *(future)*, Visual *(future)*, Interactive *(future)*, Simulation *(future)* |
| WorkPiece | Simulation | Pose, Visual *(future)*, Simulation *(future)* |

An entity's role is determined by which components it carries — there is no discriminator field. `FactoryScene` is the only place that creates entities and attaches components, which prevents nonsense combinations.

---

## Components

### PoseComponent

Every entity carries a `PoseComponent`. It holds position, orientation, and a parent entity reference, forming the scene hierarchy.

```
PoseComponent
  position     Vec3          — in the parent entity's reference frame (mm)
  orientation  Quat          — identity by default
  parent       entt::entity  — see encoding below
```

Parent encoding:
- `parent == entt::null` — unallocated: entity exists but has no world position yet
- `parent == self` — world root: exactly one per FactoryScene (the scene entity, ID 1)
- `parent == other` — placed in that entity's local frame

Axis convention: **x = forward, y = left, z = up** (right-hand, Z-up / ROS industrial). The RenderSystem applies the Z-up → Y-up conversion when building threepp scene objects; nothing outside the RenderSystem should reference Y-up.

`world_transform(entity, registry)` walks the parent chain and returns the world-space matrix. Returns identity for unallocated entities.

### NodeComponent

Carried by node entities. Holds the solver-assigned node type (always solver output, never user input).

```
NodeComponent
  type  NodeType   — Post | Straight | RoundedCorner | AngledPost | Gap | None
```

### EdgeComponent

Carried by edge entities. Holds fence topology and catalog reference; no geometry or rendering data.

```
EdgeComponent
  node_a       entt::entity             — first endpoint
  node_b       entt::entity             — second endpoint
  spans_mm     vector<vector<int>>      — solved panel width sequences per span
  catalog_ref  string                   — edge-scoped catalog path
```

`spans_mm` and `SpanType` are solver output, written by `apply()`. They are rendering-independent — the VisualUpdateSystem reads them to select catalog variants and write `VisualComponent`.

### DeclaredOpeningComponent

Carried by declared opening entities. Holds the user's intent for a reserved space on an edge.

```
DeclaredOpeningComponent
  parent_edge          entt::entity   — absent (null) if unallocated
  desired_position_mm  optional<int>  — absent if solver-assigned, set when anchored
  width_mm             int            — always present
  mobility             float          — 0.0 = immovable (default and fail-safe)
```

Allocation state is determined entirely by which optional fields are set — no separate flag. See [LAYOUT_SOLVER.md](LAYOUT_SOLVER.md) for how the solver handles each allocation state.

### VisualComponent *(future)*

Owns the threepp scene object for this entity. The VisualUpdateSystem writes the asset key and visibility state here after each solve; the RenderSystem reads it every frame.

Holds the threepp `Object3D`, an asset key for catalog-driven geometry, and visibility/highlight state. 3D geometry and animations live here — never in the data model or the solver.

### InteractiveComponent *(future)*

Owns hit geometry, gizmo configuration, and event callbacks. An entity without an `InteractiveComponent` cannot be selected or dragged. Callbacks mutate component data (NodeComponent, EdgeComponent, DeclaredOpeningComponent, etc.) and notify LayoutSolverSystem. They do not touch the solver directly.

### SimulationComponent *(future)*

Carries kinematics state for robots, motion state for conveyors, and pick/place task state. Additive — does not change existing component behaviour.

---

## Systems

### LayoutSolverSystem

**Trigger:** any layout-relevant component mutation (NodeComponent, EdgeComponent, DeclaredOpeningComponent, WorldFeatureComponent, SpanConstraintComponent).

Uses `SolverInputBuilder` to translate entity components into a `SolverInput`, then calls the solver. The last `SolverOutput` stored on the FactoryScene is passed in as a warm start. The solver returns a new `SolverOutput`; `FactoryScene.apply()` then translates it into updated ECS components (PoseComponent, NodeComponent, EdgeComponent, DeclaredOpeningComponent, etc.). The ECS is the single source of truth after `apply()` completes. The raw `SolverOutput` is retained only as a warm-start hint for the next solve.

### VisualUpdateSystem *(future)*

**Trigger:** after `apply()` completes.

Reads layout components (EdgeComponent, NodeComponent, DeclaredOpeningComponent) and the solver's SpanType decisions to write or update VisualComponents — selecting asset keys from the catalog and setting visibility state. This is the only place that translates layout semantics into rendering assets. The RenderSystem never reads layout components directly.

### RenderSystem

**Trigger:** every frame (or when a VisualComponent changes).

Reads PoseComponent and VisualComponent for every entity that has both. Applies the world transform (from `world_transform()`) to the threepp Object3D owned by VisualComponent. The Z-up → Y-up coordinate conversion happens here and nowhere else. The RenderSystem has no knowledge of NodeComponent, EdgeComponent, or solver output.

### InteractionSystem

**Trigger:** user input events (mouse, touch, keyboard).

Raycasts against hit geometry in InteractiveComponents. On hit, invokes the entity's callbacks. Callbacks mutate component data (e.g. updating `desired_position_mm` on a DeclaredOpeningComponent) and notify LayoutSolverSystem.

### SimulationSystem *(future)*

Steps the physics simulation and robot kinematics. Writes PoseComponents back from simulation state. Reads EdgeComponent and NodeComponent to know the fence boundary — not the raw SolverOutput.

---

## Data Flow

```
Entity components (NodeComponent, EdgeComponent, DeclaredOpeningComponent, …)
    │
    ▼  SolverInputBuilder
SolverInput + warm_start SolverOutput?
    │
    ▼  Solver  (pure function, no EnTT dependency)
SolverOutput  ─── retained on FactoryScene as warm-start hint only
    │
    ▼  FactoryScene.apply()
Updated ECS components (PoseComponent, NodeComponent, EdgeComponent, …)  ← single source of truth
    │
    ▼  VisualUpdateSystem  (future)
VisualComponent  (asset keys, visibility)
    │
    ├──▶  RenderSystem  reads PoseComponent + VisualComponent → threepp scene graph
    └──▶  InteractionSystem  reads InteractiveComponent
              │  mutates entity components
              └──────────────────────────────┐
                                             ▼
                                     LayoutSolverSystem
                                     (triggered again)
```

---

## Entity References

All cross-entity references use **`entt::entity` handles** from the EnTT registry. These handles are stable across insertion and deletion — the generation counter detects use of a stale handle after its entity has been destroyed.

Raw integer indices into vectors are explicitly prohibited:
- They silently invalidate when any element is inserted or removed
- They carry no type safety
- They require a full scan for reverse navigation (e.g. finding all openings on an edge)

---

## Extension Rules

Before adding any new entity type or component:

1. **Every visible or interactive element is an entity in the EnTT registry.** No parallel lists, no special-cased scene objects outside the ECS.
2. **Rendering logic belongs in VisualComponent and RenderSystem.** The solver and layout components must remain rendering-independent.
3. **Interaction logic belongs in InteractiveComponent and InteractionSystem.** Selectability and drag behaviour are not properties of the data model.
4. **All cross-entity references use `entt::entity` handles.** Never raw integers.
5. **New entity types are defined by their component composition.** Do not add new special cases to existing systems — add a component.
6. **New entity types get their own component struct.** Never add a discriminator field to an existing component. A new entity type is defined entirely by which components it carries.
7. **SimulationComponent is additive.** Adding simulation to an entity type does not change its layout or visual behaviour.
