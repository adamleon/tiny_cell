# Entity System

## Overview

The scene is managed as an Entity-Component-System using **EnTT**. The `FactoryScene` wraps an EnTT registry and is the **single source of truth** for all scene state. Every visible or interactive element in the scene is an entity.

This document covers the entity types, components, systems, and the rules for extending the system. For how the layout solver uses entity data, see [LAYOUT_SOLVER.md](LAYOUT_SOLVER.md).

---

## Why EnTT

- Dynamic component composition: not every entity has every component. The renderer iterates only entities with a VisualComponent; the solver reads only entities with a LayoutComponent.
- Stable `entt::entity` handles with generation counters — safe across insertion and deletion, with built-in detection of stale references.
- Header-only, MIT licensed, widely used in production.
- Views and queries scale cleanly as entity count grows.

---

## Entities

Every visible or interactive element is an entity, referenced by a stable `entt::entity` handle. This includes elements that are *derived* by the system rather than explicitly placed by the user — an Edge entity is created automatically when two Node entities are connected, but it is still a full scene participant with rendering and interaction.

| Entity | Created by | Components |
|---|---|---|
| Node | User / blueprint import | Visual, Interactive, Layout |
| Edge | System (when two nodes connect) | Visual, Interactive, Layout |
| DeclaredOpening | User (may be unallocated — no edge yet) | Visual, Interactive, Layout |
| WorldFeature | User / blueprint import | Visual, Interactive |
| SpanConstraint | User / blueprint import | Interactive |
| Robot | User / blueprint import | Visual, Interactive, Simulation (future) |
| ConveyorBelt | User / blueprint import | Visual, Interactive, Simulation (future) |
| WorkPiece | Simulation | Visual, Simulation (future) |

WorldFeature and SpanConstraint entities have no LayoutComponent. They influence the layout solver through their world positions and footprints, but they are not fence system participants — the solver reads them directly.

---

## Components

### VisualComponent

Owns the threepp scene object for this entity. The RenderSystem iterates all VisualComponents each frame and updates geometry from the current solved layout.

Holds:
- The threepp `Object3D` (or a reference to a prototype in the catalog)
- An asset key for catalog-driven geometry (fence panels, posts, robot meshes)
- Visibility and highlight state

3D geometry and animations live here, not in the data model or the solver.

### InteractiveComponent

Owns the hit geometry, gizmo state, and event callbacks. The InteractionSystem raycasts against hit geometry owned by InteractiveComponents. An entity without an InteractiveComponent cannot be selected or dragged.

Holds:
- Hit geometry (threepp object used for raycasting)
- Gizmo configuration (which handles are shown, their axes and constraints)
- Callbacks: `on_click`, `on_drag_begin`, `on_drag`, `on_drag_end`, `on_hover`

The callbacks mutate LayoutComponent data and trigger LayoutSolverSystem. They do not touch the solver directly.

### LayoutComponent

Carries the entity's role in the fence layout and the constraint data the solver needs. LayoutComponents are **never read directly by the solver** — a `SolverInputBuilder` translates them into a typed `SolverInput` before each solve. This decouples the solver from EnTT and means the solver can be tested without a registry.

Each entity type uses a subset of the fields:

```
LayoutRole — Node | Edge | DeclaredOpening | WorldFeature | SpanConstraint

Node
  (position is read from the scene transform — no extra fields)

Edge
  node_a          entt::entity
  node_b          entt::entity
  catalog_ref     string

DeclaredOpening
  parent_edge          entt::entity?   — absent if unallocated (solver assigns edge)
  desired_position_mm  int?            — absent if floating  (solver assigns position)
  width_mm             int             — always present
  mobility             float           — 0.0 = immovable (default and fail-safe)

WorldFeature
  type                 enum (Machine | Pillar | SolidWall | ConveyorIO)
  footprint            polygon or AABB in world space

SpanConstraint
  footprint            polygon or AABB in world space
  max_height_mm        int     (or other constraint property)
```

A DeclaredOpening's allocation state is determined by which optional fields are set — no separate flag is needed. See [LAYOUT_SOLVER.md](LAYOUT_SOLVER.md) for how the solver handles each state.

The LayoutComponent is serialisable and rendering-independent. Node positions are read from the entity's world transform rather than stored separately, keeping position as the single source of truth.

### SimulationComponent *(future)*

Carries kinematics state for robots, motion state for conveyors, and pick/place task state. Added when simulation is introduced; existing components do not change.

---

## Systems

### LayoutSolverSystem

**Trigger:** any LayoutComponent mutation.

Uses `SolverInputBuilder` to translate entity LayoutComponents into a `SolverInput`, then calls the solver. The current `CellLayout` stored on the FactoryScene is passed in as a warm start. The solver returns a new `CellLayout` which replaces the previous one on the FactoryScene. CellLayout is never serialised — it is always recomputed from entity state.

### RenderSystem

**Trigger:** every frame (or when CellLayout changes).

Reads the current CellLayout and entity VisualComponents. Builds or updates threepp scene objects to match the solved layout. Does not read LayoutComponents directly — it reads the solver's output.

### InteractionSystem

**Trigger:** user input events (mouse, touch, keyboard).

Raycasts against hit geometry in InteractiveComponents. On hit, invokes the entity's callbacks. Callbacks mutate LayoutComponents (e.g. updating `desired_position_mm` on a DeclaredOpening) and notify LayoutSolverSystem.

### SimulationSystem *(future)*

Steps the physics simulation and robot kinematics. Writes world transforms back to entity scene objects. Reads the solved CellLayout to know where the fence boundary is.

---

## Data Flow

```
Entity state (LayoutComponents)
    │
    ▼  SolverInputBuilder  (part of LayoutSolverSystem)
SolverInput + warm_start CellLayout?
    │
    ▼  Solver  (pure function, no EnTT dependency)
CellLayout  ─── stored on FactoryScene, never serialised
    │
    ▼  RenderSystem
threepp scene graph
    │
    ▼  user input
InteractionSystem
    │  mutates LayoutComponents
    └─────────────────────────────┐
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
2. **Rendering logic belongs in VisualComponent and RenderSystem.** The solver and LayoutComponent must remain rendering-independent.
3. **Interaction logic belongs in InteractiveComponent and InteractionSystem.** Selectability and drag behaviour are not properties of the data model.
4. **All cross-entity references use `entt::entity` handles.** Never raw integers.
5. **New entity types are defined by their component composition.** Do not add new special cases to existing systems — add a component.
6. **SimulationComponent is additive.** Adding simulation to an entity type does not change its Layout or Visual behaviour.
