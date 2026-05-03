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
| ConveyorBelt | User / workflow *(future)* | Pose, Transport, ConveyorBelt, FlowNode, Visual *(future)*, Interactive *(future)*, Simulation *(future)* |
| Station | User / workflow *(future)* | Pose, Station, *type-specific*, Visual *(future)* |
| PalletizingStation | User / workflow *(future)* | Pose, Station, PalletizingStation, Visual *(future)* |
| Mechanism | Solver | Pose, Mechanism, *type-specific*, Visual *(future)*, Simulation *(future)* |
| SpawnItem | User / workflow *(future)* | Pose, SpawnItem, FlowNode, Visual *(future)* |
| DespawnItem | User / workflow *(future)* | Pose, DespawnItem, FlowNode, Visual *(future)* |
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
  type                 OpeningType    — None (default) | Open | Solid
```

`OpeningType` controls how the VisualUpdateSystem renders the reserved space:

| Type | Rendering | Typical use |
|---|---|---|
| `None` | Nothing rendered in the gap | Walls, pillars, machine footprints |
| `Open` | Physical gap; filled with beams or half-panels *(future)* | Doors, belt pass-throughs, personnel access |
| `Solid` | Fence section rendered; something is mounted here | Control boxes, racks, cable trays attached to the wall |

`None` is the default and the only type currently implemented. `Open` and `Solid` are defined in the schema now so that future rendering work has a stable target.

The solver may group multiple belt pass-throughs into a single `Open` opening when the combined width plus clearances fits within a standard panel span (e.g. five 100 mm belts grouped into one 750 mm opening). When this happens, the solver writes a single `DeclaredOpeningComponent` with the combined width; the individual belt connections are tracked via their `FlowNodeComponent` references, not by separate openings.

Allocation state is determined entirely by which optional fields are set — no separate flag. See [LAYOUT_SOLVER.md](LAYOUT_SOLVER.md) for how the solver handles each allocation state.

### FlowNodeComponent

Carried by every entity that participates in item flow: `ConveyorBelt`, `SpawnItem`, `DespawnItem`, and any future station type (robot cells, buffers, etc.).

```
FlowNodeComponent
  entry  entt::entity   — upstream entity (null if this is a source)
  exit   entt::entity   — downstream entity (null if this is a sink)
```

This forms a singly-linked directed graph of item flow using plain `entt::entity` handles. No special "workflow node" base type is needed — any entity carrying `FlowNodeComponent` can appear anywhere in the graph.

Example — two belts in series:

```
SpawnItem  →  Belt1  →  Belt2  →  DespawnItem
  exit=Belt1   entry=SpawnItem   entry=Belt1    entry=Belt2
               exit=Belt2        exit=DespawnItem  exit=null
```

The workflow solver (future) will build and validate these graphs. For current demos, the graph is wired manually in code.

### StationComponent

Carried by every station entity alongside a station-type-specific component. Holds what all stations share.

```
StationComponent
  name:      string
  mechanism: entt::entity   — the actor performing work (null until solver places one)
```

The mechanism is a separate entity carrying its own specific component. What kind of mechanism it is — robot arm, diverter flap, pneumatic pusher — is determined entirely by which components that entity carries. The station does not discriminate.

### MechanismComponent

Marker component on mechanism entities. Every mechanism carries this alongside its type-specific component.

```
MechanismComponent
  (no fields — presence identifies the entity as a mechanism)
```

### RobotArmComponent *(future)*

```
RobotArmComponent
  model_ref:  string   — catalog reference for the robot model
  reach_mm:   int      — maximum reach radius
  pose:       (in PoseComponent, solver output — never user-placed)
```

Robot position is always **solver output**. The robot placement solver takes the set of required reach poses (pick and place positions derived from belt geometry and stack pattern) and finds the optimal base position within the robot's workspace. The objective function is configurable: minimise energy (minimise total joint travel), minimise cycle time, or minimise footprint. The solver must also verify that all required poses lie within the reachable annulus — not just the reach sphere, which has a dead zone below the base and joint-limit exclusions.

### PalletizingStationComponent

```
PalletizingStationComponent
  box_belt:           entt::entity   — belt delivering boxes; robot picks from end
  pallet_belt:        entt::entity   — belt that carries pallet in and out
  item_definition:    entt::entity   — the box type being stacked
  pallet_definition:  entt::entity   — the pallet type
  stack_layers:       int            — user-configurable; solver derives stack pattern
```

Staging position (where the pallet stops on the pallet belt during stacking) is **derived at runtime** from belt geometry — not stored in this component. For the first implementation: it is the intersection of the box belt's axis extended with the pallet belt's axis, projected onto the pallet belt segment. Parallel belts produce no intersection; that case is deferred.

### TransportComponent

Carried by every transport entity alongside a transport-type-specific component. Holds what all transports share.

```
TransportComponent
  speed_mm_s:    float   — current transport speed
  running:       bool    — true = moving, false = stopped/paused
  capacity:      int     — max items in transit simultaneously (0 = unlimited)
```

### ConveyorBeltComponent

```
ConveyorBeltComponent
  catalog_ref              string         — "generic/flat-belt" | "mk/guf-p-2000"
  width_mm                 int            — must be in catalog discrete list
  length_mm                int            — within [catalog.min_length_mm, catalog.max_length_mm]
  belt_surface_height_mm   int            — floor to belt top; leg height is derived
  opening_clearance_mm     int = 50       — added each side for fence opening width
  belt_speed_mm_s          float = 200    — for UV animation; 0 = stopped
  direction_a_to_b         bool = true    — travel direction relative to PoseComponent orientation
```

See [CONVEYOR_BELTS.md](CONVEYOR_BELTS.md) for catalog structure and procedural mesh generation.

### SpawnItemComponent *(future)*

Marks an entity as a source of items entering the scene. Carries item type and spawn rate. Always appears as `entry = null` in the flow graph.

### DespawnItemComponent *(future)*

Marks an entity as a sink that removes items from the scene. Always appears as `exit = null` in the flow graph.

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
