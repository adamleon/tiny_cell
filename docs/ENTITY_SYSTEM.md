# Entity System

## Overview

The scene is managed as an Entity-Component-System using **EnTT**. The `FactoryScene` wraps an EnTT registry and is the **single source of truth** for all scene state. Every visible or interactive element in the scene is an entity.

This document covers the entity types, components, systems, and the rules for extending the system. For how the layout solver uses entity data, see [LAYOUT_SOLVER.md](LAYOUT_SOLVER.md). For the simulation architecture (transports, ports, sensors, station dispatch), see [TRANSPORT_MODEL.md](TRANSPORT_MODEL.md) — that document is the source of truth for everything transport- and station-related; this document gives the catalogue summary.

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
| Robot | User / blueprint import | Pose, Robot *(future)*, Visual *(future)*, Interactive *(future)* |
| ConveyorBelt | User / workflow *(future)* | Pose, Transport, ConveyorBelt, Visual *(future)*, Interactive *(future)* |
| Port | User / workflow *(future)* | Pose, Port |
| Sensor | User / workflow *(future)* | Pose, Sensor, LaserSensor *(physical sensors only)* |
| Picker | User / workflow *(future)* | Pose, Transport, PickerTransport, Visual *(future)* |
| Station | User / workflow *(future)* | Pose, Station, *type-specific* (Palletize, …), Visual *(future)* |
| Source | User / workflow *(future)* | Pose, Source, Visual *(future)* |
| Sink | User / workflow *(future)* | Pose, Sink, Visual *(future)* |
| ItemPrototype | User / workflow *(future)* | ItemPrototype |
| SpawnedItem | LifecycleSystem | Pose, SpawnedItem, ItemOnTransport (when riding a transport), Visual *(future)* |
| Pallet | LifecycleSystem (as a SpawnedItem variant) | Pose, SpawnedItem, Pallet, ItemOnTransport (when riding), Visual *(future)* |

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

The solver may group multiple belt pass-throughs into a single `Open` opening when the combined width plus clearances fits within a standard panel span (e.g. five 100 mm belts grouped into one 750 mm opening). When this happens, the solver writes a single `DeclaredOpeningComponent` with the combined width; the individual belt connections are tracked via each belt's own `entry_port` / `exit_port` references, not by separate openings.

Allocation state is determined entirely by which optional fields are set — no separate flag. See [LAYOUT_SOLVER.md](LAYOUT_SOLVER.md) for how the solver handles each allocation state.

### PortComponent

Carried by every entity that exposes a connection point along or between transports. Hosts a list of sensors (sensor entities) that gate flow at this port.

```
PortComponent
  name        string
  transport   entt::entity         — destination transport for items handed off here; null at sink-side or station-arrival points
  sensors     vector<entt::entity> — sensor entities; the port is "clear" iff every sensor reads not-blocked
```

Transport flow connectivity is expressed via PortComponents on transports: a belt's `entry_port` and `exit_port` reference Port entities, a port's `transport` field references the next transport in the chain, sources reference an `out_port`, and sinks reference an `in_port`. No separate "FlowNode" component is needed.

The same sensor entity may appear in multiple ports' lists — this is how a single virtual sensor (e.g. "station busy") can gate several belts at once. See [TRANSPORT_MODEL.md](TRANSPORT_MODEL.md) for the full sensor and gating model.

### StationComponent

Carried by every station entity alongside a station-type-specific component (PalletizeComponent for palletizers, etc.). The station is a **dispatcher**: it reads sensors, runs assignment logic, and writes virtual sensors. It does **not** own items.

```
StationComponent
  arrival_port              entt::entity         — where items arrive for processing
  arrival_virtual_sensor    entt::entity         — virtual sensor on arrival_port that the station drives
  pickers                   vector<entt::entity> — pool of pickers this station dispatches to
```

All in-flight work lives on the picker(s). Multi-picker stations are a matter of populating `pickers` with more than one entity. See [TRANSPORT_MODEL.md](TRANSPORT_MODEL.md) for the dispatch model.

### PickerTransportComponent

Carried by picker entities — autonomous transports that grasp and carry items between named locations. A picker is a self-driving state machine; once a station populates its job fields and sets `state = MovingToBox`, the picker handles its own transitions.

```
PickerTransportComponent
  home_pose          Vec3
  pickup_target      Vec3
  drop_target        Vec3
  drop_container     entt::entity   — entity to parent the box to on placement; null = leave at world pose
  drop_orientation   Quat
  speed_mm_s         float
  state              PickerState    — Idle | MovingToBox | Carrying | Returning
  current_box        entt::entity   — null when Idle
```

### RobotArmComponent *(future)*

```
RobotArmComponent
  model_ref:  string   — catalog reference for the robot model
  reach_mm:   int      — maximum reach radius
  pose:       (in PoseComponent, solver output — never user-placed)
```

Robot position is always **solver output**. The robot placement solver takes the set of required reach poses (pick and place positions derived from belt geometry and stack pattern) and finds the optimal base position within the robot's workspace. The objective function is configurable: minimise energy (minimise total joint travel), minimise cycle time, or minimise footprint. The solver must also verify that all required poses lie within the reachable annulus — not just the reach sphere, which has a dead zone below the base and joint-limit exclusions.

When a robot arm is added, it will be a **transport variant** — likely composed as `Pose + Transport + RobotArm + PickerTransport` so the dispatch logic in `StationSystem` does not need to discriminate between a generic picker and a robot-arm picker.

### PalletizeComponent

Type-specific addition to a station for palletizing. Combined with `StationComponent` on the same entity.

```
PalletizeComponent
  pallet_arrival_port           entt::entity
  pallet_tap_virtual_sensor     entt::entity      — gates the pallet belt while the station has a pallet to fill
  current_pallet                entt::entity      — null until a pallet is at the arrival port
  pattern                       PlacementPattern* — slot allocator
  pallet_length_mm              int
  pallet_width_mm               int
  pallet_height_mm              int
  pallet_max_stack_mm           int
```

The station has a pallet iff `current_pallet != null` — no separate state enum is needed. The staging position (where the pallet stops on the pallet belt) is the world pose of `pallet_arrival_port`, which is parented to the pallet belt — moving the belt moves the staging position automatically. See [TRANSPORT_MODEL.md](TRANSPORT_MODEL.md) for dispatch and pallet release.

### TransportComponent

Carried by every transport entity (belts, pickers, future robots/AGVs) alongside a transport-flavour-specific component.

```
TransportComponent
  running   bool   — controller intent; default true
```

`running == true` is necessary but not sufficient for items to move — the actual motion gate is per-flavour: belts move iff `running && every gate port is clear`; pickers move under their own state-machine control. Capacity-as-item-count has been replaced by sensor-based gating; there is no `capacity` field. See [TRANSPORT_MODEL.md](TRANSPORT_MODEL.md).

### ConveyorBeltComponent

```
ConveyorBeltComponent
  catalog_ref              string                 — "generic/flat-belt" | "mk/guf-p-2000"
  width_mm                 int                    — must be in catalog discrete list
  length_mm                int                    — within [catalog.min_length_mm, catalog.max_length_mm]
  surface_height_mm        int                    — floor to belt top; leg height is derived
  opening_clearance_mm     int = 50               — added each side for fence opening width
  belt_speed_mm_s          float = 200            — items advance at this rate when the belt is moving
  dir                      Vec3                   — unit vector in scene-root frame
  entry_port               entt::entity
  exit_port                entt::entity
  tap_ports                vector<entt::entity>   — ports along the belt's length; t derived from each port's pose
  gate_ports               vector<entt::entity>   — subset whose blocked sensors freeze the belt; defaults to {exit_port}
```

A belt is moving iff `running_() && every gate port is clear`. Tap-port positions are derived from each port's `PoseComponent` (parent-chained to the belt) — not stored explicitly. See [CONVEYOR_BELTS.md](CONVEYOR_BELTS.md) for catalog and mesh details, and [TRANSPORT_MODEL.md](TRANSPORT_MODEL.md) for the motion and gating model.

### SourceComponent

Marks an entity as a source of items entering the scene. Each tick, accumulates spawn debt at `rate_per_hour`; spawns when debt ≥ 1 and `out_port` is clear.

```
SourceComponent
  rate_per_hour   float
  prototype       entt::entity   — ItemPrototype to instantiate
  out_port        entt::entity   — port where new items are placed
  spawn_debt      float          — accumulated fractional spawns
```

### SinkComponent

Marks an entity as a sink that removes items arriving at its `in_port`.

```
SinkComponent
  in_port    entt::entity
  received   int            — running count, monotonic
```

### SensorComponent

A detection device. Returns a `blocked: bool` that gates ports.

```
SensorComponent
  blocked   bool   — current reading; true = "do not flow into here"
```

Physical sensors (those that also carry `LaserSensorComponent`) are auto-updated by `SensorScanSystem` — they trigger when an item's bounding box encloses the sensor's world position. Virtual sensors (no `LaserSensorComponent`) have `blocked` written by orchestration code (typically `StationSystem`).

### LaserSensorComponent

Empty marker. Its presence on a sensor entity makes it a **laser**: a real point in 3D at the sensor's world position. The scan system tests every spawned item's bounding box against this point and sets `blocked` accordingly. The sensor itself carries no extents — the item's collision box (from `ItemPrototypeComponent`) decides whether it counts as occupying the laser.

```
LaserSensorComponent
  (no fields — marker)
```

The sensor's `PoseComponent` is parented to the port it gates, so the world position tracks the port's parent-chain transform automatically.

### ItemOnTransportComponent

Records that an item is currently riding a specific transport. The transport's tick code is responsible for advancing the item's pose.

```
ItemOnTransportComponent
  transport   entt::entity   — belt or picker
```

The core rule: an item either has `ItemOnTransportComponent` **or** is parented to a container item that has it (transitively). Items at rest in the scene live inside containers — they are never floating in world space without a parent.

### ItemPrototypeComponent

Defines a class of items. Source entities reference a prototype to know what to spawn.

```
ItemPrototypeComponent
  length_mm   int
  width_mm    int
  height_mm   int
  color_hex   uint32
```

### SpawnedItemComponent

Marks an entity as an instance of a prototype. Carries a reference back to its prototype.

```
SpawnedItemComponent
  prototype   entt::entity
```

### PalletComponent

Marks a `SpawnedItem` as a pallet — a container that may hold other items via parent-chain.

```
PalletComponent
  length_mm            int
  width_mm             int
  height_mm            int
  max_stack_height_mm  int
  items                vector<entt::entity>   — children placed on this pallet
```

A child item placed on a pallet has its `PoseComponent.parent` set to the pallet entity; `world_transform()` traverses the chain. The child has no `ItemOnTransportComponent` while at rest in the pallet — the pallet itself is the thing on a transport.

### VisualComponent *(future)*

Owns the threepp scene object for this entity. The VisualUpdateSystem writes the asset key and visibility state here after each solve; the RenderSystem reads it every frame.

Holds the threepp `Object3D`, an asset key for catalog-driven geometry, and visibility/highlight state. 3D geometry and animations live here — never in the data model or the solver.

### InteractiveComponent *(future)*

Owns hit geometry, gizmo configuration, and event callbacks. An entity without an `InteractiveComponent` cannot be selected or dragged. Callbacks mutate component data (NodeComponent, EdgeComponent, DeclaredOpeningComponent, etc.) and notify LayoutSolverSystem. They do not touch the solver directly.

### Simulation state

There is no monolithic `SimulationComponent`. Simulation state is partitioned across the transport-flavour components: `TransportComponent::running_` is controller intent; `ConveyorBeltComponent::belt_speed_mm_s` and `dir` drive belt motion; `PickerTransportComponent::state` and the target fields drive picker motion; `SensorComponent::blocked` carries gate state. See [TRANSPORT_MODEL.md](TRANSPORT_MODEL.md).

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

### SensorScanSystem *(future)*

**Trigger:** each simulation tick, before TransportSystem.

For every entity carrying `SensorComponent + LaserSensorComponent + PoseComponent`, project `(sensor_pos − item_centre)` onto each item-local axis (from `ItemPrototypeComponent` half-extents) and set `blocked = true` if all three projections are within range — i.e. the laser point sits inside the item's oriented bounding box. Virtual sensors (no `LaserSensorComponent`) are skipped.

### TransportSystem *(future)*

**Trigger:** each simulation tick, after SensorScanSystem.

Owns all transport-flavour motion in one place. **Belts** advance items by `dir × belt_speed_mm_s × dt` iff `running_() && every gate port is clear`; detect tap-port crossings by comparing `prev_t` and `new_t`; hand items off at `exit_port` via `ItemOnTransportComponent::transport` reassignment when the destination port is clear. **Pickers** step toward the current target at `speed_mm_s`, transition state on arrival, and perform the matching transport/parent reassignment side-effects (see [TRANSPORT_MODEL.md](TRANSPORT_MODEL.md)).

This is intentionally one system covering both belts and pickers — they are different motion math but the same conceptual role (transports moving items). The only sim work it does NOT cover is item creation / destruction.

### StationSystem *(future)*

**Trigger:** each simulation tick, after TransportSystem.

Pure dispatch. For each station: claim a pallet if one is at the pallet arrival port and no current pallet is held; assign idle pickers to boxes waiting at the arrival port; maintain virtual sensor readings (`arrival_virtual_sensor` and any tap-port virtual sensors) from station state; release a full pallet by clearing the pallet-tap virtual sensor.

### LifecycleSystem *(future)*

**Trigger:** each simulation tick, after StationSystem.

The only system that creates or destroys item entities. Sources accumulate debt and spawn when `port_is_clear(out_port)`; sinks consume items arriving at their `in_port`. Everything else is motion or orchestration.

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
