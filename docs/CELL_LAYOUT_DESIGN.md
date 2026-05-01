# Robot Cell Layout System — Design Document

## Purpose

This document records the agreed architectural decisions for the robot cell layout generation system. It should be consulted before any implementation work on the cell layout, constraint solving, or blueprint import subsystems. Deviations from this design require explicit discussion and sign-off.

---

## Core Philosophy

**Features and constraints are the primary input. Cell dimensions are emergent.**

The user declares what exists: a machine here, a conveyor input there, a door somewhere on the east wall. The system finds the smallest valid enclosure that satisfies all constraints, optimised by an objective function. Target dimensions ("approximately 4000mm wide") are soft inputs, not hard requirements. Panel snapping takes priority over target dimensions.

---

## Entity/Component Architecture

The scene is managed as an Entity-Component-System using **EnTT**. The `FactoryScene` wraps an EnTT registry and is the single source of truth for all scene state.

### Entities

Every visible or interactive element in the scene is an entity, referenced by a stable `entt::entity` handle. Entities are never referred to by integer indices.

| Entity | Created by | Components |
|---|---|---|
| Node | User / blueprint import | Visual, Interactive, Layout |
| Edge | System (when two nodes connect) | Visual, Interactive, Layout |
| DeclaredOpening | User (within an edge) | Visual, Interactive, Layout |
| WorldFeature | User / blueprint import | Visual, Interactive |
| Robot, Belt, WorkPiece, … | User / simulation | Visual, Interactive, Simulation (future) |

Edge entities are *derived* — the system creates and destroys them as node topology changes — but while they exist they are full scene participants with rendering and interaction. WorldFeature entities have no LayoutComponent: they influence the layout through their footprint but are not fence system participants.

### Components

**VisualComponent** — owns the threepp scene object for this entity. The render system iterates all VisualComponents each frame.

**InteractiveComponent** — owns the hit geometry, gizmo state, and callbacks for click, hover, and drag. The interaction system processes these against user input. An entity without an InteractiveComponent cannot be selected.

**LayoutComponent** — carries the entity's role in the fence layout and the constraint data the solver needs:

- *Node*: world position (X, Z). NodeType is solver output, not stored here.
- *Edge*: references to its two node entities; catalog ref.
- *DeclaredOpening*: reference to parent edge entity; `desired_position_mm`; `width_mm`; `mobility`.
- *WorldFeature*: world position and footprint geometry.

**SimulationComponent** *(future)* — kinematics state for robots, motion state for conveyors, etc.

### Systems

**LayoutSolverSystem** — triggered when any LayoutComponent mutates. Reads all layout-related entities, runs the five-layer solver, and writes the result into a `CellLayout` value stored on the FactoryScene.

**RenderSystem** — runs each frame. Reads the current `CellLayout` and updates VisualComponents accordingly.

**InteractionSystem** — processes input events. Raycasts against InteractiveComponents, invokes callbacks, mutates LayoutComponents, and triggers LayoutSolverSystem.

### Data flow

```
Entity state (LayoutComponents)
    ↓  LayoutSolverSystem
CellLayout  ←  solver output; never serialised
    ↓  RenderSystem
threepp scene graph
    ↓  user input → InteractionSystem → entity mutation
Entity state (updated)
    ↓  LayoutSolverSystem (triggered again)
    …
```

`CellLayout` is the solver's output, not the primary state. Primary state lives in entity LayoutComponents. `CellLayout` is always recomputed on load.

---

## Naming

The elements that occupy reserved space on an edge are called **Openings** (`Opening`). This replaces the earlier term "wall component," which implied subordination to a wall.

An `Opening` is any contiguous section of an edge where no fence panels are built. Its subclass determines where it came from and how it behaves. See the Opening section below.

---

## Data Model

`CellLayout` is the **output of the solver**, not the primary state. Primary state lives in entity LayoutComponents (see Entity/Component Architecture). `CellLayout` is what the render system reads; it is always recomputed from entity state and never serialised.

### Topology

```
FactoryScene (EnTT registry — primary state)
├── Node entities        — world position (X, Z); NodeType is solver output
├── Edge entities        — refs to nodeA, nodeB; catalog ref
├── DeclaredOpening entities — ref to parent edge; desired_position_mm; width_mm; mobility
├── WorldFeature entities    — world position, footprint; no layout role
└── SpanConstraint entities  — world-space constraint (e.g. height limit); no layout role

CellLayout (solver output — derived, never serialised)
└── edges[]
      ├── nodeA, nodeB   — entt::entity handles
      ├── solve_cache    — merged span/opening sequence, actual positions, panel combinations
      └── span_constraints — annotations per sub-span
```

**The cell is a closed polygon.** Edges connect nodes in a loop. This is a first-class constraint, not an afterthought.

**Node**: a point in scene space (world X, Z). Nodes exist only at corners. NodeType is never stored on the node entity — it is selected by the solver after edge angles are known (see Corner Resolution).

**Edge**: one full side of the cell polygon. Connects two node entities. References a fence catalog. The edge entity is the primary unit of user interaction — clicking any span or opening on an edge selects the whole side. DeclaredOpening entities reference their parent edge; all other openings are derived at solve time.

**FenceSpan**: fully derived — never stored. Spans are the gaps between openings after world feature projections and span constraint splits are applied. Solved panel combinations live in the edge's solve cache.

**Opening**: reserved space on an edge where no fence is built. A polymorphic base class used internally by the solver — subclass determines origin and width. See below.

**SpanConstraint**: a world-space entity that modifies how spans are solved without removing fence. Does not create an opening. Projects onto edges at solve time, forcing span splits and restricting catalog candidates (e.g. a low ceiling limits available panel heights). Multiple span constraints can overlap; the most restrictive applies.

### Opening — Polymorphic Interface

`Opening` is a **solver-internal base class**, not a stored data structure. The solver instantiates openings transiently when computing the edge element sequence. Selectability, hit geometry, and visual assets are handled by ECS components — not by the Opening class.

```
Opening (base)
├── getWidth()    const — width of reserved space in mm
├── getPosition() const — actual solved position from edge start in mm
└── mobility()    const — returns 0.0f by default (immovable fail-safe)
```

Two concrete subclasses:

**`DeclaredOpening`** — constructed by the solver from a DeclaredOpening entity's LayoutComponent.
- `desired_position_mm`: read from the entity (persistent user intent)
- `width_mm`: read from the entity
- `mobility()`: read from the entity

**`WorldFeatureOpening`** — constructed by the solver when an edge intersects a world feature footprint. Not stored anywhere; derived at solve time. Covers machines, pillars, conveyors, solid walls.
- `getWidth()`: queries the referenced world feature entity for the projected intersection length
- `mobility()`: returns 0 — position is driven by the world feature, never by the solver

### Mobility

`mobility` controls how much a `DeclaredOpening` resists displacement when the solver must redistribute available space.

| Value | Behaviour |
|-------|-----------|
| `0.0` | Immovable — hard constraint. Position is always `desired_position_mm`. |
| `1.0` | Standard soft constraint. |
| `> 1.0` | More easily displaced relative to lower-mobility neighbours. |

**Default is `0.0` (immovable).** This is a deliberate fail-safe: if an opening is incorrectly deserialised with a missing or zero mobility value, it stays fixed rather than drifting. An immovable conveyor opening is a safe failure; a freely floating one is not.

`WorldFeatureOpening` always returns `0.0` from the base class default — its position is driven by the world feature, not the solver.

**Displacement distribution**: when the solver must shift openings to satisfy panel snapping or closure constraints, it distributes the required displacement among soft openings in a region proportional to their mobility. Equal-mobility openings move equally; a high-mobility door adjacent to a low-mobility conveyor absorbs most of the displacement.

**Desired vs. actual position**: `desired_position_mm` on `DeclaredOpening` stores the user's intent and persists across solves. The solver attempts to honour it. When constraints force displacement, the actual (solved) position is stored in the edge's solve cache — not written back to `desired_position_mm`. When constraints relax, the solver can restore openings closer to their desired positions.

### 3D Models and Animations

3D geometry and animations live in **VisualComponent**, not in the data model or the solver. The VisualComponent holds an asset key that the render system resolves to threepp geometry. The solver and LayoutComponent are rendering-independent and serialisable.

### Solve Cache

The edge's solve cache stores all derived results: the merged ordered sequence of spans and openings (with actual positions), solved panel combinations per span, and any span constraint annotations. It is invalidated whenever the edge's user openings change, a world feature moves, or a span constraint changes. It is never serialized — always recomputed on load.

### World Features

World features exist independently of edges. They have a world position and a footprint. When an edge line passes through a world feature's footprint, a `WorldFeatureOpening` is automatically generated in the solve cache. When the edge moves away from the footprint, the opening disappears.

| Type | Examples | NodeType implication |
|------|---------|----------------------|
| Machine | Robot, CNC, press | Adjacent nodes may become `None` |
| Pillar | Column, post | Node at pillar position |
| SolidWall | Concrete wall, building wall | Edge becomes `[WorldFeatureOpening]` |
| ConveyorIO | Fixed conveyor entry/exit | Hard-positioned opening of known width |

### Span Constraints

Span constraints are world-space objects that affect how spans are solved without removing them. They do not create openings. They project onto any edge they intersect and force span splits at their boundaries, so each resulting sub-span can be solved with the appropriate catalog restrictions.

Example — low ceiling:
```
world: HeightConstraint(position, footprint, max_height=750mm)

derived spans for south edge after split:
  [0 → 3000mm]      catalog: full
  [3000 → 3800mm]   catalog: filtered to panels ≤ 750mm high
  [3800 → end]      catalog: full
```

Multiple span constraints can overlap; the most restrictive constraint wins per sub-span.

### Corner: NodeType::None

When world feature openings consume the end of one edge and the start of the adjacent edge, the shared node becomes `NodeType::None` — no post, no hardware. The machine or wall is the effective corner. Not tracked explicitly; emerges from the opening geometry.

Example — machine in a corner:
```
south edge:  [FenceSpan, WorldFeatureOpening(machine)]
SE node:     NodeType::None
east edge:   [WorldFeatureOpening(machine), FenceSpan]
```

Example — entire edge inside a solid wall:
```
edge:  [WorldFeatureOpening(solid_wall)]
```

### NodeType (priority-ordered)

When the solver determines the angle and occupancy at a node, it selects the best available type:

1. **None** — fully consumed by adjacent world feature openings. No hardware.
2. **Straight** — edges collinear (≈180°). Post inline, no corner piece.
3. **RoundedCorner** — a rounded corner piece fits the angle.
4. **Post** — standard 90° post.
5. **AngledPost** — two posts at a non-standard angle.
6. **Gap** — small physical gap between wall ends. Last resort; better than a forced bad angle.

**A 50mm gap between a machine and a wall is preferable to a forced awkward corner angle.**

Node type is never a user input for a generated cell. It is always a solver output.

### Entity References

All cross-entity references use **`entt::entity` handles** from the EnTT registry. These are stable across insertion and deletion (generation counters detect stale handles). Raw integer indices are explicitly rejected — they silently invalidate on insertion or removal, carry no type safety, and require a full scan for reverse navigation.

---

## Solver Architecture (Five Layers)

### Layer 0 — Blueprint Import *(future)*

Reads a factory floor plan (DXF, IFC, image) and produces:
- World features and span constraints from geometry (walls, pillars, machines, height restrictions)
- **Search space boundary**: a `FloorPlan` polygon that defines where the cell can legally exist

No coupling to the solver. Produces the same objects a user would declare manually.

### Layer 1 — Topology Definition

The solver reads from entity LayoutComponents:
- WorldFeature and SpanConstraint entities (positions, footprints, properties)
- Node and Edge entities forming the closed polygon
- DeclaredOpening entities on specific edges
- Catalog reference from each Edge entity's LayoutComponent

WorldFeatureOpenings are derived automatically. The user does not place them.

### Layer 2 — Per-Span Candidate Generation

For each derived span:
1. Project world features → `WorldFeatureOpenings`
2. Project span constraints → split spans at constraint boundaries
3. Distribute `DeclaredOpenings` using mobility weights
4. For each remaining span, compute desired length and return the **top K valid panel combinations** ranked by the active objective function

This replaces the `prefer_over` / `prefer_under` flag with a proper scored candidate set.

If a world feature opening leaves insufficient room for even the smallest valid span, that side's span length is zero. The node may become `None` if both adjacent edges have zero-length spans at that corner.

### Layer 3 — Global Closure Solving

Select one combination per span such that the polygon closes exactly (sum of edge vectors = 0). Search over the discrete candidate space. For small cells: exhaustive. For larger cells: constraint propagation or branch-and-bound.

The global objective function is applied here: among all closure-consistent solutions, pick the best-scoring one.

### Layer 4 — Corner Resolution

Given solved edge lengths and opening occupancy at each node, walk the `NodeType` priority list and select the best type. If the result is undesirable (AngledPost or Gap), optionally re-enter Layer 3 with a nudge constraint on adjacent spans.

---

## User Interaction Model

Interaction is handled entirely through **InteractiveComponent**. Entities without an InteractiveComponent cannot be selected. The interaction system raycasts against hit geometry owned by InteractiveComponents — no special-casing per entity type.

The edge is the primary selection unit. Clicking any span or opening on an edge selects the whole edge entity. Clicking again narrows to the specific child entity (e.g. a DeclaredOpening entity).

- **Gizmos on an edge entity**: extend or contract the whole side. Adjacent edges adjust. WorldFeatureOpening positions are fixed; DeclaredOpening entities redistribute by mobility.
- **Dragging a DeclaredOpening entity**: the InteractiveComponent updates `desired_position_mm` in the entity's LayoutComponent, triggering LayoutSolverSystem. Neighbouring soft openings absorb displacement proportional to their mobility.
- **WorldFeature entities**: dragging moves the world feature. The LayoutSolverSystem re-derives WorldFeatureOpenings on affected edges.
- **SpanConstraint entities**: no direct drag; properties (e.g. max height) edited via inspector.

Alignment is implicit: all entities on the same edge are on the same side by definition.

---

## Objective Functions

The solver is parameterised by an objective function. Default: **minimum cost**.

- **Cost** (default): minimise total panel + hardware cost
- **Ergonomic**: minimise awkward angles, maximise door accessibility
- **Volume**: minimise enclosed floor area
- **Panel count**: minimise number of distinct panel widths
- **Custom**: user-defined scoring function

Objective functions score a complete candidate solution. They must be composable (e.g. 70% cost + 30% ergonomic).

---

## Catalogs

Fence catalogs are **user-defined and edge-scoped**. Different edges can use different catalogs. The system does not hard-code any catalog structure beyond the JSON schema.

The current Axelent X-Guard catalog is an example instance, not a template.

---

## Blueprint Import (Layer 0, Extended)

A separate module with no coupling to the solver. Responsibilities:

**World feature and span constraint extraction**: parse geometry and classify into world feature and span constraint objects with world positions, footprints, and properties (e.g. ceiling height).

**Search space**: produce a `FloorPlan` polygon feeding a pre-solve **placement step**. The configuration solver (Layers 1–4) operates within that placed boundary and never knows about the factory floor.

Placement (where does the cell go?) and configuration (how is the fence arranged?) are **distinct problems** and must remain separate.

---

## What This Design Does Not Cover Yet

- Placement solver (deciding *where* to put the cell on the factory floor)
- Multi-cell layouts
- Vertical features beyond height constraints
- Dynamic real-time re-solving during interactive drag
- Specific blueprint file format parsers
- Diagonal / corner-spanning openings (door dragged to a corner)

These are explicitly deferred. The architecture must not be coupled to any of them prematurely.

---

## Extension Rules

Before adding any new capability, verify:

1. Does it belong in an existing layer, or does it need a new one?
2. Does it respect the features-first / dimensions-emergent philosophy?
3. Does it preserve the independence of Layer 0 from Layers 1–4?
4. Does it preserve the objective function interface (any function, composable)?
5. Does `NodeType` selection remain a solver output, never a user input?
6. Do new opening types subclass `Opening` rather than introducing parallel element lists?
7. Are world features and span constraints kept independent of edges (derived at solve time, not stored on edges)?
8. Does new data stored on `DeclaredOpening` distinguish user intent (desired) from solver output (actual)?
9. Does any new mobility-like property default to `0.0` (immovable) as the fail-safe?
10. Is every new visible or interactive element an entity in the EnTT registry?
11. Is new rendering logic in VisualComponent / RenderSystem, not in the solver or data model?
12. Is new interaction logic in InteractiveComponent / InteractionSystem, not in the solver or data model?
13. Does any new cross-entity reference use `entt::entity` handles, never raw integer indices?
