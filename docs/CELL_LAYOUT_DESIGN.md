# Robot Cell Layout System — Design Document

## Purpose

This document records the agreed architectural decisions for the robot cell layout generation system. It should be consulted before any implementation work on the cell layout, constraint solving, or blueprint import subsystems. Deviations from this design require explicit discussion and sign-off.

---

## Core Philosophy

**Features and constraints are the primary input. Cell dimensions are emergent.**

The user declares what exists: a machine here, a conveyor input there, a door somewhere on the east wall. The system finds the smallest valid enclosure that satisfies all constraints, optimised by an objective function. Target dimensions ("approximately 4000mm wide") are soft inputs, not hard requirements. Panel snapping takes priority over target dimensions.

---

## Naming

The elements that occupy reserved space on an edge are called **Openings** (`Opening`). This replaces the earlier term "wall component," which implied subordination to a wall.

An `Opening` is any contiguous section of an edge where no fence panels are built. Its subclass determines where it came from and how it behaves. See the Opening section below.

---

## Data Model

### Topology

```
CellLayout
├── world_features[]   — world objects with positions/footprints (machine, pillar, solid wall)
├── span_constraints[] — world-space constraints that modify span solving (height limits, etc.)
└── edges[]            — one per side of the cell polygon (Edge)
      ├── nodeA, nodeB         — stable handles, not integer indices
      ├── user_openings[]      — DeclaredOpenings only; WorldFeatureOpenings derived at solve time
      ├── catalog_ref          — which fence catalog this edge uses
      └── solve_cache          — derived result, invalidated on any mutation
```

**The cell is a closed polygon.** Edges connect nodes in a loop. This is a first-class constraint, not an afterthought.

**Node**: a point in scene space (world X, Z) with a `NodeType`. Nodes exist only at corners — places where the wall direction changes. Adding a door or machine cutout to an edge does NOT add a node. Node type is never a user input; it is selected by the solver after edge angles are known (see Corner Resolution).

**Edge**: one full side of the cell polygon. Connects two nodes. References a fence catalog. The edge is the primary unit of user interaction — selecting any span or opening on an edge selects the whole side. The edge stores only user-placed openings; all other elements are derived at solve time.

**FenceSpan**: a section of the edge where fence panels are built. Fully derived — never stored in the primary data model. Spans are the gaps between openings after world feature projections and span constraint splits are applied. Solved panel combinations are cached in the edge's solve cache.

**Opening**: reserved space on an edge where no fence is built. A polymorphic base class — subclass determines origin, width computation, selectability, and appearance. See below.

**SpanConstraint**: a world-space object that modifies how spans are solved without removing fence. Does not create an opening. Projects onto edges at solve time, forcing span splits and restricting catalog candidates (e.g. a low ceiling limits available panel heights). Multiple span constraints can overlap; the most restrictive applies.

### Opening — Polymorphic Interface

`Opening` is a **base class with virtual methods**, not a tagged union. The interface grows meaningfully per subclass (width computation, selectability, collision box, asset reference) and virtual dispatch is the appropriate mechanism.

```
Opening (base)
├── getWidth() const        — width of reserved space in mm
├── getPosition() const     — position from edge start in mm (actual/solved position)
├── mobility() const        — returns 0.0f by default (immovable); see Mobility below
├── collisionBox() const    — returns empty box by default (not selectable)
├── isEditable() const      — returns false by default
└── assetKey() const        — reference to visualization asset (3D model, etc.)
```

Three concrete subclasses:

**`DeclaredOpening`** — placed by the user on a specific edge. Stored on the edge.
- `desired_position_mm`: where the user placed it (persistent intent, separate from solved position)
- `width_mm`: user-defined
- `mobility`: user-defined (see Mobility below)
- `collisionBox()`: returns the opening's footprint — selectable and draggable
- `isEditable()`: true

**`WorldFeatureOpening`** — auto-generated when an edge intersects a world feature footprint. Not stored on the edge; derived at solve time.
- `getWidth()`: queries the referenced world feature for the projected intersection length
- `collisionBox()`: returns empty — not independently selectable (the world feature is)
- `mobility()`: returns 0 — position is driven by the world feature, never by the solver

**`WorldFeatureOpening`** covers all world-driven openings: machines, pillars, conveyors, and solid walls. `SolidWall` is a world feature type — its opening behaves identically to any other `WorldFeatureOpening` (not selectable, mobility 0, width from projected intersection). There is no separate `SolidWallOpening` class; the world feature's type determines any downstream behaviour.

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

3D geometry and animations are **not stored in the data model**. `Opening` and `WorldFeature` carry an `assetKey` — a catalog reference or path — that the visualization layer resolves to geometry. This is consistent with the existing pattern: `fence_catalog.hpp` reads JSON, `cell_builder.hpp` builds scene objects. The data model is serializable and rendering-independent.

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

### Node References

Nodes are referenced by **stable handles**, not integer indices into a vector. Raw integer indices are explicitly rejected:
- Inserting or removing nodes invalidates all indices silently
- No type safety (`int` is `int`)
- No reverse navigation (node → its connected edges) without a full scan

The specific mechanism (typed ID, stable-pointer storage, etc.) is an implementation decision, but the interface must not expose raw integers.

---

## Solver Architecture (Five Layers)

### Layer 0 — Blueprint Import *(future)*

Reads a factory floor plan (DXF, IFC, image) and produces:
- World features and span constraints from geometry (walls, pillars, machines, height restrictions)
- **Search space boundary**: a `FloorPlan` polygon that defines where the cell can legally exist

No coupling to the solver. Produces the same objects a user would declare manually.

### Layer 1 — Topology Definition

The user (or Layer 0) declares:
- World features and span constraints (positions, footprints, properties)
- Edge topology (closed polygon)
- `DeclaredOpenings` on specific edges
- Catalog reference per edge

World feature and solid wall openings are derived automatically. The user does not place them.

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

The edge is the primary selection unit. Clicking any span or opening on an edge selects the whole side. Clicking again narrows to the specific element.

- **Gizmos on an edge**: extend or contract the whole side. Adjacent edges adjust. World feature openings remain fixed; user-placed openings redistribute by mobility.
- **Dragging a `DeclaredOpening`**: updates `desired_position_mm`, triggers re-solve. Neighbouring soft openings absorb displacement proportional to their mobility.
- **`WorldFeature` openings**: not draggable. Follow the world feature. Disappear if the edge moves away.
- **Collision boxes**: each opening exposes `collisionBox()`. Empty box = not independently selectable. The selection system needs no special-casing.

Alignment is implicit: all elements on the same edge are on the same side by definition.

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
