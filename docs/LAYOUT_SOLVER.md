# Layout Solver

## Core Philosophy

**Features and constraints are the primary input. Cell dimensions are emergent.**

The user declares what exists: a machine here, a conveyor input there, a door somewhere on the east wall. The solver finds the smallest valid enclosure that satisfies all constraints, optimised by an objective function. Target dimensions ("approximately 4000mm wide") are soft inputs, not hard requirements. Panel snapping takes priority over target dimensions.

The solver reads entity LayoutComponents as its input (see [ENTITY_SYSTEM.md](ENTITY_SYSTEM.md)) and writes a `CellLayout` as its output. It has no knowledge of rendering, interaction, or scene state beyond what LayoutComponents provide.

---

## Naming

Reserved space on an edge — any contiguous section where no fence panels are built — is called an **Opening**. This name replaces "wall component" (which implied subordination to a wall) and "hole" (which implies an absence rather than a first-class element).

An `Opening` is a solver-internal base class. The solver instantiates openings transiently when computing the edge element sequence. They are not stored in entity state or in the solved output.

---

## Opening Interface

`Opening` is a **base class with virtual methods**, not a tagged union. The interface covers only what the solver needs:

```
Opening (base)
├── getWidth()    const — width of reserved space in mm
├── getPosition() const — actual solved position from edge start in mm
└── mobility()    const — returns 0.0f by default (immovable fail-safe)
```

Selectability, hit geometry, and visual assets are handled by ECS components — not by the Opening class.

### DeclaredOpening

Constructed by the solver from a DeclaredOpening entity's LayoutComponent. Represents space the user has intentionally reserved on an edge: a door, a conveyor pass-through, a slab panel, or any other user-defined gap.

- `desired_position_mm` — read from the entity; stores the user's intent; persists across solves
- `width_mm` — read from the entity
- `mobility()` — read from the entity (see Mobility below)

### WorldFeatureOpening

Constructed by the solver when an edge line passes through a world feature entity's footprint. Not stored anywhere; derived at solve time and discarded when the solve is complete. Covers all world-driven openings: machines, pillars, conveyors, solid walls.

There is no separate `SolidWallOpening` — a solid wall is a WorldFeature type, and its opening behaves identically to any other WorldFeatureOpening.

- `getWidth()` — queries the world feature entity for the projected intersection length
- `mobility()` — returns 0; position is driven by the world feature, never by the solver

---

## Mobility

`mobility` controls how much a DeclaredOpening resists displacement when the solver must redistribute available space.

| Value | Behaviour |
|---|---|
| `0.0` | Immovable — hard constraint. Position is always `desired_position_mm`. |
| `1.0` | Standard soft constraint. |
| `> 1.0` | More easily displaced relative to lower-mobility neighbours. |

**The default is `0.0` (immovable).** This is a deliberate fail-safe: if an opening is incorrectly deserialised with a missing mobility value, it stays fixed rather than drifting. An immovable conveyor opening is a safe failure mode; a freely-floating one is not.

WorldFeatureOpenings always return `0.0` — their position is driven by the world feature, not the solver.

**Displacement distribution:** when the solver must shift openings to satisfy panel snapping or closure constraints, it distributes the required displacement among soft openings proportional to their mobility. A high-mobility door adjacent to a low-mobility conveyor opening absorbs most of the displacement.

**Desired vs. actual position:** `desired_position_mm` on a DeclaredOpening entity stores the user's intent and persists across solves. The solver attempts to honour it. When constraints force displacement, the actual solved position lives only in the edge's solve cache — it is never written back to `desired_position_mm`. When constraints relax, the solver can restore openings closer to their desired positions.

---

## Solve Cache

The edge's solve cache stores all derived results: the merged ordered sequence of spans and openings with actual positions, solved panel combinations per span, and span constraint annotations. It is:

- Invalidated whenever the edge's declared openings change, a world feature moves, or a span constraint changes
- Never serialised — always recomputed from entity state on load
- The sole source of "actual position" for any element on an edge

---

## World Features

World features exist independently of edges. When an edge line passes through a world feature's footprint, a WorldFeatureOpening is automatically generated in the solve cache. When the edge moves away from the footprint, the opening disappears.

| Type | Examples | NodeType implication |
|---|---|---|
| Machine | Robot, CNC, press | Adjacent nodes may become `None` |
| Pillar | Column, structural post | Node at pillar position |
| SolidWall | Concrete wall, building wall | Edge becomes `[WorldFeatureOpening]` |
| ConveyorIO | Fixed conveyor entry/exit | Hard-positioned opening of known width |

---

## Span Constraints

Span constraints are world-space entities that affect how spans are solved without removing them. They do not create openings. They project onto any edge they intersect and force span splits at their boundaries, so each resulting sub-span can be solved with the appropriate catalog restrictions.

Example — low ceiling:
```
entity: SpanConstraint(footprint, max_height=750mm)

derived spans for south edge after split:
  [0 → 3000mm]      catalog: full
  [3000 → 3800mm]   catalog: filtered to panels ≤ 750mm
  [3800 → end]      catalog: full
```

Multiple span constraints can overlap; the most restrictive constraint wins per sub-span.

---

## Corner: NodeType::None

When world feature openings consume the end of one edge and the start of the adjacent edge, the shared node becomes `NodeType::None` — no post, no hardware. The machine or wall is the effective corner.

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

---

## NodeType

NodeType is **always solver output, never user input**. After edge lengths and opening occupancy are known, the solver walks this priority list and selects the best available type at each node:

1. **None** — fully consumed by adjacent world feature openings. No hardware.
2. **Straight** — edges collinear (~180°). Post inline, no corner piece.
3. **RoundedCorner** — a rounded corner piece fits the angle.
4. **Post** — standard 90° post.
5. **AngledPost** — two posts at a non-standard angle.
6. **Gap** — small physical gap between wall ends. Last resort; preferable to a forced bad angle.

A 50mm gap between a machine and a wall is preferable to a forced awkward corner angle.

---

## Solver Architecture — Five Layers

### Layer 0 — Blueprint Import *(future)*

Reads a factory floor plan (DXF, IFC, image) and produces world feature and span constraint entities from geometry. Also produces a `FloorPlan` polygon defining the search space for cell placement.

No coupling to the solver. Produces the same entity state a user would declare manually. See [VISION.md](VISION.md) Phase 5.

### Layer 1 — Topology Definition

The solver reads from entity LayoutComponents:
- WorldFeature and SpanConstraint entities (positions, footprints, properties)
- Node and Edge entities forming the closed polygon
- DeclaredOpening entities on specific edges
- Catalog reference from each Edge entity's LayoutComponent

WorldFeatureOpenings are derived automatically at this stage. The user does not place them.

### Layer 2 — Per-Span Candidate Generation

For each derived span:
1. Project world features → WorldFeatureOpenings
2. Project span constraints → split spans at constraint boundaries
3. Distribute DeclaredOpenings using mobility weights
4. For each remaining span, compute desired length and return the **top K valid panel combinations** ranked by the active objective function

If a world feature opening leaves insufficient room for even the smallest valid span, that side's span length is zero. The node may become `None` if both adjacent edges have zero-length spans at that corner.

### Layer 3 — Global Closure Solving

Select one combination per span such that the polygon closes exactly (sum of edge vectors = 0). For small cells: exhaustive search. For larger cells: constraint propagation or branch-and-bound.

The global objective function is applied here: among all closure-consistent solutions, pick the best-scoring one.

### Layer 4 — Corner Resolution

Given solved edge lengths and opening occupancy at each node, walk the NodeType priority list and select the best type. If the result is undesirable (AngledPost or Gap), optionally re-enter Layer 3 with a nudge constraint on adjacent spans.

---

## Objective Functions

The solver is parameterised by an objective function. The default is **minimum cost**.

| Function | Description |
|---|---|
| Cost (default) | Minimise total panel and hardware cost |
| Ergonomic | Minimise awkward angles, maximise door accessibility |
| Volume | Minimise enclosed floor area |
| Panel count | Minimise the number of distinct panel widths |
| Custom | User-defined scoring function |

Objective functions score a complete candidate solution. They must be **composable**: a solution can be scored as 70% cost + 30% ergonomic.

---

## Catalogs

Fence catalogs are **user-defined and edge-scoped**. Different edges can use different catalogs. The system does not hard-code any catalog structure beyond the JSON schema.

The current Axelent X-Guard catalog is an example instance, not a template.

**Edge height** equals `post.height_mm` in the catalog. This is derived, not stored as a separate field — use `catalogEdgeHeight()` in code. Panels may vary in height within a catalog, but the post defines the structural height of the cell boundary.

---

## Blueprint Import — Extended Notes

Layer 0 is strictly separate from the solver. Its responsibilities:

**World feature and span constraint extraction** — parse geometry and classify into entity state (world positions, footprints, properties such as ceiling height).

**Search space** — produce a `FloorPlan` polygon feeding a pre-solve placement step. The configuration solver (Layers 1–4) operates within the placed boundary and never knows about the factory floor geometry directly.

Placement (where does the cell go on the factory floor?) and configuration (how is the fence arranged?) are **distinct problems** and must remain separate.

---

## What This Design Does Not Cover Yet

- Placement solver (deciding *where* to put the cell on the factory floor)
- Multi-cell layouts
- Vertical features beyond height constraints (full 3D span constraints)
- Real-time incremental re-solving during interactive drag
- Diagonal / corner-spanning openings (opening dragged to a node)

These are explicitly deferred. The architecture must not couple to them prematurely.

---

## Extension Rules

Before adding any new solver capability, verify:

1. Does it belong in an existing layer, or does it need a new one?
2. Does it respect the features-first / dimensions-emergent philosophy?
3. Does it preserve the independence of Layer 0 from Layers 1–4?
4. Does it preserve the objective function interface — any function, composable?
5. Does `NodeType` selection remain a solver output, never a user input?
6. Do new opening types subclass `Opening` rather than introducing parallel element lists?
7. Are world features and span constraints kept independent of edges — derived at solve time, not stored on edges?
8. Does new data on a DeclaredOpening entity distinguish user intent (`desired_position_mm`) from solver output (solve cache)?
9. Does any new mobility-like property default to `0.0` (immovable) as the fail-safe?
