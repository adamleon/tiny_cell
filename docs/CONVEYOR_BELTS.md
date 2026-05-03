# Conveyor Belts

## Scope

This document covers the data model, catalog structure, and mesh generation for straight flat-belt conveyors. Two catalogs are defined: a **generic catalog** using procedural box geometry (available now, no CAD files required) and a placeholder for the **mk GUF-P 2000 catalog** (future, requires real CAD files from mk). The workflow solver that will eventually place belts is not designed here — belts are wired manually in current demos.

Inclined belts, Z-frame elevation changers, and chain/pallet conveyors are deferred.

---

## Research Summary — mk GUF-P 2000

**Manufacturer:** mk Technology Group, Troisdorf, Germany. Metric, European market. UR+ certified. Widely used in robot cell integration.

### Dimensions

| Parameter | Value |
|---|---|
| Available widths | 50, 75, 100, 150, 200, 250, 300, 400, 500, 600, 700, 800 mm |
| Length range | 380 – 10 000 mm (continuous, 1 mm increments) |
| Frame profile height | 50 mm |
| Belt surface offset above frame top | ~3 mm (belt thickness) |
| Belt surface height above mounting (no legs) | ~53 mm |
| Frame outer width | belt_width_mm + 40 mm (20 mm aluminium overhang each side) |
| Max recommended leg span | 1 500 mm |
| Max incline (flat smooth belt) | 20° |
| Motor overhang at drive end | ~180 mm |

### Leg System

Specified by the resulting **top-of-belt height** (floor to belt surface), not raw leg length. Practical range for robot cell use: 500 – 1 050 mm.

`leg_height_mm = belt_surface_height_mm − 53`

### Cost

Configure-and-quote. Indicative: €1 200 – €2 800 new for a 1 m × 200 mm unit. Rough budget: €20 – €40 per mm of belt width for 1–2 m lengths.

### CAD Models

mk provides parametric STEP files via CADENAS PartCommunity (account required). Because belt length is continuous and we want live resizing and UV animation, the mesh is **procedurally generated** from catalog parameters even when real mk geometry is eventually available. The mk catalog's visual contribution is primarily the end-cap and drive-unit OBJ meshes — the belt surface and legs remain procedural regardless of catalog.

### Fence Opening Conventions

No codified standard. Practical defaults used here:

| Belt width | Opening width | Guarding assumption |
|---|---|---|
| ≤ 200 mm | belt_width + 60 mm | Tunnel guard |
| > 200 mm | belt_width + 100 mm | Light curtain |

Default implementation: `opening_width_mm = belt_width_mm + 100`. Overridable per entity.

---

## Where Belts Come From

The full workflow system design is not yet settled and is not specified here. What is known:

- A future workflow solver will derive belt positions and connections from higher-level declarations ("item A enters station B, outputs C")
- Belt entities and their fence openings will be **workflow solver output**, not direct user placement
- For current demos, belts are placed manually in code

The current entity model is designed so that the workflow solver can be added later without restructuring. The `FlowNodeComponent` on every belt entity is the hook point — the workflow solver will build the directed flow graph by setting `entry` and `exit` references.

### Belt Placement Drives Fence Openings

Wherever a belt's axis intersects a fence edge, a `DeclaredOpeningComponent` is created on that edge (by the workflow solver in future; by demo setup code now):

```
opening.type                 = OpeningType::Open
opening.width_mm             = belt_width_mm + 2 × opening_clearance_mm
opening.mobility             = 0.0
opening.parent_edge          = intersected edge entity
opening.desired_position_mm  = intersection distance from node_a (mm)
```

The fence layout solver treats these exactly like any other anchored opening. No new solver path is needed. A belt passing through the cell on both sides creates two such openings on two different edges from a single belt entity.

The solver may group multiple belts into one opening when their combined widths plus clearances fit within a span. Individual belt connections are tracked via `FlowNodeComponent` references, not by separate openings.

### Item Flow Graph

Belts connect to each other and to source/sink entities via `FlowNodeComponent`. The graph is a directed chain of `entt::entity` references:

```
SpawnItem  →  Belt1  →  Belt2  →  DespawnItem
```

- `Belt1.FlowNode.entry` = SpawnItem entity
- `Belt1.FlowNode.exit`  = Belt2 entity
- `Belt2.FlowNode.entry` = Belt1 entity
- `Belt2.FlowNode.exit`  = DespawnItem entity

Any entity carrying `FlowNodeComponent` can appear in the graph. Future station types (robot cells, buffers, machines) slot in without changing the model.

---

## ECS Model

### Entity Composition

```
ConveyorBeltEntity:
  PoseComponent              — center of belt (floor level), yaw = travel direction
  ConveyorBeltComponent      — belt dimensions and catalog reference
  FlowNodeComponent          — entry and exit in the item-flow graph
  VisualComponent  (future)  — owns threepp mesh, asset key from catalog
  InteractiveComponent (future)
  SimulationComponent (future)
```

See [ENTITY_SYSTEM.md](ENTITY_SYSTEM.md) for component field definitions.

### PoseComponent for Conveyors

`PoseComponent.position` = world-space centre of the belt at floor level (Z = 0, base of legs). Orientation encodes the travel direction yaw. Z-up convention as everywhere else.

Belt endpoints (used for fence intersection):

```
end_a = position + orientation * (−length_mm * 0.5, 0, 0)   // mm, Z-up
end_b = position + orientation * (+length_mm * 0.5, 0, 0)
```

---

## Catalogs

### Catalog Interface

All belt catalogs satisfy the same interface. The mesh generator reads catalog data; the rest of the system reads only `ConveyorBeltComponent`.

```jsonc
{
  "catalog_ref": string,
  "manufacturer": string,
  "series": string,

  "widths_mm": [int, ...],          // discrete list; width_mm must be in this list
  "length": { "min_mm": int, "max_mm": int },

  "cross_section": {
    "frame_height_mm": int,
    "belt_surface_offset_mm": int,  // top of frame to top of belt
    "frame_side_overhang_mm": int   // aluminium overhang beyond belt width each side
  },

  "legs": {
    "max_span_mm": int,
    "leg_profile_mm": int,
    "min_belt_surface_height_mm": int,
    "max_belt_surface_height_mm": int
  },

  "drive": {
    "motor_overhang_mm": int,
    "motor_height_mm": int
  },

  "belt_texture": {
    "tile_pitch_mm": int
  },

  "materials": {
    "belt":  { "color_hex": string, "roughness": float, "metalness": float },
    "frame": { "color_hex": string, "roughness": float, "metalness": float },
    "legs":  { "color_hex": string, "roughness": float, "metalness": float }
  },

  "mesh_source": "procedural" | "obj"
  // "procedural" = all geometry generated from the dimensions above
  // "obj"        = end caps and drive unit loaded from OBJ files; belt + legs remain procedural
}
```

### Generic Catalog (`generic/flat-belt`)

Fully procedural. No CAD files required. Suitable as a placeholder until manufacturer-specific catalogs are available. Uses round metric dimensions.

```jsonc
{
  "catalog_ref": "generic/flat-belt",
  "manufacturer": "Generic",
  "series": "Flat Belt",

  "widths_mm": [100, 150, 200, 300, 400, 500, 600],
  "length": { "min_mm": 400, "max_mm": 10000 },

  "cross_section": {
    "frame_height_mm": 50,
    "belt_surface_offset_mm": 3,
    "frame_side_overhang_mm": 20
  },

  "legs": {
    "max_span_mm": 1500,
    "leg_profile_mm": 40,
    "min_belt_surface_height_mm": 500,
    "max_belt_surface_height_mm": 1200
  },

  "drive": {
    "motor_overhang_mm": 180,
    "motor_height_mm": 120
  },

  "belt_texture": {
    "tile_pitch_mm": 200
  },

  "materials": {
    "belt":  { "color_hex": "0x3a3a3a", "roughness": 0.7, "metalness": 0.0 },
    "frame": { "color_hex": "0xcccccc", "roughness": 0.3, "metalness": 0.7 },
    "legs":  { "color_hex": "0xbbbbbb", "roughness": 0.4, "metalness": 0.6 }
  },

  "mesh_source": "procedural"
}
```

### mk GUF-P 2000 Catalog (`mk/guf-p-2000`) — Placeholder

Not usable until OBJ files (end caps, drive unit) are sourced from mk CADENAS. The entry below reserves the `catalog_ref` string and records the correct dimensional data.

```jsonc
{
  "catalog_ref": "mk/guf-p-2000",
  "manufacturer": "mk Technology Group",
  "series": "GUF-P 2000",

  "widths_mm": [50, 75, 100, 150, 200, 250, 300, 400, 500, 600, 700, 800],
  "length": { "min_mm": 380, "max_mm": 10000 },

  "cross_section": {
    "frame_height_mm": 50,
    "belt_surface_offset_mm": 3,
    "frame_side_overhang_mm": 20
  },

  "legs": {
    "max_span_mm": 1500,
    "leg_profile_mm": 40,
    "min_belt_surface_height_mm": 500,
    "max_belt_surface_height_mm": 1200
  },

  "drive": {
    "motor_overhang_mm": 180,
    "motor_height_mm": 120
  },

  "belt_texture": {
    "tile_pitch_mm": 200
  },

  "materials": {
    "belt":  { "color_hex": "0x2a2a2a", "roughness": 0.65, "metalness": 0.0 },
    "frame": { "color_hex": "0xd4d4d4", "roughness": 0.25, "metalness": 0.75 },
    "legs":  { "color_hex": "0xc8c8c8", "roughness": 0.35, "metalness": 0.65 }
  },

  "mesh_source": "obj",
  "obj": {
    "STATUS": "PLACEHOLDER — no OBJ files yet; fall back to procedural",
    "end_cap": "assets/components/conveyors/mk/guf-p-2000/end_cap_{width_mm}.obj",
    "drive_unit": "assets/components/conveyors/mk/guf-p-2000/drive_{width_mm}.obj"
  }
}
```

When `mesh_source = "obj"` but the OBJ files are absent, the mesh generator falls back to `procedural` and logs a warning. This prevents the mk catalog entry from blocking any work before the files are available.

---

## Procedural Mesh Generation

The generator is a pure function: `(ConveyorBeltComponent, CatalogData) → shared_ptr<Object3D>`. No ECS access. All coordinates in local space, travel axis along **+X**, belt surface at `belt_surface_height_mm` above origin. Dimensions in metres.

### Components

**Frame body**
```
BoxGeometry(
  x = length_mm * 0.001,
  y = frame_height_mm * 0.001,
  z = (belt_width_mm + 2 × frame_side_overhang_mm) * 0.001
)
center Y = belt_surface_height_mm * 0.001 − frame_height_mm * 0.0005
```

**Belt surface**
```
PlaneGeometry(
  width  = length_mm * 0.001,     // along travel (X)
  height = belt_width_mm * 0.001  // across travel (Z)
)
rotated to lie in XZ plane
Y = belt_surface_height_mm * 0.001

UV: u ∈ [0, length_mm / tile_pitch_mm]  (travel direction)
    v ∈ [0, belt_width_mm / tile_pitch_mm]
wrapS = wrapT = Repeat
```

**End rollers** (×2)
```
CylinderGeometry(
  radius = frame_height_mm * 0.35 * 0.001,
  height = belt_width_mm * 0.001
)
rotated 90° to align with Z
X = ±length_mm * 0.0005
Y = belt_surface_height_mm * 0.001 − radius
```

**Drive unit** (at end_b by default)
```
BoxGeometry(
  x = motor_overhang_mm * 0.001,
  y = motor_height_mm * 0.001,
  z = (belt_width_mm + 2 × frame_side_overhang_mm) * 0.001
)
X = length_mm * 0.0005 + motor_overhang_mm * 0.0005
Y = belt_surface_height_mm * 0.001 − frame_height_mm * 0.0005
```

**Legs**
```
n = ceil(length_mm / max_leg_span_mm) + 1  (minimum 2)
spacing = length_mm / (n − 1)
positions X = [−length/2, −length/2 + spacing, ..., +length/2]  (in mm, then × 0.001)

BoxGeometry(
  x = leg_profile_mm * 0.001,
  y = (belt_surface_height_mm − frame_height_mm − belt_surface_offset_mm) * 0.001,
  z = leg_profile_mm * 0.001
)
Y = leg height * 0.0005  (centred vertically)
```

---

## Belt Animation

UV offset scrolling. In the animate loop (or `SimulationSystem` once implemented):

```cpp
float scroll = (belt_speed_mm_s * delta_s) / tile_pitch_mm;
belt_material->map->offset.x += direction_a_to_b ? scroll : -scroll;
```

`wrapS = Repeat` handles the periodicity — no modulo needed. The offset accumulates indefinitely without artefact.

---

## Open Questions

Deferred until workflow solver design is settled:

1. **Workflow solver ordering** — does the workflow solver run before, after, or in a joint loop with the fence layout solver?
2. **Belt routing** — first iteration assumes straight lines. When does obstacle routing become necessary?
3. **Belt endpoint at a fence node** — if the belt axis lands exactly on a corner node the opening geometry is degenerate. Define expected behaviour.
4. **Workflow node types** — machines, depots, spawn points are not world features. How they are defined and positioned is deferred.

Resolved:

- Belt-generated openings use `DeclaredOpeningComponent` with `mobility = 0.0` and `type = Open`. No new type.
- Multiple belts may share one opening; individual connections tracked via `FlowNodeComponent`.
- Belt-to-belt handoff is modelled by sharing a `FlowNodeComponent` exit/entry reference. Sufficient for phase 1.
- Workflow solver details are TBD; current demos wire the flow graph manually in code.

---

## Extension Rules

1. Do not add workflow solver code until that design is settled. Current demos wire belts manually.
2. Belt-generated fence openings are `DeclaredOpeningComponent` entities with `mobility = 0.0`, `type = Open`, and solver-assigned `desired_position_mm`. No new opening type.
3. `ConveyorBeltComponent` holds intent (width, height, speed). Leg positions, opening widths, and mesh geometry are derived — never stored in the component.
4. Width must be validated against the catalog discrete list at entity creation.
5. The mesh generator is a pure function with no ECS access.
6. Belt animation belongs in the animate loop (now) and `SimulationSystem` (future). Not in the mesh generator.
7. When `mesh_source = "obj"` but files are absent, fall back to procedural and log a warning. Never fail silently or crash.
8. The mk catalog entry must not be used for rendering until its OBJ files are present. Code that reads `catalog_ref = "mk/guf-p-2000"` must check `obj.STATUS` or test file existence before attempting to load.
