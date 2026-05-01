# tiny_cell — Vision and Goals

## What This Is

tiny_cell is a **generative design tool for robot cell integrators**. The user manipulates constraints — cell size, conveyor input position, pallet locations, product dimensions, cycle time target — and the system generates and evaluates a valid cell layout in real time: fence placement, robot selection and positioning, pick-and-place motion, and safety zones.

The inspiration is *Tiny Glade*: the user interacts directly with the cell and the system responds immediately with an updated, visually coherent result. Nothing feels like filling in a form. Everything feels like sculpting.

---

## Audience

**Robot integrators** designing palletizing cells. They understand robots, conveyors, and safety standards. They do not want to use a CAD tool. They want to explore the design space quickly, see whether a robot reaches its pick and place points, understand the cycle time implications, and share a convincing visual with a customer — all before committing to detailed engineering.

---

## The Core Interaction Model

The user controls constraints. The system owns dimensions.

| User sets | System generates |
|---|---|
| Conveyor position and width | Robot model and mounting position |
| Pallet position(s) | Pallet stacking pattern |
| Box dimensions and weight | Pick-place cycle animation |
| Target cycle rate (units/min) | Cycle time estimate |
| Cell boundary (approximate) | Fence panel layout and corner hardware |
| | Workspace overlay (green/yellow/red) |
| | Safety zone boundaries (ISO 10218-2) |

The user does not place fence panels. The user does not select a robot model. These emerge from the constraints. If the user drags the conveyor closer to the pallet, the robot arm shortens, the cycle time improves, and the fence adjusts — all immediately.

---

## Initial Scope: Palletizing

The first application is a **single-robot palletizing cell**: boxes arrive on a conveyor, a robot picks them and stacks them on one or more pallets, the cell is enclosed by a safety fence.

This is deliberately narrow. One application, done well, is more convincing than five applications done poorly.

### What the demo shows

- A robot picking boxes from a conveyor and stacking them on a pallet
- Animated pick-place cycle at the configured cycle rate
- Fence panels snapping to valid combinations around the cell boundary
- Corner hardware resolving automatically based on angles
- Workspace reachability feedback as the user moves elements
- Safety zone boundaries derived from robot stop distance

### Explicitly out of scope for now

- Multiple robots
- Welding, machine tending, or other applications
- Full kinematic simulation (approximate motion paths are sufficient for the demo)
- Blueprint import from DXF or IFC
- Export to CAD or robot programming systems

These are not ruled out forever — the architecture must support them — but they are not in the demo.

---

## Visual Quality

The demo must look professional. A robot integrator who sees something polished takes the concept seriously.

- Good lighting with soft shadows
- Subtle floor grid
- Colour-coded workspace overlay: green = reachable, yellow = marginal, red = out of reach
- Smooth robot motion animations
- Snap feedback when parameters change
- Clean, uncluttered UI — parameters visible but not overwhelming

Visual quality is not a finishing step. It is a design constraint from the start.

---

## Design Principles

**1. Responsiveness over precision.**
The tool must feel immediate. Use simplified calculations that update in real time. A cycle time estimate accurate to ±5% that updates instantly is more useful than a precise simulation that takes three seconds.

**2. Visual quality sells the concept.**
An integrator's first impression is visual. A cell that looks right and moves plausibly is more convincing than one that is numerically correct but visually rough.

**3. One application, done well.**
Do not generalise the demo to cover welding, machine tending, or assembly. Palletizing only. Depth over breadth.

**4. Architecture for the future.**
The demo scope is narrow; the architecture must not be. Data models, interfaces, and component boundaries should support adding new applications, robot types, and simulation fidelity without rewriting the core. See the entity system and solver design documents for the specific constraints this places on the code.

**5. Test algorithms in isolation.**
Pallet pattern generation, reachability checking, and cycle time estimation are pure logic. They must have unit tests that run without a window or a scene.

---

## Technology Stack

| Concern | Technology |
|---|---|
| 3D rendering and scene management | threepp (C++20) |
| Robot models | URDF via threepp's robotics module |
| Entity/component management | EnTT |
| Physics simulation (future) | NVIDIA PhysX via threepp integration |
| Design space optimisation (future) | Python |

threepp is developed by a project collaborator and has unpublished robotics modules including URDF loading, path planning, and ROS2 support. Treat it as the primary library — all visualisation and robot interaction goes through it.

---

## Relevant Standards

| Standard | Relevance |
|---|---|
| ISO 10218-2:2025 | Safety requirements for robot cells — layout, risk assessment |
| ISO 13855 | Safety distances based on approach speed |
| ISO/TS 15066 | Collaborative robots — force and speed limits |
| ISO 12100 | General machinery risk assessment methodology |

Safety zone calculations in the tool should reference these standards explicitly, even if the initial implementation is approximate.

---

## Roadmap

### Phase 1 — Fence layout (current)
Parametric fence panel solver. Nodes, edges, declared openings. Panel combinations from catalog. Corner hardware resolution. Interactive demo with a dummy slab opening.

### Phase 2 — Robot placement
Robot catalog, reachability check, automatic placement within the cell boundary. Workspace overlay.

### Phase 3 — Pick-and-place simulation
Animated pick-place cycle. Cycle time estimation. Pallet pattern generation.

### Phase 4 — Simulation integration
PhysX for box dynamics (stacking, settling). More accurate cycle time via physics-based motion.

### Phase 5 — Blueprint import
DXF/IFC floor plan import. World feature extraction. Search space boundary for cell placement.
