# Transport, Sensor & Station Model

## Scope

This document specifies the simulation architecture that governs how items move through the factory: belts, pickers, ports, sensors, and stations. It supersedes the older capacity-based gating model in `TransportComponent::capacity` / `ConveyorBeltComponent::capacity_blocked` and the older "stations hold items" model in `StationComponent::held_item_`.

This is a **design specification**. The codebase is being refactored to match it; existing code may still reflect the older model in places.

For the broader entity-system overview see [ENTITY_SYSTEM.md](ENTITY_SYSTEM.md). For belt geometry and catalog data see [CONVEYOR_BELTS.md](CONVEYOR_BELTS.md).

---

## The unified mental model

```
                 sensors (own pose, optionally a laser-sensor marker)
                          ▲ scanned/written by
                          │
   port ─── exposes ───── sensor list (AND-gated; clear iff every sensor is unblocked)
    ▲
    │ along its length, a belt has
    │
   belt ─ entry, exit, [tap...] ports
    │ motion gated by all gate_ports being clear
    │
   item ─ ItemOnTransportComponent → some transport
            │
            └── if no ItemOnTransportComponent, must be parented to a container

   station ─ orchestrator: sensors, dispatch logic, drives picker(s)
              │ never owns items; only references them
              │
              └─ picker ─ a transport entity with target/state
                          items aboard ride via their ItemOnTransportComponent
```

### The core rule

**An item is either on a transport** (carries `ItemOnTransportComponent` whose `transport` field points at a belt or a picker) **or contained inside another item** (parented to it via `PoseComponent::parent`) that is itself on a transport, transitively.

Items at rest are never floating in world space — they are always in some container. The container chain ends at something that does have `ItemOnTransportComponent`.

### Roles in one line each

| Concept | Role |
|---|---|
| **Item** | Passive thing being moved or processed. Has pose + dimensions. |
| **Transport** | Anything that moves items: belt, picker, future robot/AGV/lift. Owns the motion math. |
| **Port** | Reference point along or between transports. Hosts a list of sensors. |
| **Sensor** | Detection device. Returns a `blocked: bool`. Physical sensors scan their volume; virtual sensors are written by other code. |
| **Station** | Orchestrator. Reads sensors, runs dispatch logic, assigns work to its pickers, writes virtual sensors. **Does not own items.** |
| **Container** | An item (e.g. pallet) that holds other items via parent-chain. The container itself rides a transport; children inherit transform. |

---

## Components

### `SensorComponent`

```
SensorComponent
  blocked        bool   — current reading; true = "do not flow into here"
```

Every sensor entity has `SensorComponent + PoseComponent`. The pose is parented to the port that lists this sensor, so the sensor's world pose follows the port's parent chain.

### `LaserSensorComponent`

Empty marker — its **presence on a sensor entity makes it a laser**: a real point in 3D at the sensor's world position. The scan system (see *Systems*) updates the sensor's `blocked` field each tick by checking whether any item's bounding box (from `ItemPrototypeComponent`) currently encloses the laser point. The sensor itself carries no extents; the item's collision box decides whether it counts as "on" the laser.

A sensor without `LaserSensorComponent` is **virtual** — its `blocked` is written by station code or other systems.

This matches a real photoeye: the beam is at a specific (x, y, z), and an item triggers it iff the item's body is occupying that point. Items at different heights or cross-belt offsets do not interfere with each other's lasers.

### `PortComponent`

```
PortComponent
  name         string
  transport    entt::entity   — destination transport for items handed off here; null if the port is a sink-side or station-arrival point
  sensors      vector<entt::entity>   — list of sensor entities
```

A port is **clear** iff every sensor in `sensors` reports `blocked == false`. A port with no sensors is always clear. The same sensor entity may appear in multiple ports' lists; this is how a single virtual sensor (e.g. "station busy") can gate several belts at once.

### `TransportComponent`

```
TransportComponent
  running   bool   — controller intent; default true
```

`running == true` is *necessary but not sufficient* for items to move. The actual motion gate per transport flavour is described below.

### `ConveyorBeltComponent`

```
ConveyorBeltComponent
  catalog_ref              string
  width_mm                 int
  length_mm                int
  surface_height_mm        int
  opening_clearance_mm     int
  belt_speed_mm_s          float
  dir                      Vec3            — unit vector in scene-root frame
  entry_port               entt::entity
  exit_port                entt::entity
  tap_ports                vector<entt::entity>   — ports along the belt's length; t derived from each port's pose
  gate_ports               vector<entt::entity>   — subset of {exit_port} ∪ tap_ports whose blocked sensors freeze the belt; default {exit_port}
```

A belt is **moving** iff `running_() && every gate port is clear`. An item is advanced this tick only if the belt is moving.

### `ItemOnTransportComponent`

```
ItemOnTransportComponent
  transport   entt::entity   — the belt or picker entity carrying this item
```

An item with this component is on the named transport. The transport's tick code is responsible for advancing the item's pose. An item without this component is at rest in a container (see core rule above).

### `PickerTransportComponent`

```
PickerTransportComponent
  home_pose          Vec3
  pickup_target      Vec3            — world pose to grab from
  drop_target        Vec3            — world pose to release at
  drop_container     entt::entity    — entity to parent the box to on placement; null = remain free at world pose
  drop_orientation   Quat
  speed_mm_s         float
  state              PickerState     — Idle | MovingToBox | Carrying | Returning
  current_box        entt::entity    — null when Idle
```

A picker is an autonomous transport with a self-contained state machine. Once dispatched (fields populated, `state := MovingToBox`), it drives itself through the rest of the cycle without further station input.

### `StationComponent`

```
StationComponent
  arrival_port             entt::entity         — where items arrive for processing
  arrival_virtual_sensor   entt::entity         — the virtual sensor on the arrival port that the station drives
  pickers                  vector<entt::entity> — pool of pickers this station dispatches to
```

The station holds **no per-item state**. All in-flight work lives on the picker(s). Multi-picker palletizers are a matter of populating `pickers` with more than one entity.

### `PalletizeComponent`

```
PalletizeComponent
  pallet_arrival_port           entt::entity
  pallet_tap_virtual_sensor     entt::entity      — gates the pallet belt while the station has a pallet to fill
  current_pallet                entt::entity      — observation reference; null while waiting
  pattern                       PlacementPattern* — slot allocator, supports reservation
  pallet_length_mm              int
  pallet_width_mm               int
  pallet_height_mm              int
  pallet_max_stack_mm           int
```

Station has a pallet to fill iff `current_pallet != null`. No separate state enum is needed.

---

## Systems

There are four system entry points. They run in a fixed order each tick (see *Tick order* below).

### `sensor::scan(scene)`

For every entity carrying `SensorComponent + LaserSensorComponent + PoseComponent`:
1. Compute the sensor's world position from its parent-chain transform.
2. For every spawned item, project `(sensor_pos − item_centre)` onto each item-local axis and compare against the item's half-extent on that axis (from `ItemPrototypeComponent`).
3. Set `blocked = true` if all three projections are within range — i.e. the laser point is inside the item's oriented bounding box.

Virtual sensors (no `LaserSensorComponent`) are skipped — their `blocked` is written by other systems.

### `transport::step(scene, dt)`

Handles **all** transport-flavour motion in one place. Belts and pickers are both transports; their motion logic differs but their conceptual role is the same. (The previous draft of this design split belt sim and picker sim into separate systems; that split has been collapsed.)

For each transport entity:

**If it is a belt** (`ConveyorBeltComponent`):
1. Check belt-moving predicate: `running_() && every gate port is clear`. If false, skip — items on this belt do not advance.
2. For each item with `ItemOnTransportComponent::transport == this_belt`:
   - Compute `prev_t = (item.world_pos - entry_port.world_pos) · dir` (before advance).
   - Advance: `pose.position += dir * belt_speed_mm_s * dt`.
   - `new_t` similarly.
   - For each tap port on the belt: if `prev_t < tap_t <= new_t`, the item has crossed the tap this tick. (No event is emitted — stations poll sensors directly.)
   - If `new_t > length_mm`:
     - If `exit_port.transport != null` and `port_is_clear(exit_port)`: reassign `ItemOnTransportComponent::transport` to `exit_port.transport`; snap pose to exit_port's pose.
     - If `exit_port.transport == null`: this is a sink or station-arrival point; sink consumption happens in `lifecycle::step`, station capture happens in `station::step`. The item simply stops advancing.

**If it is a picker** (`PickerTransportComponent`):
1. Move pose toward whichever target the current state requires (`pickup_target` if `MovingToBox`, `drop_target` if `Carrying`, `home_pose` if `Returning`) at `speed_mm_s` per second, clamped to the target.
2. On reach, transition state and apply side-effects:

| From → To | Trigger | Side-effects |
|---|---|---|
| `MovingToBox → Carrying` | reached `pickup_target` | reassign box's `ItemOnTransportComponent::transport` to this picker; reparent box to picker (poses match → no jump) |
| `Carrying → Returning` | reached `drop_target` | remove `ItemOnTransportComponent`; if `drop_container != null` reparent box to container with `drop_orientation`, else leave at world pose |
| `Returning → Idle` | reached `home_pose` | clear `current_box` |

The picker never asks the station what to do next. Once dispatched it self-drives.

### `station::step(scene, dt)`

Pure dispatch — runs after sensors and transports have updated.

For each station:
1. **Pallet acquisition** (if `PalletizeComponent`): if `current_pallet == null` and `pallet_arrival_port`'s sensor is blocked, find the item in the port's volume, claim it as `current_pallet`. Set `pallet_tap_virtual_sensor.blocked = true` to keep the pallet belt frozen while filling.
2. **Box dispatch**: if there is a pallet AND any picker is `Idle` AND the box arrival port's sensor reports a present item:
   - Identify the candidate box (item in `arrival_port`'s volume, not already assigned to another picker).
   - Reserve a slot from the placement pattern → world pose for `drop_target`.
   - Populate the picker's job fields: `pickup_target`, `drop_target`, `drop_container = current_pallet`, `drop_orientation`, `current_box`. Set `state = MovingToBox`.
3. **Virtual-sensor maintenance**:
   - `arrival_virtual_sensor.blocked = (no idle picker available) || (current_pallet == null)`.
   - `pallet_tap_virtual_sensor.blocked = (current_pallet != null)`.
4. **Pallet release**: if pattern is full and every picker is `Idle`, set `current_pallet = null` and clear `pallet_tap_virtual_sensor`. The pallet belt then resumes; the full pallet rides on under standard belt logic.

### `lifecycle::step(scene, dt)`

The only system that does **not** relate to transport motion. Handles item creation and destruction:

- **Sources**: each `SourceComponent` accumulates spawn debt; while debt ≥ 1 and `port_is_clear(out_port)`, spawn a new item, attach `ItemOnTransportComponent` pointing at `out_port.transport`, decrement debt.
- **Sinks**: items whose pose lies within a sink port's volume (or that have been emitted from a belt with `exit_port.transport == null` and a sink registered on that port) are destroyed; `received` is incremented.

This system is the one place where items come into and leave the simulation. Everything else is motion or orchestration.

---

## Tick order

```
sensor::scan(scene)        // physical sensors refresh from current item poses
transport::step(scene, dt) // belts advance items, pickers run state machines, handovers happen
station::step(scene, dt)   // dispatch new work, write virtual sensors, release full pallets
lifecycle::step(scene, dt) // sources spawn, sinks despawn
```

There is intentionally a **one-tick lag** between station decisions (writing a virtual sensor) and visible motion changes (transport reading that sensor on the next tick). At ~16 ms ticks and 200 mm/s belt speed, items move ~3.2 mm per tick — well inside any reasonable sensor volume — so the lag is invisible.

The same lag applies to handovers: an item handed from belt A to belt B in tick N enters B's entry sensor volume, but the sensor still reads "clear" this tick (it scanned before the advance). Tick N+1's scan sees the item at the entry; further handovers are correctly blocked. As long as no item can fully traverse a sensor volume within a single tick, gating is correct.

---

## Tap ports and mid-belt stations

A tap port is a `PortComponent` parented to a belt entity. Its position along the belt is **derived from its pose**, not stored explicitly:

```
tap_t = (tap.world_pos - belt.entry_port.world_pos) · belt.dir
```

This means a belt and its tap ports are one rigid spatial unit — moving the belt moves all its taps automatically.

### Two patterns for stations on a belt

**Gate-and-process** (palletizer, press, drill): the tap port is in the belt's `gate_ports` list. When the station's virtual sensor at the tap is `blocked`, the belt freezes; items already past that point also freeze (the belt-moving predicate is global per belt). The station drives the virtual sensor based on whether it has work to do.

**Pass-through** (CV inspection, spray paint): the tap port is *not* in `gate_ports`. The belt keeps running. A physical sensor at the tap detects items entering its volume; the station observes each tick and runs concurrent logic without affecting flow. Multiple items may be in the station's zone simultaneously.

The two patterns share the same component machinery — the only difference is membership in `gate_ports`.

---

## Sensor patterns at a station's arrival port

How sensors are wired at a station's arrival port depends on what the station does to the item — specifically, whether the station **removes** the item from its arrival location or **releases it in place**.

### Pattern A — workpiece is removed (one port, two sensors)

For "gate-and-process" stations where the picker physically picks the item up and carries it elsewhere (boxes feeding a palletizer, parts feeding a press, …), the arrival port carries **two sensors on the same port**:

| Sensor | Type | Role |
|---|---|---|
| `arrival_physical_sensor` | physical | Detects whether an item is currently sitting at the port. Used by the station's poll logic to find what to capture. |
| `arrival_virtual_sensor` | virtual, station-driven | Encodes "the station can/can't accept new work right now" — gates the upstream belt. |

The belt's `gate_ports` includes the port; both sensors gate motion. The belt freezes if either is blocked:

- A box arriving and sitting at the port → physical blocked → belt freezes → box waits.
- Station busy or no pallet present → virtual blocked → belt freezes upstream → no new box arrives.

The chicken-and-egg "station can never release the held item" doesn't bite here because the *physical* sensor clears the moment the picker carries the item out of the volume.

### Pattern B — workpiece is released in place (two ports, one sensor each)

For "claim-and-fill" stations where the workpiece **stays** while the station works on it and then is *released back into flow* rather than removed (a palletizer's pallet, a CV station that triggers on entry but lets the item ride past, …), the one-port two-sensor split locks up: after the station clears its virtual sensor, the physical sensor still sees the workpiece sitting in the volume, so the gate stays closed and the belt never resumes.

Use **two ports at the same location** instead:

| Port | Membership | Sensors |
|---|---|---|
| `tap_gate`   | **In** `belt.gate_ports` | One **virtual** sensor (station-driven). Controls belt motion. |
| `tap_detect` | **Not** in `belt.gate_ports` | One **physical** sensor. Read by the station to detect arrival. Has no gating effect. |

Both are tap ports parented to the belt's entry port at the same `position_mm`, so they share a world location and orientation but play different roles:

- Pallet arrives at the tap. Physical sensor on `tap_detect` blocks → station polls and claims the pallet → station sets virtual sensor on `tap_gate` blocked → belt freezes.
- Pallet fills. Station clears virtual sensor on `tap_gate` → belt resumes → pallet moves forward → physical sensor on `tap_detect` clears as the pallet leaves the volume.

`StationComponent::arrival_port` (or `PalletizeComponent::pallet_arrival_port`) points at the **detect** port; the station's virtual sensor reference points at the sensor on the **gate** port.

---

## Picker dispatch and the multi-picker path

A station's `pickers` is a list. Today's palletizing demo populates it with one picker; nothing else assumes single-picker.

### Going from one to N pickers

The architectural changes required to dispatch *N* pickers in parallel are contained:

| Change | What it does |
|---|---|
| `station.add_picker(p1); station.add_picker(p2);` | Wiring |
| `PlacementPattern::reserve_slot()` / `release_slot(id)` (replacing single-shot `next_pose`) | Two pickers in flight need non-conflicting destinations |
| `station::step` dispatch loop iterates idle pickers and assigns up to *N* jobs per tick | One-line generalisation |
| `arrival_virtual_sensor` predicate becomes `(no idle picker available) \|\| (no pallet)` | Same predicate, different boolean source |

`StationComponent`, `PickerTransportComponent`, and `transport::step` are unchanged. The picker entity does not know whether it is working alone or alongside siblings.

---

## Future: physics-driven simulation

The kinematic motion in this model is confined to two functions: `transport::step` (belts and pickers) and `sensor::scan` (overlap testing). Physics integration replaces those two functions and degrades two component fields without touching the dispatch layer:

| Today | Under physics |
|---|---|
| `transport::step` advances item poses by `dir * speed * dt` | Belt is a kinematic surface imparting friction; physics step writes positions |
| `sensor::scan` does AABB tests in sensor-local space | Sensors become collision triggers; `blocked` is written from overlap callbacks |
| `ItemOnTransportComponent::transport` is authoritative | Becomes a cached hint, written by a contact-tracking system |
| Container parent-chain (`box.parent = pallet`) | Replaced by rigid-body stacking via contact + gravity |

Discipline maintained today to keep this swap small:

1. **Confine motion to `transport::step`.** No motion equations elsewhere.
2. **Sensors are read only via `port_is_clear()` and `SensorComponent::blocked()`.** Callers never reach into `LaserSensorComponent` or iterate items themselves.
3. **No APIs that depend on kinematic determinism** (e.g. "compute time-until-arrival"). Physics has no deterministic ETA.

The component contracts, port/sensor model, and station dispatch carry over unchanged.

---

## Out of scope

The following are intentionally not part of this model. They may be added later as additive changes:

- **Per-item gap simulation on belts.** When a belt freezes, every item on it freezes together — there is no item-vs-item collision or pile-up modelling.
- **Item-shape-aware sensor detection.** Sensors test item *centres*, not their full bounding boxes. Sensors must be sized with margin.
- **Sink-side gating sensors.** Sinks consume whatever arrives; their ports carry no sensors and are always clear.
- **Station-to-station handover signals.** A robot picking up output from another station (S3-style) requires a "ready" signal that this model does not specify. Likely future addition: producer station emits a synthetic arrival event, or attaches a marker component to the completed item.
- **Single-belt mid-stream physics.** Mid-belt taps are supported, but a belt cannot meaningfully "pause only past tap-position" — the belt-moving predicate is global per belt. Localised pile-ups need physics.
- **Physical sensors with arbitrary OBB rotations** beyond what the parent-chain already provides.
