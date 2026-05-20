# HTN sandbox

Throwaway program to make the HTN-with-geometry solver architecture
**legible** before we build it for real in tiny_cell. **Iteration 3**:
task positions are **solved**, not declared. The outer loop enumerates
candidate positions for each task; the inner loop runs the strategy
search for each. The winner is the combination of (task positions,
strategy choices, equipment placement) with the lowest total cost.

## Run

After building (`cmake --build build --target htn_sandbox`):

```
./build/Debug/htn_sandbox.exe                           # default input.json
./build/Debug/htn_sandbox.exe path/to/other_input.json  # any path
```

Edit `input.json`, re-run — no rebuild.

## What's in the toy domain

- **Workflow**: source position + sink position on a 1-D line, plus a list
  of tasks declared **without positions**. The user states *what* needs
  to happen, not *where*. Tasks are ordered (workflow order = topological
  order); the first task lands between source and second task, etc.
- **Task kinds**:
  - `Palletize`, `Assemble` — root tasks the user declared
  - `Transport`, `DetectPresence` — sub-tasks emitted by `PushStrategy`
- **Strategies**:
  - `ArmStrategy` — solves `Palletize` *or* `Assemble`. Enumerates
    candidate anchor positions; at each anchor, claims every unclaimed
    task whose position falls in the work area, provided the summed
    rate ≤ arm capacity.
  - `PushStrategy` — solves `Palletize` only. Emits `Transport` +
    `DetectPresence` as sub-tasks.
  - `BeltStrategy` — solves `Transport`. Belt length = source→target
    distance; cost scales with length.
  - `LaserStrategy` — solves `DetectPresence`. Co-located with the task.
- **Outer loop**: enumerates task positions on a configurable grid
  (`outer_grid_step`) with an `outer_edge_margin` from source/sink.
  Topologically valid only (task `i` strictly upstream of task `i+1`).
  For each combination, runs the inner search silently and records its
  cost. After all combinations, picks the cheapest and re-runs the inner
  search with the full verbose trace for display.

## What the output looks like

```
=== Outer search: 14 task-position combinations ===

Feasible: 14 / 14

Top 5 cheapest:
  palletize_A=9.0  assemble_B=10.0  → €1000.0
  palletize_A=9.0  assemble_B=12.0  → €1000.0
  ...

Top 3 most expensive (feasible only):
  palletize_A=11.0  assemble_B=18.0  → €1800.0
  ...

==========================================
Detailed trace for the winning configuration:
  palletize_A=9.0  assemble_B=10.0  → €1000.0

Dispatch palletize_A (Palletize) at 9.0
  Try:
    [ArmStrategy ] pos=9.0 fp=[8.0..10.0] wa=[6.0..12.0] cost=1000.0  TERMINAL
        + 1×Arm  €1000.0
        * claims palletize_A
        * claims assemble_B
  >> COMPLETE solution cost=1000.0 (best so far)
  ...

Final layout:
    tasks:        *    *
    workzn:  =====================
    footpr:  ###########
    axis:    |                                                          |
             8---------10--------12--------14--------16--------18--------
```

Symbols:
- `o` = unclaimed task; `*` = claimed
- `=` = work area; `#` = footprint
- `|` = source / sink (pinned)

Each proposal in the trace shows: `[Strategy] pos fp wa cost`, equipment
added, tasks claimed, and any sub-tasks (`? needs …`) the strategy emits
for further dispatch.

## What to try

Default values (`palletize_A` + `assemble_B`, both 4/min, arms cheap and
fast) produce a clear winner: **one shared arm somewhere in the middle**
of the workflow, €1000/year.

### 1. Force tasks apart with the grid

Set `outer_grid_step` to **4.0**. Fewer candidate positions for each task;
some "good" anchors no longer reachable. Watch the cost climb if the
grid step skips over the sweet-spot zone for the shared arm.

### 2. Widen the workflow

Set `workflow.sink_position` to **30.0**. The outer loop now has more
positions to try; the cheapest combinations cluster near the source side
because long belts get expensive.

### 3. Raise the per-arm rate to force splitting

Set both task rates to **6.0**. Combined rate 12/min exceeds arm
capacity 10; the shared-arm proposal disappears. Outer loop now picks
configurations where push+arm is cheaper than two arms — typically
short belt distance (palletize close to source).

### 4. Make arms expensive

Set `arm.cost_each_eur` to **3000**. Push becomes attractive even at
moderate belt length. The outer winner shifts to push+arm with
palletize near the source.

### 5. Make pushers expensive

Set `push.pusher_cost_eur` to **2000**. Push is uncompetitive even with
short belts; arms dominate. Shared-arm wins when geometry allows.

### 6. Shrink the workflow until something goes infeasible

Set `workflow.sink_position` to **11.0**. Only ~1m of corridor between
source and sink. Most position combinations have no room for footprints.
Watch the feasibility count drop.

## What this NOT

- Not production code. Will be deleted once tiny_cell has the real
  solver.
- Not a complete model: 1-D only, no real throughput modelling, no
  catalog. Equipment costs are scalars in the JSON, not real catalog
  entries.
- Not optimal in absolute terms: outer-grid resolution caps the
  achievable precision. Finer grid → more combinations → more compute.
  For 2 tasks, even step=0.5 is instant.

## What this validates

After running through the experiments above, all the architectural
pieces we've discussed across the sessions are now concrete and
testable:

1. **Strategy** as a polymorphic interface: each strategy receives a
   task + solver state + workflow + params, returns proposals.
2. **Proposal** as a value type carrying equipment + cost + position +
   footprint + work area + claimed tasks + remaining sub-tasks.
3. **SolverState** as the running partial layout: occupancy intervals,
   task positions, claimed/pinned sets.
4. **Multi-task claims**: one proposal can cover many tasks if its work
   area + capacity allow it. Shared equipment emerges from geometry,
   not from a separate strategy.
5. **Footprint-aware dispatch**: new proposals must not overlap existing
   footprints; alternative anchors are tried.
6. **Branch-and-bound search**: cheap-first DFS with pruning by
   best-so-far cost.
7. **Position-as-decision-variable**: the outer loop enumerates task
   positions; the winning configuration has both positions and
   equipment chosen by the solver. The user declares only the
   workflow's intent, not its geometry.

These are the pieces we'll port back to tiny_cell with the real
throughput models, real catalogs, and 2-D geometry. The shapes stay
the same; the values become richer.
