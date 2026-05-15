# HTN sandbox

Throwaway program to make the HTN solver architecture **legible** before we
build it for real in tiny_cell. Toy domain: 1-D world, three task kinds,
four strategies. Search algorithm prints every decision to stdout so you can
follow what it's doing.

## Run

After building (see project root CMake):

```
./build/Debug/htn_sandbox.exe                            # uses default input.json
./build/Debug/htn_sandbox.exe path/to/other_input.json   # use a different file
```

Edit `input.json`, re-run the exe — no rebuild needed.

## What's in the toy domain

- **Tasks**: `Palletize`, `Transport`, `DetectPresence`.
- **Strategies**:
  - `ArmStrategy` solves `Palletize` directly (no sub-tasks). Returns one
    proposal per `N` in 1..`max_count`. Each proposal is terminal.
  - `PushStrategy` solves `Palletize` with a pusher + emits sub-tasks for
    `Transport` and `DetectPresence`.
  - `BeltStrategy` solves `Transport` directly. Cost scales with distance.
  - `LaserStrategy` solves `DetectPresence` directly.

This is enough to exercise:
- Multiple strategies competing for one task (Arm vs Push for `Palletize`)
- Sub-task chaining (Push → Transport → Belt, Push → DetectPresence → Laser)
- Branch-and-bound search (best-first by lower bound, terminate when best
  complete solution ≤ all open partials' lower bounds)

## What to try

Default values: pusher cheap (€500), belts cheap, lasers cheap. Push wins.

1. **Make the pusher expensive.** Set `pusher_cost_eur` to 2000. Re-run.
   Arms win because the pusher's total package no longer competes.

2. **Lengthen the source distance.** Set `source_distance_m` to 10.0. Belt
   cost goes up linearly (€100 base + €50/m × 10 = €600 belt alone). Push
   total approaches arm total. Tweak till the winner flips.

3. **Cheap belts.** Set `per_metre_eur` to 5. Belts become trivial; Push
   wins even at huge distances.

4. **Stress arm count.** Set `rate_per_minute` to 8.0 (above one arm's
   capacity at default `max_rate_per_minute=10`). N=1 still works. Bump to
   20 (above one arm's capacity) — N=2 forced. Bump to 50 — infeasible for
   arms (`max_count=3` × 10/min = 30/min). Push may still be feasible.

5. **Push capacity limit.** Set `push_max_rate_per_minute` to 5 and ask for
   `rate_per_minute=8`. Push declares itself infeasible — only Arms remain.

The trace shows you exactly what each strategy proposed at each step, what
the search picked, and why.

## Reading the output

```
=== Initial dispatch on root task: Palletize ===
  [ArmStrategy(N=1)        ] cost=1000.0  lb= 1000.0  TERMINAL
    + 1×Arm               €1000.0
  [PushStrategy            ] cost= 500.0  lb=  650.0
    + Pusher              €500.0
    ? needs Transport
    ? needs DetectPresence
```

- **`cost`** = sum of equipment cost realised so far in this proposal
- **`lb`** = lower bound on this proposal's total cost (own + minimum-possible
  remaining). Used by the search to pick what to expand next.
- **`TERMINAL`** = no unresolved sub-tasks; this is a complete solution.
- **`? needs X`** = sub-task this proposal emitted; must be dispatched to
  another strategy.

The search picks the proposal with the lowest `lb`. If it's terminal and no
other partial's `lb` is lower, that's the winner. If it's a partial, the
search dispatches one of its `? needs` sub-tasks to all matching strategies,
splices their results back in, and continues.

## What this is NOT

- Not production code. Will be deleted (or moved to `archive/`) once the
  real solver in tiny_cell is built.
- Not a complete model. No geometry, no real cost model, no catalog of
  KUKA arms, no feasibility checks beyond a rate gate. Single hardcoded
  "pusher" and "laser" with no variants.
- Not a complete search algorithm. The lower-bound bookkeeping in `splice()`
  is approximate (we subtract the budgeted floor and add the realised cost).
  Good enough to demonstrate the loop; a real implementation would track
  remaining-floor more carefully per sub-task.

## What we learn

After running this with several different inputs:

- The shape of `Proposal` (equipment, cost, lower bound, remaining sub-tasks)
- The shape of `Strategy` (`name`, `can_solve`, `propose`)
- The mechanics of `splice` (combining a child proposal into a parent partial)
- The mechanics of best-first search with lower-bound termination

Whatever survives this 400-line file in spirit is what we'll port back into
tiny_cell. Whatever doesn't was wrong.
