# Roadmap

**Plan, not spec.** This sequences the build and defines milestones; it points into the design docs rather than restating them. Design docs describe the *end state* and don't change as you build; this file describes the *path* and gets checked off.

---

## Build order

1. **Equipment catalog + strategy library** (`data-model.md` §2–3). Rigorously type what each strategy consumes (`requires_state`, guards, structural preconditions) and produces (`effect`, equipment, energy). The solver is generic; this domain knowledge determines whether output is usable. **Do this first.** *Notes:* catalog structs use raw mp-units quantity types in step 1; range invariants are enforced in the JSON loader at the `io/` boundary. Validated value-type wrappers (`architecture.md` §8) are staged in at step 4 — see `decisions.md`. **PARTIAL feasibility + precondition spawning are skipped here** and staged to step 4 along with the analytic throughput model (also `decisions.md`); step 1 strategies return only `FULL` / `INFEASIBLE`.
2. **Brute-force enumerator** over tiny problems (3–5 tasks) to validate the strategy library produces sensible plans before any metaheuristic can hide bad engineering. *Note:* the enumerator loads each catalog via its per-category loader (`decisions.md` — per-category dispatch); strategies match tasks by capability and consume only their own category.
3. **Item-state propagation** pass over the workflow DAG (`data-model.md` §4) with the MVP-scoped state vector.
4. **Layer 2 allocation** — assignment/packing with sharing (`solver.md` Layer 2). **Introduce validated value-type wrappers (`architecture.md` §8) for the fields Layer 2 can produce or mutate**: this is the first point where solver code can emit a value that fails a range invariant (per `decisions.md`). Grow the wrapper set field-by-field as Layer 2 touches them — don't wrap fields no solver code consumes yet. **Introduce the analytic per-archetype throughput models** (`belt`, `picker`, `palletizer`-style functions giving items/min for a given parameter set) **and turn on PARTIAL handling** in strategies that can serve a task partially: `evaluate` emits `feasibility=PARTIAL` with `partial_info` and a residual sub-task in `preconditions`, enabling the "n× small equipment chained" branch of the OR-tree (per `decisions.md`). These two are tightly coupled — PARTIAL can't be computed honestly without the throughput model.
5. **Layer 3 placement** — 2D NLP with positional-prior seeding (`solver.md` Layer 3 + positional prior). Build warm-start + partial-freeze support now (needed by both LNS and future interaction).
6. **LNS + annealing** outer loop tying it together (`solver.md` outer loop).

## Supporting tools

Built incrementally alongside the build steps, not separate milestones.
Add capability when the layer it serves first needs it.

- **`svg/` — 2D vector export of solver output.** Debug oracle for every
  solver layer that carries geometry. Pure C++ stdlib (no new
  dependencies); browser/phone viewable; diff-friendly, so SVGs can
  serve as CI regression artifacts.
  - Step 1: each catalog entry's footprint + (for arms) reach envelope —
    visual sanity check on `load_*_catalog`.
  - Step 4: station-local accumulated footprints (hull + union per
    `data-model.md` §3.1) — verify intra-station packing before
    inter-station placement runs.
  - Step 5: the headline use — full `LayoutSolution` in world coords
    (equipment polygons, transfer arrows, clearance buffers, floor
    bounds).
  - Step 6: same as step 5; before/after pairs make LNS regressions
    visible by diff.

  Distinct from `render/` (threepp, post-MVP, interactive 3D, reads from
  the EnTT registry) — `svg/` reads solver-internal structs + catalogs
  directly and emits text. The two coexist after MVP: SVG for batch /
  CI / diffs, threepp for interactive editing.

## MVP milestone — "done" means

- Catalog + strategy library loadable and validated.
- Given a workflow + blueprint + catalog + cycle-time target, produces a `LayoutSolution` (`data-model.md` §5): equipment list, poses, explicit transfer graph.
- Hard standards constraints enforced in placement (`standards.md`).
- Seed-reproducible.
- **Not** in MVP: 3D, full simulation, BOM/costing report, GUI, interactive editing (`interaction.md`).

## Open questions (resolve as you reach them)

- **Energy model granularity:** per-motion physics, or flat `power_draw × occupancy_time` for MVP? (Latter likely enough initially.)
- **Cycle-time allocation:** when the user gives only a whole-cell target, how is it split across stations for Layer 2's capacity packing? Even split (wrong — stations differ in work content), bottleneck-driven, or solver-decided (turns allocation into a min-max balancing problem)? **Resolve before building Layer 2.**
- **Conflict learning:** should resisted Layer-3 conflicts feed a no-good list to prune Layer 1/2 search, or is plain LNS re-sampling enough for MVP?
- **Partial-solution scoring:** lexicographic is provisional (`solver.md`). Revisit when partial cases appear in practice.

## Known limitations / deferred

- Strict lexicographic partial-handling can miss A-frees-resource-for-B cases. Accepted for MVP.
- Item state is a fixed small vector, not a general planner. Generalize only on demand.
- Symmetry limited to z-rotation + flippable. Full SO(3) deferred.
- 2D geometry, no IK. 3D deferred.
- Approximate optimum only — LNS/annealing gives no global guarantee (acceptable by requirement).
