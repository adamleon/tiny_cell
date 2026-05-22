# Roadmap

**Plan, not spec.** This sequences the build and defines milestones; it points into the design docs rather than restating them. Design docs describe the *end state* and don't change as you build; this file describes the *path* and gets checked off.

---

## Build order

1. **Equipment catalog + strategy library** (`data-model.md` §2–3). Rigorously type what each strategy consumes (`requires_state`, guards, structural preconditions) and produces (`effect`, equipment, energy). The solver is generic; this domain knowledge determines whether output is usable. **Do this first.**
2. **Brute-force enumerator** over tiny problems (3–5 tasks) to validate the strategy library produces sensible plans before any metaheuristic can hide bad engineering.
3. **Item-state propagation** pass over the workflow DAG (`data-model.md` §4) with the MVP-scoped state vector.
4. **Layer 2 allocation** — assignment/packing with sharing (`solver.md` Layer 2).
5. **Layer 3 placement** — 2D NLP with positional-prior seeding (`solver.md` Layer 3 + positional prior). Build warm-start + partial-freeze support now (needed by both LNS and future interaction).
6. **LNS + annealing** outer loop tying it together (`solver.md` outer loop).

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
