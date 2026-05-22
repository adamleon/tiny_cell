# Standards Compliance  — ⚠ STUB, NEEDS YOUR INPUT

Standards are encoded as a **structured rules file** the solver reads (not hardcoded), parameterized per equipment category so the base set can be expanded. Distances are **hard constraints** in Layer 3 (`solver.md`) — the solver must refuse to return a violating solution, never trade compliance off against cost.

This stub needs (a) confirmation of which standards are in scope and (b) the actual numeric clearances, which depend on your jurisdiction, equipment, and whether cobots are involved. I can draft the schema; **I can't supply authoritative regulatory numbers** — those must come from the standards themselves or a safety engineer.

---

## Proposed base set (expandable) — confirm or adjust

- **ISO 10218-2** — industrial robot cell integration: clearances, safeguarded space.
- **ISO 13857** — safety distances to prevent reaching hazard zones (guarding).
- **ISO/TS 15066** — collaborative operation. *Only if cobots are in the catalog.*
- **Generic aisle-width / egress** rule.

**Deferred** (don't materially affect 2D layout): NFPA 79 / IEC 60204 (electrical). ATEX, food-safe, etc. — out of scope unless your domain requires them.

## Rules-file schema (draft — refine)

```
StandardRule {
  id,                       // e.g. "ISO13857.upper_limb_reach"
  applies_to:  [category],  // which equipment categories it constrains
  type:        CLEARANCE | AISLE_WIDTH | SAFEGUARD_ZONE | EGRESS | ...,
  value:       mm,          // the authoritative number — TO BE FILLED from the standard
  reference,                // citation for traceability
}
```

## What you need to fill in

1. Confirm the in-scope standard set (and whether cobots → include 15066).
2. Confirm jurisdiction (affects which regional standards apply — e.g. NA vs. EU).
3. Supply the authoritative numeric values per rule (from the standards or a safety engineer — do **not** let an implementation invent these).
4. Decide how rules map to geometry: a clearance is a buffer polygon around an equipment footprint; an aisle/egress is a free-corridor constraint across the floor. Specify each rule's geometric meaning so Layer 3 can enforce it.

## How standards enter the solver

Loaded as data → each rule becomes a hard constraint in Layer 3 placement: clearances inflate footprints into keep-out buffers; aisle/egress rules reserve free corridors; safeguard zones forbid overlap with operator areas. Infeasible-if-violated, never penalized-and-traded.
