# Interactive Editing

> **Status note:** the chosen stack (EnTT + threepp + ImGui, see `architecture.md`) is an interactive C++ application from day one, so editing is **implied by the architecture**, not a bolt-on. The *graded re-solve tiers* below (especially Tiers 2–3) are post-MVP refinements; basic drag + live constraint checking (Tier 0–1) arrives with the app. This doc still constrains how **Layer 3 must be built**: it must support **warm starts** and **partial freezing** from day one (far easier to design in than retrofit). See `solver.md` Layer 3.

The tool is meant to be played with: generate a solution, then let the user drag equipment and have the layout respond. A full cold solve is seconds-to-minutes — too slow for a drag, which needs sub-100 ms response. Resolution: **a manipulation invalidates only the cheapest layer whose assumptions it breaks; every layer above is reused untouched.** Re-solve scope is *graded* by what the drag actually changes.

## The tiers

| Tier | Trigger (example) | Invalidated | Re-runs | Target latency |
|---|---|---|---|---|
| **0 — free move** | Drag a camera along a belt; nudge an arm within its reach band | Nothing — same equipment, assignment, cost, feasibility | Live constraint check only (overlap / reach / clearance), red on violation | < 16 ms (per frame) |
| **1 — local geometry** | Move equipment enough to shift poses but not change *what* equipment exists | Layer 3 geometry, locally | Warm-started Layer 3, everything outside a radius **frozen**; pinned object held as hard pose | tens of ms |
| **2 — partial structural** | Drag the **output point**, or an endpoint a conveyor depends on | Segment equipment depending on the moved anchor (belt length/routing) | **Partial Layer 1** re-expansion of affected transport tasks + Layer 2 rebind of just those + local Layer 3 | ~100 ms – 1 s |
| **3 — full re-solve** | Drag a station **far enough** that strategy set, sharing, or flow topology plausibly change | Everything | Full LNS, **async in background**, offered as a suggestion | seconds – minutes |

## Trigger thresholds are explicit, not emergent

Decide *which* path to run **before** running it — don't start a cheap repair and discover it was insufficient.
- **Tier 2** fires when the moved object is an *anchor a segment-equipment depends on* (output point; station owning a conveyor endpoint).
- **Tier 3** fires when a station crosses a distance threshold that could change reachability-based sharing or flow monotonicity — e.g. it moves out of an arm's reach (breaking an instance-sharing assumption) or past a sibling (reordering flow). Below that threshold, the same station drag is Tier 1.

This pre-check (does the move break a reachability-based sharing assumption, or cross a sibling in flow order?) is its own small piece of work — don't underestimate it.

## Two override philosophies, both needed

- **Tiers 0–2 = "respect my placement, just repair around it."** User's pose is hard-fixed; the optimizer works around it.
- **Tier 3 = "reconsider everything."** Necessarily a re-optimization → runs in background, surfaces as an explicit suggestion (or behind a "re-optimize" button). **Never auto-apply a Tier-3 result** — silently rearranging the whole cell after the user moved one thing is hostile.

## Implication for Layer 3

Must accept: (a) a warm-start initial layout, (b) a set of hard-pinned poses, (c) a freeze radius / active-set so only a neighborhood re-solves. These are the **same mechanisms LNS destroy-and-repair already needs**, so the interactive path reuses the batch solver's internals rather than duplicating them.

## Drag tiers map onto the footprint cache levels

The graded tiers reuse the solver's two-level footprint cache (`solver.md` "Geometry flattening & caching", `data-model.md` §3.1) directly:

- **Dragging equipment *within* a station** = an **intra-station** invalidation: rebuild that station's `*_local` footprint, re-check against neighbors via `world_*`. This is Tier 1 (local geometry).
- **Dragging a *station*** = an **inter-station** invalidation: re-transform only that station's `world_*` footprint (one transform), re-check against neighbors; the station's internals are untouched. Tier 1 if it stays within thresholds; escalates to Tier 3 if it crosses a sharing/flow threshold (§ trigger thresholds).

So the same cache machinery and the same move-then-reorganize repair (`solver.md`) serve both batch LNS and interactive editing — no separate interactive geometry path.
