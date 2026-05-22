# Decision Log

One entry per resolved design choice + the one-line *why*. Insurance against silently reversing a deliberate choice. When something seems odd, check here before "fixing" it.

---

**Candidate binding, not commitment.** Strategies propose a specific equipment model but as a candidate; the allocation layer (Layer 2) decides instance sharing. *Why:* committing to an instance inside the strategy makes cross-station equipment sharing impossible — two strategies would each spawn their own arm.

**Guards vs. preconditions are distinct.** Guards = unsolvable pass/fail predicates (reject immediately); preconditions = solvable sub-goals (spawn a child Task). *Why:* prevents the solver from trying to spawn equipment to fix unfixable geometry (e.g. an item that simply can't fit the pallet pattern).

**Capex on the catalog entry, not the strategy.** *Why:* same strategy + same model = same price regardless of task; storing it on the strategy duplicates and risks drift. Energy stays on the strategy because it depends on the task's required motion.

**Energy stored in joules, displayed in kWh.** *Why:* joules is an amount (composes cleanly, folds in duration); watts is a rate (can't cost without separately tracking time). kWh is the priced, human-legible unit — convert only at the cost/UI boundary. 1 kWh = 3.6 MJ.

**Standby power is a separate energy term.** *Why:* idle equipment still draws power; the `standby_power × idle_time` term penalizes underutilization, reinforcing the sharing objective.

**Strategies matched by capability (`applies_to`), not by task name.** *Why:* one strategy (e.g. a pusher) legitimately applies to multiple task kinds; name-wiring blocks that reuse.

**Item state = fixed small vector, forward-propagated.** *Why:* a full predicate/effect planner is scope creep; a fixed vector (pose/carrier/orientation) handles the real feasibility-gating cases for MVP.

**Orientation modeled as symmetry-aware precision.** *Why:* you don't need a bottle's rotation; a square box only mod 90°. Reducing required precision by symmetry avoids over-speccing sensing equipment.

**Standards are data (rules file), enforced as hard constraints.** *Why:* expandable base set without code changes; compliance must not be traded off against cost.

**2D geometry for MVP, reachability-first then 30–70 % reach band.** *Why:* 3D + IK-in-the-loop is a different, much harder beast; 2D footprints + reach circles are tractable and sufficient for first layouts.

**Approximate optimum acceptable (LNS + simulated annealing).** *Why:* true global optimum on a mixed discrete-continuous (MINLP) problem is intractable at useful scale; the requirement explicitly allows "approximately optimal."

**Lexicographic partial-handling (provisional).** Maximize fully-solved tasks, then minimize cost. *Why:* simple and matches "full solution prioritized over partial regardless of cost." Flagged provisional — can miss A-frees-resource-for-B cases.

**Graded interaction over single re-solve path.** A drag invalidates only the cheapest layer it breaks. *Why:* full re-solve per drag is far too slow for interactivity; tiered scope keeps free moves at frame rate and reserves full LNS for moves that genuinely change structure.

---

## Architecture decisions

**PARTIAL feasibility + precondition spawning staged to step 4, with the analytic throughput model.** *Why:* PARTIAL is the spec's mechanism (`data-model.md` §2) for "this strategy can serve some but not all of the task" — it returns `feasibility=PARTIAL` with `partial_info: { achievable_ct, target_ct }` and emits a residual `Task` in `preconditions` for the remaining work. This is how the OR-tree captures "1× big arm covers the task FULL" vs "3× small arms chained via PARTIAL" — *not* via multiple candidate bindings per `evaluate` call. The mechanism is only honest with a real throughput model (`achievable_ct` is not invent-able per `engineering.md` §3), and the throughput model is itself staged to step 4 along with cycle-time-as-hard-constraint. Step 1's `ArmStrategy` therefore returns only FULL or INFEASIBLE; the `PARTIAL` enumerator stays in `Feasibility` so the type doesn't change at step 4. Step 2's brute-force enumerator does not need PARTIAL — it validates strategy *matching* and *equipment selection* on tasks that fit FULL in a single binding.

**Strategies named `<EquipmentType>Strategy`, not `<EquipmentType><TaskKind>Strategy`.** *Why:* strategy-class reuse depends on one strategy applying to several task kinds (`ArmStrategy` covers `Palletize`, `Transport`, `Assemble`; `PusherStrategy` covers `Palletize`, `PushOff`). Naming a strategy after a single task forecloses that reuse and forces a combinatorial explosion of classes. Tasks must in turn be shaped equipment-agnostically (`Palletize(item, pallet, count)`, not `PalletizeWithArm`) — the OR-node fan-out in Layer 1 (`data-model.md` §2) only works if many strategies legitimately apply to one task. When tempted to name a strategy after a task, redesign the task or the strategy first.

**Per-category catalog loaders, not polymorphic dispatch.** *Why:* every consumer in the current design reads catalogs per category — strategies match tasks by capability and only consume their own category; asset files are already split per category (`assets/<type>/<vendor>/catalog.json`). A `std::variant<ArmSpec, GripperEntry, …>` (or class hierarchy) buys exhaustive-match safety only where a heterogeneous walk exists; we have none. Revisit when a consumer emerges that genuinely iterates over multiple categories in one pass — most likely candidate is `sync/` writing all equipment into the ECS registry (post-MVP, `architecture.md` §11). Refactor A→variant is mechanical at that point and we'll know which categories actually need to be in the variant. C++ class hierarchy with virtual dispatch (Option C) is rejected outright: equipment entries are pure data, behavior lives on strategies (`data-model.md` §2).

**Validated value types staged in at step 4 / Layer 2 allocation, not step 1.** *Why:* the wrapper layer (`PositiveLength`, `Payload`, `ReachRadius`, … — `architecture.md` §8) protects against *solver-produced* invalid values. In step 1 the only producer is the JSON loader, which validates at the `io/` boundary anyway; an extra wrapper layer is ~80 LOC enforcing nothing the loader doesn't already enforce. Step 4 (Layer 2 allocation) is the first point where solver code can violate a range invariant — that's when wrappers earn their cost. ArmSpec uses raw mp-units quantity types until then; promoting fields to wrapped value types is a mechanical per-field refactor when the time comes. (Reverses nothing in `architecture.md` §8 — that section describes the end state; this entry sequences when the layer is introduced.)

**Catalog & workflow input format: JSON, not YAML.** *Why:* nlohmann/json is already part of the stack via the io/ parser, has stronger Windows/MSVC traction than yaml-cpp, and avoids adding a second text-config dependency. Authored bad data is still rejected with a typed `ParseError` at the `io/` boundary (`engineering.md` §3); the format choice is independent of the error mechanism.

**C++ throughout; interactive app from day one.** *Why:* EnTT + threepp + ImGui is a C++ GUI stack; OR-Tools and IPOPT are natively C++. No headless-CLI-first phase — interactivity is implied by the stack.

**Solver is a pure module on its own structs, syncs into the EnTT registry.** *Why:* LNS explores thousands of discarded candidates; mutating the live registry per candidate would thrash the renderer and couple search to ECS identity. The `sync/` step is the boundary.

**Frames have their own id space, not entity ids.** *Why:* the solver creates/discards candidate frames freely and may reference a frame before any entity is committed; one entity can both be-placed-in and define a frame (belt + its sensor). Coupling frames to `entt::entity` would drag the solver into ECS identity space.

**Frame-in-pose kept, but as FrameId.** *Why:* the original "pose carries its frame" instinct was right; the fix is the frame is a `FrameId`, never an entity id.

**Frame resolution composes transforms, not (x,y,θ) triplets.** *Why:* a parent's rotation rotates the child's offset; triplet addition silently mis-places rotated children (the rotated-belt/sensor bug).

**Geometry abstraction: abstract stored types, not algorithms; one adapter per foreign lib.** *Why:* makes Boost/glm/threepp swappable and core testable; foreign types leaking outside adapters would kill swappability. Convert at boundaries, never mid-kernel (performance).

**mp-units wired into the geometry types; units stripped only in adapters.** *Why:* compile-time dimensional safety (the joules-vs-watts class of bug); confining unit-stripping to adapters keeps stray unitless numbers out of core. Units and frames are orthogonal guards.

**Core/solver Z-up meters; Z-up↔Y-up confined to render adapter; handedness locked after confirming threepp.** *Why:* axis bugs are invisible until something renders mirrored; isolating the conversion to one file + a round-trip test contains them.

**Validated components via constrained value types, not public-field PODs.** *Why:* invalid geometry (length ≤ 0 etc.) produces silently-wrong layouts — the worst failure. Invariants pushed into value types (`PositiveLength`…) layered on mp-units, written once per concept; construction-time + immutable where possible; throw/assert never silent clamp; doesn't tax the solver because validation is at the sync boundary, not the LNS loop.

**Optimizer libraries (Layer 2/3) left open.** *Why:* they sit inside the solver and never touch core stored types, so the choice has no architectural cost; defer pending validation.

**Two-level lazy footprint flatten/cache (station-local, then world).** *Why:* intra-station and inter-station placement are independent problems; flattening at the station-footprint granularity means a station move is one polygon-transform, not a re-flatten of its equipment. World transforms touch only the moved station's single cached polygon. Keeps the LNS inner loop and interactive drag fast. Hull (broad-phase reject) + union (narrow-phase) cached together; clearance buffered in once at build time, never per collision check. Repair tries cheap inter-station nudge before expensive intra-station reorganization. See `solver.md`.
