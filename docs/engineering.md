# Engineering Mechanics

The **how-it's-built** layer: toolchain, dependency acquisition, error-handling mechanism, and the test harness. The other docs describe *what* the system is and *why*; this one describes the machinery that compiles, fails, and verifies it. `CLAUDE.md` is the rules layer for code *content*; this is the rules layer for code *infrastructure*.

Read this when standing up the build, adding a dependency, deciding how something should fail, or writing a test. Sections marked **[OPEN]** are unresolved decisions — do not lock them in code; treat them like the `standards.md` stub and the `CLAUDE.md` open-decisions list.

---

## 1. Language standard

**Fixed: C++20.** Not a floor to drift upward from — the target is C++20, full stop. Newer features may be used *only* behind a feature-test macro with a C++20 fallback path, never as a hard requirement. Nothing in any module may *require* C++23+.

*Why C++20:* mp-units is the binding constraint and C++20 is sufficient to compile and use all of its functionality. It's also the realistic ceiling for the Windows requirement (§2.2) — MSVC's C++20 conformance is already shaky enough that reaching past it for C++23 features would multiply the portability pain. EnTT, threepp, glm, and Boost.Geometry are all comfortably C++20.

*Consequence for §3:* C++20 means **no `std::expected`** (it's C++23). The recoverable-error mechanism therefore resolves to either a typed boundary exception or a `tl::expected`-style polyfill — see §3.1. The default chosen here is the typed exception, avoiding the extra dependency.

*Portability constraint:* do **not** use C++ modules or `import std;`. mp-units supports modules only on Clang, and not at all on MSVC; with Windows as a day-one target, old-style header includes are the only portable choice.

---

## 2. Toolchain & build system

### 2.1 Build system

**CMake.** *Why:* every chosen dependency (mp-units, EnTT, threepp, OR-Tools, IPOPT, Boost) ships or assumes CMake; it is the path of least resistance for the whole stack and the only one with first-class support across all of them.

- Minimum CMake version: **3.28+**. *Why concrete now:* mp-units' recent CMake requires a modern version, FetchContent for threepp is mature well before this, and 3.28 is the floor for the C++ standard-library module handling you'd want available even though you're not using modules (§1) — it costs nothing to require and avoids a class of older-CMake friction. Bump only if a dependency's current release demands higher.
- One top-level `CMakeLists.txt`; each module (`core/`, `solver/`, `ecs/`, …) is its own target with explicit dependency edges. **The module-dependency graph in `CLAUDE.md` §0 is enforced here as target link rules** — `core` and `solver` link *nothing* external; foreign libs link only into `adapters`, `ecs`, `render`, `gui`. A link edge that violates the module index is a build-level bug, not just a convention. This is the cheapest possible enforcement of the §1/§2 prohibitions: make the wrong include fail to link.

### 2.2 Compilers — Windows is a day-one target

**Supported set: MSVC (VS 2022 17.4+ / toolset 194+), plus GCC 13+ and Clang 17+** as the conformance reference on Linux/macOS. Windows support is required from the start, so MSVC is in CI from the first commit, not deferred to a GUI milestone.

**The MSVC caveat you must internalize:** mp-units itself states MSVC has poor C++20 conformance. They patched the mp-units *library* code to compile on MSVC (so *using* the library on Windows works — that's all you need), but its own tests and examples still don't build there. The practical implications for your code:

- **Treat GCC/Clang as the conformance oracle.** Write to standard C++20; if MSVC chokes on advanced template / non-type-template-parameter code (the exact area mp-units hit bugs in), that's an MSVC bug to work around, not a signal your code is wrong. Don't let "it compiles on MSVC" be the bar — let "it compiles clean on GCC and Clang" be the bar, then make MSVC work.
- **Avoid Clang 19 and Apple-Clang 17.0** specifically — both have an unfixable compiler bug that prevents mp-units from building. Use Clang 17–18 or 20+, and Apple-Clang 15–16 or 17.1+.
- **No modules / no `import std;`** (already stated §1) — partly an MSVC portability constraint.
- mp-units' text formatting backend is settled in §2.4 (`std::format`, with a fmtlib fallback specifically for MSVC gaps).

The heavier Windows risk is **not** mp-units — it's OR-Tools and IPOPT native builds on Windows (§2.3), which is the real reason to settle the dependency manager early. The GUI stack (threepp/ImGui) is cross-platform and low-risk by comparison.

### 2.3 Dependency acquisition

**threepp is FetchContent — fixed.** It isn't packaged in vcpkg/Conan in a form you'd rely on, and it's a clean CMake-FetchContent consumer. Pin it to a specific commit/tag in `CMakeLists.txt`. This is settled regardless of what the *rest* of the dependencies use.

**Everything else: vcpkg — RESOLVED.** The Phase A spike at the start of step 5 (`spike/layer3-optimizer/FINDINGS.md`, decision recorded in `decisions.md` "Layer-3 NLP backend") validated NLopt on Windows via vcpkg — installs cleanly, ships a CMake config, single runtime DLL. Boost.Geometry similarly installs cleanly via vcpkg. The OR-Tools/IPOPT-on-Windows risk that originally motivated leaning-but-deferring is moot: IPOPT was rejected (vcpkg's `coin-or-ipopt` port has no usable linear solver out-of-the-box on Windows — HSL not redistributable, MUMPS not built in, Pardiso licensed); OR-Tools isn't needed yet either (Layer 2 is hand-rolled per `decisions.md`).

Net stack: **vcpkg manifest mode (`vcpkg.json` at the repo root)** for packaged libraries (nlopt, boost-geometry); **FetchContent for source-pinned libraries** (gsl-lite, mp-units, nlohmann_json, googletest, eventually threepp). Versions pinned via vcpkg `builtin-baseline` + per-port `version>=` AND FetchContent `GIT_TAG`. The vcpkg checkout is provided by the user via `VCPKG_ROOT` env var; `CMakePresets.json` references it as `$env{VCPKG_ROOT}`.

**Conan reserved as a fallback if vcpkg ever fails on a needed dependency.** Not needed at MVP; revisit if a future Layer-3 algorithm swap (e.g. real IPOPT-with-MUMPS) requires a port vcpkg can't carry.

**Pin exact versions in the manifest/lockfile and commit them.** This stack is version-sensitive — mp-units had a breaking V2 engine change and a V3 is on the horizon; an unpinned mp-units upgrade could break `core/` silently.

### 2.4 Text formatting backend

**Decision: `std::format`, with fmtlib as an MSVC-only fallback.** mp-units doesn't hardcode a formatting backend; it can use the standard library's `std::format` or the third-party fmtlib, selected by the `MP_UNITS_API_STD_FORMAT` build option. The two are near-identical by design — fmtlib is the library `std::format` was standardized from, same `{}` syntax — so this is "use the standard one or vendor the external one," not an architectural fork.

*Why `std::format`:* it's available across the entire pinned toolchain (MSVC 194+, GCC 13+, Clang 17+ — all already required in §2.2), so there's no availability gap to route around, and avoiding an added dependency beats fmtlib's marginal extras (colored output, etc.) for a layout solver that does no fancy terminal output.

*Why keep fmtlib in mind:* MSVC's `std::format` implementation has historically had rough edges. If a formatting bug or gap shows up specifically on the Windows side, flipping mp-units to fmtlib is a one-flag build-option change — far cheaper than fighting MSVC's `std::format`. Nothing to set up now; just know the lever exists. This is a low-stakes knob (worst case, one build-option line later — it never touches your code).

### 2.5 Mechanical style — tooling, not prose

Naming, formatting, include ordering, `const`-correctness, and the rest of the mechanical-style surface are enforced by **committed `.clang-format` and `.clang-tidy` config files**, not by a prose style guide. *Why:* machine-enforced rules don't rot and don't drift from the code; a prose convention duplicating a linter is waste that eventually contradicts the linter. CI runs both; a violation fails the build.

The only naming rules that live in *prose* (because a linter can't check them) are the semantic ones already in `CLAUDE.md`: invariant-carrying value types are named for their invariant (`PositiveLength`, `Angle`, …); `FrameId` is its own id space and is never spelled `entt::entity`. Those are correctness, not style — keep them in `CLAUDE.md` §2, not here.

---

## 3. Error handling — the three mechanisms

The design docs are emphatic and consistent: **never silently clamp a bad value** (`CLAUDE.md` §1, `decisions.md`). But "don't clamp" names three *different* failure situations that need three *different* mechanisms. Getting the mechanism right per situation is the point of this section.

| Situation | Origin | Recoverable? | Mechanism | Why |
|---|---|---|---|---|
| **Authored bad data** | user JSON / catalog (`io/`) | Yes — report and reject | **Reject at parse boundary with a diagnostic** (error value or typed parse exception carrying a message) | The user can fix their input; they need a message saying what and where, not a crash. Caught at the `io/` boundary (`CLAUDE.md` §0, §8). |
| **Solver-produced invariant violation** | a bug in our code (`solver/` emits a value that fails a validated-component invariant at the `sync/` boundary) | No — it's a defect | **Assert / throw** | This can never happen if the solver is correct; if it happens, the solver is wrong and must fail loudly, not limp on. (`CLAUDE.md` §1, §8; `decisions.md` validated-components entry.) |
| **Missing catalog spec / standards number** | absent input entry | Yes — report | **Reject as an input error; never fabricate** | A missing entry is an input error, not a default to invent (`CLAUDE.md` §1, `data-model.md` §3, `standards.md`). Same path as authored-bad-data. |

### 3.1 The split that matters

Two of the three rows (authored-bad-data, missing-spec) are **recoverable, user-facing, and caught at a boundary** (`io/`). One row (solver invariant violation) is **a bug, caught at the `sync/` boundary, and should be fatal in debug.** Same prohibition ("don't clamp"), opposite handling.

- **Recoverable / user-facing → diagnostic-carrying rejection via a typed exception.** A `ParseError` (or similar) carrying a message + source location, thrown at the `io/` boundary. *Why exception not return-value:* `std::expected` isn't available in C++20 (§1), and a `tl::expected` polyfill would be the only reason to add a dependency for this — not worth it when the error is caught at exactly one boundary (`io/`) and propagated to one place (the user-facing message). Parse errors are rare and terminal-to-the-parse, so exception cost is irrelevant. If the parse pipeline ever grows enough that exception-as-control-flow becomes awkward, revisit with `tl::expected` then.
- **Bug / invariant violation → assert + throw.** Construction-time invariant checks in the value types (`PositiveLength` etc.) throw on violation; this is acceptable precisely because validation sits at the `sync/` boundary, **not** in the LNS inner loop (`decisions.md`), so it never taxes the hot path. In debug builds, also `assert` so the failure is caught at the call site with a stack trace. A solver that produces an invariant-violating value is a defect to fix, not an error to handle gracefully.

### 3.2 Interaction with mp-units contract checking

mp-units does its own internal contract checking (backed by gsl-lite or ms-gsl). This is the *runtime* category of safety — value-dependent checks the type system can't catch, chiefly precision-losing or overflowing unit conversions and affine-space preconditions. (The famous compile-time dimensional checks are separate, always on, zero runtime cost — this toggle does not touch them.)

**Decision: keep contract checking ON in both debug and release.** *Why:* it is the same silent-wrong-value failure mode the entire project is built to prevent (`CLAUDE.md` §1, `decisions.md` validated-components entry) — a precision-losing conversion that slips through produces exactly the kind of quietly-wrong layout that is the worst outcome. The usual "off in release for speed" argument barely applies here, because the unit-stripping boundary discipline (§3 of `CLAUDE.md`: convert once in / once out at kernel boundaries) keeps mp-units operations *out* of the LNS inner loop and collision kernels by design — the checks fire at the `io/`/`sync/` boundaries where cost is irrelevant, not in the hot path. Turning safety off should require a profiler showing a specific check is measurably hot, not a pre-emptive guess; the burden of proof runs toward keeping it on.

**Pin it explicitly in the build config for both build types** — do not inherit mp-units' default. A future V3 upgrade could flip the default and silently change the safety posture; an explicit setting makes that impossible. This is the part that matters most: not which way it's set, but that it's set on purpose.

Do not let mp-units' contract mechanism and our value-type invariant mechanism diverge in style — they should read as one error-handling philosophy.

### 3.3 The one absolute

No path — recoverable or buggy — ever **clamps, defaults, or fabricates** its way past a bad value. Reject (input) or fail loud (bug). This is the single rule that unifies all three rows and is non-negotiable per `CLAUDE.md` §1.

---

## 4. Testing

The design treats specific tests as **load-bearing correctness guarantees**, not optional extras. Two are named directly in the docs and are mandatory:

- **Frame-composition regression (`CLAUDE.md` §2, `architecture.md` §4.3):** a sensor at `(belt_length/2, 0, 0)` in a belt frame rotated 90° must land along the belt's *rotated* axis. This guards the #1 frame bug (triplet-addition instead of transform composition). Required.
- **Render round-trip identity (`CLAUDE.md` §2, §7):** `Pose2D → threepp → Pose2D` must be identity within epsilon. Guards the axis/handedness conversion. Required once `render/`/`adapters/` exist.

### 4.1 Framework

**GoogleTest.** You already know it, it integrates cleanly with CMake/CTest, and its built-in mocking (gmock) is there if the `sync/`/solver-seam work later wants to mock the abstract `solve()` boundary (§9 of `CLAUDE.md`). One framework, no mixing.

Compile-time checks (dimensional-correctness "this must not compile" cases) sit outside GoogleTest — use `static_assert` in dedicated compile tests, since a wrong-dimension operation failing to compile can't be expressed as a runtime assertion.

### 4.2 What must be tested, by layer

- **`core/`:** value-type invariants (rejection on length ≤ 0, etc.); frame-transform composition (the §4.3 regression above); unit dimensional correctness (these are largely compile-time via mp-units, but include `static_assert`-style checks that wrong-dimension operations fail to compile).
- **`solver/`:** the brute-force enumerator over 3–5-task problems (`roadmap.md` step 2) *is* a test harness — it validates the strategy library produces sensible plans before any metaheuristic can hide bad engineering. Treat it as such. Plus: footprint-cache invalidation logic (dirty-flag transitions, hull-vs-union lazy build), and seed-reproducibility (same seed → same `LayoutSolution`).
- **`adapters/`:** the round-trip identity test (§7) per adapter; unit strip/reattach correctness.
- **`sync/` (post-MVP):** FrameId stability across re-solve (surviving IDs not renumbered, `CLAUDE.md` §1); atomic apply (all-or-nothing); ordering (frames → poses → transfers → metrics).

### 4.3 CI

Every commit: build under the supported compiler set (§2.2), run clang-format/clang-tidy (§2.5) as a gate, run the full test suite via CTest. The link-rule enforcement of the module graph (§2.1) means many `CLAUDE.md` prohibitions are caught at compile/link time before tests even run — lean on that.

---

## 5. Decisions — resolved and remaining

**Resolved (this revision):**

- **Language standard:** C++20, fixed. No modules, no `import std;` (§1).
- **Build system:** CMake 3.28+ (§2.1).
- **Compilers:** MSVC 194+ / GCC 13+ / Clang 17+ (avoiding Clang 19 and Apple-Clang 17.0); Windows in CI from day one; GCC/Clang as the conformance reference (§2.2).
- **Dependency manager:** vcpkg manifest mode for packaged libraries (nlopt, boost-geometry); FetchContent for source-pinned libraries (gsl-lite, mp-units, nlohmann_json, googletest, threepp). Settled by the Phase A spike at start of step 5 (§2.3, `decisions.md` "Layer-3 NLP backend").
- **Layer-3 NLP backend:** NLopt — algorithm choice is local to `solver/src/layout_problem.cpp` behind the `solve()` seam (currently `LN_BOBYQA`; `decisions.md` "Layer-3 algorithm").
- **threepp:** FetchContent, pinned to a tag (§2.3).
- **Test framework:** GoogleTest (§4.1).
- **Recoverable-error mechanism:** typed `ParseError` exception at the `io/` boundary; invariant violations assert/throw at the `sync/` boundary (§3).
- **mp-units contract checking:** ON in both debug and release, pinned explicitly in the build config (§3.2).
- **Text formatting backend:** `std::format`, with fmtlib as a one-flag MSVC fallback (§2.4).

**No remaining [OPEN] infrastructure decisions at MVP.** Layer-3 library + algorithm choices are local to one file behind the `solve()` seam and can be revisited without disturbing the rest.

---

## 6. MVP scope note

Consistent with `roadmap.md` and `CLAUDE.md` §6: the first milestone is **batch solver + `core/`, no GUI**. For *this* doc that means:

- **Windows is still day-one** (§2.2) even though the GUI is post-MVP — `core/` and the solver must build and test on MSVC from the first commit. The GUI being deferred does *not* defer Windows; it only defers threepp/ImGui.
- The MVP dependency surface is small: mp-units, Boost.Geometry, glm. EnTT/threepp/ImGui arrive with later modules. So the vcpkg-vs-Conan question (§2.3) can be settled on the *light* dependencies now; the OR-Tools/IPOPT spike that's the real decider waits until Layer 3.
- §4.2 `sync/`/`render`/`adapters` tests bind *when those modules are built*, not in milestone one. The §4.2 `core/` and `solver/` tests — value-type invariants, frame composition, the brute-force enumerator as a strategy-library validator — are milestone-one work.
- The language + error-handling decisions (§1, §3) and CMake/compiler setup (§2.1, §2.2) **block the first line of `core/`** — they're settled above precisely so they don't.
