# Layer-3 Optimizer Spike — Findings

**Date:** 2026-05-25
**Branch:** `feature/layer-3-placement`
**Scope:** The PoC required by `engineering.md` §2.3 and `decisions.md`
"Hand-rolled allocator at Layer 2; library choice remains open for Layer 3."
**Question asked:** which optimizer library builds + runs on Windows via
vcpkg, with what pain?

---

## Bottom line

**Recommendation: NLopt (SLSQP) for Layer 3, not IPOPT.**

Driven by the spike, not by ergonomic argument: vcpkg's `coin-or-ipopt`
port is **non-functional on Windows out of the box** (details below).
NLopt installs cleanly, exports a proper CMake config, and solves the
spike NLP to the analytic optimum.

Rest of `engineering.md` §2.3 framing carries over unchanged — vcpkg as
the dep manager (confirmed by this spike), FetchContent for threepp,
explicit version pinning in `vcpkg.json`.

---

## Numbers from this spike

| Metric | IPOPT (`coin-or-ipopt 2023-02-01#1`) | NLopt (`nlopt 2.10.1#1`) |
|---|---|---|
| Time to first build (vcpkg install + deps) | **~84 min** | bundled in the same install — incremental ≈ few min on top |
| Required toolchain | MSVC **+ MSYS2 + MinGW gfortran + autoconf/m4/perl** (~half a GB of MSYS2 packages) | MSVC only |
| Runtime DLL fan-out | `ipopt-3.dll`, `CoinUtils-0.dll`, `liblapack.dll`, `openblas.dll`, `bz2.dll`, `z.dll`, `libgfortran-5.dll`, `libgcc_s_seh-1.dll`, `libquadmath-0.dll`, `libwinpthread-1.dll` | `nlopt.dll` |
| CMake integration | None — ships `ipopt.pc` (pkg-config) only; needs `find_library` / `find_path` | Native CMake config; `find_package(NLopt CONFIG)` + `NLopt::nlopt` target |
| Solves the trivial NLP? | **No** (see below) | **Yes** — converges to (2.5, 2.5), f = 12.5, result code `NLOPT_FTOL_REACHED` |
| Port freshness | 2023-02-01 (3+ yr stale in this vcpkg snapshot) | 2.10.1 (fresh) |

---

## IPOPT failure modes encountered

In install order:

1. **Wrong port name.** vcpkg ships the port as `coin-or-ipopt`, not
   `ipopt`. The README in `engineering.md` §2.3 calls it "IPOPT" — the
   reader should know `vcpkg install ipopt` fails immediately.
2. **Long, heavy build.** ≈84 minutes wall clock. vcpkg downloads MSYS2
   to provision a Unix toolchain (autoconf, m4, perl, gcc-libs,
   gfortran, …) because IPOPT uses GNU autotools, not CMake. The
   resulting `ipopt-3.dll` was built by MinGW gcc/gfortran, not MSVC.
3. **Mixed-toolchain DLL boundary.** An MSVC-built consumer links to a
   MinGW-built IPOPT DLL through the C ABI. Functional, but the
   executable now needs four MinGW runtime DLLs
   (`libgfortran-5`, `libgcc_s_seh-1`, `libquadmath-0`,
   `libwinpthread-1`) on its DLL search path. Deployment story is worse
   than a pure-MSVC build.
4. **No CMake config exported.** Only pkg-config. Required
   `find_library`/`find_path` and a manual `target_include_directories`
   pointing at `include/coin-or`. Workable, not clean.
5. **No usable linear solver.** This is the hard stop.
   - Default `linear_solver=ma27` (and all HSL variants `ma27`, `ma57`,
     `ma77`, `ma86`, `ma97`) fail at runtime:
     `Error 126 while loading DLL libhsl.dll`. HSL is closed-source and
     vcpkg cannot redistribute it.
   - The FOSS fallback `mumps` is **not** in the runtime's
     `linear_solver` valid-settings list — vcpkg didn't build IPOPT
     with MUMPS support.
   - The remaining option `pardiso` requires a user-supplied Pardiso
     library at runtime.
   - **Net:** vcpkg's `coin-or-ipopt` can be *installed* but cannot
     actually solve anything without the user separately acquiring HSL
     or Pardiso. We could not run our trivial NLP through it at all.

To make IPOPT usable on Windows, viable paths are:

- **Patch the vcpkg port** to enable MUMPS. Maintenance burden + we go
  off the beaten path.
- **Acquire an HSL license** (free academic, commercial for products).
  Complicates redistribution if TinyCell ever ships.
- **Build IPOPT manually from source** with MUMPS enabled, bypassing
  vcpkg. Loses the dep-manager benefit.
- **Switch dep manager to Conan** to see if its IPOPT bundles MUMPS.
  Reverses the §2.3 dependency-manager tiebreaker.

None of these are appropriate effort for an MVP that needs a 2D NLP
over ≲ a few dozen rigid bodies.

---

## What NLopt looked like for the same problem

```cmake
find_package(NLopt CONFIG REQUIRED)
target_link_libraries(nlopt_smoke PRIVATE NLopt::nlopt)
```

```cpp
nlopt::opt opt(nlopt::LD_SLSQP, 2);
opt.set_min_objective(objective, nullptr);
opt.add_inequality_constraint(constraint, nullptr, 1e-12);
opt.optimize(x, f);  // x → (2.5, 2.5), f → 12.5
```

That's the entire integration surface. One DLL at runtime
(`nlopt.dll`). Compiled in seconds. Solves the smoke problem to the
analytic optimum on the first try.

NLopt also exposes a wide menu of other algorithms behind the same API
— useful as a hedge if SLSQP's local-minimum behavior bites later (we
can swap to a different `nlopt::algorithm` enum without changing the
problem formulation).

---

## Implications for Layer 3 build sequence

Phase A goal achieved with a clear answer: NLopt via vcpkg, SLSQP as
the starting algorithm.

Carry-overs into Phase B–G of the step-5 plan:

- **Phase A3 (decision write-up to `decisions.md`)** — the entry below
  is the canonical record; copy/refine and add a "Layer 3 NLP backend"
  bullet pointing at this file.
- **Phase D2/D3 (`solve()` seam):** keep the seam neutral.
  - NLopt-specific things (algorithm enum, ftol/xtol) stay behind the
    seam.
  - The problem statement (variables, constraints, objective) crosses
    the seam library-agnostic.
  - If SLSQP turns out to be inadequate at real scale, swapping to
    another NLopt algorithm (LD_MMA, LD_LBFGS for unconstrained
    subproblems, GD/GN globals) is a one-line change. Swapping to IPOPT
    later (under a different dep-acquisition strategy) is still
    possible *because* the seam is neutral — but is not the planned
    path.
- **Top-level CMakeLists.txt:** Layer 3 lands behind the seam, so the
  top-level build needs vcpkg integration. That's the **first time the
  main project takes a vcpkg dependency** (everything else is
  FetchContent). Introduce `vcpkg.json` at the repo root + adjust
  `CMakePresets.json` to include the vcpkg toolchain — separate
  commit, before any `solver/` code links nlopt.
- **CI:** add the spike's `~/.cache/vcpkg/archives` (or the equivalent
  binary-cache config) to the CI cache key so we don't pay 84 min on
  every clean build. NLopt is fast but anything bundled into the
  vcpkg.json will pull it through the same path. (NLopt alone takes
  only minutes.)

---

## Decision-log entry (draft for `decisions.md`)

> **Layer-3 NLP backend: NLopt (SLSQP), via vcpkg.** The PoC in
> `spike/layer3-optimizer/` confirmed NLopt installs and links cleanly
> on Windows via the vcpkg `nlopt` port (fresh, ships a CMake config,
> `NLopt::nlopt` target, single runtime DLL). The vcpkg `coin-or-ipopt`
> port is unusable on Windows out of the box: build takes ~84 min and
> pulls in an MSYS2 + MinGW runtime, and the resulting IPOPT has no
> usable linear solver (all HSL variants fail with
> `libhsl.dll not found`; MUMPS not enabled in the port build). NLopt
> is sufficient for our regime — Layer 3 is a few dozen rigid bodies in
> 2D — and exposes alternate algorithms behind the same API if SLSQP
> proves inadequate. The `solve()` seam stays library-agnostic so a
> future move (manual-build IPOPT with MUMPS, Conan, hand-rolled) is
> mechanical. *See* `spike/layer3-optimizer/FINDINGS.md`.

---

## What this spike did **not** test

Honest deferrals — flagged so we don't pretend they were resolved:

- **SLSQP on the actual Layer-3 problem.** A trivial QP says nothing
  about how SLSQP handles non-smooth overlap penalties, the
  positional-prior soft term, or several-dozen-variable problems with
  many active inequalities. The first time we hit real workloads we'll
  learn whether SLSQP convergence is acceptable; if not, swap the
  `nlopt::algorithm` and revisit.
- **Hand-rolled fallback.** Not built. The user's prior pattern
  (`decisions.md` Layer-2 entry) suggests hand-rolled is a viable
  fallback if NLopt also disappoints — the same `solve()` seam admits
  it without churn.
- **Binary cache for CI.** vcpkg has a binary-cache mechanism; we
  haven't wired it. Worth doing before CI runs IPOPT-flavored steps.
- **Toolchain pinning.** vcpkg baseline / port versions weren't pinned
  in this spike. The main repo's `vcpkg.json` should pin both via a
  `builtin-baseline` + per-port `version>=` to avoid version drift
  (`engineering.md` §2.3 "pin exact versions in a manifest/lockfile and
  commit it").
