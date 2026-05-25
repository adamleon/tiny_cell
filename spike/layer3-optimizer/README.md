# Layer-3 Optimizer Spike

Self-contained sandbox for the optimizer-library PoC required by
`engineering.md` §2.3 and `decisions.md` "Hand-rolled allocator at
Layer 2; library choice remains open for Layer 3":

> "The decision really hinges on **one question: which manager builds
> OR-Tools and IPOPT on Windows with least pain?** ... a proof-of-concept
> build is worth more than either of our priors here."

This directory is **isolated from the main TinyCell build** on purpose:
the spike is allowed to fail — a nasty native-build surprise must not
contaminate solver code that's already working.

## What it tests

Both candidates solve the same trivial constrained NLP:

```
minimize    x² + y²
subject to  x + y >= 5
```

Closed-form optimum: `(x*, y*) = (2.5, 2.5)`, `f* = 12.5`. Both smoke
tests assert this within tolerance.

- `src/ipopt_smoke.cpp` — IPOPT (interior point, `Ipopt::TNLP`)
- `src/nlopt_smoke.cpp` — SLSQP (sequential least-squares quadratic
  programming, via the NLopt library)

If a library doesn't build under vcpkg on Windows, the rest of the
spike still works — we record the failure and move on.

## How to reproduce

From this directory in a Developer PowerShell for VS 2022:

```powershell
# 1. Bootstrap vcpkg (first time only)
git clone https://github.com/microsoft/vcpkg.git
.\vcpkg\bootstrap-vcpkg.bat

# 2. Configure (pulls in ipopt + nlopt via the vcpkg.json manifest)
cmake --preset default

# 3. Build
cmake --build build/default --config Release

# 4. Run both smokes
.\build\default\Release\ipopt_smoke.exe
.\build\default\Release\nlopt_smoke.exe
```

## Outputs

- `FINDINGS.md` — decision write-up: which library, why, what hurt.
  Authoritative input to `decisions.md` once we lock the choice.
