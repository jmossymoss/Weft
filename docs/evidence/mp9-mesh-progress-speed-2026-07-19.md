# Evidence — MP9 mesh progress + interval speed (2026-07-19)

## Problem
App showed **meshing 0 / 4270** for minutes with no advancement; felt hung.
Root causes: (1) no progress during intervals/boundaries, (2) O(n²)
ownership scans in `buildIntervalProblem` (~55s).

## Fix
- Progress phases: edge intervals → boundaries → faces N/M
- `facesByEdge` index; preview circle/ellipse without per-edge OCCT radius
  eval; plane-owned circles keep ≥16 samples
- Parallel planes remain disabled (OCCT not thread-safe)

## Timing (CLI MP9, this machine)
| Stage | Before | After |
|-------|--------|-------|
| intervals.problem | ~55 s | ~0.3 s |
| intervals.solve | ~0.01 s | ~0.01 s |
| boundaries | ~28–30 s | ~28 s (still OCCT sample-bound) |
| faces | ~50 s | ~45 s |
| **total mesh** | ~2–6 min | **~80 s** |

## Still needed for “few seconds”
- Speed/simplify `buildCanonicalBoundaries` sample evaluation
- Safe parallel face templates or per-solid workers
- Warm cache (already in app) for ms regenerate
