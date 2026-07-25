# WP6 — multi-tooth open-band tooth-wall folds (2026-07-23)

## Class

Dense multi-tooth open-band drums (ABC `00008536` face 57 / reducer
`notched_drum_iso_band_r0`): with only ~4 columns per tooth, mid-split
parks neighbouring notches on a shared kept column past each tooth's
true wall. Left-wall web ribbons then UV-overlap the neighbour wall /
land lattice. On REVERSED cylinders that yields ~32 local Newell folds
under sparse-fold protect.

No filename / face-id specials. No new `MesherKind`. Authoritative path:
`weft::generate()` / open-band RevolutionGrid.

## Fix

Raise multi-tooth open-band `nu` (and the plain `bandDriver` densify that
keeps `passPlain`) from `4 * notches + 2` to `12 * notches + 2` so each
inter-tooth land keeps its own column after pad / mid-split. Slot-U
outer-rail experiments cleared folds on the open-shell reducer but opened
~210 edges on the closed ABC solid — densify preserves watertightness.

## Metrics (CAD profile)

| Asset | Before (4×) | After (12×) |
| --- | --- | --- |
| `notched_drum_iso_band_r0` | 385 polys, 32 folds, nu=102 | 914 polys, 0 folds, nu=302; slivers 12→10 |
| ABC `00008536` whole | WT, 32 folds face 57, 136 slivers | WT, 0 folds, 134 slivers; face 57 RevolutionGrid |
| `testNotchedDrumOpenBand` | `nFolded < 50` | `nFolded == 0` |

Sphere×fillet reducer / `testSphereFilletFullPeriodNoFloor` unchanged.

## Test

`testNotchedDrumOpenBand` in `tests/test_pipeline.cpp`.
Reducer: `tests/regressions/abc/notched_drum_iso_band_r0.step`.
ABC nightly: `WEFT_RUN_PUBLIC=1 tools/public_corpus_gate.sh abc`.
