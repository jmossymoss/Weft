# WP6 — multi-tooth open-band tooth-wall folds (2026-07-23)

## Class

Dense multi-tooth open-band drums (ABC `00008536` face 57 / reducer
`notched_drum_iso_band_r0`): after mid-split / side-clamp, neighbouring
notches share a kept column whose U sits past each tooth's true wall
(`slotU0` / `slotU1`). Notch web outer rails on those columns UV-overlap
the neighbour's opposite wall (and the inter-tooth land lattice). On
REVERSED cylinders the left-wall ribbon winds opposite the lattice →
~32 local Newell folds under sparse-fold protect.

No filename / face-id specials. No new `MesherKind`. Authoritative path:
`weft::generate()` / open-band RevolutionGrid.

## Fix

In the open-band notch web ribbon:

1. When `slotU0 < uk[colL]` (or `slotU1 > uk[colR]`), build the outer
   vertical at the slot U iso-line instead of the bounding column so each
   wall stays inside its own slot.
2. Iso-U outer on an iso-U cut wall yields zero-UV-area spans. Fan those
   cut edges to the notch's feature-row apex with lattice winding so
   border contract keeps the wall edges and Newell stays non-folded.
3. Any remaining positive-UV-area ribbon cell is reversed to match the
   lattice hand.

## Metrics (CAD profile)

| Asset | Before | After |
| --- | --- | --- |
| `notched_drum_iso_band_r0` | 385 polys, 32 folds, sparse-fold keep | 384 polys, 0 folds, build=0; slivers 12→33 |
| `testNotchedDrumOpenBand` | `nFolded < 50` | `nFolded == 0` |

Sphere×fillet reducer / `testSphereFilletFullPeriodNoFloor` unchanged
(0 floors, 0 folds).

## Test

`testNotchedDrumOpenBand` in `tests/test_pipeline.cpp`.
Reducer: `tests/regressions/abc/notched_drum_iso_band_r0.step`.
