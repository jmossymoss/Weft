# Foam drum spans — sparse-fold keep (2026-07-22)

## Problem

Foam CAD's tall body (`Drum×FullPeriod` revolution) built a clean
annulus-body column grid with one inverted cell, then lost the fold
self-heal tournament to a zero-fold contract floor. Visually: large
cylinder → triangle soup. Other faces still hit planned / sphere-bowl
floors (MVP-legal when editable).

## Class fix

In `meshers.cpp` fold self-heal (invertedCells + foldedPolys paths):

- When `RevolutionGrid` + `FeatureClass::Drum` has sparse folds
  (`1..8` and `folds*4 <= polyCount`), refuse contract-floor swap.
- SphereCap / FilletStrip intentionally excluded (WP5: sphere skip
  reopens foam seams; teleporter FS×FullPeriod ships hundreds of folds).

## Evidence

| Check | Result |
|---|---|
| Foam CAD | watertight; body drum off floor (~183 quads + n-gon notch strip) |
| Teleporter CAD | watertight; fold count unchanged vs pre-fix |
| `testSparseFoldKeepsStructuredCharts` | extract + full foam |
| Golden | foam CAD 6115q / 1046t / 304n (intentional) |

Screenshots: `foam_v0.png` … `foam_v2.png` (app `--finalize`).

Residual floors (planned freeform/drum bands, sphere-bowl fold heals,
top fillet strip) remain — follow-ups, not this class.
