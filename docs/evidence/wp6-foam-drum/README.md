# Foam drum / fillet / wedge floors (2026-07-22 → 2026-07-23)

## Problem

Foam CAD residual floors after the tall-body drum sparse-fold keep:

1. Top `FilletStrip×FullPeriod` (face 184) — coons with ~5 folds lost the
   soft floor tournament.
2. Sphere bowls (314/315/322/681) — soft fold self-heal → contract floor.
3. Planned drum wedges (501/505/510/525) — open-band bail (one meridian)
   then coons `no clear fourth corner` / `interior probe outside`.

## Class fixes

### Sparse-fold keep (invertedCells + foldedPolys)

Refuse contract-floor swap when folds are sparse (`1..8` and
`folds*4 <= polyCount`) for:

- `RevolutionGrid` + `FeatureClass::Drum`
- `CoonsGrid` + `FilletStrip` + `FullPeriod`

Absolute fold cap keeps teleporter's heavily-folded FS×FullPeriod faces
on the floor (74–139 folds). SphereCap intentionally excluded — keeping
soft-path folded bowls drops flap polys and reopens foam seams.

### Drum wedge → triangle Coons

`makeCoonsPatch`: analytic cylinder/cone/revolution with exactly one
full-height u-iso side and three clear corners takes collapsed-last
triangle Coons instead of rejecting / probing outside a four-sided
patch.

- Fuzzy 4th corner: always (iso-band 505/525 class).
- Clear 4th corner: only when caller passes `allowDrumWedge`
  (`FeatureClass::Drum` at plan + mesh). FilletStrip cylinders share the
  one-meridian signature and must stay four-sided (teleporter face 208).

Mesh path threads `allowDrumWedge` into `meshCoonsGrid` so plan and mesh
rebuild the same patch.

## Evidence

| Check | Result |
|---|---|
| Foam CAD | watertight; floors 12 → 7 |
| Off floor | 183 drum, 184 fillet, 501/505/510/525 drum wedges |
| Residual floors | sphere bowls (fold heal), freeform 308/846, fillet iso-band 514 |
| Teleporter CAD | watertight; floor count unchanged (28) |
| `testSparseFoldKeepsStructuredCharts` | drum + fillet + wedge extracts + foam |
| Golden | foam CAD 6869q / 816t / 313n (intentional: fillet keep + drum wedges) |

Reducers: `tests/regressions/foam/{fillet_fullperiod,drum_isoband_bail,drum_freetrim_probe}_r1.step`.

## Sphere bowls (still open)

Soft-path `SphereCap×Pole` grids still lose to the floor. Sparse keep
reopens seams; loft experiments (constant-V / strip-bridge) raised fold
ratios or demoted hole-plate neighbors. Hard-path sphere rescue unchanged.
Follow-up: clear folds at source without flap drops.
