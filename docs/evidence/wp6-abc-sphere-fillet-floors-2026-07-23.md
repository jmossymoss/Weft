# WP6 — ABC sphere×fillet contract floors (2026-07-23)

## Class

ABC nightly `00006051`: `SphereCap × FullPeriod` with unequal rim totals
(72/36 → strip) plus `FilletStrip × FullPeriod` torus between sphere and
cylinder. Prior path: RevolutionGrid hard-failed on sphere rim mismatch;
Coons on the fillet mass-inverted (~520/1280) → both demoted to contract
floor.

Reducer: `tests/regressions/abc/sphere_fillet_fullperiod_r1.step`
(extract faces 15 with rings=1 from the ABC STEP).
Test: `testSphereFilletFullPeriodNoFloor`.

No filename / face-id specials. No new `MesherKind`.

## Fixes

1. Sphere two-rim zones join cylinder/cone/torus in revolution
   `stripReconcile` (transition strip over the flatter rim).
2. `FilletStrip × FullPeriod` on closed torus/cylinder plans as
   `RevolutionGrid` with `isFillet` + `acrossIsU` (blend ownership kept).
   Capsule / iso-band fillets stay on Coons.

## Multi-tooth densify (same change set)

ABC `00008536` open-band: densify the plain `bandDriver` (now
`12 * estNotches + 2` with the tooth-fold follow-up) so mesher `raise nu`
keeps `passPlain` and inter-tooth land keeps a column. See
`docs/evidence/wp6-abc-tooth-wall-folds-2026-07-23.md`.

## Metrics (CAD profile)

| Asset | Before | After |
| --- | --- | --- |
| `sphere_fillet_fullperiod_r1` | 2 floors, ~64 slivers | 0 floors, 0 folds, 0 slivers; all RevolutionGrid |
| ABC `00006051` whole | 2 floors | 0 floors, WT, 0 folds, 0 slivers |
| ABC `00008536` face 57 | RevolutionGrid, 32 folds | see tooth-wall-folds evidence (0 folds, WT) |

## Visual

`docs/evidence/wp6-abc-visual/` — CLI `--profile cad` on the reducers and
ABC public models.
