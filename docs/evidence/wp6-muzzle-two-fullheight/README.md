# Orthogonal drum gate: allow ≥1 full-height side (2026-07-31)

## Failure

`mp9_Edited` muzzle (`Drum×IsoBand`, many capsule insets) rejected early
orthogonal planning with `drum needs one full-height side (2 full, 23 inset)`.
It then took late open-band and emitted diagonal chord closures between
columns. `MP9.stp` #3432 passed the same gate only because one meridian
measured under `0.9*v`.

Flaregun open-band barrels #43/#50 hit the same class (2 full-height
meridians + notch insets) and also move onto column cells.

## Fix

In `planOrthogonalTrimGrid`, require `fullHeightVertical >= 1` instead of
`== 1`, still demanding inset walls or a multi-piece horizontal rim.

## Evidence

- Reducer `tests/regressions/mp9/muzzle_two_fullheight_sides.step`:
  32 column cells, 0 off-axis shared closures.
- Existing `testMp9MuzzleColumnCells` still green.
- Structure retention unchanged; polygon goldens banked for the arity shift
  to arc-following n-gons on flaregun/foam/teleporter.
