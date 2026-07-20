# MP9 gate — convex simple-region freeform (`freeform_553`) — 2026-07-20

## Gate status

Extract GREEN. Parent relaunches fail-closed MP9.

```
weft mesh tests/fixtures/mp9_extracts/freeform_553.step -o build/f553.obj
→ EXIT 0
WEFT_G3_FREEFORM_UVTRIM_LATTICE tris=1970 uvWith=1970 uvAgainst=0 windingsMatch=1 relax=0
  1102 vertices, 1027 polygons (943 quads, 84 tris)
```

## Subclass

Extract: `tests/fixtures/mp9_extracts/freeform_553.step` (MP9 face 553)

- family `bspline`, 6 bspline edges
- `convex_simple_region` / `freeform.uv_trim_candidate`
- non-periodic (no `curvedUvUPeriod`) — circular-cap band / period-gated
  split did not apply

## Cause

Lawson CDT leaves against-N ears after invert + coarsen (down to 2–4 against).
Without a U period the freeform recovery stopped before lattice/split.

## Fix (general — not `faceId==553`)

Extend `buildPeriodicUvIsoLattice` to non-periodic charts (raw UV bbox densify,
no fold/wrap) and call it from freeform UV-trim after coarsen. Append CDT
boundary vertices for interval consumption (same as cylinder multi-rim).

## Matrix

Wave D row: convex simple-region n-gon → HARD (`freeform_553`).

`WEFT_FREEFORM_MATRIX` locked count includes `freeform_553.step`.

## Regressions

`freeform_399` / `186` / `135`–`138` / `cyl_441` still EXIT 0.

## Out of scope

Full fail-closed MP9 until the next residual after face 553.
