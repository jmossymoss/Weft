# MP9 gate — multi-rim ellipse-cut cylinder (`cyl_441`) — 2026-07-20

## Gate status

Extract GREEN. Parent relaunches fail-closed MP9.

```
weft mesh tests/fixtures/mp9_extracts/cyl_441.step -o build/cyl_441.obj
→ EXIT 0
WEFT_G2_CYL_UVTRIM_LATTICE tris=1160 uvWith=1160 uvAgainst=0 windingsMatch=1 relax=0
  818 vertices, 589 polygons (571 quads, 18 tris)
```

## Subclass

Extract: `tests/fixtures/mp9_extracts/cyl_441.step` (MP9 face 441)

- family `cylinder`, `full_periodic_with_cap_boundaries`
- 13 edges: 5 circle + 2 ellipse + 6 line (multi-rim / elliptical cuts)
- Consumer: `buildPeriodicUvIsoLattice` after CDT invert refuses hardOrient

## Cause

Lawson CDT emits full-height V fans from hole corners (du small, dv≈full band)
that disagree with the oriented cylinder normal. Invert leaves ~10 against-N
ears; coarsen/centroid-split do not clear them.

CDT unwrap U spans >1 period (e.g. `[−π, 2π)`), so a naive iso-lattice
double-covers the cylinder → `certified.triangle_proper_intersection`.

## Fix (general — not `faceId==441`)

In cylinder UV-trim recovery (`secure_meshing.cpp`):

1. Invert CDT chart; if still refused, build a period-folded `[0,P)` iso-U×V
   lattice clipped by even-odd inclusion of all CDT boundary loops (outer +
   holes), with corner/edge-midpoint inside checks.
2. Append every CDT boundary vertex (canonical identity) so interval
   consumption stays complete.
3. Half-open U + guarded wrap; skip wrap cells that already disagree with N.

Wave 0 hardOrient unchanged. Prior cylinder extracts still HARD via CDT path
when they orient without lattice.

## Matrix

Wave B row: multi-rim ellipse-cut full-period UV-trim → HARD (`cyl_441.step`).

`WEFT_CYLINDER_MATRIX` locked count = 8/8.

## Out of scope

Full fail-closed MP9 until the next residual after face 441.
