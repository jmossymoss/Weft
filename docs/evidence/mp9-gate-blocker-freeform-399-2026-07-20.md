# MP9 gate — shared-seam U-periodic freeform with circular caps (`freeform_399`) — 2026-07-20

## Gate status

Extract GREEN. Fail-closed MP9 still blocked later (do not claim full MP9).

```
weft mesh tests/fixtures/mp9_extracts/freeform_399.step -o build/f399.obj
→ EXIT 0
WEFT_G3_FREEFORM_UVTRIM_BAND … tris=288
WEFT_G3_FREEFORM_UVTRIM_ORIENT tris=288 uvWith=288 uvAgainst=0 windingsMatch=1 relax=0
```

## Subclass

Extract: `tests/fixtures/mp9_extracts/freeform_399.step` (MP9 face 399)

- family `bspline`, unique edges: 1 bspline seam + 2 circle caps
- `full_periodic_with_cap_boundaries`, U period `1`, UVBounds `[0,1]×[0,1]`
- Consumer: `buildCircularCapPeriodicUvBand` — iso-parametric UV band from the
  CDT-unwrapped circular-cap / seam boundary (general; not `faceId==399`)

## Cause

Lawson CDT on the UV rectangle emits ≈full-V interior diagonals (3D diameter
chords) that fail Wave-0 hardOrient. A full-period tensor grid with both U=0
and U=P seam columns duplicated 3D seam geometry after densified interiors and
failed `certified.triangle_proper_intersection`.

## Fix

1. Prior session: junction keep-candidate, circle interval cap (12), coarsen
   iso-gap guard, hardOrient corner-vote at `0.15·P`.
2. This session: when CDT/invert/coarsen refuse hardOrient on U-periodic
   freeform, build a matched-U circular-cap band with densified V, half-open U
   (drop U=P column), and wrap tris that reuse the U=0 seam verts with
   `cornerUv` at U+P — hardOrients and certifies without duplicate seam hulls.
3. Centroid-split of against-N CDT ears remains a fallback when the band
   classifier does not apply.

## Matrix

Wave D row: shared-seam U-periodic + circular caps → HARD (`freeform_399`).

`WEFT_FREEFORM_MATRIX` locked count includes `freeform_399.step`.

## Regressions

`freeform_135` / `137` / `138` / `186` / `freeform_hex` still EXIT 0 (CDT path;
band requires iso-V caps + iso-U seams with no diagonal leftovers).

## Out of scope

Full fail-closed MP9 until the next residual after face 399.
