# MP9 gate — offset convex freeform (`freeform_1555`) — 2026-07-20

## Gate status

Extract RED. Fail-closed MP9 refused at `subjects=[6:1555]` after clearing
441 (cylinder lattice) and 553 (bspline lattice).

```
weft mesh tests/fixtures/mp9_extracts/freeform_1555.step
WEFT_G3_FREEFORM_UVTRIM_LATTICE tris=1920 uvWith=1911 uvAgainst=9 windingsMatch=0
→ freeform.uv_trim_orientation_unresolved
```

## Subclass

Extract: `tests/fixtures/mp9_extracts/freeform_1555.step` (MP9 face 1555)

- family `offset`, 3 bspline edges
- `convex_simple_region` / `freeform.uv_trim_candidate`
- non-periodic; corner-strict iso-lattice emits empty (even-odd polarity),
  centroid-only fallback yields 1920 tris with 9 residual against-N ears
- centroid-split on the lattice grows against count (does not clear)

## Next action

General consumer for offset/thin UV charts: clear the last against-N lattice
ears without face-id (diagonal precheck, denser centroid grid, or offset-
specific UV promote) → matrix HARD → relaunch MP9.

## Related greens this session

- `cyl_441.step` — period-folded iso-lattice HARD
- `freeform_553.step` — non-periodic iso-lattice HARD
