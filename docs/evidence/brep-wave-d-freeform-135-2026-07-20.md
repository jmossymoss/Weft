# Wave D — periodic-band freeform pent (`freeform_135`) — 2026-07-20

## Gap

MP9 fail-closed stopped at face 135 with
`freeform.uv_trim_orientation_unresolved` after invert retry
(`uvAgainst≠0`). Matrix row: Periodic band pent +
`freeform.general_attempted` / `FullPeriodicWithCapBoundaries`.

Extract: `tests/fixtures/mp9_extracts/freeform_135.step`
(inventory: `freeform.uv_trim_candidate`, `freeform.general_attempted`,
`full_periodic_with_cap_boundaries`, 5 bspline edges).

Neighbor `crash_face_136_r0` remains HARD via invert; 135 is a distinct
subclass (dense rim samples → against-N CDT ears).

## Cause

Product radial 32 sets `minimumClosedCurveSegments=8`. On periodic-band
freeform UV-trim edges that floor densifies rims enough for the curved UV
CDT to emit two against-N ears that stay manifold-consistent with +N
neighbors. Wave 0 `hardOrientUvTrimMesh` cannot flip them without
minority drop / residual admit. Coarser floors (≤6) certify.

## Fix (general — not `faceId==135`)

1. Cap bspline/bezier interval counts at 6 when the owner face is
   periodic-band + `freeform.uv_trim_candidate` /
   `freeform.general_attempted` (`secure_meshing` interval builder).
2. After invert fails, coarsen mid-edge UV stations (keep wire corners)
   and re-CDT up to two steps (`coarsenCurvedUvTrimLoops` +
   `WEFT_G3_FREEFORM_UVTRIM_COARSEN`).

Wave 0 hardOrient contract unchanged (no minority drop, no residual
admit). Torus 115 densify path untouched.

## Proof (vs2022 Release)

```
weft mesh tests/fixtures/mp9_extracts/freeform_135.step -o build/f135.obj
WEFT_G3_FREEFORM_UVTRIM_ORIENT tris=33 uvWith=33 uvAgainst=0 windingsMatch=1 relax=0
→ EXIT 0

weft_mapped_template_tests / WEFT_FREEFORM_MATRIX includes freeform_135.step
```

## Matrix

`docs/governance/brep-consumer-matrix.md` Wave D row → HARD.
`WEFT_FREEFORM_MATRIX` scaffold requires `freeform_135.step`.

## Out of scope

Full MP9 body (parent relaunches `mp9-gate`).
