# Wave D — periodic-band freeform hex (`freeform_137`) — 2026-07-20

## Gap

MP9 fail-closed stopped at face 137 with
`freeform.uv_trim_orientation_unresolved` after invert + coarsen
(`uvAgainst≠0`). Neighbor `freeform_135` already HARD; 137 is a distinct
subclass (hex wire + non-2π U period).

Extract: `tests/fixtures/mp9_extracts/freeform_137.step`
(inventory: `freeform.uv_trim_candidate`, `freeform.general_attempted`,
`full_periodic_with_cap_boundaries`, `u_periodic`, 6 bspline edges).

## Cause

Curved UV CDT seam unwrap hard-coded period `2π`. This bspline face’s
authoritative U period is `1`. Principal-value jumps across the seam were
never unwrapped, so the CDT polygon stayed seam-crossed and emitted a
folded against-N ear from a mid-V corner across the rim. Coarsen/densify
could not repair a wrong chart.

## Fix (general — not `faceId==137`)

1. Thread `PlanarTrimDomain::curvedUvUPeriod` from reconnaissance face
   domains in `assemblePlanarTrimDomain`.
2. `ExactLawsonReferencePlanarCdtBackend` unwraps curved UV loops with that
   period (fallback `2π` for analytic charts that omit the field).

Wave 0 hardOrient contract unchanged. Wave D interval-cap + coarsen for
`freeform_135` untouched.

## Proof (vs2022 Release)

```
weft mesh tests/fixtures/mp9_extracts/freeform_137.step -o build/f137.obj
WEFT_G3_FREEFORM_UVTRIM_ORIENT tris=40 uvWith=40 uvAgainst=0 windingsMatch=1 relax=0
→ EXIT 0; 35 verts / 22 polys (18q/4t)

weft_planar_cdt_tests includes testCurvedUvAuthoritativePeriodUnwrap
weft_mapped_template_tests / WEFT_FREEFORM_MATRIX includes freeform_137.step
```

## Matrix

`docs/governance/brep-consumer-matrix.md` Wave D row → HARD.
`WEFT_FREEFORM_MATRIX` scaffold requires `freeform_137.step`.
