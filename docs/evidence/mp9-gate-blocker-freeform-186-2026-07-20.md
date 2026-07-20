# MP9 gate — V-periodic freeform tiny period (`freeform_186`) — 2026-07-20

## Gate status

Extract certifies HARD. Parent relaunches fail-closed MP9.

```
weft mesh tests/fixtures/mp9_extracts/freeform_186.step -o build/f186.obj
WEFT_G3_FREEFORM_UVTRIM_ORIENT tris=40 uvWith=28 uvAgainst=0 windingsMatch=1 relax=0
→ EXIT 0; Independent modelling polygons
```

## Subclass

Extract: `tests/fixtures/mp9_extracts/freeform_186.step`

- family `bspline`, 6 edges
- `full_periodic_with_cap_boundaries`, `v_periodic` (not `u_periodic`)
- OCCT `VPeriod ≈ 0.016`

## Cause

Nearest-period V unwrap alone collapses the full-period seam cut
(`|ΔV|≈P`) and leaves both UV images of the seam canonical vertex on one
sheet. The wire then forms a digon spur (out-and-back) between the two
visits → near-zero CDT area → ~40% against-N under hardOrient.

Contrast `freeform_137` (U period=1): consecutive unwrap advances the
second seam image by one period along the spanning edge.

## Fix (general — not `faceId==186`)

In `ExactLawsonReferencePlanarCdtBackend` curved UV unwrap:

1. Enable authoritative `curvedUvVPeriod` even when tiny (U still guards
   `<0.25` for unit/2π charts).
2. Keep intentional ≈1-period seam-cut steps (0.5% band) so the cut is
   not nearest-collapsed.
3. When a canonical vertex reappears on the same V sheet with a digon
   spur (no progress on U), place the second image at `±VPeriod` and
   reverse the spur interior so the cut advances monotonically — preserve
   sample UV/3D (no off-surface relocate, no sample drop).

Wave 0 hardOrient unchanged. `faceUvPeriods` still ignores tiny periods
for `periodicNear` (skinny triangles span `>P/2`).

## Proof (vs2022 Release)

```
weft mesh tests/fixtures/mp9_extracts/freeform_186.step -o build/f186.obj
→ EXIT 0; windingsMatch=1 relax=0

Regressions freeform_135 / 137 / 138 → EXIT 0
weft_planar_cdt_tests includes testCurvedUvTinyVPeriodDualImage
weft_mapped_template_tests / WEFT_FREEFORM_MATRIX includes freeform_186.step as HARD
```

## Matrix

`docs/governance/brep-consumer-matrix.md` Wave D row → HARD.
`WEFT_FREEFORM_MATRIX` scaffold requires `freeform_186.step`.

## Out of scope

Full MP9 body (parent relaunches `mp9-gate`).
