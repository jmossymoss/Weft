# Wave D — periodic-band freeform pent period-1 (`freeform_138`) — 2026-07-20

## Gap

MP9 fail-closed stopped at face 138 with
`freeform.uv_trim_orientation_unresolved` after face 137 went HARD.
Same subclass tags as `freeform_135` / `freeform_137`
(`full_periodic_with_cap_boundaries`, `u_periodic`,
`freeform.general_attempted`), but U period is `1` (not `2π`) and a
mid-seam corner still fans a wide rim ear after correct unwrap.

Extract: `tests/fixtures/mp9_extracts/freeform_138.step` (5 bspline edges).

## Cause

After authoritative period unwrap, hardOrient always sampled the surface
normal at the triangle UV centroid whenever `uPeriod` was set. For ears
with U-span ≳ 0.35·period the centroid lies on a folded chord and
false-againsts a manifold ear (`uvAgainst=1`). Corner voting agrees with
the oriented face.

## Fix (general — not `faceId==138`)

In `hardOrientUvTrimMesh`: when `uPeriod` is set and the triangle U-span
exceeds `0.35 * period`, use corner normal voting instead of the centroid
sample. Also fold normal-sample UVs into the principal period before
`evaluateSurface`.

Depends on `curvedUvUPeriod` threading (`freeform_137`). Steiner period-span
splits were tried and rejected (`certified.vertex_position_mismatch`).

## Proof (vs2022 Release)

```
weft mesh tests/fixtures/mp9_extracts/freeform_138.step -o build/f138.obj
WEFT_G3_FREEFORM_UVTRIM_ORIENT tris=34 uvWith=32 uvAgainst=0 windingsMatch=1 relax=0
→ EXIT 0; 29 verts / 18 polys (14q/4t)

WEFT_FREEFORM_MATRIX includes freeform_138.step
```

## Matrix

`docs/governance/brep-consumer-matrix.md` Wave D row → HARD.
