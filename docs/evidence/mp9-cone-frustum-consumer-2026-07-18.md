# Evidence — truncated cone (frustum) consumer (2026-07-18)

## Consumer

Non-apex cones with `FullPeriodicWithCapBoundaries` or
`PeriodicBandCrossingSeam` reuse the revolved-band wall builder
(`buildFullCylinderWall`) instead of the apex fan.

## Proof

- Fixture `truncated_cone` (r1=10, r2=4, h=20): `WEFT_CONE_FRUSTUM tris=76
  quads=36` with Independent modelling provenance.
- MP9 extract `tests/fixtures/mp9_extracts/cone_frustum.step` (face 274):
  meshes (32 tris on default LOD).
- Apex cones still use `buildApexConeWall`.
- Complex partial cone bands (>4 edges) stay
  `cone.complex_boundary_deferred`.

## Quad bias

Structured two-rim band produces triangle strips that pair into modelling
quads (36/36 cells on the fixture). Certified floor remains triangles.
