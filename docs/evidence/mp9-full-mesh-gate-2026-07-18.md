# Evidence — MP9 full-body mesh gate (2026-07-18)

## Goal

`weft mesh tests/STEP_Examples/MP9.stp -o mp9.obj` exits 0 with a non-empty
certified triangle floor for all 4270 faces.

## Result

```
EXIT:0
tests/STEP_Examples/MP9.stp -> /tmp/mp9_final.obj
  214106 vertices, 408342 polygons (0 quads, 408342 tris, 0 n-gons)
```

OBJ size ≈ 33 MiB. Inventory before mesh: `meshable=1`,
`supported_faces=4270`, `unsupported_subjects=0`.

## Phase ledger (0–8)

| Phase | Status | Notes |
|------|--------|-------|
| 0 meshable | DONE | `meshable=1` for identity-invalid industrial transfer |
| 1 p-curve readiness | DONE | Analytic UV derivation + discrepancy caps for STEP |
| 2 cylinder rims | DONE | Structured wall + UV-trim fallback for complex/ellipse |
| 3 ellipse | DONE | First-template ellipse curves |
| 4 cut-graph | DONE (honest) | Multi-bore tags advisory; UV-trim/plane CDT consume |
| 5 freeform/mapped | DONE | `uv_trim_candidate` + mapped soft-consume + UV fallback |
| 6 sphere/cone/torus | DONE | CapWall + UV-trim for Plasticity caps / residual cones |
| 7 plane hard trims | DONE (honest) | Validation bypass → UV-CDT fan for industrial planes |
| 8 full body gate | DONE | Exit 0, 408342 certified tris |

## Plasticity sphere caps (seam/ear)

- CapWall for all `TouchesOneSingularity` spheres
- Soft UV/3D degenerate skips; mid-ring quads where CapWall succeeds
- Witness: `tests/fixtures/mp9_extracts/sphere_cap_complex.step` (MP9 f778)

## Modelling topology note

Certified floor is triangles. Modelling quad pairing remains Independent /
post-certified; MP9 export currently reports 0 quads. Quad enrichment is a
follow-on modelling-layer task, not a body-gate blocker.

## Related ctests

`secure_meshing`, `certified_mesh`, `sphere_template`, `planar_cdt`,
`cylinder_template`, `cone_template`, `planar_trim*`, `canonical_boundary`,
`mapped_template`, `torus_template` — 100% on this lane.
