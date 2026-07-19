# Evidence — Modelling quads + preview density progress (2026-07-19)

## Delivered

### Modelling quads (B1/B2)
- `makeCertifiedPolyMeshAdapter` exports Independent modelling polygons.
- Cylinder fixture: **48 quads / 28 tris**.
- MP9 preview: **103502 quads / 24810 tris** (128312 polygons, 121684 verts).

### Density / radial (D1)
- UI/default **radial = 32** → `revolutionRadialSegments`.
- General curve floor separate (`minimumClosedCurveSegments` ≈ 8).
- Aggressive interval budget scaling disabled (broke plane hole bridges).
- Current MP9 preview with UI radial 32 + active preview revolution cap 8: **87959 polygons (70074 quads / 17885 tris)**, 84729 verts, 923 folded.
- Target ~70k: within ~25%; next freeform-only coarsening (safe) to close the gap.

### Quality track (partial)
- A1: no blanket relax on unit fixtures; industrial omitDeferred still relaxes.
- A2: `cutout.multi_bore_cylinder` advisory (not deferred).
- A3: strict mapped seam unmatched; freeform soft-skip retained.
- A4: torus chord restored.
- A5: hard self-intersecting planes deferred + omit on compounds; body-scale
  plane UV-fan residual when nesting proofs fail.

### Perf (not started)
- Import still ~35s; mesh ~6 min. Track C next.

## Screenshot
`/opt/cursor/artifacts/mp9_quads_preview.png`
