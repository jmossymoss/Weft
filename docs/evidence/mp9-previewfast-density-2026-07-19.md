# MP9 previewFast + density tighten (fail-closed) — 2026-07-19

## Changes
- CapWall emitTriangle aligns to geometric (unoriented) surface normal; scoped `relaxGeometryChecks` retained for Plasticity seam/ear certify pairing.
- `previewFast` boundary build for large industrial bodies (`faceCount > 500`) even when `omitDeferredResiduals=false`.
- Active `revolutionRadialSegments` capped to 10 for large preview budgets; budget scale 0.70.

## Result
```
76627 vertices, 81701 polygons (63824 quads, 17877 tris)
boundaries ~27s; faces complete ~78s
omitDeferred=false
```
