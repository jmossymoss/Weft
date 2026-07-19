# Projection cache + density ~79k (fail-closed) — 2026-07-19

## Changes
- `previewFast` caches `projectPointToSurface` UV by face + quantized 3D.
- Projection uses ElSLib/GeomAPI foot points directly (no second `evaluateSurface`).
- CapWall: `windingsMatchOrientedFaceNormal` contract (certify skips double Reversed); scoped relax retained for Plasticity ears.
- Large preview: active radial cap 8, budget scale 0.60.

## Result
```
74377 vertices, 79047 polygons (61824 quads, 17223 tris)
boundaries ~25s; faces ~76s; omitDeferred=false
```
