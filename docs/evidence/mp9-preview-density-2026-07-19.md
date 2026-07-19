# Evidence — MP9 preview density (~82k polys) 2026-07-19

## Settings
- UI radial = 32 (quality intent)
- Active `revolutionRadialSegments` capped at 8 under `omitDeferredResiduals`
- Mapped/extrusion UV grids no longer forced to 32×32 in preview
- Modelling Independent quads exported

## Result
```
78485 vertices, 81834 polygons (64287 quads, 17547 tris)
folded ≈ 926
wall mesh ≈ 124 s (was ~280–360 s at 400k tris)
```

Within the plan band of 55k–85k for preview polygon count (±20% of 70k).

## Warm cache
App `secureWarmCache` hits on regenerate with identical path/mtime/LOD
(settings) for millisecond adopt of the prior MeshingResult/PolyMesh.
