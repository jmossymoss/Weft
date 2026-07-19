# MP9 preview density under fail-closed — 2026-07-19

## Change
- Enable `previewTriangleBudget` coarsening (was disabled).
- Cap active `revolutionRadialSegments` to 12 for large bodies when a preview budget is set (UI default remains 32).
- Cap mapped UV intervals for large industrial models without requiring `omitDeferredResiduals`.

## Result (`omitDeferred=false`)
```
79667 vertices, 84958 polygons (66775 quads, 18183 tris)
face phase ~87s (was ~170s at ~126k polys)
```

Target soft budget ~70k; remaining headroom from freeform UV-trim density.
