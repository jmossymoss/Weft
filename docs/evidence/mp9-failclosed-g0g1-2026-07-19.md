# MP9 fail-closed mesh (omitDeferred=false) — 2026-07-19

## Gate
G0: product default does not omit deferred faces.
G1: plane multi-outer + self-intersect consumers without skipping faces.

## Command
```
build/linux-gcc/cli/weft mesh tests/STEP_Examples/MP9.stp -o /tmp/mp9_g1s.obj --progress
```

## Result
- EXIT 0
- omitDeferredResiduals = false (app/CLI defaults)
- 4270/4270 faces reported in progress
- Output: 121690 vertices, 130110 polygons (104994 quads, 25116 tris)

## Remaining soft paths (not face omission)
- Curved-UV CDT recovery for self-intersecting / collapsed plane wires
- Scoped `relaxGeometryChecks` on complex cylinder UV-trim and sphere UV fallback
- Industrial near-surface acceptance (~5 cm) in certified assembly
- Validation coverage bookkeeping for import repair / failed template attempts

## Next
G2: unify cylinder split-rail corners; remove scoped relax.
