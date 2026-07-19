# G2: complex cylinder UV-trim without relax — 2026-07-19

## Change
- Cylinder leftover sample attach gated to 1 mm 3D proximity (no distant STEP-id attach).
- Complex cylinder UV-trim no longer sets `relaxGeometryChecks`.
- Sphere CapWall leftovers similarly gated; CapWall retains scoped relax for Plasticity seam/ear orientation until CapWall manifold/orientation is proven hard.

## Proof
```
weft mesh tests/fixtures/mp9_extracts/cylinder_complex.step  # OK without relax
weft mesh tests/STEP_Examples/MP9.stp                        # EXIT 0, omitDeferred=false
```

## Remaining soft (G3–G5)
- CapWall / plane curved-UV / mapped `relaxGeometryChecks`
- Industrial near off-surface envelope
