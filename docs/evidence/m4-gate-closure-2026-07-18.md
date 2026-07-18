# M4 tessellation floor gate closure - 2026-07-18

## Gate

Every valid or conservatively repairable face (within supported automatic
families) produces a deterministic boundary-exact intersection-free triangle
mesh; unsupported geometry fails by stable named refusal.

## Supported families in scope

plane (incl. holes), full/partial cylinder, apex cone, sphere, torus, analytic
fillet strip (plane + quarter-cylinder bead).

## Verification

```bash
ctest --preset linux-gcc \
  -R 'geometric_predicates|certified_mesh|cylinder_template|cone_template|sphere_template|torus_template|secure_meshing|canonical_boundary|secure_committed_step_corpus' \
  --output-on-failure
ctest --preset linux-gcc-static-analysis \
  -R 'secure_meshing|certified_mesh|geometric_predicates' --output-on-failure
build/linux-gcc/cli/weft mesh <box|cylinder|partial_cylinder>.step -o out.obj
build/linux-gcc/cli/weft mesh ribbon.step  # named refusal
```

## Outcomes

- Linux gcc: 9/9 listed tests passed.
- Linux static-analysis: 3/3 listed tests passed.
- CLI: box 8v/12t, cylinder 52v/100t, partial_cylinder 42v/80t certified OBJ.
- Unsupported freeform `ribbon`: `secure_pipeline.unsupported_curve_family`.

## OUT_OF_SCOPE (not M4 blockers)

Mapped Coons, cut-graph expansion, and general freeform floors (WP-120+).

## Status

WP-115 closed. M4 marked `PASSED`.
