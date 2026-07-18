# M8 selected-release gate closure - 2026-07-18

## Proven increment

Selected M8 release scope is complete:

- plane / cylinder (incl. partial) / cone / sphere / torus / analytic fillet strip

Mapped/freeform/cut-graph remain named refusals (OUT_OF_SCOPE for this gate).

## Commands

```bash
ctest --preset linux-gcc -R 'cone_template|sphere_template|torus_template|secure_meshing|secure_core|canonical_boundary|secure_committed_step_corpus' --output-on-failure
```

## Outcomes

Sphere/torus/cone/fillet product smokes and family packets passed on Linux.

## Status boundary

M8 marked PASSED.
