# M3 canonical-boundary evidence - 2026-07-17

## Proven increment

The secure core now records exact edge endpoint topology and constructs one
atomic canonical sample sequence per accepted working edge. Adjacent coedges
consume the same sample records and deterministic vertex indices. Each UV use
distinguishes a stored p-curve from an evaluator-derived planar projection,
carries periodic lift and source/working provenance, and is checked against the
bounded source tolerance envelope.

The increment also ports azimuth registration for equal-count periodic rings.
It returns a cyclic permutation only; reflected or non-uniform inputs fail by
`boundary.azimuth_registration_failed`.

## Verification

Commands run from `D:\Weft`:

```text
cmake --build --preset vs2022 --target weft_secure_core_tests weft_interval_solver_tests weft_canonical_boundary_tests
ctest --preset vs2022 -R 'secure_core|interval_solver|canonical_boundary' --output-on-failure
cmake --build --preset vs2022-static-analysis --target weft_secure_core_tests weft_interval_solver_tests weft_canonical_boundary_tests
ctest --preset vs2022-static-analysis -R 'secure_core|interval_solver|canonical_boundary' --output-on-failure
```

Both lanes passed all three focused tests. Fixtures cover:

- a box whose planar coedges lack stored p-curves;
- a cylinder combining planar projection and stored curved-face p-curves;
- missing solved-count refusal with no partial value;
- a quarter-circle whose supporting curve is periodic but whose B-rep edge is
  topologically open;
- aligned, phase-shifted, reflected, and non-uniform azimuth rings.

## Status boundary

M3 remains in progress. Counts are deliberately uniform within each edge in
these fixtures; critical-interval adaptive placement, exact predicates,
periodic-loop closure proof, sum constraints, and the exact CDT research backend
remain open. Degenerate singular edges fail closed by name.
