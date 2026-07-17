# M3 interval-solver evidence - 2026-07-17

## Proven increment

The first M3 increment introduces a deterministic bounded integer solver for:

- equality classes;
- per-boundary minimum counts;
- even-parity requirements;
- L1 distance from requested counts with a smaller-count tie break;
- explicit configured maximum refusal.

Circular-arc demands use analytic chord-sagitta and normal-turn bounds. Sum
constraints are represented but refuse by the stable
`interval.unsupported_constraint` code until the exact extension is complete.

## Verification

Commands run from `D:\Weft`:

```text
cmake --build --preset vs2022 --target weft_interval_solver_tests
ctest --preset vs2022 -R interval_solver --output-on-failure
cmake --build --preset vs2022-static-analysis --target weft_interval_solver_tests
ctest --preset vs2022-static-analysis -R interval_solver --output-on-failure
```

Both test lanes passed. The suite includes 200 deterministic property cases
checked against an independent exhaustive reference and explicit analytic,
tie-break, cap, and unsupported-constraint witnesses.

## Status boundary

This starts M3 but does not pass it. Exact predicates, monotone/critical trim
segmentation, periodic UV lifting, azimuth registration, canonical shared-edge
sampling, and the exact CDT prototype remain open.
