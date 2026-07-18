# M3 bounded coupled count evidence - 2026-07-18

## Proven increment

The interval solver accepts one connected coupled component after equality
reduction. The component is exact and finite: at most eight classes, eight
equations, and 1,000,000 candidate assignments. It minimizes global L1 cost
and uses stable ascending class/count order for ties. Independent sums retain
their existing convex allocation path.

A planar box consumes `edge1 + edge2 = edge3` together with `edge3 = edge4`;
the coupled solution drives canonical boundaries and certified template
generation, and repeated generation reproduces the topology fingerprint.

## Oracle and adversaries

An independent exhaustive enumerator checks 100 deterministic coupled
minimum/parity/fixed-count problems over small domains. Solver and oracle agree
on feasibility, global objective, and the complete stable ordered assignment.

Explicit fixtures cover:

- satisfiable and infeasible coupled chains;
- equality aliases shared across coupled equations;
- coefficient multiplicity refusal as
  `interval.sum_aliasing_unsupported`;
- a second connected component as `interval.coupled_sum_unsupported`;
- class/equation budget and assignment-product overflow as
  `interval.sum_complexity_exceeded`;
- deterministic replay and exact planar-template consumption.

## Verification

Commands run from `D:\Weft` at revision `8180f90` with WP-012 through WP-014
present as uncommitted working-tree changes:

```text
cmake --build --preset vs2022 --config Release --parallel 2 --target weft_interval_solver_tests weft_secure_meshing_tests
ctest --test-dir build/vs2022 -C Release -R "interval_solver|secure_meshing" --output-on-failure
cmake --build --preset vs2022-static-analysis --config Release --parallel 2 --target weft_interval_solver_tests weft_secure_meshing_tests
ctest --test-dir build/vs2022-static-analysis -C Release -R "interval_solver|secure_meshing" --output-on-failure
cmake --build --preset vs2022 --config Release --parallel 2
cmake --build --preset vs2022-static-analysis --config Release --parallel 2
ctest --preset vs2022
ctest --preset vs2022-static-analysis
```

Focused Release and MSVC static-analysis suites pass 2/2. Complete Release and
MSVC static-analysis lanes each pass 14/14, including committed and generated
STEP corpus tests.

## Status boundary

WP-014 is closed. Additional coupled components, net coefficient
multiplicities, over-budget systems, and Linux determinism remain refused or
open. M3 remains `IN_PROGRESS`.
