# M3 exact-predicate evidence - 2026-07-17

## Proven increment

The geometry core now exposes a narrow exact-predicate interface and a compiled
distribution-safe reference backend. Finite IEEE-754 coordinates are converted
to exact dyadic integers before evaluating orientation and incircle determinant
signs. Exact segment classification distinguishes proper crossing, endpoint
contact, collinear overlap, and disjoint segments. Non-finite input refuses by
stable code.

No CGAL installation was found in the current environment, so no CGAL or CDT
claim is made. The reference backend exists to lock correctness independently
of the eventual adaptive/performance implementation.

## Verification

Commands run from `D:\Weft`:

```text
cmake --build --preset vs2022 --target weft_secure_core_tests weft_interval_solver_tests weft_canonical_boundary_tests weft_geometric_predicate_tests
ctest --preset vs2022 -R 'secure_core|interval_solver|canonical_boundary|geometric_predicates' --output-on-failure
cmake --build --preset vs2022-static-analysis --target weft_geometric_predicate_tests
ctest --preset vs2022-static-analysis -R 'secure_core|interval_solver|canonical_boundary|geometric_predicates' --output-on-failure
```

Both lanes passed all four focused tests. Predicate-specific coverage includes
1,500 independent integer-reference property cases, cancellation and underflow
witnesses, wrong-sign incircle input, exact segment relations, and non-finite
refusal.

## Status boundary

M3 remains in progress. The exact signs are ready for loop validation and a CDT
adapter, but no exact-construction CDT backend, critical trim segmentation, or
cross-platform determinism rerun has yet passed.
