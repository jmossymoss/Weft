# M3 template chain-sum consumer evidence - 2026-07-18

## Proven increment

One certified planar template path now consumes the existing exact independent
chain-sum solver. A box fixture whose opposite-edge density is only correct
when `edge1 + edge2 = edge3` solves through canonical boundaries, certifies
non-vacuous requested/solved/consumed coverage, and refuses minimum, parity,
conflict, invalid-sum, and tampered-consumption adversaries by name.
Recipe-v2 edge pins plus the same chain sum replay to an identical topology
fingerprint.

## Verification

Commands run from `D:\Weft` at revision `8180f90` working tree (WP-012/WP-013
uncommitted):

```text
cmake --build --preset vs2022 --config Release --parallel 2 --target weft_secure_meshing_tests weft_secure_recipe_tests weft_interval_solver_tests
ctest --test-dir build/vs2022 -C Release -R "interval_solver|secure_meshing|secure_recipe" --output-on-failure
cmake --build --preset vs2022-static-analysis --config Release --parallel 2 --target weft_secure_meshing_tests weft_secure_recipe_tests weft_interval_solver_tests
ctest --test-dir build/vs2022-static-analysis -C Release -R "interval_solver|secure_meshing|secure_recipe" --output-on-failure
```

Both Release and MSVC static-analysis lanes pass the focused suites (3/3).

Focused proof:

- without the sum, pinned edges 1=2 and 2=3 leave edge 3 at its floor of 1;
- with the sum, edge 3 becomes 5 and six canonical samples are shared by owning
  faces;
- coverage codes `secure_pipeline.chain_sum_requested`,
  `secure_pipeline.chain_sum_solved`, and
  `secure_pipeline.chain_sum_consumed` are non-vacuous and complete;
- `interval.sum_infeasible`, `interval.exact_parity_conflict`,
  `interval.exact_below_minimum`, `secure_pipeline.chain_sum_invalid`, and
  `secure_pipeline.interval_consumption_mismatch` refuse by name;
- repeated secure meshing and recipe-v2 reload+resolve replay produce identical
  topology fingerprints.

## Status boundary

WP-013 is closed. Coupled/aliased sum systems (WP-014 / BR-011) and Linux
determinism remain open. M3 remains `IN_PROGRESS`.
