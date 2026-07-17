# M3 exact chain-sum evidence - 2026-07-17

## Proven increment

The bounded interval solver now supports independent equal-sum boundary chains
after deterministic equality-class reduction. It reconciles minimum, parity,
fixed-count, cap, and L1 objectives exactly before canonical sampling. Coupled,
net-aliased, infeasible, invalid, and over-budget systems refuse by name.

## Algorithm evidence

For a single step size, each equality-class L1 objective has non-decreasing
marginal costs. Merging those marginal sequences constructs the exact minimum
allocation cost for every reachable side total. Unit- and even-step curves are
then combined over every transition inside a fixed budget. The two sides are
matched over every common reachable total, selecting the minimum combined L1
objective and then the smaller total. A documented relative `1e-12` objective
tie prevents insignificant binary round-off in decimal desires from selecting
a different topology.

The implementation caps each side at 2,000,000 states and a mixed-parity side
at 4,000,000 transitions. Refusal occurs before the large state allocation or
transition loop.

## Verification

Commands run from `D:\Weft`:

```text
cmake --build --preset vs2022 --config Release --parallel 2
cmake --build --preset vs2022-static-analysis --config Release --parallel 2
ctest --test-dir build/vs2022 -C Release -E "^pipeline$" --output-on-failure
ctest --test-dir build/vs2022-static-analysis -C Release -E "^pipeline$" --output-on-failure
```

Both complete-product builds pass. All 14 secure/corpus tests pass in each
lane. The interval solver battery includes:

- the existing 200 deterministic equality/minimum/parity problems checked
  against an independent exhaustive reference;
- 100 deterministic sum problems checked by a separate exhaustive enumerator
  for feasibility, global L1 objective, and smallest optimal common total;
- repeated-solve ordered-count equality for every generated case;
- explicit chain-to-single, mixed unit/even, exact-total,
  equality-cancellation, and multiple-independent-sum witnesses; and
- stable `sum_infeasible`, `sum_aliasing_unsupported`,
  `coupled_sum_unsupported`, `sum_complexity_exceeded`, and invalid-input
  witnesses.

## Status boundary

No automatic template emits a sum constraint yet. Net equality-class
multiplicity, constraint graphs sharing a class, general integer coefficients,
and Linux determinism remain open. M3 remains `IN_PROGRESS`.
