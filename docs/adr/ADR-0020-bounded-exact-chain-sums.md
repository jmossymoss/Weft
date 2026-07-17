# ADR-0020: Bounded exact chain-sum assignment

- Status: Accepted
- Date: 2026-07-17
- Owners: Weft geometry core

## Context

Structured transitions need the total interval count along one boundary chain
to equal the total along another. Solving the edges independently and adjusting
one afterward would invalidate canonical samples, exact recipe pins, and error-
derived minimums. A general coupled integer program is also an unsafe first
step: its cost can grow exponentially and an accidental large model must not
turn a deterministic meshing request into an unbounded search.

## Decision

- Equality classes are reduced before a sum is solved. Classes occurring once
  on each side cancel algebraically.
- Each remaining sum side is a disjoint set of equality classes with net
  coefficient one. Multiple sums may be solved together only when their class
  sets are independent.
- Every class retains its certified minimum, parity, configured cap, and exact
  recipe constraint. Its objective is the sum of absolute deviations of all
  equality-class members from their desired counts.
- For classes with the same step, the solver constructs the exact convex
  allocation curve by merging non-decreasing marginal costs. Mixed unit/even
  allocation curves are combined by exhaustive bounded transitions.
- The two sides meet at the feasible common total with minimum combined L1
  objective. Objectives within a relative scale of `1e-12` are treated as a
  numerical tie so binary representations of decimal requests cannot change
  topology on negligible round-off. The smaller common total wins that tie.
  Marginal ties increment later stable classes first; mixed-step ties use fewer
  even-step actions. These rules make repeated output deterministic.
- State storage is capped at 2,000,000 total states per side and mixed-parity
  combination at 4,000,000 transitions. Exceeding either budget refuses as
  `interval.sum_complexity_exceeded` before the large allocation/search.
- Net coefficients larger than one and classes shared by multiple non-trivial
  sums refuse as `interval.sum_aliasing_unsupported` and
  `interval.coupled_sum_unsupported`. Infeasible bounded systems refuse as
  `interval.sum_infeasible`.

This record supersedes ADR-0003's temporary decision to reject every sum. The
original equality, minimum, parity, exact-count, cap, and no-approximation
decisions remain in force.

## Invariants

- every successful result satisfies every original sum in a final independent
  self-check;
- no minimum, parity, exact count, equality, or cap is weakened to make a sum
  feasible;
- unsupported algebraic coupling and exhausted complexity budgets fail by
  stable code;
- no canonical boundary is built until all participating counts solve;
- ordered input produces ordered, repeatable output independently of traversal
  order.

## Alternatives rejected

- greedy post-adjustment of one edge in a chain;
- rounding a fractional or relaxed linear-program solution;
- silently dropping equality aliases from the objective;
- invoking a general-purpose integer optimiser without a deterministic budget;
- accepting coupled systems while solving their constraints one at a time;
- allowing memory consumption to scale to the full cap product.

## Verification

The solver is checked against an independent exhaustive enumerator on 100
deterministically generated sum problems containing minimum, parity, and fixed-
count combinations. The oracle compares feasibility, global L1 optimum, and
the smaller optimal common total; repeated solves must return identical ordered
counts. Explicit witnesses cover chain-to-single reconciliation, mixed parity,
fixed totals, equality-class cancellation, multiple independent sums,
infeasibility, aliasing, coupling, invalid empty sides, and complexity refusal.

The pre-existing 200-case equality/minimum/parity exhaustive battery remains
green. Complete strict and MSVC static-analysis builds pass, and all 14
secure/corpus tests pass in both lanes.

## Consequences

Template work can now express exact independent boundary-chain reconciliation
without resampling. The first template consumer, coupled sum graphs, and net
class multiplicities remain gated follow-up work. Those cases cannot be
approximated through this solver.
