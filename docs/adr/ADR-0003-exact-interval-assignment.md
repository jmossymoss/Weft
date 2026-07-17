# ADR-0003: Exact bounded interval assignment

- Status: Accepted
- Date: 2026-07-17
- Owners: Weft geometry core

## Context

Boundary sequences that meet by index must agree before geometry is sampled.
Choosing counts independently and repairing the mismatch later creates cracks,
T-junctions, or a hidden welding dependency. Count assignment also has to be
deterministic across platforms and expose unsupported template constraints.

## Decision

Weft solves equality, minimum, and even-parity interval constraints as a bounded
integer problem. Equality classes are formed deterministically. Every feasible
integer in the configured range is considered and the class minimises the sum
of absolute deviations from requested counts; the smaller count wins an exact
tie.

Analytic circular-arc demands are derived from both the chord-sagitta bound and
the normal-turn bound, then rounded upward. Lines require one interval.

Sum constraints remain present in the public problem contract but fail with
`interval.unsupported_constraint` until their bounded exact extension lands.
They are never approximated or silently ignored.

## Invariants

- variables are stable boundary IDs in strict deterministic order;
- equality is transitive and all members receive the same count;
- every result satisfies its minimum, parity, and configured cap;
- infeasible counts and unsupported constraint families fail by stable code;
- repeated solves of identical input produce identical ordered output.

## Alternatives rejected

- independent rounding followed by edge resampling or welding;
- greedy propagation whose answer depends on traversal order;
- accepting sum constraints while ignoring them;
- platform-specific optimisation libraries in the core count contract.

## Verification

`weft_interval_solver_tests` compares 200 deterministic generated problems
against an independent exhaustive reference, covers equality transitivity,
parity tie-breaking, cap refusal, named sum refusal, and analytic circle counts.
The test passes in the MSVC warnings-as-errors and static-analysis presets.

## Consequences

Canonical-boundary construction can consume a single solved count per stable
boundary. Template sum constraints must be implemented and property-tested
before templates that require them are enabled.
