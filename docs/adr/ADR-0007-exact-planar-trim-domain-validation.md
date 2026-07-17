# ADR-0007: Exact planar trim-domain validation

- Status: Accepted
- Date: 2026-07-17
- Owners: Weft geometry core

## Context

A constrained triangulator can only preserve a valid planar straight-line
graph. Sending it an open wire, repeated vertex, crossing constraint, touching
hole, or incorrectly nested loop makes any later triangulation or repair
ambiguous. Sampling a handful of points or relying on a triangulator error is
not sufficient evidence that the complete trim domain was checked.

Canonical boundaries already provide immutable sample identities and shared
vertex indices. ADR-0005 provides exact signs and segment relations for the
finite UV coordinates actually supplied by OCCT evaluation. The pre-CDT
contract must join those two proofs without changing boundary order or hiding a
failed predicate behind a tolerance.

## Decision

`validatePlanarTrimDomain` is the mandatory precondition for a planar CDT
backend. Its input vertices carry canonical boundary sample IDs, working and
optional source edge IDs, canonical vertex indices, and lifted UV coordinates.
A topology assembler must explicitly mark each wire closed; merely providing
three points does not imply closure.

The validator requires a backend that reports exact finite-double predicates
and checks:

- valid face, wire, sample, and canonical-vertex identities;
- finite UV coordinates and at least three unique vertices per loop;
- every unordered edge pair within every loop, including the expected endpoint
  contact of adjacent edges;
- every edge pair between different loops;
- exact winding containment for every ordered loop pair;
- exactly one outer loop and only directly contained holes;
- counter-clockwise outer and clockwise hole orientation in lifted UV.

The orientation sign is the exact turn at the unique lexicographically minimal
vertex after simplicity and uniqueness have been proven. For a simple polygon,
that extreme turn has the polygon's orientation; a zero turn at that point
would imply a rejected overlap or degeneracy.

The validator does not repair, reorder, reverse, snap, merge, or discard input.
Any failed or unavailable check returns no validated domain. Every check family
reports `expected`, `checked`, `skipped`, and `failed` counts.

## Invariants

- no non-exact predicate backend may certify a trim domain;
- every sample boundary ID must resolve to the vertex's working edge ID;
- each input edge relation is checked exactly once in deterministic order;
- boundary contact between separate loops is a failure, not a tolerance case;
- a failed structural check causes dependent checks to be reported as skipped;
- successful non-empty evidence has non-zero coverage wherever evidence was
  expected;
- success returns the original sample order and coordinates unchanged.
- canonical vertex index zero is retained as valid zero-based provenance;

## Alternatives rejected

- allowing a CDT package to interpret invalid constraints;
- epsilon-based segment intersection or point-in-polygon tests;
- silently closing an open wire or deleting a repeated endpoint;
- reversing loop order inside validation;
- accepting multiple disjoint outers or nested islands as one OCCT face without
  an explicit decomposition contract;
- stopping at the first failure and losing validation-coverage evidence.

## Verification

`weft_planar_trim_validation_tests` covers a valid outer/hole domain, a valid
triangle, a subnormal-coordinate domain, open and empty inputs, invalid
provenance, repeated vertices, non-finite UV, a bow tie, adjacent overlap,
crossing loops, an external hole, a nested hole, wrong orientation, and a
missing predicate backend. Valid fixtures require complete non-vacuous
coverage. Invalid fixtures require stable named diagnostics and explicit
skipped counts for checks that could not safely run.

The target passes MSVC warnings-as-errors and `/analyze` builds and tests.

## Consequences

The next CDT backend receives a proof-carrying simple planar domain rather than
raw sampled rings. The current contract deliberately supports one outer with
direct holes; cut graphs and multi-region decomposition remain later gated
work. Assembly of these loop records from face/coedge topology is still an M3
dependency and no M4 tessellation claim follows from this ADR alone.
