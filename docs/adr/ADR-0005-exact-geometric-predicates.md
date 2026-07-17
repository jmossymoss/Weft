# ADR-0005: Exact finite-double geometric predicates

- Status: Accepted
- Date: 2026-07-17
- Owners: Weft geometry core

## Context

Orientation, segment-intersection, and incircle signs control trim validity and
CDT topology. A wrong near-zero sign can change containment, accept a crossing,
or flip a Delaunay edge. Ordinary floating-point determinant evaluation is not
sign-correct under cancellation or underflow.

CGAL 6.2 is not installed in the current development environment. Treating an
epsilon or OCCT's legacy face triangulator as an equivalent fallback would
violate the secure-core contract. Shewchuk's adaptive predicates remain the
planned performance replacement; his research explains adaptive exact sign
evaluation and publishes the predicate code in the public domain:
[Fast Robust Predicates for Computational Geometry](https://www.cs.cmu.edu/~quake/robust.html).

## Decision

`GeometricPredicates` is the narrow geometry-core interface for `orient2d`,
`incircle`, and segment classification. The first compiled reference backend
decodes every finite IEEE-754 double into its exact dyadic rational and evaluates
the determinant using a self-contained unbounded signed integer.

The backend is exact for the supplied finite doubles. Non-finite coordinates
fail with `predicate.non_finite_input`. Segment classification is derived only
from exact orientations and exact comparisons and distinguishes no contact,
proper crossing, endpoint contact, and collinear overlap.

This reference backend is distribution-safe and intentionally prioritises
auditability over throughput. It does not claim adaptive-predicate performance,
exact constructions for generated intersection coordinates, or a CDT backend.

## Invariants

- no epsilon changes a determinant sign;
- zero means the determinant of the actual input doubles is exactly zero;
- subnormal and widely separated exponents retain their exact dyadic value;
- segment relation uses the same exact orientation contract as CDT validators;
- backend identity and exactness are queryable at runtime;
- invalid coordinates fail by stable code.

## Alternatives rejected

- fixed or scale-relative orientation epsilons;
- `long double`, whose precision and behaviour vary by platform;
- calling ordinary determinants exact because most fixtures are well-scaled;
- enabling an uninstalled/unverified CGAL route;
- importing Triangle itself: Triangle is copyrighted even though the separate
  Shewchuk predicate code is public domain.

## Verification

`weft_geometric_predicate_tests` checks 1,000 integer orientation determinants
and 500 integer incircle determinants against an independent bounded-integer
reference. Adversarial cases prove:

- an orientation where ordinary double evaluation returns zero but the exact
  sign is negative;
- an incircle determinant where ordinary evaluation is negative but the exact
  sign is positive;
- a positive subnormal orientation whose double products underflow to zero;
- exact proper, endpoint, overlap, disjoint, and point-segment relations;
- named refusal for a NaN coordinate.

The suite passes both MSVC warnings-as-errors and static-analysis lanes.

## Consequences

Trim validation and every CDT backend must receive `GeometricPredicates`
instead of embedding arithmetic. A later Shewchuk adaptive backend can optimise
the same interface and must reproduce this reference battery. CGAL research CDT
remains a separate, explicitly licensed backend milestone.
