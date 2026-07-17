# ADR-0008: Exact single-loop reference CDT

- Status: Accepted
- Date: 2026-07-17
- Owners: Weft geometry core

## Context

The rewrite needs a boundary-preserving constrained-Delaunay safety floor that
is independent of OCCT triangle soup and post-hoc welding. CGAL 6.2 remains the
planned research comparison, but it is not installed and its GPL/commercial
licensing prevents treating it as an unresolved distributable dependency.

The exact trim validator now proves a planar straight-line boundary, and the
exact dyadic predicate backend provides orientation, incircle, and segment
signs. A first compiled triangulation increment can therefore operate entirely
on existing boundary vertices without constructing intersection coordinates.

Lawson's original triangulation work introduced diagonal improvement by local
flips; the primary 1977 report is archived by
[NASA NTRS](https://ntrs.nasa.gov/citations/19770025881). Later constrained
triangulation results establish local Delaunay legality away from required PSLG
segments; the implementation remains behind a narrow backend so it can be
compared with [CGAL's constrained-Delaunay package](https://doc.cgal.org/latest/Triangulation_2/index.html)
without changing public geometry contracts.

## Decision

`PlanarCdtBackend` is the narrow planar triangulation interface. The first
distribution-safe backend is `exact_lawson_single_loop_reference`:

1. run exact planar trim validation;
2. accept exactly one counter-clockwise outer loop and retain its canonical
   vertex order and source/working provenance;
3. create a deterministic ear triangulation using exact orientation and
   inclusive exact point-in-triangle decisions;
4. preserve every boundary edge as an immutable constraint;
5. visit internal edges in sorted order and flip a convex edge only when the
   exact incircle sign is strictly positive;
6. retain the existing diagonal for an exact cocircular tie;
7. independently validate the complete result before returning any mesh.

The independent validation covers vertex and triangle provenance, `n-2`
triangle cardinality, positive exact orientation, boundary identity, edge
incidence, every unordered mesh-edge relation, and exact local Delaunay
legality for every internal edge. Each family reports
`expected/checked/skipped/failed`.

## Invariants

- no boundary constraint is flipped, resampled, snapped, welded, or omitted;
- no new geometric point is constructed; all triangle vertices resolve to an
  input canonical sample and working/source edge provenance;
- every triangle carries working and optional source face provenance;
- strict positive incircle is the only flip trigger;
- deterministic input and predicate results produce deterministic triangle
  ordering;
- any trim, predicate, ear, flip, provenance, incidence, intersection, or
  Delaunay failure returns no mesh and a stable refusal code.

## Alternatives rejected

- OCCT face triangulation as an authoritative correctness floor;
- unconstrained Delaunay followed by boundary snapping;
- epsilon-based ear, convexity, or incircle decisions;
- importing an unavailable CGAL configuration while claiming it was tested;
- distributing a prototype-only GPL dependency;
- treating a structured-template failure as permission to bypass certified
  triangulation.

## Verification

`weft_planar_cdt_tests` covers triangles, an exactly cocircular square, a
quadrilateral that requires a Lawson flip, a concave polygon, 12 collinear
boundary-density levels, 64 deterministic star-shaped domains run twice, and a
subnormal-coordinate square. It also proves named refusal of an invalid trim,
a valid perforated domain, and a missing predicate backend.

Every successful case requires complete non-vacuous validation evidence. The
target passes MSVC warnings-as-errors and `/analyze` builds and tests.

## Consequences

M4 can begin with a real compiled, exact-predicate, boundary-preserving planar
triangle floor. This reference is intentionally not the release-performance
backend and does not yet support holes. Constraint recovery or a proven cut
graph, face/coedge loop assembly, surface lifting and adaptive error bounds,
cross-face canonical assembly, self-intersection protection, and independent
body validators remain required before M4 can pass or production routing can
use the result.
