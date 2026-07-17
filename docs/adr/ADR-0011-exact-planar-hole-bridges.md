# ADR-0011: Exact planar hole bridges

- Status: Accepted
- Date: 2026-07-17
- Owners: Weft geometry core

## Context

A production tessellation floor cannot reject every valid perforated face.
Validated planar domains already prove one outer loop, directly contained
holes, disjoint constraints, canonical orientation, and domain-wide unique
sample/vertex identity. The single-loop reference CDT can triangulate any
simple boundary walk and independently certify its constraints and topology.

Naively keyholing a hole can duplicate mesh vertices, cross a concavity, or
silently choose a tolerance-based bridge. A secure cut must leave original
constraints untouched, introduce no geometric point, and fail when visibility
cannot be proven.

## Decision

The exact Lawson reference backend now supports directly contained planar
holes through deterministic visibility bridges:

1. sort holes by their exact lexicographically minimal vertex and stable wire
   identity;
2. choose that minimum as the hole endpoint;
3. test every outer-loop vertex using exact local-cone predicates at both the
   outer and clockwise hole endpoint;
4. reject any candidate whose segment has an exact crossing, overlap, or
   non-incident touch with an input constraint or prior bridge;
5. choose the shortest visible candidate using exact dyadic squared-distance
   comparison, breaking exact ties by stable local index;
6. splice the clockwise hole and the bridge twice into a weakly simple ear-cut
   walk;
7. keep only one mesh vertex per original canonical vertex - the duplicate bridge
   endpoints are walk occurrences, not output vertices;
8. preserve all outer/hole edges as immutable constraints, while allowing the
   artificial bridge to participate in exact Lawson flips;
9. independently recheck triangle count `V + 2H - 2`, boundary identity,
   incidence, every mesh-edge relation, and local Delaunay legality.

The algorithm constructs no intersection or midpoint coordinate. All topology
decisions use the exact finite-double predicate interface. Failure to find or
triangulate a proven bridge returns no mesh by stable code.

## Invariants

- original outer and hole sample IDs, coordinates, and edges are unchanged;
- no output vertex is duplicated to form a cut;
- a bridge cannot cross or overlap an input constraint or earlier bridge;
- bridges leave the outer interior cone and hole exterior cone correctly;
- hole ordering supplied by a caller does not change flattened vertex or
  triangle ordering;
- all hole constraints have one face-triangle incidence and all internal edges
  have two;
- every successful perforated result passes the same independent certificate as
  an unperforated result.

## Alternatives rejected

- continuing to refuse all planar holes;
- keyholing with spatially duplicated output vertices;
- ray intersections or bridge midpoints computed and trusted in ordinary
  floating point;
- connecting to the nearest vertex without cone and complete intersection
  checks;
- constraining the artificial bridge permanently when exact Delaunay flips can
  improve it safely;
- falling back to OCCT triangulation for perforated faces.

## Verification

The CDT battery now includes one square hole, two disjoint rectangular holes,
the same two holes in reversed caller order, and a hole inside a concave
U-shaped outer loop. It requires complete validation, exact triangle counts,
all original constraints, and deterministic triangle ordering.

The secure STEP through-hole fixture assembles a real topology-identified outer
and hole, triangulates it, and then passes certified open-face provenance,
surface, incidence, winding, and fingerprint validation. Requiring that lone
face product to be a closed body refuses by edge-incidence code.

Both warnings-as-errors and static-analysis lanes cover the implementation.

## Consequences

Ordinary directly contained planar holes no longer block the certified floor.
This does not prove bridge completeness for every highly degenerate or
multi-level cut graph; the validator still refuses nested islands, boundary
touches, and any domain without an exact visible bridge. Curved holes, critical
surface regions, and full body self-intersection remain open.
