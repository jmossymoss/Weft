# ADR-0009: Canonical planar trim assembly

- Status: Accepted
- Date: 2026-07-17
- Owners: Weft geometry core

## Context

Canonical boundaries are edge-owned, while a planar CDT consumes face-owned
ordered loops. Building those loops is not a simple concatenation: coedge
orientation controls sample traversal, adjacent edges contribute different
sample records to the same topological corner, a closed circular edge has no
repeated terminal sample, and reversed face topology may disagree with the
canonical CDT orientation convention.

Discarding one corner record would lose edge provenance. Comparing corner UVs
with a tolerance and selecting one would silently change the face boundary.
Inferring holes from orientation alone would also confuse topology orientation
with geometric containment.

## Decision

`assemblePlanarTrimDomain` is the only initial adapter from secure B-rep face
topology and `CanonicalBoundarySet` into planar trim loops.

For a proven planar face it:

- groups face coedges by wire and requires contiguous `ordinalInWire` order;
- traverses each immutable boundary forward or backward from the coedge's
  recorded orientation;
- requires exactly one matching lifted UV use for every coedge/sample pair;
- merges adjacent endpoints only when their canonical vertex indices and lifted
  UV doubles agree exactly;
- retains all incident `(sample, working edge, source edge, coedge, UV)` records
  on the merged trim vertex;
- recognises a one-coedge closed canonical boundary without inventing a
  duplicate endpoint;
- obtains the outer-wire identity from exact OCCT face topology and labels all
  other wires as holes;
- reverses a complete loop only when needed to reach the CDT convention, and
  records `reversedForCanonicalCdt`;
- runs the independent exact trim-domain validator before returning output.

Every stage reports `expected/checked/skipped/failed` coverage. Any missing or
ambiguous mapping, invalid traversal, open junction, exact UV disagreement,
orientation failure, or validation failure returns no domain.

## Invariants

- every canonical boundary sample consumed by the face is accounted for;
- shared corners retain both incident boundary-use records;
- no UV value is averaged, snapped, wrapped again, or replaced during assembly;
- each non-closed coedge junction is the same canonical vertex on both sides;
- the final-to-first junction is proved, except for the explicit closed-edge
  canonical representation;
- topology identifies outer versus hole; exact containment independently checks
  that declaration;
- loop reversal changes traversal convention only and is always visible.

## Alternatives rejected

- face-local resampling;
- retaining only the preceding or following edge at a shared corner;
- tolerance-welding endpoint UVs;
- classifying outer/hole solely from signed UV orientation;
- adding a duplicate endpoint to a closed canonical circle;
- passing partially assembled loops to triangulation.

## Verification

`weft_planar_trim_assembly_tests` builds secure imports, reconnaissance,
intervals, and canonical boundaries from real generated STEP fixtures. It
proves:

- all six box faces assemble, retain two boundary uses at each of four corners,
  pass exact trim validation, and pass the reference CDT;
- both cylinder caps assemble from one closed circular coedge with 16 samples
  and pass the reference CDT;
- the cylindrical wall refuses planar assembly by name;
- at least one perforated face in the through-hole fixture assembles a topology-
  identified outer and hole, passes trim validation, and reaches the current
  CDT's named hole-support boundary;
- changing one incident lifted UV by `1e-12` causes
  `trim_assembly.vertex_uv_mismatch` and no output.

The target passes MSVC warnings-as-errors and `/analyze` builds and tests.

## Consequences

The planar reference floor now consumes actual secure B-rep topology rather
than hand-authored polygon tests. General periodic/singular faces, repeated wire
occurrences, hole-capable CDT recovery, and corpus-wide face assembly remain
open. Exact endpoint disagreement is intentionally a repair/import issue, not an
assembly convenience.
