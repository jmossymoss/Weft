# ADR-0023: Topology-isolate the conservative working B-rep

- Status: Accepted
- Date: 2026-07-17
- Owners: Weft geometry core

## Context

`SourceBRep` and `WorkingBRep` were separate C++ objects, but conservative
import still placed the same OCCT TShapes in both. A later tolerance,
orientation, or representation edit through the working shape could therefore
alter retained source evidence.

The obvious OCCT copy routes were not representation identities.
`BRepBuilderAPI_Copy` with geometry copying enabled and disabled was exercised
against processing-disabled STEP input. Both routes materialized computed
planar p-curves as stored edge representations and changed the canonical native
B-rep digest. That is a representation change, not a valid identity copy.

## Decision

- Conservative import clones each unique source TShape exactly once with
  `TopoDS_Shape::EmptyCopied()`, rebuilds the direct child graph with
  `BRep_Builder`, and then restores all source TShape bookkeeping flags.
- Occurrence orientation and location stay on each occurrence; cloned TShapes
  preserve the source sharing graph while remaining distinct from every source
  TShape.
- Exact curve, surface, and curve-representation handles remain shared and
  read-only during identity import. A future conservative repair may mutate
  cloned topology state or replace a representation copy-on-write, but may not
  mutate a shared geometry handle in place.
- Exact XDE leaf uses are rebound through the copier's complete TShape map.
  OCCT `BRepTools_History` is used only for its supported vertex, edge, face,
  and solid families.
- Higher-order correspondence uses the copier's exact TShape-derivation map
  plus equal independently validated occurrence paths and metadata. Equal
  canonical native B-rep digests determine whether the mapped relation is an
  identity rather than a modification; a digest is never mapping evidence by
  itself. ADR-0024 supersedes the original digest/order mapping mechanism.
- Compatibility repair continues to receive a geometry-deep copy before any
  historical mutation and remains non-meshable until its non-identity
  occurrence correspondence is complete.

## Invariants

- source and conservative working roots are neither `IsSame` nor `IsPartner`;
- no authoritative source occurrence is a TShape partner of its working
  occurrence;
- conservative identity requires equal native B-rep SHA-256 digests;
- conservative identity has zero tolerance, stored-p-curve, and topology-count
  changes;
- every source and working topology account validates independently;
- every XDE leaf retains an exact working topology root;
- no computed p-curve is stored merely to create the working copy;
- a shared exact geometry handle is immutable and reachable only through const
  evaluator contracts during this stage.

## Alternatives rejected

- retaining shared source/working TShapes;
- `BRepBuilderAPI_Copy` with geometry copying enabled;
- `BRepBuilderAPI_Copy` with geometry copying disabled;
- serialization and re-import as a copy mechanism, because it changes the
  evidence channel and permits translator normalization;
- relying on `BRepTools_History` for compound, compsolid, shell, or wire
  identity, which OCCT does not support;
- calling a digest-changing copy an identity derivation.

## Verification

Commit `a62e6af` implements the copier, correspondence lane, XDE exact-use
rebinding, and corpus assertions. Every one of the 77 generated STEP fixtures
requires equal source/working native B-rep digests, zero repair deltas, complete
source/working topology validation and correspondence, distinct root TShapes,
and distinct TShapes for every authoritative occurrence.

An intermediate run deliberately exposed 18 assembly, compound, compsolid,
freeform, and orientation fixtures with
`topology.instance.topology_roots_missing`. The cause was using OCCT history for
unsupported higher-order shapes. Direct exact-use rebinding through the full
copy map corrected the defect; no validator or identity condition was relaxed.

The complete core, CLI, and desktop app compile under strict MSVC and MSVC
`/analyze`; both lanes pass all 14 registered tests. The identity evaluator
proof requires equal source/working curve, vertex, p-curve, surface, and
curve-on-surface results.

## Consequences

The conservative working topology can now evolve without rewriting source
TShapes, while identity import remains byte-for-byte representation exact.
M1 is not passed: bounded conservative repair operations, copy-on-write rules
for any replacement geometry, and complete non-identity correspondence still
require fixture-backed increments.
