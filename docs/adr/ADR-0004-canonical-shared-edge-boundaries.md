# ADR-0004: Canonical shared-edge boundaries

- Status: Accepted
- Date: 2026-07-17
- Owners: Weft geometry core

## Context

Sampling each face trim independently cannot guarantee identical positions,
ordering, or vertex identity on a shared B-rep edge. Positional welding hides
that representation loss and cannot recover a proof that adjacent faces use the
same boundary. Planar STEP faces also commonly lack stored p-curves even when
their exact 3D edge and plane are valid.

## Decision

Each working B-rep edge owns one immutable increasing-curve-parameter sample
sequence. Its stable boundary ID is derived from the stable working edge ID.
Every sample carries:

- a stable sample ID and deterministic canonical vertex index;
- exact working-edge parameter and position;
- source and working edge provenance;
- every owning coedge/face use, its traversal orientation, raw and lifted UV,
  mapping kind, source face, and measured curve-on-surface discrepancy.

Open-edge endpoints reuse indices derived from their B-rep vertex IDs. Interior
indices are allocated once after the topological vertex range. Supporting-curve
periodicity does not close a trimmed edge: only identical B-rep endpoint
vertices do. This distinction prevents a partial circular arc from losing its
upper endpoint.

Stored p-curves compose through `GeometryEvaluator`. A proven plane with no
stored p-curve uses the evaluator's named planar inverse mapping and records it
as `DerivedPlanarProjection`. Curved faces never receive that exception.
Periodic surface coordinates are lifted to the nearest continuous covering-
space use. Discrepancy is bounded against the source edge/face tolerance
envelope and an absolute safety cap.

Construction is atomic. Any missing count, endpoint, coedge, provenance,
mapping, evaluation, or bounded tolerance returns a stable named refusal and no
partial boundary set. Degenerate singular edges remain explicitly unsupported.

## Invariants

- one canonical boundary exists per accepted working edge;
- a sample's 3D identity is independent of the number of owning faces;
- topologically shared endpoints have the same canonical vertex index;
- every UV use maps back to source and working geometry;
- expected and checked edge, sample, and UV-use counts are equal and non-zero;
- no canonical boundary path welds or substitutes face-local samples.

## Alternatives rejected

- face-local trim sampling followed by positional welding;
- treating every periodic supporting curve as a topologically closed edge;
- synthesising curved-face p-curves when source representations are missing;
- returning completed edges when another edge fails;
- accepting large repair tolerances as an unbounded discrepancy allowance.

## Verification

`weft_canonical_boundary_tests` proves atomic box and cylinder construction,
shared endpoint indices, source/working provenance, stored and derived planar UV
uses, non-vacuous coverage, and named missing-count refusal. An adversarial
quarter-circle fixture proves that a trimmed periodic curve remains open. The
ported azimuth registration proves identity and cyclic phase alignment while
refusing reflection and non-uniform input on either ring.

## Consequences

Face CDT and templates must consume canonical sample IDs and vertex indices.
Critical-interval adaptive sampling, degenerate singularities, and complete
periodic-loop closure validation remain later M3 increments.
