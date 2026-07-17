# ADR-0012: Canonical topological vertex positions

- Status: Accepted
- Date: 2026-07-17
- Owners: Weft geometry core

## Context

Sampling each incident edge at a shared B-rep vertex does not guarantee
bit-identical coordinates. Distinct exact curve evaluations may differ within
the model's tolerance even though topology says they are one vertex. Assigning
the same canonical index to those different coordinates postpones the conflict
until body assembly and invites a spatial weld.

The source/working correspondence originally covered faces and edges only, so
there was also no audited source vertex through which to bound normalization.

## Decision

- `BRepSnapshot` retains immutable indexed wire and vertex maps.
- `GeometryEvaluator::evaluateVertex` is the only route used to obtain an
  exact B-rep vertex position.
- Source/working correspondence now accounts solids, faces, wires, edges, and
  vertices. Coedge-occurrence correspondence remains a separate open gate.
- Every canonical edge endpoint evaluates both its exact curve and topological
  vertex. The discrepancy must fit the scaled source edge-plus-vertex tolerance
  envelope and the configured absolute cap.
- After that proof, all incident endpoint samples use the topological vertex
  position as their one authoritative coordinate.
- Face-use discrepancy envelopes include the source vertex tolerance at an
  endpoint.
- The canonical report records non-vacuous expected and checked vertex/curve
  comparisons.

## Invariants

- one canonical topological vertex index has one bit-identical 3D position;
- normalization never uses proximity search or a mesh weld;
- an out-of-envelope curve/vertex disagreement refuses the complete boundary
  set by `boundary.vertex_curve_discrepancy`;
- missing source/working vertex correspondence refuses by name;
- interior edge samples remain exact curve evaluations.

## Alternatives rejected

- retaining separately evaluated endpoint positions under one index;
- choosing the first incident edge position without a tolerance proof;
- averaging incident positions;
- welding the resulting mesh;
- accepting the working vertex without source correspondence.

## Verification

The canonical box, cylinder, and partial-arc fixtures now require every sample
sharing a canonical index to have exactly the same coordinate. They also
require non-zero, complete vertex/curve coverage. The immutable evaluator test
resolves a source vertex directly, and the repair certificate remains complete
with the expanded solid/wire/vertex correspondence.

Both warnings-as-errors and MSVC static-analysis suites cover the change.

## Consequences

Canonical boundary identity is now strong enough for curved faces whose seam
curve and rim curves meet only within source tolerance. Compatibility repairs
that cannot prove vertex correspondence remain non-meshable. Full assembly,
instance, and coedge-occurrence correspondence are still open M1 work.
