# ADR-0028: Copy-on-write representation rules

- Status: Accepted
- Date: 2026-07-17
- Owners: Weft geometry core

## Context

ADR-0023 isolates conservative working topology while sharing exact curve,
surface, and curve-representation handles with the immutable source. Flag,
tolerance, and orientation repairs mutate only cloned topology state. Any
future repair that must replace stored geometry still needs an enforceable
rule so shared handles are never mutated in place and unexplained
representation deltas cannot become meshable.

## Decision

- During conservative identity and topology-only repairs, corresponding edge
  3D curves and face surfaces must remain the same OCCT handles.
- A geometry replacement is allowed only as copy-on-write: `EmptyCopied` the
  working TShape first, attach replacement handles to that clone, record a
  `RepresentationChange` with `certifiedCopyOnWrite=true`, and emit
  `repair.representation_copy_on_write`.
- Certificate validation proves shared-handle immutability for every
  conservative one-to-one edge/face correspondence that is not covered by a
  certified copy-on-write change.
- Any `RepresentationChange` without certified copy-on-write evidence fails
  `repair.representation_copy_on_write_reconciliation` and cannot be meshable.
- Compatibility remains a geometry-deep historical derivation and is exempt
  from the shared-handle immutability gate while its correspondence proof is
  incomplete.

## Invariants

- conservative shared geometry handles are reachable only through const
  evaluator contracts;
- unexplained stored-p-curve cardinality deltas fail closed;
- certified representation replacement cannot mutate a source-reachable
  handle in place;
- identity certificates retain empty representation changes.

## Alternatives rejected

- mutating shared `Geom_Curve` / `Geom_Surface` handles through the working
  edge or face;
- treating p-curve cardinality drift as an automatic identity-preserving
  repair;
- requiring geometry-deep copies for every conservative repair;
- applying the shared-handle gate to the incomplete compatibility lane.

## Verification

Identity native B-rep import, SameParameter/SameRange repair, tolerance
envelope repair, and face-adjacency orientation repair all produce complete
non-zero `repair.shared_geometry_immutable` evidence with empty unexplained
representation changes. See
`docs/evidence/m1-copy-on-write-representation-rules-2026-07-17.md`.

## Consequences

M1 now has an enforceable representation ownership contract ahead of any
geometry-replacing conservative repair. Sewing and other representation
mutations must implement the certified copy-on-write path before they can
become meshable.
