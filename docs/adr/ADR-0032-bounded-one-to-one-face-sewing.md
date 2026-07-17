# ADR-0032: Bounded one-to-one face sewing

- Status: Accepted
- Date: 2026-07-17
- Owners: Weft geometry core

## Context

M1 still required a conservative sewing path that preserves face identity and
refuses the reviewed gap fixtures. Legacy `healWithHistory` sewing can demote
solids and is confined to Compatibility. A first bounded sew must prove a
face-preserving free-edge merge without silently closing authored gaps.

## Decision

- Conservative sewing applies only to open face compounds/shells with no
  solids and at least two free edges.
- Sew tolerance is fixed at `1e-4` mm. Faces are added individually.
- Commit only when face count is preserved and free-edge count decreases.
- Exact-shape rebinds cover faces, face-local wires, edges, and vertices;
  `repair.sewing_one_to_one` records the free-edge delta.
- Reviewed gap fixtures (`corrupt.wire.gap_within_beyond`, large trim gaps)
  remain unsewn because they are not face-compound sew targets or exceed the
  tolerance.
- Topology correspondence may use derivation-backed matching when StableId
  ordinals shift after merges. Meshable certificates still require complete
  absorbed-edge/vertex correspondence; that last mile remains open when sewing
  leaves Introduced working edges/vertices.

## Invariants

- closed solids are never sewn on the conservative path;
- face cardinality does not change;
- gap wire/trim fixtures do not receive `repair.sewing_one_to_one`;
- meshability still demands complete correspondence.

## Alternatives rejected

- always sewing through `healWithHistory` on conservative imports;
- sewing valid closed solids;
- treating free-edge merge as meshable without complete correspondence.

## Verification

`testBoundedSewingRepair` proves a two-face compound with a `1e-5` mm gap
records `repair.sewing_one_to_one`, reduces edge count, preserves faces, and
does not sew `corrupt.wire.gap_within_beyond`. See
`docs/evidence/m1-bounded-sewing-2026-07-17.md`.

## Consequences

Provable sewing is partially closed: geometric face-preserving sew is
certified as an operation, but meshable sew certificates wait on complete
absorbed-edge/vertex correspondence.
