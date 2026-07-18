# ADR-0032: Bounded one-to-one face sewing

- Status: Accepted
- Date: 2026-07-17
- Owners: Weft geometry core

## Context

M1 required a conservative sewing path that preserves face identity and
refuses the reviewed gap fixtures. Legacy `healWithHistory` sewing can demote
solids and is confined to Compatibility.

## Decision

- Conservative sewing applies only to open face compounds/shells with no
  solids and at least two free edges.
- Sew tolerance is fixed at `1e-4` mm. Faces are added individually.
- Commit only when face count is preserved and free-edge count decreases.
- Exact-shape rebinds cover faces, face-local wires, edges, and vertices.
  Absorbed free edges rebind through `ContigousEdgeCouple` /
  `SectionToBoundary` onto the shared seam; remaining vertices snap within
  the sew tolerance envelope.
- Topology correspondence treats partner-identical working occurrences of one
  sewn seam TShape as one logical image (multiple face uses are not
  Introduced duplicates).
- Representation correspondence claims the sewed shell from the mapped source
  root when `Model.solids` gains a free shell. Sew-owned edge representation
  and tolerance deltas are certified through
  `repair.representation_copy_on_write` so shared-geometry audits remain
  complete.
- Reviewed gap fixtures (`corrupt.wire.gap_within_beyond`, large trim gaps)
  remain unsewn because they are not face-compound sew targets or exceed the
  tolerance.
- Operation code: `repair.sewing_one_to_one`.

## Invariants

- closed solids are never sewn on the conservative path;
- face cardinality does not change;
- gap wire/trim fixtures do not receive `repair.sewing_one_to_one`;
- meshable sew certificates require complete correspondence and complete
  repair validation evidence.

## Alternatives rejected

- always sewing through `healWithHistory` on conservative imports;
- sewing valid closed solids;
- treating free-edge merge as meshable without complete correspondence.

## Verification

`testBoundedSewingRepair` proves a two-face compound with a `1e-5` mm gap
records `repair.sewing_one_to_one`, reduces edge count, preserves faces,
completes correspondence, and is meshable; it does not sew
`corrupt.wire.gap_within_beyond`. See
`docs/evidence/m1-bounded-sewing-2026-07-17.md`.

## Consequences

Provable face-preserving sewing is closed for the bounded open-compound
lane. Compatibility heal-lane correspondence is covered separately in
ADR-0031.
