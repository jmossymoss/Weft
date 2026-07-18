# ADR-0031: Compatibility correspondence (valid copy and heal lane)

- Status: Accepted
- Date: 2026-07-17
- Owners: Weft geometry core

## Context

Compatibility previously always ran `healWithHistory` (sew / ShapeFix /
UnifySameDomain) after a geometry-deep copy. Sewing can demote solids to
shells and rewrite wires without history entries, so StableId occurrence
ordinals diverged and topology correspondence stayed incomplete even for
already-valid cylinders and boxes. Deep copies also materialize planar
p-curves, so requiring equal coedge p-curve representation lists rejected
otherwise one-to-one accounts.

## Decision

- `CompatibilityWorkingDerivation` carries an `ExactShapeDerivationMap`.
- After `BRepBuilderAPI_Copy`, bind every unique source TShape to its copy.
- When the deep copy is already `BRepCheck`-valid, retain that copy and skip
  sew/ShapeFix so occurrence ordinals stay one-to-one.
- Invalid copies run the historical heal pipeline. The derivation map then:
  - rebinds through one-to-one Modified/Generated results;
  - collapses partner-identical multi-Modified images;
  - keeps images that still appear in the healed body;
  - demotes a sole solid to the sole healed shell when sewing removes solids;
  - pairs face-local wires after face images are known (wires are not a
    `BRepTools_History` family);
  - snaps unbound vertices to the nearest healed vertex within `1e-3` mm.
- Topology matching allows Solid↔Shell and Compound↔Shell kind pairs when
  cardinality-changing heal rewrites the body.
- Topology assembly equality requires equal coedge p-curve representations
  only for representation-identity (Conservative identity) accounts.
  Compatibility Modified accounts compare coedge topology without demanding
  equal stored p-curve lists.
- Representation copy-on-write reconciliation remains Conservative-only;
  Compatibility deep copies may change stored p-curve cardinality without a
  COW operation.

## Invariants

- valid Compatibility imports of simple solids complete topology and entity
  correspondence;
- Compatibility heal of the reviewed inverted-shell fixture completes
  correspondence and is meshable when the healed working shape is valid;
- heal paths that split one source into multiple non-partner results emit
  `import.heal.multi_way_split` (or `import.heal.correspondence_unbound` when
  subjects remain unbound) and stay non-meshable;
- Conservative identity still requires equal p-curve representation lists;
- Compatibility remains distinct from Conservative topology isolation.

## Alternatives rejected

- forcing sew on already-valid deep copies;
- treating Compatibility deep-copy p-curve materialization as unexplained
  Conservative COW failures;
- claiming complete correspondence through arbitrary multi-way splits without
  a unique partner image.

## Verification

`testCompatibilityIsAudited` on the generated cylinder and native box
Compatibility imports assert complete topology correspondence and meshability
when the deep copy is valid. `testCompatibilityHealLaneCorrespondence` on
`corrupt.orientation.inverted_shell_face.brep` asserts the historical heal
detail, complete correspondence, and meshability. See
`docs/evidence/m1-compatibility-correspondence-2026-07-17.md`.

## Consequences

M1 Compatibility correspondence is closed for valid deep-copy imports and for
the reviewed heal-lane fixture. Multi-way splits without a unique partner image
fail closed with a named heal refusal. Provable conservative sewing is covered
by ADR-0032. The M1 certificate-or-named-refusal gate is closed; see
`docs/evidence/m1-certificate-or-named-refusal-2026-07-17.md`.
