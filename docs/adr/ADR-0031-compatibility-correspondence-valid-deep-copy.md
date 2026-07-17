# ADR-0031: Compatibility correspondence for valid deep copies

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
- Invalid copies still run the historical heal pipeline; the derivation map
  rebinds only through one-to-one Modified/Generated results.
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
- invalid Compatibility imports may still be correspondence-incomplete when
  heal splits/removes shapes;
- Conservative identity still requires equal p-curve representation lists;
- Compatibility remains distinct from Conservative topology isolation.

## Alternatives rejected

- forcing sew on already-valid deep copies;
- treating Compatibility deep-copy p-curve materialization as unexplained
  Conservative COW failures;
- claiming complete correspondence through heal splits without history.

## Verification

`testCompatibilityIsAudited` on the generated cylinder and native box
Compatibility imports assert complete topology correspondence and meshability
when the deep copy is valid. See
`docs/evidence/m1-compatibility-correspondence-2026-07-17.md`.

## Consequences

M1 Compatibility correspondence is closed for valid deep-copy imports. Heal
paths that demote or split topology remain incomplete until one-to-one
history covers those cases. Provable sewing and product STEP repair witnesses
remain open.
