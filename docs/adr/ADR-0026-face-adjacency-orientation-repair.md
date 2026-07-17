# ADR-0026: Face-adjacency orientation repair

- Status: Accepted
- Date: 2026-07-17
- Owners: Weft geometry core

## Context

`BR-015` rejects whole-solid reversal as a general orientation repair. The
reviewed native witness `corrupt.orientation.inverted_shell_face.brep` reverses
the shell occurrence and one face token while retaining every geometry and
tolerance byte. Signed volume and infinite-point classification can look
repaired after a solid reverse while `BRepCheck_BadOrientationOfSubshape`
remains.

Conservative import therefore needs an orientation repair that proves
two-manifold face adjacency, rebuilds only occurrence orientations, chooses
global polarity independently, and leaves the immutable source unchanged.

## Decision

When the topology-isolated working root is an invalid closed solid, or a
compound/compsolid of free solids containing at least one invalid closed
solid, the conservative profile may repair each invalid solid independently:

1. absorb a reversed shell occurrence into solid-relative face orientations;
2. build the two-manifold face-edge adjacency of the closed shell and refuse
   open, non-manifold, non-FORWARD/REVERSED, or non-orientable shells;
3. solve a parity assignment so every shared edge is traversed in opposite
   directions;
4. rebuild a forward shell and solid from the solved face orientations only;
5. apply an independent polarity check (positive finite volume and infinite
   point outside) and reverse all solved face orientations if needed;
6. commit the rebuilt solid into the working root only when
   `BRepCheck_Analyzer` reports that solid valid.

The operation never calls `BRepLib::OrientClosedSolid` as the sole fix. It does
not change geometry handles, tolerances, stored p-curves, or topology
cardinality. Face TShapes remain partners; shell and solid TShapes are rebound
through the exact derivation map and history.

Source-to-working occurrence correspondence may differ in orientation when the
underlying TShape mapping remains one-to-one. Wire/coedge orientations stay
equal for this operation. The certificate records `OrientationChange` evidence,
matching `repair.orientation_face_adjacency` operations, and non-vacuous
`repair.orientation_reconciliation` validation coverage.

If the solved rebuild remains invalid, the working shape is left unchanged so
non-orientation pathologies keep identity correspondence.

## Invariants

- the immutable source orientations and TShapes never change;
- only already-invalid closed solids under a solid/compound/compsolid root
  are eligible; valid solids in a multi-body root stay byte-stable;
- adjacency proof requires exactly two FORWARD/REVERSED uses per non-degenerate
  shared edge;
- polarity-only mutation without an adjacency or shell-occurrence change is
  refused;
- a failed or non-validating proposal leaves that working solid unchanged;
- meshability still requires complete correspondence and repair evidence.

## Alternatives rejected

- `BRepLib::OrientClosedSolid` alone, because it can clear volume/infinite-point
  symptoms while leaving shell incoherence;
- unconditional shell rebuild for every solid, because it breaks digest identity
  for valid imports and unrelated invalid fixtures;
- polarity-only global face reversal without adjacency evidence;
- mutating wire/coedge orientations in this operation.

## Verification

`weft_secure_core_tests` proves the PAT-009 witness becomes valid and meshable
with orientation-only Modified correspondence, refuses certificate tampering
with `import.repair.validation_incomplete`, leaves
`corrupt.wire.inconsistent_orientation.brep` and non-orientation edge
pathologies unrepaired by this operation, and repairs an inverted solid that
shares a compound root with a valid box. Existing identity and parameterization
lanes remain green under strict MSVC.

## Consequences

M1 gains a second bounded conservative repair. BR-015 remains enforced: solid
reversal alone is still not an authorized repair. Multi-body compounds, open
shells, INTERNAL/EXTERNAL policy, and wire-only orientation repair remain open.
