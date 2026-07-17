# ADR-0026: Occurrence-only face-adjacency orientation repair

- Status: Accepted
- Date: 2026-07-17
- Owners: Weft geometry core

## Context

The orientation reconnaissance (see
`docs/evidence/m1-orientation-reconnaissance-2026-07-17.md`) rejected
whole-solid reversal as an orientation fix: OCCT's `OrientClosedSolid`
classifies only the infinite point and reverses the solid occurrence, so an
incoherently wound shell stays incoherent and
`BRepCheck_BadOrientationOfSubshape` persists. The reviewed witness
`corrupt.orientation.inverted_shell_face.brep` retains every geometry and
tolerance byte of the clean box yet reports signed volume `-5120 mm^3`,
because exactly two topology-token orientations were inverted.

Positive signed volume and an outside infinite point are necessary polarity
evidence, but they do not prove face-adjacency coherence. A conservative
orientation repair therefore needs two independent proofs: a two-manifold
parity solution over shared-edge uses, and a global polarity selection for
the coherent assignment.

## Decision

- Add `repairShellOrientations` as a bounded conservative repair stage after
  the topology-isolated identity copy and the bounded parameterization
  reconciliation.
- A repair candidate is a solid whose single shell violates shared-edge
  parity: some non-degenerate edge is traversed in the same direction by its
  two composed face uses. Coherent shells are never touched, so identity
  imports pay only a topology walk and no geometric cost.
- The parity proof requires a closed two-manifold shell: every
  non-degenerate edge has exactly two face uses (a seam counts as two uses by
  one face), all occurrence orientations are two-sided
  (`FORWARD`/`REVERSED`), no face definition repeats inside the shell, and
  the face-adjacency constraint graph is connected and two-colourable. The
  solution is unique up to one global sign.
- The global polarity is selected independently of the parity solve: each of
  the two coherent assignments is materialized as a trial solid over the
  same face definitions, and the accepted assignment must have a finite
  positive signed volume, classify the infinite point outside, and pass
  `BRepCheck` native validity.
- The repair rebuilds only occurrence orientations, with the fewest flips
  that realize the accepted assignment: either flip the disagreeing face
  occurrences, or flip the shell occurrence plus the complement. TShapes,
  child order, geometry handles, p-curves, tolerances, flags, locations, and
  topology cardinality are untouched.
- Every certified flip is recorded in the repair certificate as a
  `ShellOrientationRepair` with non-vacuous manifold-edge coverage and the
  numeric polarity evidence. `buildImportedModel` re-audits each record
  against the authoritative snapshots — stored orientations must differ at
  exactly the certified subjects, and the working solid must independently
  re-prove positive volume and an outside infinite point — before the
  `repair.face_adjacency_orientation` validation evidence can complete.
- Source-to-working occurrence correspondence admits an exact
  `FORWARD`/`REVERSED` inversion only for occurrences whose definition is a
  certified flip subject; every other structural difference still breaks the
  correspondence.
- Every detected candidate that cannot be proved refuses by a stable named
  code recorded as a `RepairRefusal` in the certificate
  (`repair.orientation.shell_open`,
  `repair.orientation.shell_nonmanifold_edge`,
  `repair.orientation.non_orientable`,
  `repair.orientation.adjacency_disconnected`,
  `repair.orientation.polarity_unproven`, scope refusals for multi-shell,
  shared-definition, repeated-face, non-two-sided, and non-forward-solid
  configurations, and `repair.orientation.analysis_failure`). A refusal
  emits `import.repair.face_orientation_unproven` and forces the import
  non-meshable regardless of what `BRepCheck` happens to accept.

## Invariants

- coherent shells and non-candidate solids are bit-identical to the source
  and keep their identity certificates;
- a repaired working copy differs from its immutable source only in
  occurrence orientation tokens of the certified shell and face subjects;
- the parity proof covers every non-degenerate shell edge with non-zero
  expected/checked counts; vacuous coverage cannot validate;
- polarity is never inferred from the parity solve alone, and the audit
  recomputes it from the working solid without reusing the derivation's
  measurements;
- a tampered certificate (zero checked coverage, missing flip subjects, or
  wrong polarity claims) fails `import.repair.validation_incomplete` even
  when the working shape is structurally valid;
- refusals are deterministic: manifold defects are counted before naming so
  hash iteration order cannot change the recorded code.

## Alternatives rejected

- whole-solid or whole-shell reversal keyed on signed volume or infinite
  point alone (rejected by the reconnaissance witness);
- normalizing coherent-but-inverted shells in the same increment (requires a
  signed-volume sweep over every imported solid; deferred as a named open
  item);
- repairing multi-shell solids, shared shell definitions, or free-standing
  shells in this increment;
- `ShapeFix_Shell`/`ShapeFix_Solid` orientation healing (unbounded, no
  per-subject proof, breaks source/working correspondence);
- deleting or re-sewing faces to force coherence;
- accepting the parity solution without native validity, volume, and
  infinite-point gates on the exact repaired assignment.

## Verification

`corrupt.orientation.inverted_shell_face.brep` imports through the public
native API: the source stays invalid and untouched, the working copy becomes
valid and meshable with exactly one shell-occurrence reversal plus one face
flip, twelve of twelve manifold edges checked, and the infinite point
outside. The incoherent source integrates to `-5120 mm^3` because the
inverted face's contribution flips sign; the repaired working solid recovers
the full `+7680 mm^3` of the reviewed 24x20x16 baseline box. Complete one-to-one `Modified`
correspondence holds for every occurrence, with no tolerance,
representation, p-curve, or cardinality changes.

Adversarial witnesses derived in-test from the reviewed box baseline prove
the open-shell and non-orientable refusals leave the working copy identical
and non-meshable with the named diagnostic, and a structurally valid repair
whose certificate is tampered to `checked=0` fails
`import.repair.validation_incomplete`.

## Consequences

The first working-copy orientation defect family is now repairable with a
complete proof chain, and every unproved orientation candidate fails closed
by name instead of relying on downstream validity checks. M1 remains in
progress: coherent-but-inverted polarity normalization, multi-shell and
shared-shell orientation scopes, bounded tolerance reconciliation, provable
sewing, and the remaining repair families are still open.
