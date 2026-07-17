# ADR-0027: Bounded curve-on-surface tolerance envelope repair

- Status: Accepted
- Date: 2026-07-17
- Owners: Weft geometry core

## Context

An edge can retain exact 3D/p-curve ranges and stored p-curve uses whose
measured curve-on-surface discrepancy exceeds the stored edge tolerance. The
reviewed beyond-threshold witness is invalid under
`BRepCheck_InvalidCurveOnSurface` while the within-threshold sibling remains
valid without repair. ADR-0024 already measures that discrepancy for flag
reconciliation and refuses when the maximum exceeds the source tolerance. The
conservative profile still lacked a bounded way to raise only the isolated
working tolerance to the proven envelope.

## Decision

After topology isolation and optional SameParameter/SameRange repair, the
conservative profile may raise a working edge tolerance when:

- the source edge is non-degenerate and retains a bounded 3D curve;
- every owning face and every stored p-curve use is enumerated, including both
  seam uses;
- each stored p-curve range equals the 3D range exactly;
- deterministic non-parallel `GeomLib_CheckCurveOnSurface` completes for every
  expected use with a finite non-negative maximum;
- checked uses equal expected uses and are non-zero;
- the measured maximum is strictly greater than the source edge tolerance.

The operation sets only the working TEdge tolerance to that measured maximum
via `BRep_Builder::UpdateEdge`. It does not change curves, p-curves, ranges,
SameParameter/SameRange, topology cardinality, or source TShapes. Tolerance
repair runs only while the working body is invalid. Each candidate raise is
validated first on a disposable geometry-deep copy; the live working shape is
mutated only after that probe becomes valid. Already-valid imports therefore
remain digest-identical, and failed trials cannot leave vertex-tolerance
residue.

The certificate records `ToleranceChange` with before/after, expected/checked
uses, and maximum discrepancy, plus a matching
`repair.tolerance_envelope` operation and non-vacuous
`repair.tolerance_envelope_reconciliation` evidence. Unexplained tolerance
deltas without proof fail that evidence.

Within-threshold disagreement that does not exceed the source tolerance is left
unchanged and remains an identity import.

## Invariants

- the immutable source tolerance never changes;
- working tolerance rises only to the proven measured maximum;
- no p-curve is synthesized or range-shifted in this operation;
- a failed proof or non-validating raise leaves the working edge unchanged;
- vacuous or tampered checked-use evidence blocks meshing.

## Alternatives rejected

- open-ended ShapeFix / healing tolerance inflation (BR-002);
- raising tolerance without full stored-use measurement;
- rewriting p-curves to fit the old tolerance (BR-003);
- accepting beyond-threshold residuals without a working mutation.

## Verification

`weft_secure_core_tests` proves
`corrupt.edge.pcurve_disagreement_beyond_tolerance.brep` becomes valid and
meshable with one certified tolerance envelope change,
`corrupt.edge.pcurve_disagreement_within_tolerance.brep` remains identity, and a
tampered `checked=0` certificate is non-meshable. Existing parameterization and
orientation lanes remain green under strict MSVC and MSVC `/analyze`.

## Consequences

M1 gains a third bounded conservative repair. Sewing, copy-on-write geometry
replacement, native-unit inference, IGES, multi-body orientation, product STEP
repair witnesses, and compatibility correspondence remain open.
