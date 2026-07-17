# ADR-0024: Bound SameParameter/SameRange flag reconciliation

- Status: Accepted
- Date: 2026-07-17
- Owners: Weft geometry core

## Context

An OCCT edge can retain an existing 3D curve and face p-curves whose parameter
ranges and positions already agree while its `SameParameter` and `SameRange`
flags are false. Such a body is inspectable but cannot safely use evaluator or
mesher paths that rely on those contracts. Unconditionally setting the flags,
sampling a few points, or calling a general healing routine would convert an
assumption into apparent source truth.

The topology-isolated working copy from ADR-0023 permits a working TEdge flag
change without mutating the retained source TEdge. It also supplies an exact
TShape-derivation map for every topology family, so a modified working
occurrence no longer needs digest/order inference.

## Decision

The conservative profile may set both flags on a non-degenerate working edge
only when all of the following are true:

- the source has both flags false and retains a bounded 3D curve;
- every owning face and every stored p-curve use is enumerated, including both
  representations of a seam;
- no p-curve is missing, computed on demand, or synthesized;
- each stored p-curve range is exactly equal to the stored 3D-curve range;
- deterministic, non-parallel `GeomLib_CheckCurveOnSurface` evaluation
  completes for every use and its maximum distance is finite and no greater
  than the source edge tolerance;
- checked p-curve uses equal expected p-curve uses and are non-zero.

The operation changes only `SameRange` and `SameParameter` on the isolated
working TEdge. It does not change a curve, p-curve, surface, parameter range,
tolerance, topology cardinality, or source TShape. The repair certificate
records both stable edge IDs, before/after flags, expected/checked use counts,
maximum discrepancy, tolerance envelope, and a matching operation record.

An exact source-TShape to working-TShape derivation map is the correspondence
authority for all shape families. Native B-rep digest equality remains audit
evidence for identity, not a fallback mapping mechanism. A changed digest with
an exact one-to-one derivation is recorded as `Modified`.

If any proof obligation is absent, the edge is left unchanged and the body is
non-meshable with
`import.repair.same_parameter_range_unproven`. Incomplete or vacuous
certificate evidence additionally emits `import.repair.validation_incomplete`.

## Invariants

- the immutable source flags and source TShapes never change;
- the working topology remains one-to-one with the source topology;
- source/working tolerances, stored p-curve counts, and topology counts remain
  equal for this operation;
- every existing p-curve use is checked exactly once, including two seam uses;
- the measured maximum is within the original source edge tolerance;
- an expected non-zero proof cannot pass with zero checked evidence;
- no unsupported edge is repaired by fallback or partial sampling;
- a failed proof leaves the working edge unchanged and blocks meshing.

## Alternatives rejected

- `BRepLib_CheckCurveOnSurface` before establishing `SameParameter`; its
  contract is intended for edges already carrying that flag;
- `BRepLib::SameParameter`, `ShapeFix`, or unrestricted healing, because those
  routes may rewrite or synthesize representation data;
- fixed-point sampling, because it does not establish a maximum discrepancy;
- accepting tolerance alone without exact range and full-use evidence;
- normalizing a periodic range shift in this operation;
- accepting a computed or missing p-curve as stored source evidence;
- setting either flag with partial, skipped, or zero validation coverage;
- digest/order matching as correspondence evidence.

## Verification

Commit `97a2113` implements the operation and exact derivation map. The reviewed
native fixture `corrupt.edge.sameparameter_samerange_false.brep` changes one
working edge from both flags false to both true, produces a valid working
B-rep, retains an invalid immutable source B-rep, records zero tolerance,
representation, and topology-cardinality changes, and completes one-to-one
modified occurrence correspondence.

The dual-p-curve cylindrical seam witness requires and checks exactly two
p-curve uses. `corrupt.edge.range_mismatch.brep` and
`corrupt.edge.pcurve_disagreement_beyond_tolerance.brep` remain unchanged and
non-meshable. A tampered certificate with zero checked uses is non-meshable by
name. Source evaluator access refuses the unproven source edge while working
evaluator access succeeds after the certified repair.

The complete strict MSVC and MSVC `/analyze` product lanes each pass all 14
registered tests. The 77 generated STEP imports remain 59 meshable and 18
inspectable-only, with all 1,559 classified subjects accounted; none is
silently relabelled or repaired.

## Consequences

Weft now has one deliberately narrow conservative repair operation and a
non-identity correspondence lane backed by exact derivation evidence. This
does not authorize range normalization, p-curve synthesis, tolerance changes,
orientation changes, sewing, or compatibility output. M1 remains in progress
until those separately permitted operations and product STEP witnesses have
their own bounded contracts and adversarial proof.
