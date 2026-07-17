# ADR-0006: Refuse unaccounted non-rigid STEP transforms

- Status: Accepted
- Date: 2026-07-17
- Owners: Weft geometry core

## Context

OCCT XDE locations can expose proper rigid placements while a STEP source also
declares affine mapped-item operators. If scale or reflection disappears during
processing-disabled transfer and there is no proof that it was baked into exact
geometry, accepting the resulting B-rep silently changes the represented model.

The frozen donor corpus contains reviewed scale-2 and mirrored mapped-item
witnesses specifically for this loss route.

## Decision

Before XDE transfer, the secure STEP reader scans every source
`cartesian_transformation_operator_3d`. It derives the declared basis using the
ISO 10303 base-axis construction, preserving the declared handedness, and
checks the local origin, scale, and linear determinant.

Non-finite, degenerate, singular, or non-positive-scale operators fail as
`import.transform.singular_placement`. Any finite operator that is not a proper
rigid transform fails as
`import.transform.unresolved_representation_loss`. The refusal reports the
operator count plus first determinant and scale. No working B-rep is produced.

This is deliberately conservative. A future compatibility lane may accept the
source only after a typed transform account proves either retained affine
placement or byte/geometry-backed baking and validates composition and parity.

## Invariants

- scale and reflection cannot disappear behind an identity XDE location;
- proper rigid operators remain importable;
- source-transform inspection happens before authoritative transfer;
- invalid/non-rigid operators fail with stable distinct codes;
- neither repair profile bypasses unresolved representation loss.

## Alternatives rejected

- trusting the post-transfer shape without inspecting source operators;
- converting a reflection into a rotation;
- assuming scale was baked because the resulting topology is non-empty;
- allowing compatibility repair to heal an assembly-accounting defect;
- matching known fixture hashes instead of auditing source entities.

## Verification

The committed STEP corpus test imports the six reviewed success sources twice,
refuses the syntax witness twice with `import.step.transfer_failed`, and refuses
the mirrored and scaled mapped-item witnesses twice with
`import.transform.unresolved_representation_loss`. The result passes strict and
static-analysis lanes.

## Consequences

Recipe migration and export cannot target a silently flattened affine instance.
Complete assembly/instance occurrence records and typed retained/baked transform
accounts are still required before M1 can pass.
