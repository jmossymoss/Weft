# ADR-0029: Explicit native B-rep unit resolution

- Status: Accepted
- Date: 2026-07-17
- Owners: Weft geometry core

## Context

Native OCCT ASCII B-rep carries no declared physical length unit. ADR-0025
correctly leaves `SourceMetadata::lengthUnitMm` absent and emits
`import.brep.length_unit_unspecified`. Export and workflow routing still need
an explicit millimetre scale without inventing a unit from the file bytes or
rescaling coordinates.

## Decision

- Extend `importBRepSecure` with an optional caller-supplied `lengthUnitMm`.
- Absent the argument, behaviour is unchanged: unit metadata stays empty and
  `import.brep.length_unit_unspecified` remains.
- A supplied value must be finite and positive. It is recorded on source
  metadata and both source/working model snapshots, marked
  `lengthUnitExplicitlyResolved`, and diagnosed as
  `import.brep.length_unit_resolved`.
- Coordinates are never scaled. The value is provenance and export scale only.
- Invalid values refuse as `import.brep.length_unit_invalid`. A conflict with a
  format-declared unit refuses as `import.brep.length_unit_conflict`.

## Invariants

- native files never invent a declared unit;
- caller resolution is visible in metadata and diagnostics;
- resolved units do not alter topology, digests, or geometry handles;
- STEP declared units remain format-declared, not caller-resolved.

## Alternatives rejected

- defaulting native imports to `1.0` millimetres silently;
- rescaling coordinates to millimetres during import;
- requiring a unit before any native secure import;
- treating model-unit coordinates as metres or inches by heuristic.

## Verification

The reviewed box native B-rep imports with unit `1.0` as identity/meshable,
records explicit resolution, and rejects non-positive scales. See
`docs/evidence/m1-explicit-native-brep-unit-resolution-2026-07-17.md`.

## Consequences

Workflow/export can consume an explicit native scale without weakening
immutable-source provenance. M1 native-unit resolution is closed; sewing,
product STEP repair witnesses, secure IGES, and multi-body orientation remain
open.
