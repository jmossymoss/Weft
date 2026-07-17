# ADR-0028: Explicit caller-side native unit resolution

- Status: Accepted
- Date: 2026-07-17
- Owners: Weft geometry core

## Context

ADR-0025 made the missing physical unit of native ASCII B-rep explicit:
`SourceMetadata::lengthUnitMm` stays absent and the import emits
`import.brep.length_unit_unspecified`, because inferring a unit from the
bytes would be provenance loss. That left native imports blocked from
workflow and export routing, which scale by `Model::lengthUnitMm` — the
consequence section named an explicit external physical-unit resolution
contract as the missing piece.

## Decision

- Add `NativeUnitResolution { millimetresPerModelUnit, authority }` and an
  optional resolution parameter on `importBRepSecure`. The resolution is
  caller-side evidence: the caller states the scale and names the resolving
  authority.
- A resolution must be finite, positive, and carry a non-empty authority;
  anything else refuses as `import.brep.unit_resolution_invalid` before any
  bytes are read.
- A resolved import records the scale in `SourceMetadata::lengthUnitMm`,
  sets both source and working `Model::lengthUnitMm`, appends a
  `length unit: caller-resolved ... (authority: ...)` entry to the
  effective configuration, and replaces the missing-unit warning with an
  `import.brep.length_unit_resolved` info diagnostic. Coordinates and the
  exact representation digests are untouched — the resolution is metadata
  evidence only.
- An unresolved import behaves exactly as before: absent unit, explicit
  warning.
- The base `io::Reader::resolveNativeLengthUnit` refuses as
  `import.secure.unit_resolution_unsupported`. Only unit-less native
  formats may override it, so a caller can never override the
  source-declared units of STEP (or any future format that carries its
  own).

## Invariants

- no unit is ever inferred from native bytes;
- a resolved unit always names its authority in the retained configuration;
- resolution changes no geometry byte: source and working shape digests are
  identical with and without a resolution, and identity certificates are
  preserved;
- source-declared unit evidence is not overridable through this contract;
- invalid resolutions fail closed by a stable named code.

## Alternatives rejected

- defaulting native imports to millimetres (silent unit invention);
- inferring units from bounding-box heuristics;
- a bare `double` parameter without a recorded authority (evidence without
  provenance);
- allowing the STEP reader to accept a caller override for reconciliation
  (caller convenience cannot outrank source evidence);
- storing the resolution only in `Model` without the metadata record (the
  certificate chain would lose why the scale exists).

## Verification

The reviewed box baseline imports unresolved with the absent unit and the
explicit warning, and resolved (25.4 mm per model unit, named authority)
with the unit present in metadata and both models, the info diagnostic, the
configuration entry carrying scale and authority, a preserved identity
certificate, and byte-identical source/working digests against the
unresolved import. Zero, negative, NaN, infinite, and authority-less
resolutions refuse as `import.brep.unit_resolution_invalid`; the STEP
reader refuses any resolution as
`import.secure.unit_resolution_unsupported`.

## Consequences

Native B-rep imports can now carry real physical units into the certified
workflow and exporters without inventing evidence, closing the unit-contract
item from ADR-0025. Secure IGES import and the remaining M1 repair families
are unchanged and still open.
