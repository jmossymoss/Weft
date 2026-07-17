# M1 native unit resolution evidence - 2026-07-17

## Proven increment

ADR-0028 closes the explicit external physical-unit resolution contract
named open by ADR-0025. `importBRepSecure` now accepts an optional
`NativeUnitResolution` (millimetres per model unit plus a non-empty
resolving authority). A resolved import records the scale in the source
metadata, both source and working models, and the retained effective
configuration, and downgrades the missing-unit warning to an
`import.brep.length_unit_resolved` info diagnostic. An unresolved import is
byte-for-byte unchanged behaviour.

The resolution is metadata evidence only: the reviewed box baseline
produces identical source and working shape digests with and without a
resolution, and its identity certificate is preserved. Invalid resolutions
(zero, negative, NaN, infinite, or authority-less) refuse as
`import.brep.unit_resolution_invalid` before any bytes are read, and the
STEP reader refuses any caller resolution as
`import.secure.unit_resolution_unsupported`, so source-declared unit
evidence can never be overridden.

## Verification runs

Linux container (dev vehicle only), OCCT 8.0.0 from source,
`WEFT_WARNINGS_AS_ERRORS=OFF`: all 14 registered tests passed after the
increment, including the new native-unit battery in the secure-core test.
The Windows strict MSVC and MSVC `/analyze` product lanes were NOT run in
this session and remain the active development gate; this evidence is
partial until they rerun clean.

## Status boundary

M1 remains `IN_PROGRESS`. Secure IGES import, bounded tolerance
reconciliation, provable sewing, copy-on-write representation rules,
product STEP repair witnesses, the reversed-solid/multi-shell/shared-shell
orientation scopes, and complete compatibility correspondence remain open.
