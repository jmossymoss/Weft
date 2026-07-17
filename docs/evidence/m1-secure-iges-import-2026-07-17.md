# M1 secure IGES import evidence - 2026-07-17

## Proven increment

ADR-0029 gives IGES the immutable-source secure import contract, closing
the last registered format without one. Two OCCT constraints were
established by direct source and runtime reconnaissance and shaped the
design:

- the IGES work library cannot parse a stream — the base
  `IFSelect_WorkLibrary::ReadStream` unconditionally fails, and only STEP
  overrides it — so the retained byte snapshot is materialized to a
  private scratch file, parsed, and re-verified byte-for-byte afterwards;
- the IGES file parser accepts arbitrary bytes (empty, garbage, and
  truncated inputs all return done), so acceptance is gated on transfer
  roots and malformed content refuses as
  `import.iges.no_transfer_roots` rather than pretending parse-level
  validation exists.

The transfer shares the STEP XDE path end to end (`cafToImportedModel`):
same topology-isolated conservative derivation, bounded repairs,
orientation proofs, correspondence, certificates, and diagnostics. The
declared unit arrives as source evidence (1.0 mm after translator
conversion). Conservative import does not sew: a mode-0 face soup stays
independent faces, faithful to the source representation.

## Witnesses

Committed reviewed witnesses `tests/fixtures/secure_box.igs` and
`tests/fixtures/secure_cylinder.igs` prove:

- the box imports identity, valid, meshable: six independent faces,
  twenty-four per-face edges, equal source/working digests, complete
  correspondence, declared 1.0 mm unit, and the snapshot configuration
  entry;
- path replacement after `readFile` cannot change the parsed source: the
  retained reader still reports the box while a fresh import of the
  overwritten path sees the cylinder with a different byte digest;
- compatibility imports stay audited and non-meshable;
- garbage bytes refuse as `import.iges.no_transfer_roots`; a missing path
  refuses as `import.iges.read_failed`; a reader whose second read fails
  refuses as `import.iges.no_source_snapshot`; the base reader default
  still refuses as `import.secure.reader_unsupported` (witnessed by a
  minimal reader with no secure override, since every registered format
  now has one).

The cylinder witness imports with an invalid source (a defect of the
translated representation itself) and therefore stays inspectable-only —
recorded here as honest fail-closed behaviour, not a repair claim.

## Environment note

While authoring witnesses, the product IGES writer
(`core/src/io/occ_writers.cpp`) crashed non-deterministically inside
`IGESControl_Writer::AddShape` on the Linux/OCCT 8.0 dev vehicle; a plain
`IGESControl_Writer` in an isolated process was stable and authored the
committed witnesses. The writer is unchanged by this increment; its
stability should be confirmed on the Windows gate.

## Verification runs

Linux container (dev vehicle only), OCCT 8.0.0 from source,
`WEFT_WARNINGS_AS_ERRORS=OFF`: all 14 registered tests passed, including
the new secure-IGES battery. The Windows strict MSVC and MSVC `/analyze`
product lanes were NOT run in this session and remain the active
development gate; this evidence is partial until they rerun clean.

## Status boundary

M1 remains `IN_PROGRESS`. Provable one-to-one sewing (which would give
IGES face soups closed shells), bounded tolerance reconciliation,
copy-on-write representation rules, product STEP repair witnesses, the
reversed-solid/multi-shell/shared-shell orientation scopes, and complete
compatibility correspondence remain open.
