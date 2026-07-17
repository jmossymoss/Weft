# ADR-0029: Secure IGES import

- Status: Accepted
- Date: 2026-07-17
- Owners: Weft geometry core

## Context

IGES was the last registered format without a secure import path: its
reader kept the legacy pathname-based `ReadFile`, retained no source-byte
digest, and refused `transferSecure` through the base default. Two OCCT
facts shape the contract. First, the IGES work library has no stream
parsing — the base `IFSelect_WorkLibrary::ReadStream` unconditionally
fails and IGES never overrides it — so the STEP/B-rep pattern of parsing
an in-memory snapshot stream is unavailable. Second, the IGES file parser
accepts arbitrary bytes (empty, garbage, truncated all return done) and
only materializes a translator actor for real models, so malformed input
surfaces at the transfer-roots gate, not at parse.

## Decision

- Add public `importIgesSecure`. `IgesReader::readFile` reads the path
  exactly once into a retained byte string, materializes those bytes to a
  private uniquely named scratch file, parses the scratch file, re-reads
  the scratch bytes, and accepts the parse only when they still equal the
  retained snapshot; the scratch file is always removed. Hashing and
  parsing therefore provably consumed the same bytes even though OCCT
  cannot parse IGES from a stream.
- A failed snapshot read (unopenable path, IO error, or scratch
  divergence) invalidates all retained reader state; a stale reader
  refuses `transferSecure` as `import.iges.no_source_snapshot`.
- `transferSecure` gates on transfer roots first — non-IGES bytes refuse
  as `import.iges.no_transfer_roots` — then freezes the post-transfer
  shape-processing policy to empty and verifies OCCT retained it
  (`import.processing.not_disabled` otherwise). The intrinsic
  IGES-to-BRep conversion is translator-defined and recorded as such in
  the effective configuration; the transferred B-rep is the immutable
  source evidence of that deterministic translation.
- The XDE document path is shared with STEP: `cafToImportedModel` supplies
  the same topology-isolated conservative derivation, bounded repairs,
  correspondence, certificates, and diagnostics, and the XDE length unit
  (millimetres after translator conversion) lands in the metadata as
  source-declared evidence.
- IGES keeps the refusing `resolveNativeLengthUnit` default: it declares
  its own units, so the ADR-0028 caller-side contract stays closed to it.
- Conservative IGES import does not sew: a mode-0 IGES face soup imports
  as independent faces with per-face edges, faithful to the source.

## Invariants

- replacing or deleting the original path after `readFile` cannot change
  the parsed source or its recorded digest;
- a scratch file that diverges from the retained snapshot fails the read
  instead of silently accepting foreign bytes;
- stale reader state is never transferable after any failed read;
- all existing repair, correspondence, and certificate gates apply
  unchanged through the shared XDE path;
- malformed and missing IGES inputs fail with stable named codes;
- no sewing, healing, or shape processing runs beyond the recorded
  intrinsic translation.

## Alternatives rejected

- parsing via `ReadStream` (the IGES work library cannot; it fails for
  every input);
- hashing the file and then re-reading the original path with `ReadFile`
  (the ADR-0025 replacement race);
- treating the lenient parse as validation (everything "parses"; the
  roots gate is the real acceptance point);
- sewing the translated face soup into shells during conservative import
  (provable one-to-one sewing is its own future increment);
- exposing the caller-side unit resolution to IGES.

## Verification

Committed reviewed witnesses `tests/fixtures/secure_box.igs` and
`secure_cylinder.igs` (authored with a plain `IGESControl_Writer`, mode 0,
millimetres): the box imports identity, valid, and meshable as six
independent faces with twenty-four edges and a declared 1.0 mm unit; the
path-replacement witness retains the box after the file is overwritten
with the cylinder while a fresh import sees the cylinder; compatibility
stays audited and non-meshable; garbage bytes refuse as
`import.iges.no_transfer_roots`; a missing file refuses as
`import.iges.read_failed`; a reader whose second read fails refuses as
`import.iges.no_source_snapshot`; and the base reader default still
refuses formats without an explicit secure contract.

## Consequences

Every registered import format now has an immutable-source secure path or
an explicit named refusal. IGES models enter the same certificate chain as
STEP and native B-rep; sewing their face soups into shells remains the
provable-sewing increment. The product IGES writer showed environment
instability during witness authoring on the Linux dev vehicle and the
witnesses were therefore authored with the plain OCCT writer; the writer
itself is unchanged.
