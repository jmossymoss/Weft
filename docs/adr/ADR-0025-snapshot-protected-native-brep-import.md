# ADR-0025: Snapshot-protect native B-rep secure import

- Status: Accepted
- Date: 2026-07-17
- Owners: Weft geometry core

## Context

The frozen repair catalogue contains reviewed native OCCT ASCII B-reps whose
representation defects cannot always survive STEP translation. Tests could
previously exercise them only by calling internal derivation functions. The
existing `BrepReader` reopened a pathname through `BRepTools::Read`, retained
no source-byte digest, and inherited a default `Reader::transferSecure` that
placed the same materialized model and TShapes in both source and working
slots. That default violated the immutable-source and topology-isolation
contracts.

A pathname is not an immutable source. Hashing it and reopening it for parsing
would allow a concurrent replacement to make the digest describe different
bytes from the parsed shape. Native B-rep also carries no declared physical
length unit, so silently describing its model-unit scale as source-declared
millimetres would be provenance loss.

## Decision

- Add public `importBRepSecure` for native OCCT ASCII B-rep evidence.
- `BrepReader::readFile` reads the file once into an immutable byte string and
  parses the shape from a stream over those exact bytes. It never reopens the
  path for parsing or hashing.
- A failed or throwing parse nulls the retained shape before returning or
  propagating, preventing transfer of stale or partial reader state.
- Source metadata records the snapshot byte length, SHA-256, source filename,
  importer version, and effective configuration.
- `SourceMetadata::lengthUnitMm` is typed and optional. STEP records its
  declared XDE scale; native ASCII B-rep leaves it absent and emits
  `import.brep.length_unit_unspecified`. Coordinates remain unchanged in
  native model units; no physical unit is inferred.
- Callers may supply an explicit positive millimetre scale to
  `importBRepSecure`. That value is provenance/export metadata only
  (`lengthUnitExplicitlyResolved`, `import.brep.length_unit_resolved`); see
  ADR-0029. Coordinates are still never rescaled.
- Conservative native import uses the same topology-isolated derivation and
  bounded repairs as STEP. Compatibility native import uses the same
  geometry-deep historical repair derivation and remains non-meshable while
  its correspondence proof is incomplete.
- The compatibility copy/history implementation is one shared secure-core
  stage, rather than duplicated STEP- and B-rep-specific code.
- The base `Reader::transferSecure` refuses with
  `import.secure.reader_unsupported`. Only a format reader with an explicit
  immutable-source contract may override it.

## Invariants

- source-byte hashing and native parsing consume the same retained bytes;
- replacing or deleting the original path after `readFile` cannot change the
  parsed source or its metadata;
- conservative source and working topologies never share TShapes;
- all existing repair and correspondence certificate gates apply unchanged;
- a failed parse cannot leave a transferable shape in the reader;
- absent native physical units are visible and never relabelled as declared;
- unsupported readers cannot manufacture a source/working identity pair;
- malformed and missing native inputs fail with a stable named code.

## Alternatives rejected

- hashing the file and then calling filename-based `BRepTools::Read`;
- retaining the legacy default `model, model` secure-reader alias;
- copying the reader model only after a legacy healing transfer;
- treating `lengthUnitMm = 1.0` as a unit declaration from the B-rep file;
- keeping native repair fixtures behind test-only internal derivation seams;
- allowing a malformed parse to retain whatever partial shape OCCT produced;
- silently applying compatibility healing when a reader has no secure
  override.

## Verification

Commit `43e62ed` implements the public import, reader snapshot, shared
compatibility derivation, typed unit provenance, and fail-closed base reader.
Commit `94d2cf3` adds the explicit valid-read, failed-read, refused-transfer
state-reuse witness.

The reviewed
`corrupt.edge.sameparameter_samerange_false.brep` artifact is imported through
the public API. Its exact byte SHA-256
`c400ae0bb2102165a07673bb0df8ba28a17e998c84daf1ade7b4bc95a963ba1e`
is retained, its three-face source remains immutable, and its certified
working copy becomes valid and meshable through the bounded flag repair.

The path-replacement witness reads that artifact, overwrites the path with the
reviewed box B-rep, and then transfers the original reader. The retained reader
still reports the original SHA-256 and three faces; a fresh public import sees
the box SHA-256 and six faces. Missing and malformed B-reps fail as
`import.brep.read_failed`. The default IGES reader refuses secure transfer as
`import.secure.reader_unsupported`. Native compatibility output is distinct
from the source and remains non-meshable by certificate.

The complete strict MSVC and MSVC `/analyze` product lanes each compile the
core, CLI, desktop app, and all tests, then pass all 14 registered tests.

## Consequences

Reviewed native repair evidence now traverses a production core API without
bypassing immutable import. STEP and native B-rep share repair ownership, and
unimplemented readers fail closed. Explicit caller unit resolution is defined
in ADR-0029. M1 remains in progress: IGES still lacks a secure import
override, and remaining bounded repair families such as sewing are unchanged.
