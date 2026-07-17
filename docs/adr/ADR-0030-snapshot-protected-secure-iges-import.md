# ADR-0030: Snapshot-protect secure IGES import

- Status: Accepted
- Date: 2026-07-17
- Owners: Weft geometry core

## Context

IGES remained on the legacy `ReadFile(path)` + heal path while STEP and native
B-rep already bound provenance to immutable bytes and derived topology-isolated
working copies. The default `Reader::transferSecure` refused IGES as
`import.secure.reader_unsupported`, blocking M1 format parity.

## Decision

- `IgesReader::readFile` loads the caller path once into retained bytes, then
  parses through a private temp filled from those bytes. The caller path is
  never reopened.
- `transferSecure` disables IGES shape processing, transfers into XDE, and
  reuses `cafToImportedModel` so conservative/compatibility repair matches STEP.
- Public `importIgesSecure` wraps the reader with named
  `import.iges.read_failed` / `import.iges.transfer_failed` refusals.
- Empty-reader secure transfer refuses as `import.iges.no_source_shape`.

## Invariants

- source digest and parse consume the same retained bytes;
- replacing the caller path after `readFile` cannot change the transferred
  source;
- processing-disabled policy is retained or transfer refuses;
- conservative source/working topologies remain TShape-isolated.

## Alternatives rejected

- reopening the caller path after hashing;
- leaving IGES on the heal-only legacy transfer for secure workflows;
- inventing IGES units beyond whatever XDE reports.

## Verification

A generated millimetre B-rep-mode box IGES imports as identity/meshable through
`importIgesSecure`. Path replacement after `readFile` retains the original
digest while a fresh import sees the replacement. See
`docs/evidence/m1-secure-iges-import-2026-07-17.md`.

## Consequences

M1 secure IGES import is closed for the processing-disabled identity path.
Provable sewing, product STEP repair witnesses, and complete compatibility
correspondence remain open.
