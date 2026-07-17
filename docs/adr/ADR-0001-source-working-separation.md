# ADR-0001: Separate immutable source and working B-reps

- Status: Accepted
- Date: 2026-07-17
- Owners: Weft geometry core

## Context

The historical importer ran translator processing and a broad heal pipeline
before retaining the only shape consumed by meshers. That made representation
loss, tolerance changes, and topology changes difficult to distinguish from
source evidence.

## Decision

STEP/XDE transfer must verify an empty shape-processing policy on both the XDE
reader and its base STEP reader. That result becomes `SourceBRep`. A separately
identified `WorkingBRep` is derived afterward. Conservative import begins as an
identity derivation; compatibility import may invoke the historical heal
pipeline only as an explicit repair operation on a geometry-deep copy. Copy and
repair histories are composed back to source identities.

Both shapes receive exact native-B-rep SHA-256 fingerprints. Face/edge
correspondence, topology-cardinality changes, tolerance changes, stored p-curve
use changes, and non-vacuous audit coverage are included in the repair
certificate.

## Invariants

- meshers never consume `SourceBRep`;
- no source p-curve is claimed unless OCCT reports it as stored;
- missing or ambiguous correspondence prevents an exportable working claim;
- a compatibility operation can never rewrite source evidence;
- exact evaluation fails by code when its parameter relationship is unproven.

## Alternatives rejected

- keeping one healed shape and logging the requested repair flags;
- treating OCCT-computed planar p-curves as source representations;
- using `D:/weftocct` as a runtime library.

## Verification

`weft_secure_core_tests` proves conservative identity, exact evaluation,
non-vacuous audit coverage, and explicit compatibility operation recording
under both strict MSVC and MSVC static-analysis presets.

## Consequences

Legacy `loadStep` remains temporarily available to existing workflows while
the certified meshing floor is built, so M0 and M1 remain in progress. Inputs
with missing stored p-curves may become visible refusals until an authorized
working-representation policy is proven.
