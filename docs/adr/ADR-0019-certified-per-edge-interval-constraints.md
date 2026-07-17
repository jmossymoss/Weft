# ADR-0019: Certified per-edge interval constraints

- Status: Accepted
- Date: 2026-07-17
- Owners: Weft geometry core, CLI, and desktop app

## Context

Recipe v2 can resolve an immutable source-edge reference through the audited
source-to-working correspondence map, but ADR-0018 initially stopped before
applying the resolved count. Treating that count as a face-mesher hint would
reintroduce face-local resampling, phase disagreement, and weld dependence.
Silently increasing a requested count to the certified error floor would also
make the persisted recipe differ from the generated topology.

## Decision

- A resolved recipe-v2 edge count becomes an exact constraint on the working
  edge's canonical boundary interval variable.
- Exact constraints participate in the same equality classes as error-derived
  minimums and parity requirements. A cylinder-rim constraint therefore
  propagates to the registered opposite rim before either face is built.
- Incompatible exact counts, counts below a certified minimum, parity
  conflicts, missing working edges, and non-positive counts refuse by stable
  code. No constraint is rounded, raised, discarded, or applied locally.
- Canonical boundaries remain immutable after count solving. Every owning
  coedge consumes the same sample IDs and assembled vertex indices.
- The certified generation report exposes every solved boundary interval count
  to workflow adapters. The app uses this evidence for its edge controls rather
  than an independent display default.
- The CLI, recipe reload path, app regeneration, and hot reload all capture or
  resolve the edge setting through recipe-v2 source references. Legacy compiler
  edge constraints remain blocked.
- A cylinder axial-edge count greater than one remains unsupported until axial
  interior samples carry source-surface provenance; it refuses by the existing
  named template gate.

This record supersedes only ADR-0018's rollout statement that all per-edge
settings are inapplicable. Its reference, migration, and conflict contracts
remain in force.

## Invariants

- one working edge has one solved interval count and one canonical sample
  sequence;
- adjacent faces share sample identities and indices without welding;
- a persisted exact count is either satisfied exactly or generation refuses;
- error, parity, and topology requirements cannot be weakened by a user pin;
- source references are resolved before any working-edge constraint is built;
- validation coverage for a requested constraint is non-zero and complete.

## Alternatives rejected

- passing a count separately to every owning face mesher;
- allowing the adaptive floor to override an exact recipe value silently;
- applying a pin after canonical boundary construction;
- treating equal vertex coordinates as shared boundary identity;
- accepting transient working-edge ordinals directly from recipe v2;
- enabling legacy compiler edge constraints as an alias.

## Verification

The interval solver battery proves fixed-count equality propagation and named
conflict, minimum, and parity refusals. Secure meshing tests prove a four-
interval box edge has five canonical sample identities shared by its owning
faces, a 64-interval cylinder rim propagates to both registered rims, repeated
topology fingerprints are identical, invalid IDs and zero counts refuse, and
an axial-edge request requiring interior provenance refuses by name.

Recipe tests prove an edge-only v1 record migrates, resolves, and passes the
application gate. Strict and MSVC static-analysis complete-product builds pass;
all 14 secure/corpus tests pass in both lanes. Direct CLI generation and v2
reload produce identical OBJ and validation-report SHA-256 digests. The app
screenshot harness loads the v2 sidecar, reports the solved 11-vertex/18-
triangle result, and exposes the certified feature-edge control without
reenabling face/model editing.

## Consequences

Per-edge density is the first certified entity-level recipe operation restored
end to end. Exact requests can legitimately refuse where a template lacks the
required source-proven interior construction. Per-face settings, manual
surface edits, and legacy compiler edge constraints remain blocked.
