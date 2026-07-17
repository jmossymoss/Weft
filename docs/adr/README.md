# Architecture decision records

Create one numbered Markdown record for every decision that changes source
evidence, working-shape repair, topology ownership, exactness, fallback policy,
or distributable dependency licensing.

Required fields:

```markdown
# ADR-NNNN: Title

- Status: Proposed | Accepted | Superseded
- Date: YYYY-MM-DD
- Owners: ...

## Context
## Decision
## Invariants
## Alternatives rejected
## Verification
## Consequences
```

An ADR does not mark a milestone passed; the milestone ledger does.

Current records:

- `ADR-0015-certified-workflow-adapter-and-cli-routing.md` - lossless certified
  `PolyMesh` compatibility and the single secure `mesh` CLI route.
- `ADR-0016-secure-app-and-live-link-routing.md` - audited app import, secure
  async/export routing, recipe-conflict policy, and atomic Blender link.
- `ADR-0017-secure-cli-utility-routing.md` - certified conversion and sweep,
  source-backed inspection/extraction, and explicit cache refusal.
- `ADR-0018-source-referenced-recipe-v2.md` - immutable source references,
  strict fingerprint relocation, v1 migration, and working-to-source capture.
- `ADR-0019-certified-per-edge-interval-constraints.md` - exact recipe-v2
  edge counts reconciled through the canonical interval solver and shared
  boundary identity.
- `ADR-0020-bounded-exact-chain-sums.md` - exact independent boundary-chain
  totals with deterministic complexity and unsupported-coupling refusals.
- `ADR-0021-retire-legacy-production-generators.md` - physical removal of the
  retired geometry engines after secure routing and baseline capture.
- `ADR-0022-xde-product-structure-and-brep-occurrences.md` - exact XDE
  definitions/instances, recursive B-rep occurrences, coedge uses, and
  selective assembly-container separation without geometric fallback.
- `ADR-0023-topology-isolated-conservative-working-copy.md` - faithful
  topology isolation without p-curve materialization, exact-use rebinding, and
  independently certified identity correspondence.
- `ADR-0024-bounded-parameterization-flag-reconciliation.md` - flag-only
  SameParameter/SameRange reconciliation after complete stored-p-curve range
  and source-tolerance proof.
- `ADR-0025-snapshot-protected-native-brep-import.md` - immutable-byte native
  B-rep parsing, explicit missing-unit provenance, isolated repair derivation,
  and fail-closed secure-reader defaults.
- `ADR-0026-face-adjacency-orientation-repair.md` - two-manifold closed-shell
  orientation repair with independent polarity and validity-gated commit.
- `ADR-0027-bounded-tolerance-envelope-repair.md` - raise working edge
  tolerance only to a proven curve-on-surface maximum.
- `ADR-0028-copy-on-write-representation-rules.md` - shared geometry handle
  immutability and certified representation replacement.
- `ADR-0029-explicit-native-brep-unit-resolution.md` - caller-supplied
  millimetre scale for native B-rep without inventing or rescaling units.
- `ADR-0030-snapshot-protected-secure-iges-import.md` - immutable-byte IGES
  secure import through processing-disabled XDE transfer.
