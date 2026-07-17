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
