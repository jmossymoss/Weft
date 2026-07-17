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
