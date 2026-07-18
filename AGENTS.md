# Weft agent directive

Before changing anything:

1. Work from the repository root and inspect `git status`. Preserve all
   existing uncommitted and user-owned changes.
2. Read `docs/governance/milestones.md`; it is the sole status authority.
3. Read only the authority/session sections of
   `docs/governance/agent-execution-playbook.md`.
4. Search the playbook for the first `Status: OPEN`, then read that work
   package and its named prerequisites only.
5. Read `docs/governance/blocked-routes.md`.
6. Inspect only the package-specific implementation, tests, ADRs, and evidence.

Before implementation, report:

- selected package and goal;
- prerequisites and whether they are complete;
- relevant working-tree state;
- exact first action;
- existing changes that must not be overwritten.

Execute one work package completely. Follow its tests, evidence requirements,
and binary exit gate. Update `milestones.md` only when the full milestone gate
is proven.

Do not:

- create additional plans, handoff files, investigation diaries, or summaries;
- treat chat, commits, plans, or test output as status authority;
- stage attachments, build output, generated STEP files, dumps, or unrelated
  user changes;
- add OCCT triangle-soup fallback, authoritative welding, missing faces, silent
  healing, or fail-open validation.

When switching agents, use the `RESUME` block defined in the playbook. Chat is
transport only; verify every handoff against the repository and evidence.
