# ADR-0021: Retire legacy production generators

- Status: Accepted
- Date: 2026-07-17
- Owners: Weft geometry core

## Context

The certified secure pipeline already owned every reachable product route, but
the historical compiler, face strategies, post-hoc weld/fallback machinery,
and their monolithic regression test remained in the repository. Keeping an
unlinked second geometry engine made ownership ambiguous, invited accidental
reconnection, and required warnings/static-analysis work on code that could no
longer produce an authoritative result. The frozen baseline and Git history
already preserve the pre-rewrite evidence.

## Decision

- Delete the retired generator/compiler implementations, internal strategy
  headers, and the frozen `test_pipeline.cpp` target.
- Delete the disabled CLI benchmark bodies. The public `sweep` command remains
  a secure deterministic density harness; `cache-check` continues to refuse by
  stable code until certified dependency caching exists.
- Keep only the settings, recipe-migration, report, `PolyMesh` adapter, debug-
  trace, and naming types still consumed by the product migration. They do not
  provide or select a legacy geometry implementation.
- Preserve old output and failure evidence in `docs/evidence` and preserve the
  implementation itself in Git history. No runtime selector may restore it.

The removal is recorded by commit `9ec3f3f`.

## Invariants

- every reachable STEP-to-mesh product route calls `generateSecureMesh` or
  refuses with a stable secure-workflow code;
- no retired generator translation unit appears in any generated production or
  test project;
- no authoritative result may depend on welding, face-local resampling,
  partial-shell output, or OCCT triangle-soup substitution;
- migration-only legacy fields cannot change geometry ownership;
- reintroducing a second generator requires a new ADR and complete secure-core
  proof rather than resurrecting historical source.

## Alternatives rejected

- retaining the old implementation behind a build option;
- compiling it only for the frozen regression test;
- keeping disabled CLI bodies as executable documentation;
- repairing analyzer/compiler findings in an engine that product routing can
  never call;
- deleting baseline evidence together with the implementation.

## Verification

A new Windows build directory configured from the strict MSVC preset, compiled
the complete CLI/app/test product after deletion, and passed all 14 registered
tests. The MSVC static-analysis lane also built and passed 14/14 tests. Generated
Visual Studio projects contain zero retired generator source names. The clean
CLI produced a complete certified box mesh/report, repeated density sweeps with
identical topology fingerprints, and returned the named secure cache refusal.

## Consequences

Production geometry now has one implementation owner and new compiler/analyzer
failures cannot originate in dormant mesh strategies. Historical pipeline tests
are no longer executable gates; their numeric/visual evidence and failure list
remain frozen. Migration-only types still carrying legacy names are an M7
cleanup concern, not an alternate geometry route.
