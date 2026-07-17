# Secure-core milestone ledger

Last updated: 2026-07-17.

This file is the sole authority for secure-core implementation status. Plans,
handoffs, commit messages, and test output are evidence, not status authority.

Status vocabulary: `NOT_STARTED`, `IN_PROGRESS`, `BLOCKED`, `FAILED`, `PASSED`.

| Milestone | Status | Gate | Current evidence / next dependency |
|---|---|---|---|
| M0 - baseline and governance | IN_PROGRESS | Legacy baseline is reproducible, production routing has one owner, and governance/build routes are explicit | MSVC strict and static-analysis presets build the complete product. Strict CTest reproduces exactly the eight baseline failures and passes `secure_core`; see `docs/evidence/m0-strict-build-2026-07-17.md`. Linux preset execution and removal of legacy production routing remain open. |
| M1 - immutable source and working B-rep | IN_PROGRESS | Every working model has a valid repair/correspondence certificate or a named refusal; identity input has an identity certificate | Processing-disabled STEP/XDE import, source/working snapshots, exact evaluators, exact-shape fingerprints, repair deltas, and an identity fixture are implemented; see `docs/evidence/m1-secure-import-2026-07-17.md`. Total occurrence hierarchy, bounded conservative repairs, and refusal corpus remain open. |
| M2 - total reconnaissance | IN_PROGRESS | Every imported entity is classified/accounted exactly once; unsupported geometry is named | A source-backed face/edge classifier now accounts each working subject once, preserves exact family/wrapper/domain evidence, names unknowns, and proves planar projection eligibility versus curved p-curve requirements on box/cylinder fixtures. Total occurrence hierarchy, full trim taxonomy, merged logical regions, and adversarial unknown fixtures remain open; see `docs/evidence/m2-reconnaissance-2026-07-17.md`. |
| M3 - predicates, counts, canonical boundaries | NOT_STARTED | Every valid shared edge has one bijective sample sequence and consistent coedge UV uses | Requires M2 graph. |
| M4 - certified tessellation floor | NOT_STARTED | Every valid or conservatively repairable face produces a deterministic boundary-exact intersection-free triangle mesh | Requires M3 boundary contract and CDT backend. |
| M5 - non-vacuous validation | NOT_STARTED | All required validator coverage is non-zero/complete; twisted-wall and vacuous-provenance witnesses fail before their fixes | Developed alongside M3-M4; accepted only after M4. |
| M6 - planar and cylinder templates | NOT_STARTED | Box, full/partial cylinder, mismatched-frame cylinder, and connected plane/cylinder fixtures pass end to end | Requires M4-M5. |
| M7 - workflow restoration | NOT_STARTED | CLI, recipe migration, app, editing, exports, hot reload, and Blender link use `MeshingResult` with no legacy selector | Requires M6 first usable mesh. |
| M8 - topology expansion | NOT_STARTED | Each automatic family has isolated, connected, adversarial, and density-sweep proof | Requires M7 workflow. |
| M9 - distribution | NOT_STARTED | No unresolved prototype-only dependency; clean build/corpus/package gates pass | Requires M8 selected release scope. |

## Baseline exceptions

The eight failures recorded at the M0 baseline are not accepted product
behaviour. They are frozen pre-rewrite evidence so the new core cannot hide or
relabel them:

- five MP9 routing/count assertions;
- flaregun watertightness and folded-polygon assertions;
- foam watertightness.

Any new failure is a regression. Closing an existing failure requires numeric
and visual evidence.
