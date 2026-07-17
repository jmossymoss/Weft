# Secure-core milestone ledger

Last updated: 2026-07-17.

This file is the sole authority for secure-core implementation status. Plans,
handoffs, commit messages, and test output are evidence, not status authority.

Status vocabulary: `NOT_STARTED`, `IN_PROGRESS`, `BLOCKED`, `FAILED`, `PASSED`.

| Milestone | Status | Gate | Current evidence / next dependency |
|---|---|---|---|
| M0 - baseline and governance | IN_PROGRESS | Legacy baseline is reproducible, production routing has one owner, and governance/build routes are explicit | MSVC strict and static-analysis presets build the complete product. Strict CTest reproduces exactly the eight baseline failures; process-isolated secure tests also pass concurrently across both MSVC lanes. See the M0 evidence records. Linux preset execution and removal of legacy production routing remain open. |
| M1 - immutable source and working B-rep | IN_PROGRESS | Every working model has a valid repair/correspondence certificate or a named refusal; identity input has an identity certificate | Processing-disabled source/working import, exact evaluators/hashes, certificates, non-rigid transform refusal, the 6-success/3-refusal committed STEP lane, and all 77 generated imports pass strict/static tests; see M1 and frozen-corpus evidence. Complete assembly/instance accounts, bounded conservative repairs, and derived B-rep refusal lanes remain open. |
| M2 - total reconnaissance | IN_PROGRESS | Every imported entity is classified/accounted exactly once; unsupported geometry is named | Source-backed exact family/wrapper/domain records account every edge/face; all 77 generated sources complete reconnaissance over 1,559 subjects (59 meshable, 18 inspectable-only). Oracle-level taxonomy matching, total occurrence hierarchy, full trim taxonomy, merged regions, and adversarial unknown-family injection remain open; see M2 and frozen-corpus evidence. |
| M3 - predicates, counts, canonical boundaries | IN_PROGRESS | Every valid shared edge has one bijective sample sequence and consistent coedge UV uses | Exact count assignment, canonical samples, bounded UV uses/lifts, azimuth registration, exact dyadic predicates, non-vacuous trim validation, and real planar face/coedge assembly pass strict/static adversarial batteries. General periodic/singular assembly, repeated wire occurrences, critical segmentation, sum constraints, full periodic closure proof, and Linux determinism remain open. |
| M4 - certified tessellation floor | IN_PROGRESS | Every valid or conservatively repairable face produces a deterministic boundary-exact intersection-free triangle mesh | The exact-predicate single-outer-loop reference CDT now consumes actual box and cylinder-cap B-rep assemblies, preserving constraints/provenance and proving cardinality, orientation, incidence, all edge relations, and local Delaunay legality. A real perforated face reaches the named hole-support refusal. Hole recovery/cut graphs, surface lifting/error bounds, critical regions, body assembly, triangle/triangle checks, production routing, Linux determinism, and corpus completion remain open; see the M3/M4 evidence records. |
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
