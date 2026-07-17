# Secure-core milestone ledger

Last updated: 2026-07-17.

This file is the sole authority for secure-core implementation status. Plans,
handoffs, commit messages, and test output are evidence, not status authority.

Status vocabulary: `NOT_STARTED`, `IN_PROGRESS`, `BLOCKED`, `FAILED`, `PASSED`.

| Milestone | Status | Gate | Current evidence / next dependency |
|---|---|---|---|
| M0 - baseline and governance | IN_PROGRESS | Legacy baseline is reproducible, production routing has one owner, and governance/build routes are explicit | MSVC strict and static-analysis presets build the complete product. Strict CTest reproduces exactly the eight baseline failures; process-isolated secure tests also pass concurrently across both MSVC lanes. All reachable product mesh routes are secure-only or refuse explicitly. Dormant historical source removal and Linux execution remain open. See the M0/M7 evidence records. |
| M1 - immutable source and working B-rep | IN_PROGRESS | Every working model has a valid repair/correspondence certificate or a named refusal; identity input has an identity certificate | Processing-disabled source/working import, exact evaluators/hashes, certificates, non-rigid transform refusal, solid/face/wire/edge/vertex correspondence, the 6-success/3-refusal committed STEP lane, and all 77 generated imports pass strict/static tests. Complete assembly/instance/coedge-occurrence accounts, bounded conservative repairs, and derived B-rep refusal lanes remain open. |
| M2 - total reconnaissance | IN_PROGRESS | Every imported entity is classified/accounted exactly once; unsupported geometry is named | Source-backed exact family/wrapper/domain records account every edge/face; all 77 generated sources complete reconnaissance over 1,559 subjects (59 meshable, 18 inspectable-only). Oracle-level taxonomy matching, total occurrence hierarchy, full trim taxonomy, merged regions, and adversarial unknown-family injection remain open; see M2 and frozen-corpus evidence. |
| M3 - predicates, counts, canonical boundaries | IN_PROGRESS | Every valid shared edge has one bijective sample sequence and consistent coedge UV uses | Exact count assignment, canonical samples, bounded UV uses/lifts, exact topological endpoint normalization, azimuth registration, exact dyadic predicates, non-vacuous trim validation, and real planar face/coedge assembly pass strict/static adversarial batteries. General periodic/singular assembly, repeated wire occurrences, critical segmentation, sum constraints, full periodic closure proof, and Linux determinism remain open. |
| M4 - certified tessellation floor | IN_PROGRESS | Every valid or conservatively repairable face produces a deterministic boundary-exact intersection-free triangle mesh | Exact planar/hole CDT and the registered full-cylinder wall now compose atomically through `generateSecureMesh`. A planar box, capped cylinder, and connected through-hole body certify with no weld, complete face coverage, and real OBJ/GLB/FBX CLI exports. General curved surfaces, axial interior provenance/refinement, adaptive patch bounds, critical regions, 3D triangle intersections, app routing, Linux determinism, and corpus-wide meshing remain open. |
| M5 - non-vacuous validation | IN_PROGRESS | All required validator coverage is non-zero/complete; twisted-wall and vacuous-provenance witnesses fail before their fixes | The atomic result aggregates repair, reconnaissance, intervals, boundaries, endpoint normalization, trim, CDT/cylinder error bounds, and body certificates. Count mismatch, twisted/reflected registration, extra axial samples, chord/normal excess, wrong periodic lift, missing p-curve provenance, and vacuous face coverage refuse by name. Remaining source-incidence/Euler, general curved bounds, triangle/triangle, and deliberately vacuous provenance gates remain open. |
| M6 - planar and cylinder templates | IN_PROGRESS | Box, full/partial cylinder, mismatched-frame cylinder, and connected plane/cylinder fixtures pass end to end | Secure core orchestration, CLI export, desktop viewport, and Blender link now pass the box/full-cylinder family with deterministic certified output; the connected through-hole also passes core/CLI. Modelling still aliases the safe floor. Partial cylinders, interior axial rings, independently certified modelling polygons/quads, and mismatched-reference fixtures remain open. |
| M7 - workflow restoration | IN_PROGRESS | CLI, recipe migration, app, editing, exports, hot reload, and Blender link use `MeshingResult` with no legacy selector | `mesh`, STEP-to-mesh `convert`, secure global `sweep`, app async regeneration/export, audited hot reload, and Blender live link consume certified results with no legacy selector; `cache-check` refuses explicitly. Source-referenced recipe v2 now migrates v1, fingerprints source entities, resolves/captures through audited correspondence, persists in CLI/app, and replays safe global controls deterministically. Per-entity/manual records persist but refuse application by name. Certified editing, overlays, secure caching/proxy updates, and LOD report naming remain open. |
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
