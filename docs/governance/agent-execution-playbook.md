# Secure-core agent execution playbook

This is the execution checklist for the secure-core rewrite. It exists so a
work package can move between LLM products without relying on chat history.

## Authority and scope

Read these in order:

1. `docs/governance/milestones.md` — sole implementation-status authority.
2. This file — work-package order and completion protocol.
3. `docs/governance/blocked-routes.md` — routes that may not be reopened.
4. `docs/SECURE_CORE_REWRITE_PLAN.md` — architecture and global gates.
5. Relevant ADRs and evidence — decisions and prior proof.
6. `docs/MESHING_RESEARCH_PLAN.md` — sources for later M8 work, not status.

The product route remains certified-only. OCCT triangle soup, welding, missing
faces, silent healing, and unproven preview meshes are not export authority.
The currently certified automatic families are planes (including holes) and
full periodic cylinders.

## Token and documentation discipline

- Work on one package at a time. Read only its primary files, referenced ADRs,
  relevant tests, and directly related evidence.
- Do not load this whole file for routine work. Read the authority/session
  sections, search for the first `Status: OPEN`, then read that package and its
  named prerequisites only.
- Do not create planning, handoff, diary, investigation, or summary documents.
- Create an ADR only for a durable architectural decision covered by
  `docs/adr/README.md`.
- Create one evidence file only when a package has a completed or materially
  useful partial verification run. Append its entry to `docs/evidence/README.md`.
- Put temporary notes and command output in the chat, not the repository.
- Update this file instead of creating another roadmap.

## Session protocol

### Start

1. Read the milestone ledger and the authority/session sections above.
2. Search this file for the first `Status: OPEN`; read that package and select
   it only when its prerequisites are `DONE`.
3. Check the working tree. Do not overwrite unrelated user changes or stage
   `.codex-remote-attachments`, build output, generated STEP files, or dumps.
4. Restate the package ID, goal, prerequisites, and explicit non-goals.
5. Inspect only enough implementation and tests to prove the next change.

### Implement

1. Add the smallest fixture that reproduces the missing capability or failure.
2. Make the unsupported state return a stable named refusal before widening
   accepted behavior.
3. Implement the narrowest complete path.
4. Add happy-path, boundary, adversarial, and vacuity tests as applicable.
5. Run the narrow test first, then the package verification commands.
6. Review the diff for hidden fallback, tolerance-only acceptance, missing
   provenance, nondeterminism, and unrelated changes.

### Finish

A package is `DONE` only when all of these are true:

- its binary exit gate is met;
- every new failure mode has a certificate or stable named refusal;
- required coverage reports include expected, checked, skipped, and failed
  counts and cannot pass vacuously;
- no weld, legacy selector, silent repair, or unsupported fallback was added;
- relevant strict tests pass from a cleanly configured preset;
- required cross-platform or analyzer proof passes;
- one concise evidence file records revision, commands, fixtures, and results;
- the evidence index is updated;
- milestone status changes only when the entire milestone gate is met.

Interrupted verification leaves the package `OPEN`. Record a partial result in
the existing package evidence only when it prevents the next agent from
repeating expensive work.

### Handoff block

Paste this into the next chat when switching tools:

```text
RESUME
Package:
Goal:
Status: OPEN | DONE | BLOCKED
Changed files:
Commands and results:
Evidence:
Uncommitted user changes not to touch:
Exact next action:
Known refusal/error:
END RESUME
```

Chat is transport, not authority. The receiving agent verifies this block
against the working tree, ledger, and evidence.

## Standard verification commands

Use the narrowest test regex while iterating. Use the relevant complete lane
before claiming a package.

Windows:

```powershell
cmake --preset vs2022
cmake --build --preset vs2022
ctest --preset vs2022
cmake --preset vs2022-static-analysis
cmake --build --preset vs2022-static-analysis
ctest --preset vs2022-static-analysis
```

Linux:

```bash
cmake --preset linux-gcc
cmake --build --preset linux-gcc
ctest --preset linux-gcc
cmake --preset linux-gcc-static-analysis
cmake --build --preset linux-gcc-static-analysis
ctest --preset linux-gcc-static-analysis
```

If a full lane is not required by the package, record exactly which test regex
was run and why it is sufficient. Never convert a failed check into a warning
merely to claim the gate.

## Completed foundation

Do not reimplement these. Consult their indexed evidence when a package depends
on them:

- M1: immutable source, isolated working B-rep, bounded repair,
  correspondence, and certificate-or-named-refusal — `PASSED`.
- M2: total topology/geometry reconnaissance and unsupported-family accounting
  — `PASSED`.
- M3 foundation: exact interval assignment and independent chain sums,
  canonical edge samples, endpoint identity, exact predicates, and planar trim
  validation/assembly.
- M4–M6 foundation: exact planar/hole CDT, certified planar box, registered
  full-cylinder wall, capped cylinder, and connected through-hole.
- M7 foundation: certified CLI conversion/export/sweep, app/live-link routing,
  recipe v2 migration, and certified per-edge counts.

## Ordered work packages

Package status below describes task completion, not milestone status. Change it
only with indexed evidence.

### WP-001 — Close the Linux GCC analysis gate

- Status: `DONE`
- Milestone: M0
- Prerequisites: none
- Goal: produce a complete, reproducible Linux GCC strict and static-analysis
  result for the supported product targets.
- Primary files: `CMakeLists.txt`, `CMakePresets.json`,
  `core/CMakeLists.txt`, analyzer-touched sources, tests exposing failures.
- Steps:
  1. Remove temporary debug output from corpus/meshing tests.
  2. Configure and build `linux-gcc`; run all tests.
  3. Configure and build `linux-gcc-static-analysis`; run all tests.
  4. Classify each compiler crash separately from an analyzer finding.
  5. Fix real findings. Keep any translation-unit analyzer exclusion as narrow
     as possible and record compiler/version/reproducer.
  6. Re-run both complete Linux lanes.
  7. Confirm generated and committed corpus accounting is deterministic; an
     OCCT-version capability difference must be an explicit refusal, not a
     missing occurrence or false pass.
- Tests: all registered tests in both Linux presets.
- Evidence: `docs/evidence/m0-linux-gcc-analysis-YYYY-MM-DD.md`.
- Exit gate: both supported Linux lanes build and test cleanly, with every
  analyzer exclusion justified and no debug instrumentation left behind.
- Status update: mark M0 `PASSED` only if its full ledger gate is then met.

### WP-010 — Prove periodic closure and UV lifting

- Status: `DONE`
- Milestone: M3
- Prerequisites: WP-001
- Goal: prove full-period closure and reject inconsistent seam lifts.
- Primary files: `core/src/canonical_boundary.cpp`,
  `core/src/secure_meshing.cpp`, corresponding headers,
  `tests/test_canonical_boundary.cpp`, `tests/test_secure_meshing.cpp`.
- Steps:
  1. Add fixtures for ordinary full period, reversed coedge, seam crossing,
     near-full non-closure, mismatched endpoints, and wrong integer-period lift.
  2. Record source edge identity, orientation, UV endpoints, selected lift, and
     closure witness in the certificate.
  3. Require one bijective canonical sample sequence for all owning coedges.
  4. Refuse ambiguous or inconsistent lifts by stable diagnostic code.
  5. Check deterministic certificate/report digests on repeated runs.
- Tests: `canonical_boundary|secure_meshing|cylinder_template`.
- Evidence: `docs/evidence/m3-periodic-closure-YYYY-MM-DD.md`.
- Exit gate: every valid full-period fixture has one proved closure/lift and
  every adversarial fixture fails before face generation by name.

### WP-011 — Account repeated wire occurrences

- Status: `DONE`
- Milestone: M3
- Prerequisites: WP-010
- Goal: preserve distinct coedge uses when a wire or edge occurs repeatedly.
- Primary files: `core/src/secure_topology.cpp`,
  `core/src/planar_trim_assembly.cpp`, canonical-boundary code,
  `tests/test_secure_core.cpp`, `tests/test_planar_trim_assembly.cpp`.
- Steps:
  1. Add repeated-wire and repeated-edge occurrence fixtures with orientation
     variants.
  2. Give every occurrence a distinct provenance identity while sharing only
     the correct topological edge sample sequence.
  3. Verify expected/checked occurrence and coedge-use counts.
  4. Reject alias collapse, duplicate ownership, and missing uses by name.
- Tests: `secure_core|planar_trim_assembly|canonical_boundary`.
- Evidence: `docs/evidence/m3-repeated-wire-occurrences-YYYY-MM-DD.md`.
- Exit gate: all occurrences are accounted exactly once without collapsing
  identity or duplicating mesh boundaries.

### WP-012 — Segment critical parameter intervals

- Status: `DONE`
- Milestone: M3
- Prerequisites: WP-011
- Goal: split trim/boundary evaluation at all supported monotonic, periodic,
  singular, and contact-critical events before meshing.
- Primary files: reconnaissance, canonical-boundary, trim-validation modules
  and their tests.
- Steps:
  1. Define the event record and deterministic ordering.
  2. Detect the supported line/circle/plane/cylinder event set.
  3. Insert event samples into the edge-owned sequence.
  4. Prove no event is skipped or double-counted.
  5. Return a named unsupported-critical-segmentation refusal for other
     families until their solver exists.
- Tests: `canonical_boundary|planar_trim_validation|secure_meshing`.
- Evidence: `docs/evidence/m3-critical-segmentation-2026-07-18.md`.
- Exit gate: supported critical events have complete non-vacuous coverage;
  unsupported events cannot reach CDT/template generation.

### WP-013 — Consume independent chain sums in a template

- Status: `DONE`
- Milestone: M3
- Prerequisites: WP-012
- Goal: make one certified planar/cylinder template consume the existing exact
  independent chain-sum solution.
- Primary files: `core/src/interval_solver.cpp`, selected template,
  `core/src/secure_meshing.cpp`, interval/template/recipe tests.
- Steps:
  1. Add a fixture whose valid layout requires a chain sum.
  2. Feed solved counts into canonical boundary generation.
  3. Certify requested, solved, and consumed counts.
  4. Add minimum, parity, conflict, replay, and tampered-consumption tests.
- Tests: `interval_solver|secure_meshing|secure_recipe`.
- Evidence: `docs/evidence/m3-template-chain-sum-consumer-2026-07-18.md`.
- Exit gate: the template cannot generate unless every solved interval is
  consumed exactly, and deterministic recipe replay reproduces the digest.

### WP-014 — Solve one bounded coupled count class

- Status: `DONE`
- Milestone: M3
- Prerequisites: WP-013
- Goal: unblock the narrowest coupled/aliased sum system required by current
  plane/cylinder fixtures without introducing approximate sums.
- Primary files: interval solver API/implementation and tests; BR-011.
- Steps:
  1. Specify one exact coupled class and finite complexity bounds.
  2. Build an independent exhaustive oracle for small domains.
  3. Implement deterministic solving and tie-breaking.
  4. Add satisfiable, unsatisfiable, aliased, overflow, and complexity tests.
  5. Keep all other coupled classes behind the existing named refusal.
- Tests: `interval_solver|secure_meshing`.
- Evidence: `docs/evidence/m3-bounded-coupled-counts-YYYY-MM-DD.md`.
- Exit gate: implementation equals the oracle over the exhaustive domain and
  refuses out-of-scope/over-budget systems by name.

### WP-015 — Prove cross-platform M3 determinism

- Status: `DONE`
- Milestone: M3
- Prerequisites: WP-014
- Goal: match count, boundary, lift, and report digests across Windows and
  Linux for the planar box, capped cylinder, and through-hole fixtures.
- Primary files: digest/report code and determinism tests.
- Steps:
  1. Define the compared fields and canonical serialization.
  2. Run repeated clean Windows and Linux builds.
  3. Remove locale, traversal-order, uninitialized-data, and floating-format
     variation.
  4. Store compact digest results, not generated binaries, as evidence.
- Tests: M3–M6 tests in strict Windows and Linux presets.
- Evidence: `docs/evidence/m3-cross-platform-determinism-YYYY-MM-DD.md`.
- Exit gate: all selected digests match across repeated runs and both systems.

### WP-020 — Reject body triangle intersections

- Status: `DONE`
- Milestone: M4/M5
- Prerequisites: WP-015
- Goal: independently detect non-adjacent 3D triangle intersections before a
  body is certified or exported.
- Primary files: `core/src/certified_mesh.cpp`, its header/tests,
  secure-meshing integration.
- Steps:
  1. Define allowed shared-vertex/shared-edge contacts from topology identity.
  2. Implement broad-phase candidate generation with deterministic ordering.
  3. Apply robust narrow-phase tests and account every candidate.
  4. Add crossing, coplanar overlap, legal adjacency, near-contact, duplicate,
     and vacuous-validator fixtures.
  5. Integrate the result into `MeshingResult` and export admission.
- Tests: `certified_mesh|secure_meshing`.
- Evidence: `docs/evidence/m4-m5-triangle-intersections-2026-07-18.md`.
- Exit gate: all illegal intersections refuse by name; legal adjacency passes;
  expected/checked/skipped/failed counts are non-vacuous.

### WP-021 — Certify interior axial cylinder samples

- Status: `DONE`
- Milestone: M4/M6
- Prerequisites: WP-020
- Goal: permit adaptive axial rings without losing source/template provenance.
- Primary files: cylinder template, canonical boundary, certified mesh, secure
  meshing, and their tests.
- Steps:
  1. Define provenance for non-boundary axial stations and generated vertices.
  2. Derive station placement from explicit error/count constraints.
  3. Certify station order, surface evaluation, chord/normal bounds, and face
     ownership.
  4. Add zero/one/multiple ring, reversed frame, tampered provenance, and
     excessive-error fixtures.
- Tests: `cylinder_template|certified_mesh|secure_meshing`.
- Evidence: `docs/evidence/m4-m6-axial-cylinder-samples-2026-07-18.md`.
- Exit gate: extra rings pass only with complete provenance and analytic error
  proof; current extra-sample adversaries still fail when evidence is removed.

### WP-022 — Mesh and certify partial cylinders

- Status: `DONE`
- Milestone: M4/M6
- Prerequisites: WP-010, WP-013, WP-021
- Goal: produce boundary-exact certified walls for open and seam-crossing
  partial cylindrical bands.
- Primary files: cylinder template, secure meshing/reconnaissance, interval
  solver integration, and tests.
- Steps:
  1. Freeze isolated, capped, connected, seam-crossing, reversed, and
     mismatched-count fixtures.
  2. Register angular phase and unwrap the bounded UV interval.
  3. Consume canonical samples on both rims and both side rails.
  4. Reconcile axial/circumferential counts through certified solver output.
  5. Generate triangles with complete boundary/interior provenance.
  6. Validate closure where caps exist, intersections, chord/normal bounds,
     and deterministic digests.
- Tests: `cylinder_template|interval_solver|secure_meshing|certified_mesh`.
- Evidence: `docs/evidence/m4-m6-partial-cylinder-2026-07-18.md`.
- Exit gate: the M6 partial-cylinder fixture family passes end to end with no
  weld; malformed bands refuse before generation by stable code.

### WP-023 — Prove mismatched cylinder reference frames

- Status: `DONE`
- Milestone: M6
- Prerequisites: WP-022
- Goal: register compatible cylinders whose source faces use different but
  geometrically equivalent frames.
- Primary files: reconnaissance, canonical-boundary registration, cylinder
  template, secure meshing, and tests.
- Steps:
  1. Add rotated-origin, reversed-axis, shifted-angle, reflected, and genuinely
     incompatible fixtures.
  2. Prove axis/radius equivalence and an orientation-aware phase transform.
  3. Apply it to canonical samples without changing source identity.
  4. Reject reflection/twist/incompatibility by distinct named codes.
- Tests: `secure_core|canonical_boundary|cylinder_template|secure_meshing`.
- Evidence: `docs/evidence/m6-cylinder-frame-registration-2026-07-18.md`.
- Exit gate: compatible fixtures share registered columns; all incompatible
  fixtures fail before wall assembly.

### WP-030 — Add incidence and Euler validation

- Status: `DONE`
- Milestone: M5
- Prerequisites: WP-020
- Goal: compare source topology incidence with certified mesh incidence and
  report component/Euler deltas without assuming every source is closed.
- Primary files: secure topology, certified mesh, validation integration/tests.
- Steps:
  1. Define expectations by solid, shell, face, wire, and source defect class.
  2. Account vertices, edges, faces, boundaries, and connected components.
  3. Separate source defects from meshing defects.
  4. Add closed, open, holed, multi-body, duplicated, missing-face, and vacuous
     evidence fixtures.
- Tests: `secure_core|certified_mesh|secure_meshing`.
- Evidence: `docs/evidence/m5-incidence-euler-validation-2026-07-18.md`.
- Exit gate: every certified result has complete incidence accounting and zero
  unexplained topology delta.

### WP-031 — Certify modelling-mesh provenance

- Status: `DONE`
- Milestone: M5/M6
- Prerequisites: WP-030
- Goal: stop treating an unexplained alias of the safety floor as independent
  modelling topology.
- Primary files: `MeshingResult`, certified mesh, export adapter, tests.
- Steps:
  1. Represent explicitly whether modelling output is absent, an identified
     alias, or independently generated.
  2. Require full element provenance and validation for independent output.
  3. Add vacuous, tampered, alias, and independent-output fixtures.
  4. Ensure exporters report which output they consume.
- Tests: `certified_mesh|secure_meshing` plus CLI/export tests.
- Evidence: `docs/evidence/m5-modelling-provenance-2026-07-18.md`.
- Exit gate: no result can claim modelling topology without a non-vacuous
  certificate; a deliberate floor alias is explicit and truthful.

### WP-032 — Generate first independent modelling polygons

- Status: `DONE`
- Milestone: M6
- Prerequisites: WP-022, WP-023, WP-031
- Goal: produce independently certified quads/n-gons for the planar and
  cylinder baseline while preserving the certified triangle floor.
- Primary files: plane/cylinder templates, `MeshingResult`, modelling
  validation, export adapter, tests.
- Steps:
  1. Define deterministic triangle-to-polygon ownership for existing templates.
  2. Emit planar and cylindrical strips with source/working/triangle ancestry.
  3. Validate polygon simplicity, orientation, coverage, manifold incidence,
     surface error, and correspondence to the floor.
  4. Add box, capped/full/partial cylinder, through-hole, density sweep, and
     tampered-ancestry fixtures.
- Tests: all template, certified-mesh, secure-meshing, and export tests.
- Evidence: `docs/evidence/m6-independent-modelling-topology-2026-07-18.md`.
- Exit gate: the complete M6 fixture family has independent, modelable
  polygons and an unchanged certified floor.

### WP-040 — Audit all app and export admission paths

- Status: `DONE`
- Milestone: M7
- Prerequisites: WP-031
- Goal: require a valid `MeshingResult` certificate at every regeneration,
  display, live-link, and export boundary.
- Primary files: app routing, CLI adapters, exporters, recipe application, and
  integration tests.
- Steps:
  1. Enumerate every call site that displays, caches, links, or exports mesh.
  2. Centralize certificate admission and output-selection reporting.
  3. Add missing/tampered/stale-generation/refusal integration tests.
  4. Confirm no legacy or direct OCCT triangulation route remains reachable.
- Tests: full Windows strict lane plus app/CLI smoke commands.
- Evidence: `docs/evidence/m7-admission-path-audit-2026-07-18.md`.
- Exit gate: every product route is enumerated and refuses uncertified or stale
  results before use.

### WP-041 — Apply one certified per-face recipe operation

- Status: `DONE`
- Milestone: M7
- Prerequisites: WP-032, WP-040
- Goal: unblock the narrowest useful per-face recipe setting through a proven
  source reference and certified template consumer.
- Primary files: secure recipe, selected template, CLI/app controls, tests;
  BR-010.
- Steps:
  1. Select one existing persisted per-face field with a clear M6 consumer.
  2. Resolve source-to-working identity and report conflicts by name.
  3. Apply it before count/template generation.
  4. Certify requested, resolved, consumed, and resulting values.
  5. Prove save/reload and CLI/app replay digest equality.
- Tests: `secure_recipe|secure_meshing` plus CLI/app replay.
- Evidence: `docs/evidence/m7-certified-per-face-recipe-2026-07-18.md`.
- Exit gate: the operation has one complete certified consumer and
  deterministic replay; all other unsupported operations continue to refuse.

### WP-042 — Add secure cache and proxy invalidation

- Status: `DONE`
- Milestone: M7
- Prerequisites: WP-040, WP-041
- Goal: reuse only artifacts whose source snapshot, recipe, settings,
  dependency closure, implementation version, and certificate identity match.
- Primary files: app generation/cache state, secure result/report types, tests.
- Steps:
  1. Define a canonical cache key and dependency record.
  2. Preserve generation epochs and reject stale async results.
  3. Invalidate changed faces plus the boundary/count dependency closure.
  4. Prove unaffected certified artifacts remain byte-identical.
  5. Add source/recipe/settings/version/corruption/race invalidation tests.
- Tests: app integration, `secure_recipe`, `secure_meshing`, full strict lane.
- Evidence: `docs/evidence/m7-secure-cache-invalidation-2026-07-18.md`.
- Exit gate: cache hits are reproducible and certified; every stale or partial
  entry is rejected by name.

### WP-043 — Complete editing, overlays, and LOD reporting

- Status: `DONE`
- Milestone: M7
- Prerequisites: WP-042
- Goal: complete the remaining M7 workflow gate without allowing UI state to
  bypass certified generation.
- Primary files: app editing/overlay/report paths, secure recipe, tests.
- Steps:
  1. Route supported edits through source-referenced recipe operations.
  2. Show source defects, meshing refusals, certificate coverage, and selected
     output as distinct overlays.
  3. Name every LOD/density value and its effect in the report.
  4. Reject unsupported manual operations before mutating authoritative state.
  5. Prove hot reload and Blender link preserve the same report/digest.
- Tests: app/CLI/live-link integration and full strict lane.
- Evidence: `docs/evidence/m7-certified-editing-overlays-2026-07-18.md`.
- Exit gate: the full M7 ledger gate passes and M7 can be marked `PASSED`.

### WP-050 — Classify the frozen corpus against the certified pipeline

- Status: `DONE`
- Milestone: M4/M8 readiness
- Prerequisites: WP-043
- Goal: establish the measured coverage baseline before adding another surface
  family.
- Primary files: corpus tests/sweep reporting; no mesher expansion in this WP.
- Steps:
  1. Run every frozen committed and generated subject.
  2. Record import, reconnaissance, working validity, supported family,
     meshability, refusal code, and certificate result.
  3. Assert every subject ends in certified output or one stable named refusal.
  4. Group gaps by user value and geometry family without changing status.
- Tests: committed/generated corpus plus secure global sweep.
- Evidence: `docs/evidence/m4-m8-certified-coverage-baseline-2026-07-18.md`.
- Exit gate: corpus totals reconcile exactly and no failure is unclassified.

### WP-070 — CONE-A reconnaissance

- Status: `DONE`
- Milestone: M8
- Prerequisites: WP-050
- Goal: exact family/domain/trim classification for cone subjects and stable
  named unsupported residual cases.
- Primary files: `secure_reconnaissance`, cone fixtures, classification tests.
- Steps:
  1. Characterize the OCCT topology of `makeFixture("cone")` (faces, edges,
     apex degeneracy, seam).
  2. Prove every cone fixture face/edge is classified exactly once with stable
     family/domain/trim records.
  3. Keep meshing support deferred until CONE-C; document named deferral codes.
  4. Keep non-cone residuals (sphere/torus/freeform) refusing by name.
- Tests: reconnaissance / secure_core / secure_meshing classification coverage.
- Evidence: `docs/evidence/m8-cone-a-reconnaissance-2026-07-18.md`.
- Exit gate: every cone fixture subject is accounted exactly once; non-cone
  residuals still refuse by stable name.

### WP-071 — CONE-B canonical boundaries

- Status: `DONE`
- Milestone: M8
- Prerequisites: WP-070
- Goal: critical segmentation, periodic/singular (apex) lifting, and count
  constraints for cone edges/faces.
- Primary files: `canonical_boundary`, interval integration, tests.
- Steps:
  1. Extend critical-parameter segmentation to cone-compatible curve/surface
     pairs (circle on cone, generator on cone, apex singularity).
  2. Define singular lift policy at the apex; refuse ambiguous lifts by name.
  3. Integrate cone edges into analytic interval demands without welding.
  4. Add adversaries for missing apex events, twisted seams, and unsupported
     curve/face pairs.
- Tests: `canonical_boundary|interval_solver|secure_meshing`.
- Evidence: `docs/evidence/m8-cone-b-boundaries-2026-07-18.md`.
- Exit gate: supported cone boundaries have bijective samples and UV uses;
  apex/seam adversaries refuse by name before generation.

### WP-072 — CONE-C certified floor

- Status: `OPEN`
- Milestone: M8
- Prerequisites: WP-071
- Goal: boundary-exact cone tessellation with chord/normal bounds, provenance,
  and body intersection/incidence certificates.
- Primary files: new `cone_template`, `secure_meshing`, `secure_reconnaissance`
  support bit, certified-mesh assembly, tests.
- Steps:
  1. Implement a cone wall template sibling to the cylinder contract.
  2. Promote cone to `SupportedAnalyticTemplate` only with a live consumer.
  3. Route `familyCode == "cone"` through `generateSecureMesh`.
  4. Certify isolated/capped cone fixtures with no weld; refuse malformed
     bands by name.
- Tests: `cone_template|certified_mesh|secure_meshing|canonical_boundary`.
- Evidence: `docs/evidence/m8-cone-c-certified-floor-2026-07-18.md`.
- Exit gate: the capped apex-cone fixture certifies end to end with complete
  floor certificates.

### WP-073 — CONE-D modelling topology

- Status: `OPEN`
- Milestone: M8
- Prerequisites: WP-072
- Goal: deterministic quads/n-gons above the cone floor with truthful
  provenance (Independent or explicit floor alias).
- Primary files: modelling mesh builder, provenance validation, tests.
- Steps:
  1. Pair cone wall strips into modelling quads when topology allows.
  2. Keep truthful `CertifiedFloorAlias` when pairing cannot be proved.
  3. Refuse tampered independent claims by name.
- Tests: `certified_mesh|secure_meshing|cone_template`.
- Evidence: `docs/evidence/m8-cone-d-modelling-2026-07-18.md`.
- Exit gate: cone results carry a non-vacuous modelling provenance certificate.

### WP-074 — CONE-E product integration

- Status: `OPEN`
- Milestone: M8
- Prerequisites: WP-073
- Goal: recipe/app/export/live-link/cache admit certified cone results only
  through existing admission gates.
- Primary files: CLI/app admission paths, export adapter, cache keys, tests.
- Steps:
  1. CLI `mesh` / convert for cone through `admitCertifiedMeshingResult`.
  2. App headless screenshot for cone; confirm sphere still refuses.
  3. Confirm secure cache invalidation still covers family/settings changes.
- Tests: CLI/app smoke plus `secure_meshing|secure_recipe`.
- Evidence: `docs/evidence/m8-cone-e-product-2026-07-18.md`.
- Exit gate: product routes export certified cone meshes and still refuse
  unsupported families by name.

### WP-075 — CONE-F family proof

- Status: `OPEN`
- Milestone: M8
- Prerequisites: WP-074
- Goal: isolated, connected, adversarial, density-sweep, corpus impact, Linux
  determinism, and indexed evidence; declare cone supported.
- Primary files: cone/secure/corpus tests and evidence index.
- Steps:
  1. Run the full cone proof matrix and update corpus totals.
  2. Lock Linux digests/fingerprints for cone.
  3. Record family-complete evidence; leave M8 `IN_PROGRESS` for later
     families.
- Tests: full cone/secure/corpus regex on both Linux lanes.
- Evidence: `docs/evidence/m8-cone-f-proof-2026-07-18.md`.
- Exit gate: cone is recorded as a supported automatic family; sphere/torus
  remain deferred.

## M8 family packet

After M7 passes, repeat this packet for one family at a time. Choose the next
family from WP-050 data and Plasticity-to-Blender value; do not create a new
roadmap document.

1. `FAMILY-A` reconnaissance: exact family/domain/trim classification and named
   unsupported cases.
2. `FAMILY-B` canonical boundaries: critical segmentation, periodic/singular
   lifting, and count constraints.
3. `FAMILY-C` certified floor: boundary-exact tessellation, error bounds,
   intersection and provenance validation.
4. `FAMILY-D` modelling topology: deterministic polygons/quads above the floor.
5. `FAMILY-E` product integration: recipe/app/export/live-link and cache.
6. `FAMILY-F` proof: isolated, connected, adversarial, density-sweep, corpus,
   Windows/Linux determinism, and evidence.

Initial priority after the coverage baseline: cone (WP-070…WP-075), sphere,
torus, analytic fillet/blend connectors, then mapped/freeform regions. A
family is not “supported” until all six steps pass.

## M9 release packet

M9 starts only after the selected M8 release families pass:

1. Audit every dependency and close BR-006 with a distributable predicate/CDT/
   intersection implementation or an appropriate license.
2. Reproduce clean Windows and Linux builds from empty build directories.
3. Run strict, static-analysis, corpus, CLI, app, export, and Blender-link gates.
4. Verify deterministic reports/topology digests and package contents.
5. Perform the artist-facing Plasticity-to-Blender acceptance set.
6. Record one release evidence file and mark M9 only when every gate passes.
