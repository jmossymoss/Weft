# Weft execution plan

This is the sole product and engineering roadmap for Weft. Historical plans and
session handoffs are not authoritative. If code, comments, issues, or old
branches disagree with this document, follow this document or revise it with
new test evidence before changing direction.

## 1. Mission

Weft is a B-rep-native retopology bridge for hard-surface artists:

```text
Plasticity STEP -> Weft -> editable game topology -> Blender
```

Weft keeps the source B-rep live, generates a strong starting mesh, lets the
artist correct local topology while constrained to the exact CAD surfaces, and
persists those decisions so the mesh can be regenerated after density or CAD
changes.

The primary user is a Blender artist building game-ready hard-surface assets
from Plasticity. The primary job is not universal automatic quadrangulation. It
is obtaining a predictable, editable result faster than exporting a dead mesh
and retopologizing it from scratch.

### Product strategy

- Manual-first assistance with strong automatic defaults.
- Exact B-rep projection rather than shrinkwrap approximation.
- Local, named controls for primitives and blends.
- Watertightness and determinism before topology breadth.
- Persist decisions in recipes; treat generated meshes as replaceable output.
- One production generation pipeline, with experiments kept out of the release
  path until evidence justifies changing the architecture.

### Durable foundations

Preserve these foundations while stabilizing the product:

- The headless C++ geometry and meshing core.
- OpenCASCADE B-rep import, healing, and analysis.
- Stable face and edge identity where available.
- Per-vertex `(faceId, u, v)` anchors.
- Recipe serialization and geometric remapping.
- Surface-constrained editing.
- Per-face and per-edge density controls.
- Validation, diagnostic overlays, and face-attributed reports.
- Static export and the thin Blender live-link add-on.

## 2. Active scope

The active target is an artist-usable MVP. Work that does not help meet the MVP
completion gate is deferred.

### MVP includes

- Reliable STEP import for the target asset class.
- A clean automatic starting mesh for supported hard-surface geometry.
- Safe per-face density and topology controls.
- The existing constrained correction loop needed to repair local output.
- Recipe save, reload, regeneration, and reasonable remapping after CAD edits.
- OBJ delivery and Blender live-link delivery.
- Actionable diagnostics for unsupported or invalid source geometry.
- Reproducible Linux and Windows builds and tests.
- Packaging and onboarding sufficient for an artist to complete the workflow.

### Deferred until after MVP

- A new global or decoupled meshing architecture.
- Cross-field or fully automatic freeform quad layout.
- Density painting.
- A comprehensive knife, ring, radial, spin, and pole-editing suite.
- UV seam generation, straightening, and packing.
- USD export and additional nonessential formats.
- Embedded Python or Lua scripting.
- Asset-class presets and broad pipeline APIs.
- Making every stress or research model release-perfect.

Deferred work may be reconsidered only after work package 6 passes. It must not
be inserted into an active package merely because adjacent code is being
changed.

## 3. Definition of completion

MVP is complete only when every gate below passes at the same revision.

### 3.1 Geometry correctness

For every release model whose source is a valid closed solid:

- 0 open mesh edges.
- 0 non-manifold mesh edges.
- 0 folded or inverted polygons.
- 0 faces demoted to raw OpenCASCADE triangulation.
- 0 faces emitting no output.
- Consistent winding.
- No crash, hang, or unbounded memory growth.

Contract-floor output is allowed when it is watertight, local, reported, and
visually editable. It is not equivalent to raw triangulation.

Invalid, open, or deliberately adversarial source B-reps have an explicit
expected outcome in the corpus manifest. The expected outcome must be one of:

- healed to a valid closed result;
- rejected with a specific diagnostic;
- meshed with a documented, bounded defect for a research-only case.

Do not claim that a mesher can repair arbitrary invalid input.

### 3.2 Control safety

- The supported density sweep passes without opens, non-manifold edges, folds,
  raw fallback, or empty faces.
- Per-face and per-edge overrides preserve shared-border compatibility.
- A local edit does not silently demote an adjacent face.
- Regeneration is deterministic for identical input, settings, and platform.
- Linux and Windows produce equivalent topology according to the committed
  cross-platform policy.

### 3.3 Workflow

- Load STEP, inspect/select a face, change its supported controls, regenerate,
  save a recipe, reload it, and export successfully.
- Existing constrained edits survive a density change when their anchors remain
  valid.
- A reasonable upstream CAD edit remaps surviving decisions and reports dropped
  decisions rather than silently losing them.
- OBJ import and Blender live link preserve the mesh and CAD face identity.
- Export uses the fully finalized mesh, not the reduced interactive preview.

### 3.4 Real-work validation

- A versioned set of fresh Plasticity exports representative of expected work
  passes the geometry and workflow gates.
- Weft and Plasticity outputs are compared in Blender using the visual rubric in
  section 7.
- An artist can complete the tested workflow without an engineer changing code,
  manually repairing files, or explaining undocumented controls.

### 3.5 Delivery

- Linux and Windows CI are green.
- Clean build and package instructions work on both platforms.
- The release contains the app, CLI, required runtime libraries, Blender add-on,
  and concise onboarding.
- Known limitations identify unsupported geometry classes and invalid-input
  behavior without overstating readiness.

## 4. Corpus architecture

No single model can represent all possible B-reps. Trimmed B-spline geometry,
topological combinations, and tolerance defects are unbounded. Coverage comes
from several corpus layers with distinct purposes.

### 4.1 Deterministic geometry zoo

Generate small OpenCASCADE-authored fixtures for finite implementation
categories. Each fixture isolates one behavior and records source validity,
expected mesher family, expected output invariants, and visual intent.

Surface coverage:

- plane;
- cylinder;
- cone, including apex singularity;
- sphere, including both poles;
- torus, including periodic directions;
- surface of extrusion;
- surface of revolution;
- Bezier surface;
- B-spline surface;
- offset surface;
- supported combinations of trimmed and periodic surfaces.

Curve and trim coverage:

- line;
- circle and circular arc;
- ellipse;
- parabola and hyperbola where supported by import and meshing;
- Bezier curve;
- B-spline curve;
- offset curve;
- closed and open wires;
- inner wires and multiple holes;
- seam edges;
- degenerate pole edges;
- reversed wire orientation;
- unequal opposite-side sample counts.

Interaction coverage:

- cylinder-to-cap and cylinder-to-plate;
- full and partial revolution bands;
- holes and slots through planar and curved faces;
- constant-radius fillet strips;
- closed blend rings;
- chamfers and tiny bevels;
- planar plates with multiple holes;
- notched and castellated rims;
- ribbons and curved extrusion strips;
- three-or-more-face junctions;
- multiple solids, compounds, and assemblies.

Use pairwise interaction coverage first. Do not attempt the full Cartesian
product of every surface, curve, trim, and tolerance category.

### 4.2 Adversarial geometry zoo

Keep adversarial fixtures separate from valid canonical fixtures:

- micro-edges and sliver faces;
- near-coincident and duplicate vertices or edges;
- gaps below, at, and above the configured healing tolerance;
- reversed faces and inconsistent source orientation;
- degenerate or duplicate trims;
- self-intersecting wires;
- tangent contacts and zero-width regions;
- periodic seams intersected by holes or notches;
- very small features on very large bodies;
- high aspect-ratio and near-singular patches.

Each case must say whether the expected behavior is healing, rejection, or
bounded research-only output.

### 4.3 Release set

The release gate uses a small set representative of the intended product:

- `tests/STEP_Examples/flaregun.stp`
- `tests/STEP_Examples/iso14649-demo.stp`
- the generated `torture` fixture
- `tests/STEP_Examples/foam.stp`
- `tests/STEP_Examples/teleporter.stp`

This set covers a complex hard-surface asset, bores and common mechanical
primitives, combined synthetic features, and two difficult real mechanical
models. A model may be replaced only with a written justification that preserves
or improves feature coverage.

### 4.4 Stress and research set

These remain visible and are run on an appropriate scheduled or manual cadence,
but they do not silently expand MVP:

- `tests/STEP_Examples/nasty_cheese.stp`
- `tests/STEP_Examples/MP9.stp`
- `tests/STEP_Examples/tork.stp`
- the generated `slitdrill` fixture

MP9 is a target-shaped integration and performance workload. It is not the
geometry-coverage oracle. Research failures must be recorded, not hidden by
weakening release assertions.

### 4.5 Public real-world corpus

Use reproducible manifests with upstream URL, version, checksum, license note,
selection rule, and expected local path. Do not commit an unbounded external
dataset to this repository.

- Fusion 360 Gallery Extended STEP: primary mechanical-feature corpus. Select a
  stratified subset using its operation labels for extrusion, cut, fillet,
  chamfer, and revolution, plus face-count and body-count bands.
  Source: `https://github.com/AutodeskAILab/Fusion360GalleryDataset`
- ABC STEP dataset: broad geometry and robustness corpus. Select by surface and
  curve types, face count, body count, and import result rather than taking an
  unclassified random sample.
  Source: `https://deep-geometry.github.io/abc-dataset/`
- NIST and CAx-IF STEP models: interoperability corpus for AP203/AP242, units,
  assemblies, and files produced by different kernels.
  Source: `https://www.nist.gov/ctl/smart-connected-systems-division/`
  `smart-connected-manufacturing-systems-group/mbe-pmi-0`
- MAMBO may be used as a small meshing-topology supplement, but it does not
  replace the mechanical or interoperability corpora.
  Source: `https://gitlab.com/franck.ledoux/mambo`

Start with a reviewable subset, then expand nightly breadth only after failures
are classified automatically. Dataset volume is not a substitute for coverage.

### 4.6 Fresh Plasticity set

Maintain a private or appropriately licensed test set of current Plasticity
exports matching the intended asset class. Record Plasticity version, export
settings, source validity, and expected feature classes. Convert every reduced
regression that can be shared into a deterministic fixture.

## 5. Scoreboard

`tests/CAD_CORPUS.tsv` is the machine-readable inventory. Extend it or generate a
separate machine-readable result without turning this plan into a session log.
For each case record:

- corpus tier and source validity;
- source and license/provenance;
- surface, curve, trim, and feature tags;
- load result and elapsed time;
- vertex and polygon counts;
- quad, triangle, and n-gon counts;
- open and non-manifold edge counts;
- folded, degenerate, and sliver counts;
- raw fallback and empty-face counts;
- chord deviation;
- density-sweep result;
- recipe and remap result where relevant;
- visual-review result and artifact;
- last verified commit.

Current measurements must come from scripts or test output. Do not put volatile
“currently passing” narratives in this plan.

## 6. Architecture decisions

### AD-1: production generation path

`weft::generate()` and its border-count contract are the only production
generation path for MVP. Improve it incrementally behind corpus tests.

The removed decoupled-core rewrite is not an active direction. Do not restore
it, port work from old branches, or begin another ground-up rewrite without a
new architecture decision supported by release-corpus evidence.

### AD-2: stitch experiment

`GenerationSettings::decoupleSeams` and CLI `--stitch` are quarantined
experiments. They are off by default and are not a second product architecture.
They may be used for controlled diagnosis. Promote or remove them only after an
A/B report across the release set shows a consistent advantage and no new
correctness failures.

### AD-3: mesher breadth

Do not add a new `MesherKind` during stabilization. First determine whether the
case belongs to an existing conceptual backend:

- revolution band;
- fillet or blend band;
- planar plate or hole web;
- border-exact graceful floor.

Specialized internal algorithms may remain, but fixes should reduce duplicated
routing, density, and seam behavior rather than add another parallel family.

### AD-4: monolith refactoring

Split `core/src/meshers.cpp` only with behavior locked by tests. Keep mechanical
extraction and topology changes in separate commits. Refactoring is not itself
an MVP milestone.

## 7. Visual acceptance rubric

Numeric validity is necessary but insufficient. Review release and fresh
Plasticity models in the Weft viewport and Blender:

- primitive silhouette follows the exact CAD shape at the selected density;
- cylinder and revolution columns are straight and intentional;
- fillet strips run across and along the blend in understandable directions;
- holes and slots receive local collars without global triangle fans;
- flat regions remain sparse;
- poles, triangles, and n-gons are deliberate and locally editable;
- no folded, overlapping, spiraling, or hair-thin polygons;
- normals and hard-surface shading are stable;
- density transitions do not produce visible seam artifacts;
- the result is easier to edit and bake than the Plasticity comparison export.

Record one concise artifact per meaningful comparison. Do not approve a change
only because polygon counts moved in the preferred direction.

## 8. Ordered work packages

Agents work on the first incomplete package only. A package is complete when
all its exit criteria pass in the same revision.

### WP0: restore truth

Goal: make repository status, tests, and corpus inventory trustworthy.

Tasks:

- Reconcile `tests/CAD_CORPUS.tsv` paths with fixture generation and committed
  regression assets.
- Ensure tests generate ephemeral fixtures before attempting to load them, or
  commit deterministic fixtures when generation is not appropriate.
- Separate release, stress, intentionally invalid, and performance cases.
- Add or generate the scoreboard fields required for correctness triage.
- Make Linux and Windows failures reproducible locally where practical.
- Remove stale assertions and goldens only when replaced by correct,
  evidence-backed expectations.

Exit:

- Corpus paths and documented commands exist.
- CTest runs the intended deterministic cases rather than failing on setup.
- The release gate reports all current failures precisely.
- Linux and Windows CI either pass or fail only on an explicit, reproducible
  release-blocker list owned by WP2/WP3.
- No documentation claims a red gate is green.

### WP1: build coverage

Goal: cover supported B-rep categories with small deterministic tests.

Tasks:

- Implement the geometry and adversarial zoo from section 4.
- Tag each fixture by surface, curve, topology, feature, and validity class.
- Assert import, analysis classification, selected mesher family, geometry
  invariants, and expected diagnostics.
- Add reduced cases for current release failures before changing their meshers.
- Add reproducible public-corpus manifests and a bounded smoke subset.

Exit:

- Every supported category in section 4.1 maps to at least one deterministic
  fixture.
- Every invalid-input policy has at least one adversarial test.
- Coverage reports missing categories instead of relying on model count.
- Fixture and smoke gates pass on Linux and Windows.

### WP2: stabilize one production pipeline

Goal: reduce architectural ambiguity and make fixes predictable.

Tasks:

- Freeze new mesher kinds and keep `generate()` authoritative.
- Classify release failures by topology class and shared root cause.
- Consolidate border sampling, density ownership, and fallback reporting.
- Audit raw-demotion paths and make unsupported cases explicit.
- A/B the stitch experiment; retain it only as a diagnostic unless it meets
  AD-2 promotion criteria.
- Convert useful probes into fixture assertions and retire superseded probes.

Exit:

- One documented production path serves app, CLI, export, and tests.
- Every release failure has a deterministic reduced reproducer.
- Raw fallback and empty output are reliably attributed by face and cause.
- No production behavior depends on a model filename or hard-coded face ID.
- The default path remains deterministic under parallel and repeated runs.

### WP3: pass the release set

Goal: meet geometry and control-safety gates on the five release models.

Tasks:

- Fix failures by topology class, smallest reproducer first.
- Run default and CAD profiles plus supported density and override sweeps.
- Check folds, slivers, winding, deviation, raw fallback, and empty output in
  addition to watertightness.
- Review affected visual regions after every intentional topology change.
- Keep stress results visible and reject regressions caused by release fixes.

Exit:

- All section 3.1 requirements pass for valid release models.
- All section 3.2 requirements pass.
- Intentional exceptions are limited to source-invalid cases with explicit
  diagnostics and cannot be silently broadened.
- Relevant visual rubric items pass with saved evidence.

### WP4: complete the artist correction loop

Goal: let an artist correct supported local problems without engineering help.

Tasks:

- Show only settings that affect the selected face and name axes semantically.
- Ensure density edits, existing loop insertion, constrained grab, bridge, weld,
  undo, and regeneration preserve anchors and topology invariants where the
  operation is supported.
- Make unsupported edits fail clearly without corrupting the recipe.
- Test recipe save/reload, density regeneration, CAD remap, dropped-operation
  reporting, and fully finalized export.
- Keep UI work limited to this correction and feedback loop.

Exit:

- End-to-end workflow tests in section 3.3 pass.
- A new user can identify a problem face, apply a supported correction, undo or
  save it, regenerate, and export using documented controls.
- Correction operations cannot bypass the release geometry gate.

### WP5: validate real work

Goal: demonstrate that fixture success transfers to the target asset class.

Tasks:

- Run stratified Fusion 360, ABC, and NIST smoke subsets.
- Run the fresh Plasticity set.
- Classify every failure by input validity and topology class.
- Reduce shareable failures and add them to the appropriate deterministic tier.
- Compare Weft and Plasticity output in Blender with section 7.
- Measure completion without engineering intervention, not just batch success.

Exit:

- Public smoke subsets meet their declared validity-specific expectations.
- Fresh Plasticity samples pass the release geometry and workflow gates.
- Visual comparison demonstrates a useful advantage or a materially faster
  path to an acceptable editable result.
- No new common failure class remains unrepresented in the deterministic zoo.

### WP6: ship readiness

Goal: produce a reproducible MVP release.

Tasks:

- Establish app and CLI performance budgets on the release set and MP9.
- Build and package on clean Linux and Windows environments.
- Bundle or document runtime dependencies and the Blender add-on.
- Validate onboarding from STEP import through Blender delivery.
- Write accurate limitations, troubleshooting, and release notes.
- Run every completion gate on the release candidate revision.

Exit:

- Every requirement in section 3 passes.
- A clean machine can install and complete the primary workflow.
- CI, package smoke tests, and release-candidate artifacts are green.
- Known limitations are explicit and do not contradict the completion claim.

## 9. Autonomous agent protocol

An autonomous agent must:

1. Read `AGENTS.md`, this plan, `README.md`, and `tests/CAD_CORPUS.md`.
2. Select the first incomplete work package and its highest-priority failing
   exit criterion.
3. Establish a reproducible baseline before editing behavior.
4. For a non-trivial bug, reduce or instrument it and use runtime evidence to
   identify the root cause.
5. Make the smallest coherent change that addresses the failure class.
6. Run the focused fixture, release-set checks, and broader gates proportional
   to the change.
7. Visually inspect topology changes where numeric tests cannot prove quality.
8. Update machine-readable expectations and concise status only with evidence.
9. Commit one logical change at a time.
10. Stop expanding scope when the current package exit criterion is met and
    continue to the next incomplete criterion.

The agent must not:

- special-case a filename, model name, or face ID;
- add a parallel mesher architecture without revising AD-1;
- add a mesher kind during WP0-WP3 without an approved plan revision;
- weaken a gate, exempt a valid model, or update goldens simply to make CI pass;
- treat compile success, application startup, or polygon counts alone as proof;
- append session diaries, speculative directions, or stale pass claims here;
- begin deferred work before WP6 passes.

## 10. Verification commands

Use the build configuration appropriate to the platform. The standard Linux
sequence is:

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
tools/corpus_gate.sh
```

Targeted validation:

```sh
build/cli/weft mesh model.step -o out.obj --profile cad --validate
build/cli/weft sweep model.step
```

Use `tools/corpus_gate.sh --no-golden` only for an intentional
cross-platform invariant check. It does not replace the normal golden gate.

Before accepting a topology change:

- run the smallest reproducer;
- run affected geometry-zoo categories;
- run all release models;
- inspect affected visual output;
- run CTest and the full corpus gate;
- confirm required CI jobs.

## 11. Plan maintenance

This file defines targets, order, and policy. It is not a changelog.

- Change a product boundary or architecture decision only in a dedicated,
  evidence-backed revision.
- Keep transient measurements in generated reports or the corpus scoreboard.
- Mark work-package completion only when its exit criteria pass at one commit.
- If a new failure invalidates a completion claim, reopen the earliest affected
  package.
- Keep post-MVP ideas in the deferred list until MVP ships.
