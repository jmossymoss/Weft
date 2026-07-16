# Weft meshing research plan

Last reviewed: 2026-07-16

This document turns the mesher rewrite into a research-backed engineering
programme. It supplements `MVP_PLAN.md`; it does not replace the artist-facing
acceptance bar in that document.

## 1. Research conclusion

No single published quadrangulation method covers Weft's full problem. The
strongest design is a hybrid pipeline with explicit ownership boundaries:

1. **B-rep topology is invariant.** Faces, loops, edges, edge order, and
   adjacency come from the CAD model. Geometric tolerances may change fidelity,
   never topology.
2. **The shared CAD edge owns its samples.** Adjacent face meshers consume the
   same ordered sample IDs and map them into their own parameter domains. They
   do not independently approximate or later weld the edge.
3. **A smooth size field proposes density; an integer solve makes it legal.**
   Curvature, local feature size, chord error, and artist controls propose a
   continuous target. Edge counts are then solved over the affected adjacency
   component with equality, parity, opposite-side, and template constraints.
4. **Analytic surfaces keep analytic meshers.** Cylinders, cones, tori, planes,
   fillets, corridors, collars, and caps use deterministic templates. Global
   field quadrangulation is an assist for general freeform interiors, not a
   replacement for primitive topology.
5. **Every mesher has a topologically safe floor.** When a structured quad
   layout is infeasible, the face falls back to a constrained, boundary-exact
   interior and then performs conservative quad pairing. It never substitutes
   an unrelated OCCT triangle soup or leaves a hole.
6. **Interactive and authoritative meshes are separate products.** GPU trim
   classification and tessellation provide immediate visual feedback. The CPU
   compiles only the dirty dependency closure into the authoritative topology.
   Global welding/conforming is reserved for validation/export.

This architecture preserves what is already strongest in the legacy meshers
(primitive-aware, artist-readable flow) while replacing their fragile global
heuristics, late repair passes, and face-local count mismatches with algorithms
that have published foundations.

## 2. Evidence tiers

- **Tier A — implement/adapt:** peer-reviewed work that directly addresses a
  current Weft failure mode.
- **Tier B — foundation:** older or general work that defines the mathematical
  basis, but needs adaptation to B-rep-aware hard-surface output.
- **Tier C — evaluate:** very recent or preprint work. Prototype behind a flag;
  do not make it a production dependency until its claims reproduce on Weft's
  corpus.

## 3. Paper-to-problem map

| Evidence | Contribution | Weft use | Tier / decision |
|---|---|---|---|
| [Li et al., *Robust tessellation of CAD models without self-intersections* (JCDE 2026)](https://doi.org/10.1093/jcde/qwaf134) | Samples shared 3D B-rep edges once, maps those samples into both face domains, subdivides critical trim regions, and uses constrained Delaunay triangulation (CDT) for a watertight, intersection-safe interior. | Basis for the canonical edge contract, trim-loop validation, thin-feature protection, and the guaranteed face floor. | **A — implement first.** Its triangle interior is a safety scaffold; Weft adds quad templates/pairing above it. |
| [Zhou et al., *Topology-First B-Rep Meshing* (2026 preprint)](https://arxiv.org/abs/2604.02141) | Treats the exact B-rep hierarchy as invariant while tolerance controls only geometric approximation. Evaluated on thousands of ABC and Fusion 360 models. | Architectural rule for face/loop/edge ownership and for eliminating post-hoc topology repair as the primary strategy. | **C — adopt the principle; reproduce the algorithmic claims before depending on the implementation.** |
| [Yang et al., *Boundary constrained quadrilateral mesh generation based on domain decomposition and templates* (2024)](https://doi.org/10.1016/j.compstruc.2024.107275) | Boundary-first CDT decomposition, triangle/quad/pentagon templates, and integer linear programming (ILP) to reconcile divisions across patches. | Direct basis for equal rail counts, corridor connectors, cutout webs, and explicit singularity templates instead of stretched fans. | **A — implement a small Weft-specific constraint/template subset.** |
| [Couplet, Reberol & Remacle, *Generation of High-Order Coarse Quad Meshes on CAD Models via Integer Linear Programming* (2021)](https://doi.org/10.2514/6.2021-2991) | Simplifies a dense CAD-aligned quad layout by assigning integer lengths to a T-mesh while preserving CAD features. | Basis for a later coarse-layout/finalization pass and for formulating strip collapse/retention as an optimization rather than thresholds. | **A — study after the boundary/count solver.** Not required for interactive preview. |
| [Bawin, Henrotte & Remacle, *Automatic feature-preserving size field for three-dimensional mesh generation* (2021)](https://doi.org/10.1002/nme.6747) | Builds a size field from curvature and local feature size, then bounds its gradient so density changes smoothly. | Replaces isolated pitch floors and model-specific density thresholds. Supplies a continuous target to the integer count solve. | **A — implement in surface form.** Artist overrides remain hard constraints. |
| [Couplet, Chemin & Remacle, *Size-controlled quadrilateral meshing using integrable odeco fields* (CAD 2026)](https://doi.org/10.1016/j.cad.2025.103974) | Integrable orthogonal frame fields satisfy feature/boundary sizing constraints and place the singularities needed for valid size transitions. | Scientific basis for flat and nearly developable freeform panels where a regular UV grid or current quad fill produces distortion. | **A — offline/background prototype.** Published timings are not suitable for the per-drag path. |
| [Couplet et al., *Surface Quadrilateral Meshing from Integrable Odeco Fields* (2026)](https://arxiv.org/abs/2604.03889) | Extends integrable Odeco fields to curved surfaces with feature alignment, sizing, and distortion objectives. | Candidate long-term freeform surface mesher with deliberate singularity placement. | **C — watch/prototype.** Keep behind a feature flag until peer review, code availability, and MP9 reproduction. |
| [Capouellez et al., *Feature-Aligned Parametrization in Penner Coordinates* (SIGGRAPH 2025)](https://doi.org/10.1145/3731216) | Robust seamless parametrization with hard sharp-feature alignment and soft preferred directions. | Candidate for freeform regions where CAD feature curves and artist guide strokes must control flow. | **A — benchmark against Odeco and the legacy coons path; do not run on analytic primitives.** |
| [Corman & Crane, *Rectangular Surface Parameterization* (SIGGRAPH 2025)](https://doi.org/10.1145/3731176) | Produces low-shear rectangular parameterizations that can adapt to long, thin geometry. | Candidate for fillet-like or ribbon-like freeform patches where continuous columns matter more than isotropy. | **A — targeted research prototype.** |
| [Bommes, Zimmer & Kobbelt, *Mixed-Integer Quadrangulation* (2009)](https://doi.org/10.1145/1531326.1531383) | Establishes the cross-field, seamless parametrization, integer singularity, and feature-aligned quad extraction formulation. | Mathematical foundation for Weft's future guided freeform mesher and its singularity vocabulary. | **B — foundation, not a direct production drop-in.** |
| [Jakob et al., *Instant Field-Aligned Meshes* (2015)](https://igl.ethz.ch/projects/instant-meshes/) and [Huang et al., *QuadriFlow* (2018)](https://stanford.edu/~jingweih/papers/quadriflow/) | Fast local field-aligned proposals; QuadriFlow reduces unnecessary irregular vertices relative to Instant Meshes. | Useful baselines and possible fast proposal generators for freeform regions. Their output must be re-constrained to B-rep edges and features. | **B — benchmark/proposal only.** They must not erase primitive columns or CAD identity. |
| [Ng & Tan, *Incremental tessellation of trimmed parametric surfaces* (2000)](https://doi.org/10.1016/S0010-4485(99)00089-5) | Stores tessellation/topology and updates only trimlines and affected local regions after model changes. | Basis for face generation epochs, dependency closures, persistent parameter-domain caches, and avoiding 0/3748 global remeshes. | **B — old but directly applicable. Implement the data-flow principle.** |
| [Zhu et al., *Projection-driven grid-BSP tree for real-time trimming on GPU* (2025)](https://doi.org/10.1016/j.cagd.2025.102451) | GPU trim classification with an on-surface error metric; reports trimming reduced to a small part of total rendering cost. | Basis for the GPU preview: upload NURBS/trim acceleration data once, retessellate dirty patches in compute/tessellation stages, and patch only changed draw ranges. | **A — use for preview only.** Exact topology/export remains CPU-authoritative. |

## 4. Target meshing architecture

### 4.1 `BrepTopologyContract`

- Assign stable IDs to bodies, shells, faces, loops, edges, and vertices.
- Store one ordered sample sequence per topological edge:
  `(edgeId, edgeParameter, position3D, perFaceUV[])`.
- Sample from the 3D edge by arc length/error criteria, then inverse-map into
  every owning face. Never let each face independently discretize a shared edge.
- Subdivide trim curves at monotonic/critical intervals before sampling so a
  later refinement cannot introduce a crossing.
- Validate loop orientation, containment, repeated vertices, and periodic-seam
  lifting before a face mesher runs.

This replaces global weld/conform as the normal way seams become watertight.
Welding remains a validation/export repair for imported or genuinely invalid
source geometry.

### 4.2 `SurfaceSizingField`

Compute a continuous target edge length `h(x)` from:

- chordal deviation and normal-angle budgets;
- principal curvature radii;
- local feature size/medial-distance estimates around close trims, slits, and
  thin walls;
- primitive semantics (radial/axial/across/along);
- artist minimum/maximum and explicit edge/face counts.

Apply a bounded spatial gradient to `h` so neighbouring regions grade smoothly.
The field proposes density; it does not directly create independent face counts.
On a cylinder, circumferential stations are one solved closed sequence shared by
the full primitive/patch group. Axial stations are solved separately. This makes
span spacing continuous instead of allowing each trimmed side to invent a
different column count.

### 4.3 `IntegerCountSolver`

Build constraints only for the dirty connected component:

- shared edge: identical sample sequence;
- four-sided block: opposite-side compatibility or an explicit transition
  template;
- cylinder/revolution group: one circumferential count and phase;
- paired rails: equal counts unless a named singularity template is selected;
- closed quad region: parity/even-boundary requirements where required;
- artist override: hard constraint if feasible, otherwise a visible conflict;
- objective: minimize deviation from `SurfaceSizingField`, previous-frame
  counts, and artist priorities.

Start with a deterministic bounded integer search/union-find reduction. Add a
small ILP solver only for components that cannot be resolved by equalities and
template rules. Never invoke a whole-model ILP for a one-face edit.

### 4.4 Mesher families

1. **Analytic/template:** plane n-gon, disk/annulus, cylinder/cone/revolution,
   torus/sphere, fillet strip, ribbon/corridor, cutout collar, and caps.
2. **Mapped block:** Coons/transfinite interpolation for valid 4-sided blocks
   whose count constraints have already been solved.
3. **Template decomposition:** CDT partitions a trimmed domain into a small set
   of triangle/quad/pentagon regions, then deterministic templates fill them.
4. **Field-guided freeform:** Penner/Odeco/rectangular parameterization prototype
   for genuinely freeform regions only.
5. **Safety floor:** boundary-exact CDT inside validated critical regions, with
   conservative adjacent-triangle pairing into quads/n-gons. This is a valid,
   watertight result—not a hidden OCCT fallback.

`quad-fill` must not return as a generic automatic choice. Its useful building
blocks should survive only as named, validated templates inside the decomposition
mesher.

### 4.5 `IncrementalMeshingCompiler`

Persist these independently keyed caches:

- CAD analysis/feature classification;
- trim acceleration and containment data;
- edge sample contracts;
- size-field tiles;
- integer-solver components;
- per-face topology and GPU ranges;
- validation summaries and visual-capture metadata.

An edit invalidates the selected face, changed edge constraints, and only the
neighbour closure whose boundary contract or count solution changes. Generation
epochs discard stale async results. Unaffected face topology and GPU buffers are
reused byte-for-byte.

### 4.6 `GpuProxy`

- GPU-evaluate and trim dirty parametric patches for feedback.
- Keep static CAD, proxy geometry, selection, and diagnostics in separate
  buffers so selection changes do not rebuild the model.
- Return a preview inside one frame where possible; replace it asynchronously
  with the exact local mesh.
- Never use screen-space LOD or GPU-only vertices as the exported topology.

## 5. Milestones and falsifiable experiments

### R0 — Baseline and scientific instrumentation

- Freeze MP9 reference zones: foregrip, sight housing, slotted drum/cylinder,
  receiver flats, holes/collars, fillet chains, and the known face 632 soup.
- Record selected face **and its complete one-ring of adjacent faces**.
- Record topology ownership, edge sample IDs, chosen mesher, constraints,
  fallback reason, timings, and quality metrics.
- Capture solid + wireframe renders from at least front/back/left/right and
  two close oblique views.

**Exit:** one command produces a machine report and a visual contact sheet for
the full MP9 model plus close-ups. Numeric success without visual review is not
an acceptance.

### R1 — Topology-first boundary contract

Prototype canonical shared-edge sampling and per-face UV mapping using the 2026
robust tessellation papers as the reference. Add monotonic trim segmentation,
periodic-domain handling, and a CDT safety floor.

**Exit:** 0 open edges, 0 non-manifold edges, 0 T-junctions, and a bijective
sample-ID sequence on every valid shared edge before any weld pass. Invalid CAD
topology is reported separately rather than hidden.

**Implementation checkpoint (2026-07-16):** the shared canonical edge types are
now used by both the primitive compiler and a guarded legacy adapter. After all
legacy density, phase, and pin repairs finish, an immutable table can own the
final sample sequence for ordinary two-owner line/circle edges. Freeform UV
projection and direct sample-ID ownership remain pending. The rollout is
disabled by default through
`GenerationSettings::canonicalEdgeContracts` until the full R1 exit gate is
met. The accepted MP9 handgrip fixture remains byte-identical with the guard
off. The extracted face 632 one-ring has no unexplained cracks, but its STEP
round-trip changes the source trim topology and the mapped target face remains
a 115-triangle contract floor; full-model `face_632` is therefore the
authoritative visual regression. This is an R1 foundation checkpoint, not R1
acceptance: the handgrip connector gaps remain and full MP9 still contains
open/non-manifold edges, folds, degenerate elements, and slivers.

### R2 — Scientific sizing and continuous cylinder spans

Replace mesher-local pitch floors with `SurfaceSizingField`. Solve one radial
count/phase for each connected coaxial revolution group, and keep axial/across
counts independent.

**Exit:** MP9 cylinders and cylinder-like trimmed bands have continuous,
evenly phased columns across adjacent patches and cutouts; no side mismatch or
fan is introduced. Chord/normal error stays within the chosen budget. A small
change in tolerance cannot cause unrelated faces to jump in density.

### R3 — Integer rail reconciliation

Implement the constrained count graph and the first templates: equal four-sided
block, 3/5-sided transition, corridor end, ring/collar, and explicit 3/5 pole
pair. Use the 2024 boundary/template paper as the primary reference.

**Exit:** paired rails have equal compatible counts; transitions occur at named
templates, not as stretched triangles. Per-face edits solve only the affected
component and cannot silently force every face to remesh.

### R4 — Primitive cutouts and connector library

Apply template decomposition to slots, holes, undercuts, border openings, and
fillet-to-panel/cylinder junctions. Preserve the already-good base lattice and
replace only the connector region.

**Exit:** foregrip top/bottom borders, groove ends, and undercut connectors are
closed and connected without changing unaffected base columns. Visual approval
uses the selected face plus its neighbours, not an isolated face render.

### R5 — Freeform field prototype

Build an offline comparison harness for legacy Coons, QuadriFlow, Penner,
rectangular parameterization, and Odeco on the same boundary-constrained
freeform patches. All candidates receive identical canonical boundary samples
and feature directions.

**Exit:** choose a field method only if it improves feature alignment,
orthogonality, singularity count/placement, and visual flow without losing CAD
identity or boundary conformity. Otherwise keep the legacy mapped/template path.

### R6 — Interactive compiler and GPU proxy

Introduce dependency epochs, local cache invalidation, partial GPU buffer
updates, and GPU trim classification.

Performance budgets (targets, not current claims):

- selection/highlight feedback: **<= 8 ms P95**;
- first proxy after a parameter drag: **<= 16.7 ms P95**;
- warm exact update for a simple single-face closure: **<= 100 ms P95**;
- MP9 progressive first view: **<= 500 ms** while exact background work
  continues;
- no unchanged face is replanned, re-meshed, re-welded, or re-uploaded.

If a high-quality field solve needs seconds, it runs as an optional background
refinement and never blocks the edit loop.

### R7 — Export finalization

Run expensive global validation, conservative conforming, optional welding,
normal/tangent generation, and export-only cleanup after interaction or on
explicit request.

**Exit:** the interactive mesh and final mesh have an explicit, reportable
relationship; finalization cannot silently substitute a different face mesher.

## 6. Acceptance metrics

### Topological

- open and non-manifold edge counts;
- T-junction count;
- B-rep face/loop/edge incidence preserved;
- shared-edge sample order and ownership match;
- isolated, duplicate, and zero-area elements;
- connected-component and Euler-characteristic deltas.

### Geometric

- maximum and percentile surface deviation;
- maximum normal deviation;
- boundary chord error;
- self-intersections and foldovers;
- trim containment violations;
- thin-feature clearance retained.

### Quad-flow

- quad/tri/n-gon ratios, reported by region rather than model total only;
- scaled Jacobian, aspect ratio, skew, and angle distribution;
- feature-alignment error;
- singularity count, valence, and distance from protected features;
- cylinder column phase/count continuity;
- strip/rail continuity across face boundaries.

### Stability and performance

- deterministic topology hash across repeated runs and platforms;
- topology churn after a small parameter change;
- dirty faces versus total faces;
- analysis, constraint, meshing, validation, and GPU-upload timings separately;
- cold load, progressive first view, warm edit P50/P95, and memory use.

## 7. Visual validation protocol

Every topology-affecting change must include:

1. the previous accepted render;
2. the candidate render with identical camera and settings;
3. solid and wireframe views;
4. the selected face highlighted **with all incident neighbour faces visible**;
5. close-ups of changed connectors and full-model MP9 views;
6. a written visual verdict against the Plasticity/CAD source and the supplied
   Blender reference, including any known mismatch.

Automated image diffs may flag changed areas, but they do not decide topology
quality. Human visual approval remains a release gate. Until the incremental
path is fast, use a small set of targeted density values for interactive tests;
move exhaustive sweeps to scheduled/nightly validation rather than blocking
each iteration.

## 8. Immediate implementation order

1. Extract and freeze the accepted primitive/foregrip baseline before further
   connector experiments.
2. Split the large `meshers.cpp` by responsibility without changing output:
   topology contract, sizing, count solver, primitive/template meshers,
   freeform/fallback, validation, and finalization.
3. Land R0 instrumentation and the one-ring visual capture harness.
4. Implement R1 canonical edge ownership and boundary-exact safety floor.
5. Implement R2 sizing field and revolution-group count/phase solve.
6. Implement R3/R4 constraints and templates one reproduced defect at a time.
7. Start R6 cache/GPU proxy work in parallel once invalidation boundaries are
   explicit.
8. Treat R5 global field methods as a measured research track, not as the next
   wholesale rewrite.

## 9. Explicit non-goals

- Do not replace the legacy primitive meshers with a generic auto-retopology
  pass.
- Do not make an ML mesher authoritative; current methods do not provide the
  determinism or B-rep topological guarantees Weft requires.
- Do not put exact B-rep topology entirely on the GPU. GPU preview and parallel
  evaluation are valuable; kernel topology, robust predicates, and export
  validation remain CPU responsibilities.
- Do not solve the whole MP9 constraint graph for a local edit.
- Do not use aggressive weld/conform passes to disguise incompatible boundary
  generation.
- Do not accept a numeric-only improvement without the full-model and one-ring
  visual review.
