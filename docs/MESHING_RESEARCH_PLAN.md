# Meshing research reference

This file keeps only research that can guide future secure-core work. It is not
a status ledger or execution plan:

- status: `docs/governance/milestones.md`;
- execution: `docs/governance/agent-execution-playbook.md`;
- architecture: `docs/SECURE_CORE_REWRITE_PLAN.md`.

Do not add implementation diaries or legacy rollout instructions here.

## Research conclusions

1. B-rep topology and source identity remain invariant.
2. Each topological edge owns one ordered sample sequence, consumed by every
   coedge use through a proved parameter/UV mapping.
3. A smooth size field proposes density; an exact bounded integer solve makes
   counts compatible.
4. Analytic surfaces use deterministic templates.
5. Every supported face has a boundary-exact, intersection-checked certified
   triangle floor.
6. Quad/n-gon modelling topology is independently certified above that floor.
7. Unsupported geometry fails by stable diagnostic code. OCCT triangle soup,
   welding, silent healing, or dropped faces are never correctness evidence.

No single published quadrangulation method covers the full product. Weft needs
a topology-first hybrid of exact boundary ownership, certified tessellation,
analytic templates, constrained count solving, and later freeform methods.

## Sources and intended use

### Implement or adapt

- Li et al., “Robust tessellation of CAD models without
  self-intersections” (JCDE 2026),
  <https://doi.org/10.1093/jcde/qwaf134>
  - shared 3D edge sampling, per-face parameter mapping, critical trim
    subdivision, boundary-exact CDT, and intersection safety.
- Yang et al., “Boundary constrained quadrilateral mesh generation based on
  domain decomposition and templates” (2024),
  <https://doi.org/10.1016/j.compstruc.2024.107275>
  - boundary-first decomposition, count reconciliation, and explicit
    triangle/quad/pentagon transition templates.
- Couplet, Reberol and Remacle, “Generation of High-Order Coarse Quad Meshes on
  CAD Models via Integer Linear Programming” (2021),
  <https://doi.org/10.2514/6.2021-2991>
  - later coarse-layout simplification while preserving CAD features.
- Bawin, Henrotte and Remacle, “Automatic feature-preserving size field for
  three-dimensional mesh generation” (2021),
  <https://doi.org/10.1002/nme.6747>
  - curvature/local-feature sizing with bounded spatial gradients.
- Couplet, Chemin and Remacle, “Size-controlled quadrilateral meshing using
  integrable odeco fields” (CAD 2026),
  <https://doi.org/10.1016/j.cad.2025.103974>
  - offline prototype for size transitions and singularity placement on
    suitable freeform regions.
- Capouellez et al., “Feature-Aligned Parametrization in Penner Coordinates”
  (SIGGRAPH 2025), <https://doi.org/10.1145/3731216>
  - freeform feature alignment candidate.
- Corman and Crane, “Rectangular Surface Parameterization” (SIGGRAPH 2025),
  <https://doi.org/10.1145/3731176>
  - candidate for ribbon-like and long thin freeform patches.

### Foundation or evaluation only

- Zhou et al., “Topology-First B-Rep Meshing” (2026 preprint),
  <https://arxiv.org/abs/2604.02141>
  - adopt the topology-first principle; reproduce algorithmic claims before
    introducing an implementation dependency.
- Couplet et al., “Surface Quadrilateral Meshing from Integrable Odeco Fields”
  (2026 preprint), <https://arxiv.org/abs/2604.03889>
  - evaluate only after peer review/code availability and against the frozen
    corpus.
- Bommes, Zimmer and Kobbelt, “Mixed-Integer Quadrangulation” (2009),
  <https://doi.org/10.1145/1531326.1531383>
  - mathematical foundation for cross fields, seamless parameterization, and
    singularity vocabulary.
- Jakob et al., “Instant Field-Aligned Meshes” (2015),
  <https://igl.ethz.ch/projects/instant-meshes/>, and Huang et al.,
  “QuadriFlow” (2018),
  <https://stanford.edu/~jingweih/papers/quadriflow/>
  - proposal/baseline generators only; output must be re-constrained to B-rep
    boundaries and cannot become authoritative directly.
- Ng and Tan, “Incremental tessellation of trimmed parametric surfaces”
  (2000), <https://doi.org/10.1016/S0010-4485(99)00089-5>
  - dependency closure and local invalidation principles for M7 caching.

## M8 research sequence

Research enters production one geometry family at a time through the M8 family
packet in the execution playbook:

1. analytic cones, spheres, and tori;
2. fillet/blend strips and analytic connectors;
3. mapped four-sided blocks and corridor/collar transition templates;
4. cutout and cut-graph templates;
5. freeform parameterization/field methods.

For each family, compare candidate methods only after canonical boundaries,
count constraints, and certified-floor requirements are fixed. Prototype code
stays outside production routing until isolated, connected, adversarial,
density-sweep, corpus, and cross-platform gates pass.

## Acceptance measurements

Every family reports:

- topology: open/non-manifold edges, T-junctions, source incidence, connected
  components, Euler delta, and shared-edge sample identity;
- geometry: chord/surface/normal error, foldovers, intersections, trim
  containment, and thin-feature clearance;
- modelling topology: polygon ratios by region, aspect/skew/angle quality,
  feature alignment, singularities, and strip/column continuity;
- stability: deterministic topology/report digests, density-sweep behavior,
  local-edit topology churn, timings, and memory;
- provenance: expected/checked/skipped/failed counts for every required source,
  working, floor, and modelling relationship.

Numeric metrics do not replace visual review. The Plasticity source and Blender
result must be compared using matching solid/wireframe views, changed faces
with incident neighbours visible, and a written verdict for silhouette,
cylinders, segment spans, welding, and modelling usability.

## Dependency policy

Research implementations may use prototype-only dependencies behind narrow
interfaces. They may not enter distributable binaries until licensing is
resolved under M9 and `docs/governance/blocked-routes.md`. An ML mesher may be a
proposal tool but cannot be authoritative because the product requires
deterministic B-rep topology, provenance, and fail-closed validation.
