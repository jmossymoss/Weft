# ADR-0017: Secure CLI utility routing

- Status: Accepted
- Date: 2026-07-17
- Owners: Weft geometry core and CLI

## Context

After `mesh` and the desktop app moved to the certified pipeline, three less
obvious command routes could still generate historical meshes:

- STEP-to-mesh `convert` used default OCCT tessellation;
- `sweep` invoked the old mesher repeatedly with per-face transient indices;
- `cache-check` invoked the old incremental cache and applied a per-face edit.

Leaving those routes active would let automation or users produce geometry
that bypassed the secure result contract. At the same time, exact B-rep and
mesh representation conversion should remain available when it does not
generate a mesh from a B-rep.

## Decision

- STEP-to-mesh `convert` delegates to `cmdMesh`, and therefore audited import,
  certified generation, named refusal, and existing format writers.
- Non-STEP B-rep-to-mesh conversion refuses until that reader has an equivalent
  secure import proof.
- B-rep-to-B-rep and mesh-to-mesh representation conversion remain direct IO
  operations; they do not claim secure remeshing.
- `sweep` now varies the global canonical-boundary radial minimum. Every value
  generates twice, requires complete certificates, compares topology
  fingerprints, adapts the result losslessly, and runs the independent
  workflow watertightness check.
- `--profile` remains accepted as visibly ignored migration input for sweep.
- `cache-check` validates its command shape and then refuses with
  `secure_cache.incremental_dependency_unimplemented`. It cannot call the
  historical cache while recipe-v2 entity references and secure dependency
  closure are absent.
- `inspect` and `extract` import the processing-disabled source model. The
  historical `--compiler-face` diagnostic refuses instead of planning through
  the experimental compiler.

## Invariants

- every reachable CLI B-rep-to-mesh operation uses `MeshingResult`;
- no unsupported format or surface can reach OCCT triangle soup as a fallback;
- a failed conversion emits no mesh file;
- sweep success means complete and repeatable certified topology, not only
  matching polygon counts;
- cache benchmarking cannot imply secure incremental support before it exists;
- extraction selects immutable source faces, not transient healed indices.

## Alternatives rejected

- documenting `convert` as an intentionally uncertified shortcut;
- retaining the old per-face sweep because it covered more families;
- running the legacy cache only in a diagnostic command;
- interpreting recipe-v1 face IDs as working-shape stable IDs;
- silently accepting IGES/BREP secure meshing without importer evidence.

## Verification

Strict and static-analysis complete-product builds pass.

Executable proof covers:

- box STEP through `convert` to a 684-byte STL with 8 certified vertices and 12
  certified triangles;
- cylinder global sweep at minima 8, 16, and 32, with two deterministic runs
  per value and zero failures;
- stable named cache refusal with exit status 1;
- sphere STEP-to-OBJ conversion refusal with no output;
- source-backed box inspection; and
- a five-face source extraction produced from face 1 plus one adjacency ring.

## Consequences

All reachable product mesh-generation routes are secure-only or explicitly
refused. Historical CLI benchmark bodies and core generators remain compiled
solely for frozen pre-rewrite evidence; their removal is still M0 cleanup.
Secure incremental caching, recipe-v2 per-face density sweeps, and secure
non-STEP B-rep imports remain open.
