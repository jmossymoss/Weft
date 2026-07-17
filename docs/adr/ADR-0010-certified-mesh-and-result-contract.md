# ADR-0010: Certified mesh and meshing-result contract

- Status: Accepted
- Date: 2026-07-17
- Owners: Weft geometry core

## Context

`PolyMesh` is an application/export structure whose historical generation path
welds spatially close vertices and permits mixed legacy strategies. It cannot be
the authoritative representation for the secure rewrite: a weld can hide a
boundary-identity error, vertex-level face anchors cannot represent both sides
of a seam, and downstream triangulation can choose inconsistent diagonals.

The planar reference CDT now emits per-face certified triangles keyed to
canonical boundary identities. A body-level contract must assemble those faces
without proximity operations, prove that the requested face set is complete,
carry source/working provenance, and keep certified triangles separate from
later modelling topology.

## Decision

Introduce the public contracts from the rewrite plan:

- `CertifiedMesh` contains globally indexed vertices and authoritative
  triangles;
- `ModelingMesh` contains modelling polygons and may explicitly alias the
  certified floor;
- `ValidationCertificate` contains non-vacuous named coverage records;
- `MeshingResult` packages certified, modelling, existing generation-report,
  and validation data.

`assembleCertifiedPlanarMesh` is the first body assembler. The caller supplies
both per-face CDT meshes and the complete expected working-face set. The
assembler:

- rejects missing, duplicate, extra, empty, or invalid face sets;
- resolves every trim boundary use back through canonical sample, source and
  working edge, coedge, face-side UV, and its measured/allowed discrepancy;
- creates one global vertex per zero-based canonical vertex index;
- requires every incident canonical sample for that vertex to have exactly the
  same 3D position;
- applies recorded B-rep face orientation to triangle winding;
- re-evaluates every triangle corner on the exact surface within the bounded
  source-tolerance envelope carried by its coedge uses;
- proves triangle area/orientation, edge incidence, and opposite winding;
- returns no mesh until all certificate coverage is complete.

The deterministic `topologyFingerprint` is a stable 64-bit FNV-1a evidence key
over ordered canonical indices, positions, face IDs, and triangle indices. It is
not a cryptographic provenance hash; source/working B-rep identity continues to
use SHA-256.

`makeCertifiedFloorMeshingResult` explicitly aliases certified triangles into
the modelling view with a visible safe-floor reason when no structured template
has been proven.

## Invariants

- authoritative assembly never performs a spatial search or weld;
- expected face coverage is explicit and cannot pass vacuously;
- a canonical index maps to exactly one 3D output vertex;
- every output vertex and triangle resolves to source and working provenance;
- all closed-manifold edges have incidence two and opposite traversal;
- face order supplied by a caller cannot change vertex order, triangle order,
  or topology fingerprint;
- a safe-floor modelling alias is explicit, valid, and never presented as a
  structured-template success.

## Alternatives rejected

- adapting `PolyMesh` into the authoritative core;
- welding face meshes after independent tessellation;
- accepting whatever face subset a caller happens to provide;
- using one vertex-level face UV for multi-face boundary vertices;
- trusting CDT output without independent body-level provenance and incidence
  validation;
- calling a non-cryptographic topology fingerprint a source hash.

## Verification

`weft_certified_mesh_tests` runs the complete secure plane path on a box. With
two intervals per B-rep edge it proves a deterministic closed mesh with 20
canonical vertices, 36 triangles, and 54 edges. All 54 edges have incidence two
and opposite winding. The eight corners each retain six face/coedge boundary
uses; the 12 edge-interior samples retain two.

Reversing both expected-face and face-mesh input order produces the same ordered
mesh and fingerprint. Missing, duplicate, and empty face sets refuse by name.
Changing one canonical sample position by `1e-9` refuses exact vertex identity;
reversing one certified triangle refuses oriented surface geometry. The
safe-floor `MeshingResult` aliases every certified triangle with an explicit
reason.

The target passes MSVC warnings-as-errors and `/analyze` builds and tests.

## Consequences

The secure core now has a non-welded authoritative result contract and one
closed-body proof. This does not route the existing app/CLI/export workflows,
support non-planar faces or holes, prove 3D triangle/triangle
self-intersection, or complete the independent M5 validator family. `PolyMesh`
remains a future application/export adapter only.
