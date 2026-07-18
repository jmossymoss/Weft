# M4/M5 body triangle intersection evidence - 2026-07-18

## Proven increment

Certified body assembly now refuses non-adjacent 3D triangle intersections
before a mesh can be certified or exported. Exact dyadic predicates gained
`orient3d` and `triangleIntersection3d`. `validateCertifiedTriangleIntersections`
accounts every unordered triangle pair with non-vacuous
`certified.triangle_intersection` coverage, allows contacts explained by shared
canonical vertex identity (shared edge / shared vertex), and refuses proper
crossings, coplanar area overlaps, duplicate triangles, and unexplained contact
by stable diagnostic codes.

`assembleCertifiedBoundaryMesh` integrates the check after edge incidence /
winding and before the topology fingerprint, so incomplete or failed coverage
cannot produce a `MeshingResult`.

## Environment

- Host: Cursor Cloud Linux
- Compiler: g++ 13.3.0
- OpenCASCADE: 7.6.3 (Debian/Ubuntu packages)
- Presets: `linux-gcc`, `linux-gcc-static-analysis`

OCCT 7.6 portability shims required for this lane (and included in the same
change set): `GetMessageString` failure text, `OcctShapeHash` for unordered
maps, guarded `SetParallel` / `ShapeProcess::OperationsFlags` /
`STEPCAFControl_Reader::ReadStream`, and SYSTEM include treatment for OCCT
headers under `-Werror`.

## Commands

```bash
cmake --preset linux-gcc
cmake --build --preset linux-gcc --target \
  weft_geometric_predicate_tests weft_certified_mesh_tests weft_secure_meshing_tests -j"$(nproc)"
ctest --preset linux-gcc -R 'geometric_predicates|certified_mesh|secure_meshing' \
  --output-on-failure

cmake --preset linux-gcc-static-analysis
cmake --build --preset linux-gcc-static-analysis --target \
  weft_geometric_predicate_tests weft_certified_mesh_tests weft_secure_meshing_tests -j"$(nproc)"
ctest --preset linux-gcc-static-analysis \
  -R 'geometric_predicates|certified_mesh|secure_meshing' --output-on-failure
```

## Outcomes

- `linux-gcc`: geometric_predicates, certified_mesh, and secure_meshing passed.
- `linux-gcc-static-analysis`: same three tests passed (analyzer warnings only in
  the existing non-fatal classes).
- Closed box: 630 = 36*35/2 intersection candidates checked, 0 failed.
- Synthetic adversaries refuse by name:
  - `certified.triangle_proper_intersection`
  - `certified.triangle_coplanar_overlap`
  - `certified.triangle_duplicate`
- Legal adjacency, shared-vertex, near-contact, and single-triangle vacuous
  domains pass.
- M3 report digests for box/cylinder/hole updated to include the new coverage
  record; count/boundary/lift digests unchanged.

## Status boundary

WP-020 is closed on the Linux lanes available in this environment. M4 and M5
remain `IN_PROGRESS` (axial interior provenance, incidence/Euler, modelling
provenance, and remaining curved families are still open). Windows MSVC
re-proof of the updated report digests is deferred to CI / a Windows host.
