# M4 certified planar box evidence - 2026-07-17

## Proven increment

The secure core now assembles independently certified planar face meshes into a
global `CertifiedMesh` by canonical vertex identity, never spatial proximity.
It also exposes `ModelingMesh`, `ValidationCertificate`, and `MeshingResult`,
including an explicit certified-floor modelling alias.

Canonical provenance carries sample, source/working edge, coedge, face-side UV,
and measured/allowed source-tolerance discrepancy. Body assembly independently
resolves those records, re-evaluates exact planar surfaces, applies B-rep face
orientation, and checks global incidence and winding.

## Verification

Commands run from `D:\Weft`:

```text
cmake --build --preset vs2022 --target weft_certified_mesh_tests --config Release
ctest --preset vs2022 -R certified_mesh --output-on-failure
cmake --build --preset vs2022-static-analysis --target weft_certified_mesh_tests --config Release
ctest --preset vs2022-static-analysis -R certified_mesh --output-on-failure
```

Both lanes passed. The closed box proof has:

- six expected and six checked planar faces;
- 20 global canonical vertices: eight corners with six boundary-use records
  each and 12 edge-interior vertices with two records each;
- 36 positive, correctly face-oriented triangles;
- 54 checked global edges, all with incidence two and opposite winding;
- complete boundary-provenance, vertex-identity, triangle, surface, incidence,
  winding, and fingerprint coverage;
- identical topology fingerprint after reversing all caller face ordering;
- named refusal for missing, duplicate, and empty face sets, a `1e-9` canonical
  position mismatch, and a reversed triangle;
- a modelling mesh that explicitly aliases the certified floor with a visible
  reason.

Both complete MSVC build presets then passed. Strict and static-analysis CTest
trees were run concurrently across all eleven secure/corpus tests; both passed
all eleven, including the frozen 106-record catalogue and 77 generated imports.

## Status boundary

M4 remains in progress. This proves one closed all-planar body and no-weld
global identity. Hole-capable CDT, cylinders and other curved surfaces,
adaptive surface-patch/chord/normal bounds, 3D triangle self-intersection,
critical-region BVH protection, open/source-defect accounting, Linux
determinism, corpus coverage, and production routing remain open. M5 remains
open beyond the certificate checks implemented here.
