# M6 secure orchestration evidence - 2026-07-17

## Proven increment

`generateSecureMesh` atomically composes audited import evidence, total
reconnaissance, analytic count solving, canonical boundaries, planar/hole CDT,
full-cylinder certification, and no-weld body assembly. It returns one
`MeshingResult` only when the combined validation certificate is complete.

## Verification

Commands run from `D:\Weft`:

```text
cmake --build --preset vs2022 --config Release
cmake --build --preset vs2022-static-analysis --config Release
ctest --preset vs2022 -E "^pipeline$" --output-on-failure
ctest --preset vs2022-static-analysis -E "^pipeline$" --output-on-failure
```

Both build lanes complete and all 13 secure/corpus tests pass in each lane.
End-to-end fixtures cover:

- a planar box: 8 vertices and 12 certified triangles;
- a full capped cylinder with deterministic repeated topology;
- a connected through-hole body combining perforated planes and a cylindrical
  bore;
- a named unsupported sphere refusal;
- invalid sampling configuration refusal with no substituted defaults.

Every successful result carries complete repair, reconnaissance, interval,
boundary, vertex/curve identity, trim, CDT or cylinder, incidence, winding, and
fingerprint coverage.

## Status boundary

This is not yet product routing. Legacy CLI/app generation remains disconnected
from this API, modelling output still visibly aliases certified triangles, and
recipes, exports, Blender, partial cylinders, axial interior rings, Linux
determinism, and the remaining M4/M5 validators remain open.
