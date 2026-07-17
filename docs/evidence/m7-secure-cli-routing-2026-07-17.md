# M7 secure CLI routing evidence - 2026-07-17

## Proven increment

The `weft mesh` command now has one generator: `generateSecureMesh`. Successful
certified triangles are copied verbatim into the existing export adapter; no
weld, fallback mesher, OCCT triangle soup, face omission, or downstream
retriangulation is available in this route.

## Build and automated verification

Commands run from `D:\Weft`:

```text
cmake --build --preset vs2022 --config Release --parallel
cmake --build --preset vs2022-static-analysis --config Release --parallel
ctest --test-dir build/vs2022 -C Release --output-on-failure -E "^pipeline$"
ctest --test-dir build/vs2022-static-analysis -C Release --output-on-failure -E "^pipeline$"
```

Both builds pass. All 13 secure/corpus tests pass in each lane. The secure
orchestration test also proves that the compatibility adapter preserves vertex
and triangle counts, corner UV anchors, and exact triangle index triples.

## Executable workflow proof

Fresh STEP fixtures were generated and exported through the Release CLI:

| Input | Output | Secure result | Existing workflow validation |
|---|---|---|---|
| box | OBJ | 8 vertices, 12 triangles | watertight, consistent winding, zero degenerates |
| capped cylinder | GLB | 48 vertices, 92 triangles | watertight, consistent winding, zero degenerates |
| connected through-hole | FBX | 48 vertices, 96 triangles | watertight, consistent winding, zero degenerates |

Every `--mesh-report` begins with `weft-secure-mesh-report 1`, records identical
source/working hashes for the conservative identity import, records
`repair-identity 1`, records `complete 1`, and includes the complete repair,
reconnaissance, interval, boundary, face, and body validation coverage.

`--pipeline legacy` prints a migration warning and cannot select the legacy
generator.

## Refusal proof

Three adversarial CLI calls exited with status 1 and left no output:

- sphere: `secure_pipeline.unsupported_curve_family`;
- cylinder with `--axial 2`:
  `cylinder.axial_samples_require_interior_provenance`;
- legacy edge override: `secure recipe/manual/per-face migration conflict`.

## Status boundary

This starts M7; it does not complete it. App generation, Blender live link,
recipe v2 migration, manual operations, hot reload, `convert`, `cache-check`,
LOD report naming, and independently proven modelling quads remain open. The
historical pipeline test continues to represent the frozen M0 baseline and is
not used to justify secure-core correctness.
