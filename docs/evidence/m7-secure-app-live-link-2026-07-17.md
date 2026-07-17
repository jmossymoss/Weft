# M7 secure app and Blender-link evidence - 2026-07-17

## Proven increment

STEP import, async viewport regeneration, export finalization, hot reload, and
the Blender live-link file now use the audited import plus
`generateSecureMesh`. App source contains no call to `loadStep`, `generate`,
`generatePrimitiveAware`, or `applyOps`.

## Build and regression proof

Commands run from `D:\Weft`:

```text
cmake --build --preset vs2022 --config Release --parallel
cmake --build --preset vs2022-static-analysis --config Release --parallel
ctest --test-dir build/vs2022 -C Release --output-on-failure -E "^pipeline$"
ctest --test-dir build/vs2022-static-analysis -C Release --output-on-failure -E "^pipeline$"
```

Both complete-product builds pass. All 13 secure/corpus tests pass in each
lane.

## Executable app proof

The Release desktop executable ran its screenshot harness on fresh generated
fixtures:

- box: visible certified triangulation, 8 vertices, 12 triangles, and the
  workflow validator reports watertight;
- full cylinder: visible certified curved wall/caps and watertight status;
- sphere: empty viewport and visible named refusal
  `secure_pipeline.unsupported_curve_family`.

The app UI visibly identifies the `secure certified pipeline`, removes the
legacy selector, exposes the repair profile/certificate, and labels selection
as inspect-only during recipe migration.

The cylinder was launched twice with the same `--live-link` destination. The
second run replaced the existing OBJ atomically on Windows. Both runs produced
the identical 2,359-byte file with SHA-256
`41FD38903B2353C11540EEADBDED0CE8DB1B93716EA9E89DF53B956887677EBD`.

## Status boundary

This is secure workflow routing, not recipe restoration. Recipe v1 safe global
density values can migrate, but per-face/per-edge/manual data reports a
conflict and recipe saving is blocked until recipe v2 exists. Editing, source
and working overlays, validation coverage overlays, incremental secure cache,
GPU proxy performance, and remaining surface families are open.
