# M7 secure recipe v2 evidence - 2026-07-17

## Proven increment

Recipe files now have a source-referenced v2 contract shared by the core, CLI,
and desktop app. The implementation migrates v1, resolves source references
through audited correspondence, captures current working edits back onto source
references, persists only v2, and separates reference validity from permission
to apply a still-unimplemented modelling operation.

## Build and automated verification

Commands run from `D:\Weft`:

```text
cmake --build --preset vs2022 --config Release --parallel 2
cmake --build --preset vs2022-static-analysis --config Release --parallel 2
ctest --test-dir build/vs2022 -C Release -E "^pipeline$" --output-on-failure
ctest --test-dir build/vs2022-static-analysis -C Release -E "^pipeline$" --output-on-failure
```

Both complete-product builds pass. All 14 secure/corpus tests pass in both
lanes. The `secure_recipe` battery specifically proves:

- deterministic source face and edge fingerprints;
- v1 face, edge, and surface-anchored operation migration;
- working-to-source capture through a deliberately non-equal ordinal map;
- v2 write/read/resolve round trips;
- classic-locale, 17-digit floating-point persistence without `%g` precision
  loss;
- unique relocation after a source-hash change;
- named missing, ambiguous/duplicate, malformed, non-bijective, and
  world-space-operation refusals; and
- explicit application conflicts for stored features not yet supported by the
  certified pipeline.

## Executable CLI proof

A fresh box STEP was used for three Release CLI routes.

1. A v1 recipe with compiler input and global `radial=18`, `axial=2`,
   `chord=0.05`, and `angle=12` migrated to v2. Loading the resulting v2
   produced identical files:

   - OBJ SHA-256:
     `CA1B39302CCE550ACADED94B50B8A66B04FD64DB012663624134B11B36847009`;
   - secure report SHA-256:
     `73B17AFD31F3C42170A2997C2E6BF586E8CD5ACE9F92BC4D40CE9BB1E4B58D91`.

2. Direct global CLI input wrote v2 without being mislabeled as v1 migration.

   Historical option ordering is preserved: `--radial 30` before `--recipe`
   saved the recipe's radial 18, while the same option after `--recipe` saved
   radial 30 after correspondence-aware recapture.

3. Direct `--face 1:radial=24` wrote an inspectable v2 record containing source
   face 1 and its full geometric fingerprint, then exited 1 with
   `secure_recipe.application.face_settings_unimplemented`. No OBJ was
   created.

## Executable app proof

The Release desktop app opened a fresh box STEP with a matching v2 sidecar. Its
screenshot harness displayed `recipe v2 ready`, the certified pipeline,
8 vertices, 12 triangles, and `watertight`. The app complete-product target
also passes the static-analysis build.

## Status boundary

This restores v2 persistence, v1 migration, safe global replay, and visible
conflict handling. It does not apply per-face or per-edge settings, manual
surface edits, or legacy modelling controls to secure topology. Those records
are preserved but block generation/export until their certified consumers are
implemented. M7 therefore remains `IN_PROGRESS`.
