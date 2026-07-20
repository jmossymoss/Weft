# G0 fail-closed product defaults — 2026-07-20

## Gate

Kill omit/skip as the product default:

- `omitDeferredResiduals` defaults to `false` in `SecureMeshingConfiguration`,
  app regenerate, and CLI mesh (unless `--allow-partial-body`).
- No `expectedFaces.pop_back()` success path for plane CDT failures.
- Skipping `DeferredResidualSurface` faces is gated behind
  `--allow-partial-body` only (sets `omitDeferredResiduals=true`).
- Failures must carry a stable code and face subject id.

## Changes

- `core/include/weft/secure_meshing.hpp` — document escape-hatch semantics.
- `cli/main.cpp` — add `--allow-partial-body`; default mesh path stays
  fail-closed.
- `app/main.cpp` — regenerate never enables omit (no UI toggle).
- `core/src/secure_meshing.cpp` — comments on ADR-0014 / face-loop omit gates.
- `tests/test_secure_meshing.cpp` — `testG0FailClosedDefaults`.

## Commands / results (Windows MSVC vs2022, OCCT 8.x)

```powershell
cmake --build --preset vs2022 --target weft weft_secure_meshing_tests
ctest --preset vs2022 -R "^secure_meshing$" --output-on-failure
build/vs2022/bin/Release/weft mesh <box.step> -o box.obj   # EXIT 0
build/vs2022/bin/Release/weft mesh --help | findstr allow-partial-body
```

- `testG0FailClosedDefaults`: PASS (`WEFT_G0 ribbon certified tris=4284`; default
  `omitDeferredResiduals=false` asserted).
- CLI product path: box mesh EXIT 0 without `--allow-partial-body`.
- `--allow-partial-body` appears in `weft mesh` usage.
- Full `secure_meshing` suite: FAIL on pre-existing `testApexCone`
  (`secure_pipeline.import_not_meshable` for fixture `cone` under local OCCT 8.x).
  Not introduced by G0 omit/default changes; tracked as host/OCCT debt, not a
  G0 acceptance regress.

## Remaining soft paths (not G0 face omission)

- Plane CDT curved-UV retry / `relaxGeometryChecks` (G1)
- Template-scoped `relaxGeometryChecks` (G2–G4)
- `previewFast` for large industrial bodies (perf; G0 allows import skips)

## Next

G1 plane multi-outer + hole_bridge; then self-intersect planes; then MP9 body
with `omitDeferredResiduals=false`.
