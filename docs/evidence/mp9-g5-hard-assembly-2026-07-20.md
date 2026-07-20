# G5: remove certified soft assembly — 2026-07-20

## Package

WP-177 track G5 (assembly / certificate completeness). Parallel to G1 digon.

## Goal

- No omit-driven blanket `relaxGeometryChecks` before `assembleCertifiedBoundaryMesh`
- Restore closed-manifold checks for owned solids
- Interval consumption proofs on for all face paths (including `--allow-partial-body`)
- No post-assemble coverage zeroing / expected-align soft completion
- Unit adversaries still refuse (orientation / position tamper)

## Changes

- `core/src/secure_meshing.cpp`
  - Closed-manifold: `true` when `solidCount >= 1` and not single-face /
    `--allow-partial-body`; open only for MAP-C single-face extracts and
    partial-body omit. Removed industrial face-count blanket disable.
  - Deleted post-assemble coverage zeroing loop.
  - Always run `certifySolvedIntervalConsumption` (omit no longer skips proofs).
  - Mapped refuse → UV-trim records refuse as checked `(1,1,0,0)` so the
    certificate is not poisoned without zeroing.
- `tests/test_secure_meshing.cpp`
  - `testG5HardCertifiedAssembly` (box closed incidence/Euler; omit still
    consumes intervals).
  - M3 report digests refreshed for closed-manifold coverage; hole
    boundary/lift digests track denser G1 digon/hole sampling already in tree.
  - `testApexCone` names `import_not_meshable` on OCCT 8.x host debt (unchanged
    root cause from G0).

`certified_mesh.cpp` not edited (G4 sphere Reversed seed-flip preserved).

## Proof (vs2022 Release)

```
weft_certified_mesh_tests.exe
certified planar mesh assembly checks passed
# includes testTamperedCanonicalPositionRefuses /
# testTamperedTriangleRefuses (orientation/position)

weft_secure_meshing_tests.exe
WEFT_G5 box closed-manifold tris=12
WEFT_M3_DIGEST fixture=box ... report=dde25bfd7f60eb38
secure end-to-end meshing checks passed
```

## Hardened vs body green

| Item | Status |
|---|---|
| Omit blanket relax before assemble | Hardened (absent; G0 removed, G5 keeps) |
| Closed-manifold per solid | Hardened for unit closed solids |
| Interval consumption always on | Hardened |
| Coverage zeroing at assemble | Removed |
| Orientation/position tamper refuse | Still refuse (certified_mesh tests) |
| slotted/drilled cutouts | Named refuse `certified.incidence_boundary_unexpected` (gaps exposed) |
| Full MP9 body EXIT 0 certificate | Still needs G1 digon 2732 (and related plane work) |

## Next

G1 plane digon / ear-stall blockers for MP9 body green under
`omitDeferredResiduals=false`.
