# M5 incidence and Euler validation evidence - 2026-07-18

## Proven increment

Certified assembly now accounts mesh incidence and Euler characteristic under
`certified.incidence_euler`:

- Closed bodies: no boundary edges, manifold edge incidence 2, triangle
  identity `2E = 3F`, and an Euler total consistent with closed orientable
  components (`χ ≤ 2C` with even genus delta). Genus is not forced to zero, so
  through-hole solids (`χ = 0`) certify.
- Open / source-defect assemblies (`requireClosedManifold = false`): non-vacuous
  V/E accounting with explicit open policy; boundary edges are expected.

Standalone `validateCertifiedIncidenceEuler` covers tetrahedron, open disk,
closed-policy refusal of an open disk, and vacuous empty meshes.

## Environment

- Host: Cursor Cloud Linux
- Compiler: g++ 13.3.0 / OCCT 7.6.3
- Presets: `linux-gcc`, `linux-gcc-static-analysis`

## Commands

```bash
cmake --preset linux-gcc
cmake --build --preset linux-gcc --target \
  weft_certified_mesh_tests weft_secure_meshing_tests -j"$(nproc)"
ctest --preset linux-gcc -R 'certified_mesh|secure_meshing' --output-on-failure

cmake --preset linux-gcc-static-analysis
cmake --build --preset linux-gcc-static-analysis --target \
  weft_certified_mesh_tests weft_secure_meshing_tests -j"$(nproc)"
ctest --preset linux-gcc-static-analysis -R 'certified_mesh|secure_meshing' \
  --output-on-failure
```

## Outcomes

- Both lanes passed.
- Box χ=2, hole χ=0, open perforated face under open policy, and synthetic
  adversaries behave as specified.
- M3 report digests updated for the new coverage record; count/boundary/lift
  digests unchanged.

## Status boundary

WP-030 is closed on Linux. M5 remains `IN_PROGRESS` (modelling provenance and
remaining vacuity gates are still open).
