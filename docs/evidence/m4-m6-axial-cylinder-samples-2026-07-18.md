# M4/M6 axial cylinder interior samples evidence - 2026-07-18

## Proven increment

Full periodic cylinders may now carry certified interior axial rings when
`cylinderAxialIntervals > 1`. Each generated station records a
`CylinderInteriorStation` (face, UV, position, ring, column). Seam samples at
the same station attach as ordinary boundary uses. Body assembly accepts
interior stations, reconciles seam-backed positions through boundary
provenance, and still refuses leftover axial samples when interior provenance
is not requested (`axialIntervals == 1`).

## Environment

- Host: Cursor Cloud Linux
- Compiler: g++ 13.3.0
- OpenCASCADE: 7.6.3
- Presets: `linux-gcc`, `linux-gcc-static-analysis`

## Commands

```bash
cmake --preset linux-gcc
cmake --build --preset linux-gcc --target \
  weft_cylinder_template_tests weft_certified_mesh_tests weft_secure_meshing_tests -j"$(nproc)"
ctest --preset linux-gcc -R 'cylinder_template|certified_mesh|secure_meshing' \
  --output-on-failure

cmake --preset linux-gcc-static-analysis
cmake --build --preset linux-gcc-static-analysis --target \
  weft_cylinder_template_tests weft_certified_mesh_tests weft_secure_meshing_tests -j"$(nproc)"
ctest --preset linux-gcc-static-analysis \
  -R 'cylinder_template|certified_mesh|secure_meshing' --output-on-failure
```

## Outcomes

- Both Linux lanes: `cylinder_template`, `certified_mesh`, and `secure_meshing`
  passed.
- `axialIntervals == 1` with extra seam samples still refuses
  `cylinder.axial_samples_require_interior_provenance`.
- `axialIntervals == 2` certifies a 3-ring wall (64×3 vertices, 64×4 triangles
  at the template fixture density) with interior stations at ring 1.
- Secure pipeline with `cylinderAxialIntervals = 2` produces a complete
  validation certificate including interior-station vertices.

## Status boundary

WP-021 is closed on the Linux lanes. M4/M6 remain `IN_PROGRESS` (partial
cylinders, mismatched frames, modelling polygons, and remaining validators are
still open). Windows re-proof is deferred to CI / a Windows host.
