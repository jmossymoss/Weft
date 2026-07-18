# M4/M6 partial cylinder evidence - 2026-07-18

## Proven increment

`buildFullCylinderWall` now accepts both full-periodic cylinders
(`FullPeriodicWithCapBoundaries`) and open cylindrical bands
(`PeriodicBandCrossingSeam`). Partial bands require exactly two open circular
rim arcs and two linear side rails, emit non-wrapping column spans, and still
consume every face/coedge sample use. Secure meshing routes both trim classes,
equalizes open-arc interval counts, and certifies the `partial_cylinder`
fixture end to end without welding.

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
- New fixture `partial_cylinder` (r=10, h=30, 270°) certifies through
  `generateSecureMesh`.
- Unequal open-arc exact edge overrides refuse before generation.
- Full-cylinder regression paths remain green.

## Status boundary

WP-022 is closed on the Linux lanes for the isolated/capped partial solid
fixture family used here. Seam-crossing and reversed-frame partial variants
remain covered by later packages (WP-023+). M4/M6 stay `IN_PROGRESS`.
