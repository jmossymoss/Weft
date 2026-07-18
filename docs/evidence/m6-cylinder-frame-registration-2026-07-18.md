# M6 cylinder frame registration evidence - 2026-07-18

## Proven increment

Azimuth registration now distinguishes incompatible frame classes by stable
diagnostic code:

- `boundary.azimuth_reflection` — opposite ring winding
- `boundary.azimuth_twist` — no cyclic rotation aligns the rings
- `boundary.azimuth_incompatible` — count/axis/reference failures

Compatible shifted-angle, reversed-axis, and translated-origin rings still
produce a pure cyclic column permutation. The cylinder template propagates the
named registration code unchanged.

## Environment

- Host: Cursor Cloud Linux
- Compiler: g++ 13.3.0
- OpenCASCADE: 7.6.3
- Presets: `linux-gcc`, `linux-gcc-static-analysis`

## Commands

```bash
cmake --preset linux-gcc
cmake --build --preset linux-gcc --target \
  weft_canonical_boundary_tests weft_cylinder_template_tests weft_secure_meshing_tests -j"$(nproc)"
ctest --preset linux-gcc -R 'canonical_boundary|cylinder_template|secure_meshing' \
  --output-on-failure

cmake --preset linux-gcc-static-analysis
cmake --build --preset linux-gcc-static-analysis --target \
  weft_canonical_boundary_tests weft_cylinder_template_tests weft_secure_meshing_tests -j"$(nproc)"
ctest --preset linux-gcc-static-analysis \
  -R 'canonical_boundary|cylinder_template|secure_meshing' --output-on-failure
```

## Outcomes

- Both Linux lanes: the three test targets passed.
- Compatible fixtures: identity, cyclic phase shift, reversed axis order, and
  translated ring origins register successfully.
- Incompatible fixtures refuse before wall assembly with the named codes above.
- Reflected cylinder-template adversary now expects
  `boundary.azimuth_reflection`.

## Status boundary

WP-023 is closed on the Linux lanes. M6 remains `IN_PROGRESS` until modelling
polygons and the full M6 fixture family gate pass.
