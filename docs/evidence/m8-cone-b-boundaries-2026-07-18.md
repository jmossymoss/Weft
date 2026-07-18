# M8 CONE-B canonical boundaries evidence - 2026-07-18

## Proven increment

Cone edges now receive certified critical segmentation and singular apex
canonical stations without promoting the cone face to a meshing template:

- Degenerate apex circles are assigned interval count `1` in
  `generateSecureMesh` and build a one-sample singular station
  (`event.cone_apex`) with stored coedge UV uses.
- Cone faces with `touches_one_singularity` no longer blanket-refuse critical
  segmentation; generators emit `event.cone_apex` at the apex endpoint.
- Base circle and seam line keep ordinary line/circle critical events.
- Adversary: degenerate apex with interval count ≠ 1 refuses
  `boundary.degenerate_interval_count_invalid`.
- Sphere remains a named refusal before generation
  (`boundary.critical_segmentation_unsupported` or a later
  `secure_pipeline.*` code).

Cone meshing is still deferred (`secure_pipeline.unsupported_surface_family`)
until CONE-C adds the certified wall template.

## Environment

- Host: Cursor Cloud Linux / g++ 13.3 / OCCT 7.6.3
- Presets: `linux-gcc`, `linux-gcc-static-analysis`

## Commands

```bash
cmake --build --preset linux-gcc --target \
  weft_canonical_boundary_tests weft_secure_meshing_tests weft_secure_core_tests \
  -j"$(nproc)"
ctest --preset linux-gcc -R 'canonical_boundary|secure_meshing|secure_core' \
  --output-on-failure
cmake --build --preset linux-gcc-static-analysis --target \
  weft_canonical_boundary_tests weft_secure_meshing_tests weft_secure_core_tests \
  -j"$(nproc)"
ctest --preset linux-gcc-static-analysis \
  -R 'canonical_boundary|secure_meshing|secure_core' --output-on-failure
```

## Outcomes

- Both Linux lanes: the three tests passed.
- `WEFT_CONE_B degenerate_apex samples=1 uv_uses=1`
- `WEFT_CONE_B adversary=boundary.degenerate_interval_count_invalid`
- `WEFT_CONE_A mesh_refusal=secure_pipeline.unsupported_surface_family`
  (progresses past the prior `circle_domain_invalid` refusal)

## Status boundary

WP-071 (CONE-B) is closed. CONE-C must implement `cone_template` and promote
cone to `SupportedAnalyticTemplate` with a live `generateSecureMesh` consumer.
