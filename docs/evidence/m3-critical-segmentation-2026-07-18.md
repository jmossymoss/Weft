# M3 critical parameter segmentation evidence - 2026-07-18

## Proven increment

Canonical boundaries for exact line/circle edges on plane/cylinder faces now
collect deterministic critical parameter events before sampling:

- domain endpoints;
- contact-critical vertex incidences;
- periodic seams (curve and supported surface-period images);
- circle quarter-turn monotone splits relative to the trimmed edge domain.

Events merge into the edge-owned sample parameter sequence without double-
counting. Coverage reports `expectedCriticalEvents` / `checkedCriticalEvents`
and cannot pass vacuously. Unsupported families, including singular trim
domains and non line/circle/plane/cylinder subjects, refuse with
`boundary.critical_segmentation_unsupported` before CDT or cylinder templates
run. Azimuth registration accepts matching cyclic rings after critical merges
while still refusing reflected or mismatched non-uniform pairs.

`secure_pipeline.critical_parameter_events` coverage is aggregated into
`MeshingResult` validation.

## Verification

Commands from `D:\Weft` after this increment:

```text
cmake --build --preset vs2022 --target weft_canonical_boundary_tests weft_secure_meshing_tests weft_planar_trim_validation_tests
ctest --preset vs2022 -R "canonical_boundary|planar_trim_validation|secure_meshing" --output-on-failure
```

All three tests passed.

Fixtures and adversaries:

- planar box and capped cylinder: non-zero critical-event coverage with every
  event landed on a sample ordinal; repeated cylinder builds reproduce event
  digests field by field;
- sphere: named `boundary.critical_segmentation_unsupported` (or an earlier
  unsupported-family refusal) before template generation;
- azimuth suite: identity and cyclic phase still succeed; reflection and
  one-sided jitter still refuse.

## Status boundary

WP-012 is closed. M3 remains open for template sum consumers, coupled sums,
and Linux boundary determinism.
