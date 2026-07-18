# M2 total reconnaissance evidence - 2026-07-17

## Gate

Every imported working edge and face is classified/accounted exactly once;
unsupported exact geometry is named and never folded into a supported family.

## Proven increment

Secure reconnaissance now closes the remaining M2 ledger items:

1. Schema-aligned trim taxonomy (`trimDomainClassName` matches
   `trim_domain_class` in the fixture expectation schema), including open,
   self-intersecting, ambiguous nesting, annulus, simple disk, concave planar,
   periodic full/band, and singularity touches.
2. Coplanar artificial-split faces keep distinct face subjects but merge into
   one logical region (`region.plane.artificial_split_merged` with
   `reason.artificial_split_merge_accounted`).
3. Coplanar same-support faces without a shared manifold edge are named
   `multiple_disconnected_domains` with
   `reason.multidomain_decomposition_unproven` rather than a false single-face
   trim claim.
4. Adversarial `GeomAbs_OtherCurve` / `GeomAbs_OtherSurface` probes stay
   `kernel_specific` / `unrecognised_exact_geometry` /
   `reason.unrecognised_exact_geometry`.
5. Oracle slice: reviewed corrupt trim fixtures and the processing-disabled
   cylinder match their expected family/trim classes.

Prior corpus accounting remains: exact occurrence hierarchy, source-backed
family/wrapper/domain records, and complete reconnaissance over all 77
generated sources (1,559 subjects).

## Tests

`weft_secure_core_tests` covers:

- `testTotalReconnaissance`
- `testUnknownExactFamilyInjection`
- `testOracleTrimTaxonomySlice`
- `testProceduralTrimTaxonomyBattery`
- `testCoplanarArtificialSplitRegionMerge`
- `testMultidomainSameSupportDeferred`
- `testM2OracleFamilyAndCylinderTrim`

## Verification

```text
cmake --build build\vs2022 --config Release --target weft_secure_core_tests
ctest --test-dir build\vs2022 -C Release -R "secure_core|secure_committed_step_corpus|secure_generated_step_corpus|secure_fixture_catalog" --output-on-failure
```

## Status boundary

This passes M2. Interval assignment, canonical boundaries, CDT, and template
consumers remain M3+.
