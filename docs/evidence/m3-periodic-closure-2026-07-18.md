# M3 periodic UV closure evidence - 2026-07-18

## Proven increment

Closed working edges with periodic surface axes now require an unambiguous
covering-space return from the last lifted sample to the first. Each proved
closure records source/working edge and face identity, coedge orientation, raw
and lifted UV endpoints, selected integer lifts, closing lifted UV, and
`periodsCrossed`.

Consecutive sample lifts refuse half-period-ambiguous and discontinuous jumps
(`boundary.periodic_lift_ambiguous`, `boundary.periodic_lift_discontinuous`).
Closure refuses ambiguous, discontinuous, overflow, and inconsistent recorded
lift witnesses. Open edges, including trimmed periodic supporting curves, still
emit zero closure expectations.

`secure_pipeline.periodic_uv_closure` coverage is aggregated into
`MeshingResult` validation.

## Verification

Commands from `D:\Weft` at revision with this increment:

```text
cmake --build --preset vs2022 --target weft_canonical_boundary_tests weft_secure_meshing_tests weft_cylinder_template_tests
ctest --preset vs2022 -R "canonical_boundary|secure_meshing|cylinder_template" --output-on-failure
```

All three tests passed.

Fixtures and adversaries:

- planar box: zero periodic closures;
- capped cylinder: non-zero closures with both forward and reversed coedges and
  at least one `|periodsCrossed| == 1` full-period witness; repeated builds
  reproduce identical closure digests;
- quarter-circle open arc: remains open with empty closure list;
- synthetic closure seeds refuse ambiguous half-period branch cuts, wrong
  recorded lifts, and mismatched first-lift witnesses; near-full continuous
  wraps still prove `periodsCrossed == 1`.

## Status boundary

WP-010 is closed. M3 remains open for repeated wire occurrences, critical
segmentation, template sum consumers, coupled sums, and Linux boundary
determinism.
