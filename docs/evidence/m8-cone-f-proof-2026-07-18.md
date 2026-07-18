# M8 CONE-F family proof evidence - 2026-07-18

## Proven increment

The apex-cone family completes the M8 family packet proof matrix:

| Proof | Result |
|---|---|
| Isolated capped apex cone | Certified (`tris=38`, `fingerprint=4206cd65287f33d3`) |
| Connected plane+cone | Same fixture (planar base + cone wall) |
| Adversarial | Tight chord → `interval.count_exceeds_maximum`; tampered modelling refuses |
| Density sweep | loose 38 tris → dense 62 tris; fingerprints differ; repeatable loose fingerprint |
| Corpus impact | `fixture:cone` terminal=`certified`; totals `mesh_certified=6`, `fixture_extra=6` |
| Linux determinism | Golden fingerprint `4206cd65287f33d3` locked in `test_cone_template` |
| Product | CLI/app evidence from CONE-E |

```text
WEFT_CONE_F density loose_tris=38 dense_tris=62 fingerprint=4206cd65287f33d3
COVERAGE subject=fixture:cone terminal=certified fingerprint=4206cd65287f33d3
COVERAGE_TOTALS import_success=12 import_refusal=3 mesh_certified=6
mesh_named_refusal=5 mesh_inspectable_only=1 case_total=9 fixture_extra=6
```

Cone is now a supported automatic family alongside plane and cylinder.
Sphere/torus remain deferred named refusals. M8 stays `IN_PROGRESS`.

## Commands

```bash
ctest --preset linux-gcc -R 'cone_template|secure_committed_step_corpus' \
  --output-on-failure
ctest --preset linux-gcc-static-analysis \
  -R 'cone_template|secure_committed_step_corpus' --output-on-failure
```

## Outcomes

Both Linux lanes passed.

## Status boundary

WP-075 (CONE-F) is closed. Next M8 packet: sphere (playbook priority).
