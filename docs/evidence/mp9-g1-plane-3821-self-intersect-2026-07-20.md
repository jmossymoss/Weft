# G1: plane 3821 self_intersecting_unmeshable — 2026-07-20

Superseded by `mp9-g1-plane-3821-ellipse-densify-2026-07-20.md` (certified
plane-owned ellipse densify). Kept as diagnosis record.

## Body blocker

After perforated 3605 recovery, fail-closed MP9 refused:

```
plane.self_intersecting_unmeshable subjects=[6:3821]
```

## Extract

```
weft extract tests/STEP_Examples/MP9.stp --faces 3821 --rings 0 \
  -o tests/fixtures/mp9_extracts/plane_3821.step
weft mesh …/plane_3821.step → EXIT 1; plane.self_intersecting_unmeshable
weft inspect → 1 face, 6 edges (4 line + 2 ellipse); concave_simple_region
```

## Diagnosis

Single outer UV loop fails exact non-adjacent segment intersection checks
(`trim.loop.self_intersection`). G1 policy: named refuse, never
`allowCurvedUv` soften / pad-fan.

Likely next certified recovery (not done here): denser plane-owned ellipse
sampling and/or coedge UV unwrap so chords no longer cross; then planar CDT
with nesting proofs on.

## Proof

```
ctest --preset vs2022 -R planar_trim_assembly
→ PASS; WEFT_G1_PLANE3821 code=plane.self_intersecting_unmeshable
```

## Context

- 3605 perforated recovery: DONE (`mp9-g1-plane-3605-perforated-recovery-2026-07-20.md`)
- Digon 2732: DONE
- G5 hard assembly: preserve (`mp9-g5-hard-assembly-2026-07-20.md`)
