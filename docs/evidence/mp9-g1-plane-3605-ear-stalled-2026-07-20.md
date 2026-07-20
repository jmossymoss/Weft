# G1: plane 3605 ear_clipping_stalled — 2026-07-20

## Symptom

`tests/fixtures/mp9_extracts/plane_3605.step` (MP9 face 3605) refused:

```
cdt.triangle_count_mismatch  (before)
cdt.ear_clipping_stalled     (after; stable)
```

Inventory on extract: `multiply_perforated_disk`, `cutout.planar_perforated`,
22 edges (circle/line/bspline mix), filleted-slot residual plane.

## Root cause

1. Planar trim validates as one outer + multiple holes; CDT builds a
   bridge-cut boundary walk.
2. Exact ear clipping stalls (`cdt.ear_clipping_stalled`) — no positive ear
   on the industrial walk.
3. Previous fan fallback skipped zero-area wedges (`ExactSign::Zero`) and
   produced fewer than `V + 2H - 2` triangles, so `validateMesh` reported
   the misleading `cdt.triangle_count_mismatch`.

## Fix

In `planar_cdt.cpp`, for `allowCurvedUv=false` and `boundaryLoops.size() > 1`,
do **not** fan after ear stall. Preserve the ear failure (stable
`cdt.ear_clipping_stalled`). Fan remains only for simple plane disks and
curved UV walks.

G3 `edgeTris` copy-before-iterate AV fix untouched.

## Proof

```
weft mesh tests/fixtures/mp9_extracts/plane_3605.step -o %TEMP%\p3605.obj
→ EXIT 1; cdt.ear_clipping_stalled subjects=[6:1]

ctest --preset vs2022 -R "planar_cdt|planar_trim"
→ PASS; WEFT_G1_PLANE3605 code=cdt.ear_clipping_stalled
```

Extract kept: `tests/fixtures/mp9_extracts/plane_3605.step`.

## Certification deferred

A certified recovery would need a stronger perforated-plane CDT (alternate
bridges / constrained subdivision) that produces an ear-clippable walk or
an exact hole-aware triangulator. Not fail-open fan.

## Filleted-slot residual status

| Result | Count |
|---|---|
| EXIT 0 | 21 |
| `cdt.ear_clipping_stalled` @ 3605 | 1 (locked with extract) |

## Fail-closed MP9 body (after 3605 settle)

```
weft mesh tests/STEP_Examples/MP9.stp -o build/mp9_g1_body.obj --progress
omitDeferredResiduals=false (product default)
```

- Progress reached `face 2732/4270 id=2732 family=plane` (~157s after import/intervals/boundaries).
- Refused: `plane.loop_collapsed_unmeshable subjects=[6:2732, 7:2921]` (LOCKED digon; extract `plane_2732.step`).
- Face 3605 was not reached (higher id than 2732). Extract gate for 3605 remains green.

### Digon policy (body green)

MP9 remains RED under fail-closed defaults while digon face 2732 is refused by name.
No certified digon recovery consumer exists (pad-fan / `allowCurvedUv` are forbidden).
Product choices for body EXIT 0:

1. Add a certified digon/slit-plane recovery consumer with proof, or
2. Keep named refuse (current) and accept body RED until topology is repaired upstream, or
3. Explicit partial-body escape (`--allow-partial-body`) — not the product default.

Recommendation: leave RED with named refuse + extracts; do not soften.
