# G1: plane 2732 loop_collapsed_unmeshable — 2026-07-20

## Body blocker

After G3 cleared the face-136 AV, fail-closed MP9 mesh refused:

```
plane.loop_collapsed_unmeshable subjects=[6:2732, 7:2921]
```

(Wire 2921 on face 2732; not bspline face 2921.)

## Extract

```
weft extract tests/STEP_Examples/MP9.stp --faces 2732 --rings 0 \
  -o tests/fixtures/mp9_extracts/plane_2732.step
weft mesh tests/fixtures/mp9_extracts/plane_2732.step -o %TEMP%\plane_2732.obj
→ EXIT 1; plane.loop_collapsed_unmeshable subjects=[6:1, 7:1]
weft inspect …/plane_2732.step → 1 face, 2 edges (digon)
```

## Diagnosis

Face 2732 is a supported plane digon: two boundary edges, one wire. After
canonical UV assembly and exact duplicate-station cleanup, the loop has fewer
than three UV stations. Inventing pad vertices or `allowCurvedUv` would be a
fail-open soften (G0/G1 forbid). Named refuse is the honest consumer.

## Changes

- Commit extract `tests/fixtures/mp9_extracts/plane_2732.step`
- `tests/test_planar_trim_assembly.cpp`: `testG1Plane2732CollapsedLoopRefuse`
- Keep prior G1 planar_cdt hard paths (multi-outer `closed=true`, multi-station
  hole bridge, validateMesh-on for curved ears; preserve G3 edgeTris copy-before-
  iterate AV fix)

## Verification

```
ctest --preset vs2022 -R "planar_cdt|planar_trim" --output-on-failure
```

Expect `WEFT_G1_PLANE2732 code=plane.loop_collapsed_unmeshable` and PASS.

## Filleted-slot plane residual sweep (same session)

`cutout.filleted_slot_residual` ×22, rings=0 extracts, current `weft` mesh:

| Result | Count | Notes |
|---|---|---|
| EXIT 0 | 21 | including multiply_perforated 277/1715/3595 and multi-outer samples |
| `cdt.triangle_count_mismatch` | 1 | face **3605** (22-edge multi-disconnected); extract `plane_3605.step` |

## Fail-closed MP9 body (2026-07-20 re-run)

Confirmed first plane blocker after AV fix:

```
face 2732/4270 → plane.loop_collapsed_unmeshable subjects=[6:2732, 7:2921]
```

Body stays RED until digon recovery or explicit partial-body policy. See
`mp9-g1-plane-3605-ear-stalled-2026-07-20.md` for digon product options.

## Remaining G1

- Certified perforated-plane ear recovery for 3605 (optional; refuse locked)
- Digon recovery consumer vs leave body RED (product decision)
- Self-intersect plane battery / next refuses after digons cleared
