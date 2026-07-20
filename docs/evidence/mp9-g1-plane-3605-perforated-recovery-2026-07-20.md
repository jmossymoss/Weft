# G1: perforated plane 3605 certified recovery — 2026-07-20

## Goal

MP9 fail-closed body stopped at `cdt.ear_clipping_stalled` subjects=`[6:3605]`
after digon 2732 recovery. Certify the multiply-perforated plane without
pad-fan or `allowCurvedUv`.

## Diagnosis (`plane_3605.step`)

- Topology: 1 plane, 22 edges, `multiply_perforated_disk` / `cutout.planar_perforated`.
- Trim: 1 outer + 6 hole wires (STEP `FACE_BOUND` ×7).
- Shortest outer-only bridges produced a weakly-simple cut walk with no exact
  positive ear. Fan under-counts vs `V+2H-2` → previously masked as
  `cdt.triangle_count_mismatch`; G1 locked honest `cdt.ear_clipping_stalled`.

## Recovery (certified)

In `planar_cdt.cpp`:

1. Bridge each hole onto the **current cut walk** (outer + already-inserted
   seam vertices), with cone/visibility predicates at the chosen walk
   position (duplicate landings stay unambiguous).
2. Prefer shortest exact bridge; on ear stall, try alternate ranks per hole
   then bounded pairwise rank pairs.
3. Still **no fan** when `allowCurvedUv=false` and `boundaryLoops.size()>1`.

Nesting proofs remain on; digon densify and G5 assembly harden untouched.

## Proof

```
weft mesh tests/fixtures/mp9_extracts/plane_3605.step -o %TEMP%\p3605.obj
→ EXIT 0; 116 vertices, 68 polygons (58 quads, 10 tris)

ctest --preset vs2022 -R "planar_cdt|planar_trim"
→ PASS; WEFT_G1_PLANE3605 tris>0
```

Supersedes locked refuse in `mp9-g1-plane-3605-ear-stalled-2026-07-20.md`.

## Body

```
weft mesh tests/STEP_Examples/MP9.stp -o build/mp9_g1_after_3605.obj --progress
omitDeferredResiduals=false
```

- Passed `id=2732` (digon recovery) and `id=3605` (this recovery; ~2.6s face).
- Next refuse: `plane.self_intersecting_unmeshable subjects=[6:3821]`
  — extract `tests/fixtures/mp9_extracts/plane_3821.step` (locked refuse;
  see `mp9-g1-plane-3821-self-intersect-2026-07-20.md`).

## Preserved

- Digon 2732 densify / UV-corner keep
- No-fan-on-perforated (fan only simple plane disks / curved UV)
- G3 `edgeTris` copy-before-iterate AV fix
- G5 closed-manifold / interval consumption / no coverage zeroing
- G2 cylinder / G3 mapped / G4 sphere paths
