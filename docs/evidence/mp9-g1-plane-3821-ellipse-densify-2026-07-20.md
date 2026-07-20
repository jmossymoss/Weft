# G1: plane 3821 ellipse densify recovery — 2026-07-20

## Goal

MP9 fail-closed refused `plane.self_intersecting_unmeshable` on face 3821
after perforated 3605 recovery. Certify without `allowCurvedUv`.

## Diagnosis (`plane_3821.step`)

- 1 plane, 6 edges (4 line + 2 ellipse), single outer, `concave_simple_region`.
- Ellipses are highly eccentric (STEP major≈54.4, minor≈4.7).
- Chord-sagitta interval demand produced sparse UV chords that crossed under
  exact segment predicates → `trim.loop.self_intersection` → named refuse.

## Recovery (certified)

`secure_meshing.cpp`: edges classified `ellipse` that bound a `plane` face get
a floor of **48** intervals (after normal elliptical demand). Nesting proofs
stay on; `allowCurvedUv` stays false.

## Proof

```
weft mesh tests/fixtures/mp9_extracts/plane_3821.step -o %TEMP%\p3821.obj
→ EXIT 0; 100 vertices, 55 polygons (43 quads, 12 tris)

ctest --preset vs2022 -R "planar_cdt|planar_trim"
→ PASS; WEFT_G1_PLANE3821 tris>0
```

Supersedes locked refuse in `mp9-g1-plane-3821-self-intersect-2026-07-20.md`.

## Body

```
weft mesh tests/STEP_Examples/MP9.stp -o build/mp9_g1_after_3821.obj --progress
omitDeferredResiduals=false
```

- Passed `id=2732`, `id=3605`, `id=3821` (this recovery), through face `4270/4270`.
- Assembly refuse (all faces produced local meshes):
  `certified.triangle_orientation_invalid subjects=[6:33]`
- Face 33 is **bspline / mapped.four_sided** (not a plane). Extract:
  `tests/fixtures/mp9_extracts/face_33_orient.step` — reproduces the same
  code on the isolated face. G3/G5 ownership for winding vs oriented normal;
  not a further G1 plane densify issue.
- Did **not** hit `certified.incidence_boundary_unexpected` on this run.

## Preserved

- Digon 2732 densify / UV-corner keep
- Perforated 3605 walk-landing / alternate-bridge recovery
- G5 closed-manifold / interval consumption / no coverage zeroing
- No pad-fan / no `allowCurvedUv` soften
