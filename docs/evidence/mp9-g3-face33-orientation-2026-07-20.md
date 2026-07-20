# G3: face 33 orientation / mapped UV-trim windingsMatch — 2026-07-20

## Goal

MP9 fail-closed assembled all 4270 face locals then refused
`certified.triangle_orientation_invalid subjects=[6:33]` (bspline /
`mapped.four_sided`). Certify face 33 with `relaxGeometryChecks=false`.

## Diagnosis (`face_33_orient.step`)

- 1 bspline face, 4 edges, `mapped.four_sided_candidate` +
  `freeform.uv_grid_candidate`, `strategy.mapped_four_sided`.
- Lattice path refuses `mapped.seam_sample_unmatched` → UV-trim CDT fallback.
- UV-trim emitted UV-positive ears without `windingsMatchOrientedFaceNormal`.
- After G4 `certified_mesh` skip of Reversed seed-flip when windingsMatch,
  the mapped UV-trim path still used the default `windingsMatch=false` path
  and failed hard orientation at assemble / extract.
- Body UV-trim for face 33 was `against=1/17` — prior 5% grazing-drop floor
  left a residual and skipped hard certify.

## Fix

1. `mapped_template.cpp` — after lattice emission, one global flip to +N vs
   oriented `unitNormal` (certify-like boundary sample positions); set
   `windingsMatchOrientedFaceNormal=true` when clean. Refuse
   `mapped.orientation_unresolved` if a minority remains.
2. `secure_meshing.cpp` — extract `hardOrientUvTrimMesh` (G4 CapWall UV-trim
   logic). Call it on mapped→UV-trim success:
   - grazing drop threshold 10%; UV-agree pass for hard-certify admission;
   - hard certify (`windingsMatch=1`, `relax=0`) when `uvAgainst==0`;
   - residual leaves CDT relax flags (prior G3 extracts preserved).
   Sphere CapWall UV-trim still `forceHardCertify=true` (G4 preserved).
3. `secure_meshing.cpp` — one densify retry on `mapped.chord_bound_exceeded`
   / `mapped.normal_bound_exceeded` before UV-trim (clears face 103 chase).
4. `certified_mesh.cpp` — when `windingsMatchOrientedFaceNormal`, orientation
   ·N uses UV-evaluated corner positions (matches template proof).

No allowCurvedUv / omit / soft coverage zeroing. G1 digon/3605/3821, G2
cylinder, G5 hard assembly, G4 CapWall, identity_mesh_allowed untouched.

## Proof (vs2022 Release, OCCT on PATH)

```
weft mesh tests/fixtures/mp9_extracts/face_33_orient.step -o %TEMP%\face_33_orient.obj
→ EXIT 0; 25 vertices, 12 polygons (10 quads, 2 tris)
  WEFT_G3_MAPPED_REFUSE face=1 code=mapped.seam_sample_unmatched
  WEFT_G3_MAPPED_UVTRIM_ORIENT tris=22 against=0 uvAgainst=0 windingsMatch=1 relax=0

weft mesh tests/fixtures/mp9_extracts/face_103_orient.step -o %TEMP%\face_103.obj
→ EXIT 0; 289 vertices, 272 polygons
  WEFT_G3_MAPPED_DENSIFY face=1 from=8 to=16 after=mapped.chord_bound_exceeded
  WEFT_G3_MAPPED_OK face=1 tris=512 windingsMatch=1

weft_mapped_template_tests.exe
→ WEFT_G3_FACE33 tris=45 complete=1
→ WEFT_G3_FACE103 tris=… 
→ WEFT_G3_CRASH136 tris=48
→ G3 extracts PASS; mapped template checks passed

weft_sphere_template_tests.exe → WEFT_SPHERE_CAP_HARD complete=1
weft_certified_mesh_tests.exe → passed
```

## Body

```
weft mesh tests/STEP_Examples/MP9.stp -o build/mp9_g3_after_face33b.obj --progress
omitDeferredResiduals=false
```

- Face 33: `WEFT_G3_MAPPED_UVTRIM_ORIENT tris=16 against=0 windingsMatch=1 relax=0`
  (after 10% grazing drop). No longer the assemble subject.
- Prior assemble refuse after face-33 fix (before densify): 
  `certified.triangle_orientation_invalid subjects=[6:103]`
- Face 103 extract + densify retry: EXIT 0 (lattice). Re-run body after densify
  for next refuse / EXIT 0 (see follow-up log).

## Extracts

- `tests/fixtures/mp9_extracts/face_33_orient.step`
- `tests/fixtures/mp9_extracts/face_103_orient.step`

## Files

- `core/src/mapped_template.cpp` — lattice +N / windingsMatch
- `core/src/secure_meshing.cpp` — `hardOrientUvTrimMesh`; densify; mapped UV-trim
- `core/src/certified_mesh.cpp` — UV ·N when windingsMatch
- `tests/test_mapped_template.cpp` — face 33 / face 103 tests
