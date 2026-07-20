# G3: MP9 face-136 bspline ACCESS_VIOLATION — 2026-07-20

## Package

WP-177 blocker (fail-closed inventory). Track G3 (freeform/mapped/UV-trim).

## Bug

`weft mesh tests/STEP_Examples/MP9.stp` with `omitDeferredResiduals=false`
stopped at progress `face 136/4270 id=136 family=bspline` with
`EXIT 0xC0000005` ACCESS_VIOLATION — no OBJ, no named refuse.

Inventory: `docs/evidence/mp9-failclosed-inventory-2026-07-20.md`.

## Reproduction (Windows vs2022 + OCCT PATH)

```
$env:PATH = "C:\OCCT\win64\vc14\bin;" + $env:PATH
build\vs2022\bin\Release\weft.exe mesh `
  tests\fixtures\mp9_extracts\crash_face_136_r0.step `
  -o $env:TEMP\crash136_r0.obj --progress
```

After G3 single-face identity-mesh allow, the extract reproduces the AV
(progress `face 1/1 id=1 family=bspline`). Inventory:

- `freeform.uv_grid_candidate` + `mapped.non_four_sided_deferred`
- face blocker `…|strategy.freeform_uv_grid|annulus` (3 edges)

## Root cause

1. Face 136 is a bspline annulus tagged `freeform.uv_grid_candidate`.
   Mapped four-sided lattice refuses (`mapped.seam_sample_unmatched`);
   caller falls through to curved UV-trim CDT (`allowCurvedUv=1`, 2 loops).
2. Ear clipping stalls; fan triangulation succeeds.
3. Post-fan curved winding pass called `rebuild()` (which `edgeTris.clear()`s)
   while a range-for still iterated `edgeTris[edge(a,b)]` → use-after-free /
   ACCESS_VIOLATION.

Site: `core/src/planar_cdt.cpp` (ExactLawsonReferencePlanarCdtBackend::triangulate
curved UV manifold winding enforcement).

## Ownership conflict

Preferred edit locus was `mapped_template.cpp` / freeform branches in
`secure_meshing.cpp`. Progress markers proved the AV is inside
`cdt->triangulate` after mapped refuse + UV-trim OK. Fix is therefore in
`planar_cdt.cpp` (G1-owned). G1 parallel WIP on the same file was preserved
where possible; this change is the winding-list copy only.

## Fix

Copy the adjacency list before iterating; defer `rebuild()` until after the
copied uses for that edge are processed:

```cpp
const std::vector<DirectedUse> uses = edgeTris[edge(a, b)];
bool rebuilt = false;
for (const DirectedUse& use : uses) { … rebuilt = true; … }
if (rebuilt) rebuild();
```

## Tests

```
build\vs2022\bin\Release\weft_planar_cdt_tests.exe
→ exact planar CDT reference checks passed
  (includes testCurvedUvAnnulusWindingNoAv)

build\vs2022\bin\Release\weft_mapped_template_tests.exe
→ WEFT_G3_CRASH136 tris=48
→ mapped template checks passed
```

Extract CLI (post-fix): EXIT 0, 24 verts / 13 polygons.

## Full MP9 (fail-closed, no --allow-partial-body)

Revision base: `750a2ab` + working-tree fix.

```
weft mesh tests/STEP_Examples/MP9.stp -o … --progress
```

| Checkpoint | Result |
|---|---|
| face 136/4270 bspline | passes (mapped refuse → UV-trim CDT ok) |
| faces after 136 | continue (thousands of faces; UV-trim/CDT ok) |
| stop | named refuse at face 2732: `plane.loop_collapsed_unmeshable` subjects=`[6:2732,7:2921]` |
| ACCESS_VIOLATION | none observed through face 2732 |
| OBJ | not written (named plane refuse — G1 residual) |

Face-136 AV blocker cleared. Later full-body gate remains open on plane
residuals (G1) and other family tracks.

## Changed files

- `core/src/planar_cdt.cpp` — winding adjacency copy (AV fix)
- `tests/test_planar_cdt.cpp` — curved UV annulus regression
- `tests/test_mapped_template.cpp` — `crash_face_136_r0` regression
- this evidence + index entry

## Exit

Face-136 AV closed: extract certifies; body progresses past 136 to a named
plane refuse (not AV). WP-177 full-body certificate still OPEN.
