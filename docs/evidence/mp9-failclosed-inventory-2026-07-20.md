# MP9 fail-closed mesh inventory — 2026-07-20

## Scope

Product-default mesh path (`omitDeferredResiduals=false`; no `--allow-partial-body`)
on Windows MSVC `vs2022` + OCCT 8.x.

- Revision: `750a2ab` (working tree also has parallel G0–G4 edits; binary rebuilt
  for this inventory).
- CLI: `build/vs2022/bin/Release/weft.exe`
- Command pattern: `weft mesh <file> -o <out.obj>` (no allow-partial)
- Supporting: `weft inventory <file> [--probe-limit 1 --probe-body]`

Track map (B-rep meshing plan):

| Track | Owns |
|---|---|
| G1 | planes (multi-outer, hole bridge, self-intersect) |
| G2 | cylinders / cut-graph / filleted-slot residuals |
| G3 | freeform / mapped / extrusion / offset |
| G4 | sphere / cone / torus (CapWall) |

## Extract matrix (original 15)

| Extract | Family | EXIT | Code | Subjects | Inventory | Suggested track |
|---|---|---|---|---|---|---|
| `bspline_139.step` | bspline | 0 | — | — | meshable=1; probe ok tris=17 | G3 (pass) |
| `cone_frustum.step` | cone | 1 | `secure_pipeline.import_not_meshable` | *(none)* | meshable=0; `import.working.non_meshable`; blocker `strategy.surface.cone\|full_periodic_with_cap_boundaries` | G4 |
| `cyl_24.step` | cylinder | 1 | `secure_pipeline.import_not_meshable` | *(none)* | meshable=0; `import.working.non_meshable`; blocker `strategy.surface.cylinder\|full_periodic_with_cap_boundaries` | G2 |
| `cylinder_band.step` | cylinder | 0 | — | — | meshable=1; probe ok tris=64 | G2 (pass) |
| `cylinder_complex.step` | cylinder | 0 | — | — | meshable=1; probe ok tris=66 | G2 (pass) |
| `cylinder_ellipse.step` | cylinder | 0 | — | — | meshable=1; probe ok tris=7; OBJ reports 1 folded polygon | G2 (pass w/ quality note) |
| `cylinder_ellipse_band.step` | cylinder | 0 | — | — | meshable=1; probe ok tris=7; OBJ reports 1 folded polygon | G2 (pass w/ quality note) |
| `freeform_hex.step` | bspline | 1 | `secure_pipeline.import_not_meshable` | *(none)* | meshable=0; blocker `strategy.freeform_uv_trim\|convex_simple_region` | G3 |
| `freeform_mapped_fallback.step` | bspline | 0 | — | — | meshable=1; probe ok tris=17 | G3 (pass) |
| `freeform_pent.step` | bspline | 0 | — | — | meshable=1; probe ok tris=23 | G3 (pass) |
| `offset_quad.step` | offset | 1 | `secure_pipeline.import_not_meshable` | *(none)* | meshable=0; blocker `strategy.mapped_four_sided\|convex_simple_region` | G3 |
| `plane_1793.step` | plane | 0 | — | — | meshable=1; probe ok tris=18; recon class `self_intersecting_or_invalid` yet meshes | G1 (pass) |
| `plane_multi.step` | plane | 0 | — | — | meshable=1; probe ok tris=2 | G1 (pass) |
| `sphere_cap_778.step` | sphere | 1 | `secure_pipeline.import_not_meshable` | *(none)* | meshable=0; blocker `strategy.surface.sphere\|periodic_band_crossing_seam` | G4 |
| `sphere_cap_complex.step` | sphere | 1 | `secure_pipeline.import_not_meshable` | *(none)* | meshable=0; blocker `strategy.surface.sphere\|periodic_band_crossing_seam` | G4 |

Success OBJ sizes (for reference):

| Extract | Vertices | Polygons |
|---|---|---|
| bspline_139 | 19 | 10 (7q/3t) |
| cylinder_band | 66 | 32 (32q) |
| cylinder_complex | 68 | 34 (32q/2t) |
| cylinder_ellipse | 8 | 4 (2q/2t) + folded |
| cylinder_ellipse_band | 8 | 4 (2q/2t) + folded |
| freeform_mapped_fallback | 19 | 10 (7q/3t) |
| freeform_pent | 25 | 13 (10q/3t) |
| plane_1793 | 20 | 12 (6q/6t) |
| plane_multi | 4 | 1 (1q) |

### Failure families (extracts)

| Code | Count | Extracts | Track |
|---|---|---|---|
| `secure_pipeline.import_not_meshable` | 6 | cone_frustum, cyl_24, freeform_hex, offset_quad, sphere_cap_778, sphere_cap_complex | G2/G3/G4 by family |
| *(no mesh refusal with subject face ids)* | — | import gate fires before face subjects are attached | — |

Notes:

- All six extract refusals share `source_valid=0 working_valid=0` and
  `import.working.non_meshable` (validity_check.done valid=0 after param loop).
  They are single-face shells; subject lists are empty on the mesh CLI error
  line. Inventory `--probe-body` / `--probe-limit 1` still reports
  `face id=1` with the same import code.
- No extract produced a named consumer refusal (`plane.*`, `cdt.*`,
  `mapped.*`, CapWall, etc.) under the product-default path — failures stop at
  import meshability.

## Added crash-locus extracts (this scout)

From full-body progress stop at face id 136:

| Extract | Contents | Mesh result | Track |
|---|---|---|---|
| `crash_face_136_r0.step` | source face 136 only (bspline, 3 edges) | EXIT 1 `secure_pipeline.import_not_meshable` (subjects empty); inspect OK | G3 |
| `crash_face_136_r1.step` | faces 135,136,138 | EXIT 1 OCCT transfer `TopoDS::Solid` (does not re-import) | G3 |
| `crash_face_136_r2.step` | face 136 + 2 rings | EXIT 1 OCCT transfer `TopoDS::Solid` | G3 |

Isolated r0 does **not** reproduce the body ACCESS_VIOLATION; it refuses at
import. Body crash appears full-path / neighbor-context dependent.

## Other untracked extracts present in tree

Parallel agents left these; recorded for completeness (not in the original 15):

| Extract | Mesh result | Note |
|---|---|---|
| `cyl_24_band.step` | EXIT 1 OCCT transfer `TopoDS::Solid` | transfer-broken; G2 WIP? |
| `cyl_filleted_slot_277.step` | EXIT 1 OCCT transfer `TopoDS::Solid` | transfer-broken; G2 WIP? |

## MP9 body (`tests/STEP_Examples/MP9.stp`)

Present (≈27 MB). Fail-closed mesh:

```
weft mesh tests/STEP_Examples/MP9.stp -o … --progress
```

| Result | Detail |
|---|---|
| EXIT | `-1073741819` (`0xC0000005` ACCESS_VIOLATION) |
| Last progress | `face 136/4270 id=136 family=bspline` (~45.3 s after recon/intervals/boundaries) |
| OBJ | not written |
| Named refusal | none (hard crash, not a fail-closed code) |

Inventory (no `--probe-body`):

- `meshable=1` with `import_error import.repair.validation_incomplete=1`
  (`source_valid=0 working_valid=0` but industrial import still marks meshable)
- `supported_faces=4270`, `unsupported_subjects=0`
- Face family counts: plane 1523, bspline 1233, cylinder 1004, torus 221,
  cone 184, sphere 70, extrusion 18, offset 17
- Notable plane blockers still classified on body: `multiple_disconnected_domains=882`,
  `self_intersecting_or_invalid=55`, `annulus=37`, `multiply_perforated_disk=3`
  (G1 body work blocked until AV is fixed)

Suggested track for body crash: **G3** (faulting face family=bspline id=136);
blocks G1–G4 full-body acceptance until stack/extract is fixed or named-refused.

## Grouped summary → tracks

| Track | Extract passes | Extract fails | Body status |
|---|---|---|---|
| G1 | plane_multi, plane_1793 | — | blocked by AV before plane residuals prove |
| G2 | cylinder_band, cylinder_complex, cylinder_ellipse*, ellipse_band | cyl_24 (`import_not_meshable`) | blocked by AV |
| G3 | bspline_139, freeform_mapped_fallback, freeform_pent | freeform_hex, offset_quad (`import_not_meshable`); crash_face_136_r0 | **owns body AV @ f136** |
| G4 | — | cone_frustum, sphere_cap_778, sphere_cap_complex (`import_not_meshable`) | blocked by AV |

## Exact next diagnostic actions (no source edits here)

1. Debug full-body mesh under AV at post-face-136 (G3); prefer neighbor-ring
   extract that re-imports (r1/r2 currently fail `TopoDS::Solid` transfer).
2. G2/G3/G4: restore or re-extract meshable STEP for the six
   `import_not_meshable` fixtures (validity_check fails on single-face shells
   under current OCCT 8.x / import path) so consumer codes can surface.
3. Re-run this inventory after consumer work; expect either EXIT 0 stats or
   named codes with non-empty `subjects=[…]`.
