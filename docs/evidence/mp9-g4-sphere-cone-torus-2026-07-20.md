# MP9 G4 sphere / cone / torus acceptance — 2026-07-20

## Scope

Close G4 on post-G3 import greens (`mp9-import-extract-meshable-2026-07-20.md`)
without reverting `import.working.invalid_identity_mesh_allowed`.

- Host: Windows MSVC `vs2022` + OCCT on PATH
- Targets: `cone_frustum.step`, `sphere_cap_778.step`, `sphere_cap_complex.step`,
  fixture torus preview LOD
- Acceptance: CapWall / hard UV-trim / cone residual with
  `relaxGeometryChecks=false`; torus chord-valid at default preview LOD;
  no soft hide path for sphere caps

## Results

| Case | Path | `relaxGeometryChecks` | Mesh EXIT | Notes |
|---|---|---|---|---|
| `cone_frustum.step` | cylinder-wall frustum band | false | 0 | Hard (prior G4); non-vacuous intersection |
| fixture `torus` preview LOD | `buildFullTorusWall` | false | 0 | `WEFT_TORUS_PREVIEW_LOD tris=4608`; chord/normal complete |
| `sphere_cap_778.step` | CapWall refuse → **hard** UV-trim | **false** | 0 | `WEFT_G4_UVTRIM_ORIENT … against=0 windingsMatch=1 relax=0`; `certified.triangle_intersection expected=2346 checked=2346` |
| `sphere_cap_complex.step` | same | **false** | 0 | `WEFT_SPHERE_CAP_HARD` complete=1 |

No torus MP9 extract in tree; torus proof is fixture preview LOD only.

## Sphere caps — hard certify (closed this session)

Classification: `family=sphere`, `trim=periodic_band_crossing_seam`,
`sphere.uv_trim_*`. Face occurrence orientation = Reversed.

### Root cause

1. `GeometryEvaluator` already applies TopoDS face orientation to `unitNormal`.
2. Certify still applied Reversed **seed-negation + global flip** even when
   `windingsMatchOrientedFaceNormal=true`, fighting templates that already
   emit +N vs oriented normals.
3. CapWall lattice still retains a geometric ·N minority under one
   combinatorial winding (`sphere.cap_orientation_unresolved`) — band caps
   therefore fall through to UV-trim.
4. Soft UV-trim (`allowCurvedUv` + `relaxGeometryChecks=true`) was the prior
   EXIT 0 path; forbidden for G4 acceptance.

### Fix

- `certified_mesh.cpp`: when `windingsMatchOrientedFaceNormal=true`, skip the
  seed-negation / seed-global-flip block (per-tri Reversed swap was already
  skipped).
- CapWall: south/north fan winding from pole V; emit +N; `windingsMatch=true`;
  refuse on assembled ·N minority (`sphere.cap_orientation_unresolved`).
- Band-cap UV-trim fallback: materialize `cornerUv`, one global flip to +N,
  drop a tiny grazing minority (preserves edge winding; no per-tri flip),
  `windingsMatch=true`, `relaxGeometryChecks=false`.

### Commands

```
weft mesh tests/fixtures/mp9_extracts/sphere_cap_778.step -o s778.obj --mesh-report s778.json
weft_sphere_template_tests
```

- Product: 72 vertices / 37 polygons; `complete 1`
- `certified.triangle_intersection expected=2346 checked=2346 failed=0`
- Tests: `WEFT_SPHERE_CAP_HARD` (no `WEFT_SPHERE_CAP_SOFT_RESIDUAL`)

## Cone / torus (unchanged hard — accepted)

Cone frustum and torus preview LOD remain hard-certified as in the prior
partial evidence; not re-litigated this session.

## Files touched (hard-gap closure)

- `core/src/certified_mesh.cpp` — skip seed flip when windingsMatch
- `core/src/sphere_template.cpp` — CapWall south/north fan; +N / windingsMatch
- `core/src/secure_meshing.cpp` — hard UV-trim orient (no soft path)
- `tests/test_sphere_template.cpp` — require hard intersection / no soft residual
