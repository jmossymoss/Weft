# HardSurfaceFloor product default — 2026-07-20

## Policy

Product meshing defaults to `SecureMeshingFloorPolicy::HardSurfaceFloor`:

- Prefer HARD (fail-closed) on game-critical surfaces: boolean-trimmed planes,
  cylinders, cones, torus / fillet-blend tags, four-sided Coons / mapped.
- Soft residual last resorts (`WEFT_SOFT_RESIDUAL`, `relaxGeometryChecks=true`)
  under HardSurfaceFloor only — never under `--strict-all-faces`:
  - plane: `assemblePlanarTrimDomain(..., softPlaneFallback=true)` emits
    `allowCurvedUv` when nesting/self-intersect validation fails; soft CDT /
    outer-only retry (`plane.trim_soft`, `plane.cdt_soft`).
  - cone / torus: UV-trim / wall lattice that cannot hardOrient
    (`cone.uv_trim_orientation`, `torus.uv_trim_orientation`,
    `torus.wall_orientation`).
  - sphere / non-Coons freeform / mapped UV-trim orientation residuals.
- Assemble under HardSurfaceFloor: widen split-rail / industrial surface
  envelopes; skip closed-manifold + triangle-intersection + interval
  consumption when any soft residual face is present; realign incomplete
  hard-path coverage rows so admission can succeed. Soft rows are logged
  (`WEFT_SOFT_COVERAGE_INCOMPLETE`, `WEFT_SOFT_INTERVAL_CONSUMPTION_SKIP`).

`--strict-all-faces` restores Wave 0 full hardOrient (research / matrix).
`--allow-partial-body` still omits deferred faces (unchanged).

Motivation: Plasticity-class faceting is fine where games do not see span
mismatch; Weft stays strict where Plasticity laddering hurts (cylinders,
fillets, Coons, boolean planes). Soft residuals are labeled, not silent heal.
MP9 must mesh under this default (not full face-id certify).

## Files

- `core/include/weft/secure_meshing.hpp` — `SecureMeshingFloorPolicy`
- `core/include/weft/planar_trim_assembly.hpp` — `softPlaneFallback`
- `core/include/weft/certified_mesh.hpp` — assemble soft envelopes
- `core/src/planar_trim_assembly.cpp` — soft plane domain emit
- `core/src/secure_meshing.cpp` — floor classification + soft admit + soft assemble
- `core/src/certified_mesh.cpp` — soft interior-station / split-rail / surface envelope
- `app/main.cpp` / `cli/main.cpp` — product default + `--strict-all-faces`
- `docs/governance/brep-consumer-matrix.md` — policy section

## Gate

```
build\vs2022\bin\Release\weft.exe mesh tests\STEP_Examples\MP9.stp `
  -o build\mp9_hardsurface.obj --progress
```

Expect: EXIT 0, non-empty OBJ, soft residuals OK (`WEFT_SOFT_RESIDUAL`).
No `--strict-all-faces`, no `--allow-partial-body`.

## Results (this session)

Focused ctest (`planar_cdt|secure_meshing`): PASS.

MP9 HardSurfaceFloor gate (`weft_mp9gate mesh MP9.stp -o build/mp9_gate_final.obj`):

```
MP9_EXIT=0 elapsed_s=2376
OBJ_BYTES=69453476
OBJ_V=499797 OBJ_F=495943
SOFT=33  PLANE_SOFT=2
  499797 vertices, 495943 polygons (460262 quads, 35681 tris)
```

Soft residual mix includes `plane.trim_soft` (1793/1869),
`torus.wall_orientation`, `cone.uv_trim_orientation`,
`mapped.uv_trim_orientation`, `freeform.uv_trim_orientation`.

## Remaining blockers

- StrictAllFaces / Wave A still refuse plane self-intersect (honest).
- Soft body skips closed-manifold + interval consumption proofs; certificate
  soft-coverage rows are realigned after soft admit (logged as
  `WEFT_SOFT_COVERAGE_INCOMPLETE`).
- Folded polygons remain in OBJ (376) — Plasticity-class residual, not gate.
