# MP9 single-face extract import meshability — 2026-07-20

## Scope

Re-verify the six extracts that previously refused at
`secure_pipeline.import_not_meshable` / `import.working.non_meshable` with empty
subjects (see `mp9-failclosed-inventory-2026-07-20.md`), after G3 landed the
single-face identity-invalid mesh allowlist in `core/src/secure_core.cpp`.

- Host: Windows MSVC `vs2022` + OCCT on PATH
- Revision: `750a2ab` (working tree includes G3 import allow + parallel G0–G4 edits)
- CLI: `build/vs2022/bin/Release/weft.exe` (binary stamped 2026-07-20 ~00:32)
- Product-default mesh: `weft mesh <extract> -o <out.obj>` (no `--allow-partial-body`)
- Inventory: `weft inventory <extract> --probe-limit 1`

## Root cause (shared)

All six fixtures are single-face Plasticity extracts. OCCT `BRepCheck` reports
`source_valid=0` / `working_valid=0` (open-shell / incomplete solid context),
while transfer remains `identity=1` with complete correspondence.

Previously `repair.meshable` stayed false → CLI
`secure_pipeline.import_not_meshable` before any face subject was attached.

G3 fix (already in tree; not re-authored here): when identity-invalid and
correspondence-complete, allow meshing entry for `workingFaces == 1` (and for
large bodies with `workingEdges > 500`), emitting
`import.working.invalid_identity_mesh_allowed`. Per-face consumers still refuse
by name.

Industrial `skipped_large` / large-body soft paths were not weakened further.

## Re-verification matrix

| Extract | Track | meshable | Mesh EXIT | Result | Notes |
|---|---|---|---|---|---|
| `freeform_hex.step` | G3 | 1 | 0 | **pass** | 34 v / 18 polys (14q/4t); probe ok tris=32 |
| `offset_quad.step` | G3 | 1 | 0 | **pass** | 1089 v / 1056 polys (992q/64t); probe ok tris=2048 |
| `cone_frustum.step` | G4 | 1 | 0 | **pass** | 64 v / 32 quads; probe ok tris=64 |
| `sphere_cap_778.step` | G4 | 1 | 0 | **pass** | 72 v / 43 polys (27q/16t); probe ok tris=70 |
| `sphere_cap_complex.step` | G4 | 1 | 0 | **pass** | same stats as `sphere_cap_778`; probe ok tris=70 |
| `cyl_24.step` | G2 | 1 | 1 | **import cleared; named consumer refuse** | `boundary.curve_evaluation_failed` `subjects=[9:3]`; probe refuse same code |

Sibling control: `cylinder_band.step` still EXIT 0 (66 v / 32 quads).

## Per-extract status

### freeform_hex.step (G3) — fixed / green

- Was: `import_not_meshable`, empty subjects.
- Now: import meshable; product mesh EXIT 0 with Independent-style quads.
- Action this session: none (G3 already fixed).

### offset_quad.step (G3) — fixed / green

- Was: `import_not_meshable`, empty subjects.
- Now: import meshable; product mesh EXIT 0.
- Action this session: none (G3 already fixed).

### cone_frustum.step (G4) — green after G3 import allow

- Was: `import_not_meshable`.
- Now: meshable=1; EXIT 0 (32 quads). Extract validity not rewritten.
- Action this session: document only.

### sphere_cap_778.step / sphere_cap_complex.step (G4) — green after G3 import allow

- Was: `import_not_meshable`.
- Now: both meshable=1; EXIT 0 (identical 72/43 mesh stats).
- Action this session: document only.

### cyl_24.step (G2) — import gate cleared; hand off to G2

- Was: `import_not_meshable` with empty subjects.
- Now: meshable=1 (same identity-invalid single-face allow). Mesh refuses with
  named code and non-empty subjects:
  `boundary.curve_evaluation_failed` — “canonical sample did not evaluate on
  the exact 3D curve” — `subjects=[9:3]`.
- Not an import/working-shape gate bug anymore. No extract rewrite or import
  change in this session (coordinate with cylinder agent; avoid template math
  here).

## Commands (representative)

```powershell
$cli = "build\vs2022\bin\Release\weft.exe"
$dir = "tests\fixtures\mp9_extracts"
$out = "build\g_import_probe"
foreach ($e in @(
  "freeform_hex.step","offset_quad.step","cyl_24.step",
  "cone_frustum.step","sphere_cap_778.step","sphere_cap_complex.step"
)) {
  & $cli inventory "$dir\$e" --probe-limit 1
  & $cli mesh "$dir\$e" -o "$out\$($e -replace '\.step$','.obj')"
}
```

## Decision / STOP

- Five of six former import refusals now product-mesh EXIT 0 without fixture repair.
- Remaining `cyl_24` failure is post-import named refusal with subject ids → G2.
- No further import-gate or extract edits in this session.
- Do not revert G3 `secure_core` / mapped / freeform changes.
