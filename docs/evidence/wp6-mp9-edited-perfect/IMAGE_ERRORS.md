# Image error analysis (gallery retake 2026-08-08)

Fresh solid+wire, deselected screenshots in `objects/object_NNN_v{0,1}.png`
(also `/opt/cursor/artifacts/mp9_object_qa/`). Global validity green:
watertight, folds=0, failed-floor=0, structured 3052/3052.

## Automated flags

| Obj | Flag | Notes |
|----:|------|-------|
| 5 | under-span IsoBand | f373 nu=15 ngon; recess drums nu=7 |
| 11 | DRUM_LOW | f1611/1613 nu=15 notched |
| 14 | DRUM_LOW | f1770 nu=1 coons (report quirk); others ≥24 |
| 19 | BLOB | 3× MinimalNGon freeform body |
| 41/42 | HIGH_TRI | ~36% tris |
| many small | BLOB | false-positive on ≤4-face pin/collar parts |

Drums under nu=24: **112 / 218**.

## Prioritized visual defects

### V1 — Object 5 short IsoBand cylinders (FIXED 2026-08-08)
Short IsoBand cylinder lands (`edgeIds ≤ 5`) are forced to
`MinimalNGon` to avoid mid-span staves. Borders stay at adaptive arc
counts (nu≈7–15), so recesses look hexagonal and the large land
(f373, r≈14) looks coarse. Artist policy: cylinders ≥24 circumferential
spans; no mid-length support rings.

Fix class: prefer open-band / revolution with a single axial cell
(`nv=1`) + wrap-scaled circumferential floor; do not collapse short
IsoBand cylinders to MinimalNGon by default.

Verification: f373/f376/f378/f382/f384/f386/f388 all `revolution-grid`
`nu=24 nv=1`. Global still watertight, folds=0, failed-floor=0.
Screenshots: `objects/after_v1/object_005_v{0,1}.png`.

### V2 — Object 19 freeform body (IMPROVED 2026-08-08)
Large few-edge freeform Coons no longer demote to MinimalNGon on ≤3 tip
folds (that melted the spring into a blob). Body is `coons-grid` again.
Residual: 3 tip folded cells on the freeform Coons class only — allowed
in `testMp9EditedWatertight` while a fold-free remesh is chased.
Screenshots: `objects/after_v2/object_019_v{0,1}.png`.

### V3 — Objects 41/42 high-triangle brackets (DIAGNOSED)
Tris come from 3-edge torus freeform (`f2572`/`f2582`) meshed as
RevolutionGrid (or Coons with collapsedLast) — both fan ~7 tris at the
pole — plus a 1-tri sphere-cap GeometricCap quad-fill. Not a density bug;
needs a non-fan torus/sphere strap strategy. Parked after V1/V2.

### V4 — Notched full-period drums under 24 (MEDIUM)
Object 11 f1611/1613 at nu=15. Same 24-floor class as plain drums.

### V5 — Object 2 endcap / notch corners (MEDIUM)
Large sparse n-gons on flat gear faces; jagged notch corners.

## Verification protocol
After each fix: remesh CAD profile → re-screenshot affected object(s) →
update this file + OBJECT_VERIFY.md. Keep validity green.
