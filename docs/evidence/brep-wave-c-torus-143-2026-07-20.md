# Wave C — torus UV-degenerate densify (`face_143_torus`) — 2026-07-20

## Gap

MP9 fail-closed stopped at face 143 with
`torus.uv_trim_orientation_unresolved` after wall refused
`torus.triangle_uv_degenerate` and UV-trim invert/coarsen could not
hard-orient (`uvAgainst≠0`).

Extract: `tests/fixtures/mp9_extracts/face_143_torus.step`
(`full_periodic_with_cap_boundaries`, `u_periodic`, `v_periodic`, 4 circles).

## Cause

Wall densify retry only armed on seam/chord/normal codes. UV-degenerate
walls skipped densify and fell through to UV-trim, which still left
against-N ears after invert + coarsen.

## Fix (general — not `faceId==143`)

Include `torus.triangle_uv_degenerate` in the torus wall densify retry
set (same major/minor doubling as face 115 orient densify).

## Proof (vs2022 Release)

```
weft mesh tests/fixtures/mp9_extracts/face_143_torus.step -o build/t143.obj
WEFT_G4_TORUS_DENSIFY … after=torus.triangle_uv_degenerate (×2)
WEFT_G4_TORUS_WALL_ORIENT tris=4608 uvWith=4608 uvAgainst=0 windingsMatch=1 relax=0
→ EXIT 0
```

## Matrix

`WEFT_TORUS_MATRIX` includes `face_143_torus.step`.
