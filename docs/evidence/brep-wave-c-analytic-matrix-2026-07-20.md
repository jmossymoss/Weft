# Wave C — cone / sphere / torus consumer matrix lock — 2026-07-20

## Goal

Lock cone apex/frustum, sphere CapWall/band UV-trim, and torus wall including
`face_116_torus.step` under matrix ctests (not fixture-only preview LOD).
No full-body MP9. Preserve Wave 0 CapWall refuse-not-drop.

## Batteries

| Marker | Host | Cases | Outcome |
|---|---|---|---|
| `WEFT_CONE_MATRIX` | `cone_template` / `testConeMatrix` | `cone_apex.step`, `cone_frustum.step`, frustum fixture | HARD |
| `WEFT_SPHERE_MATRIX` | `sphere_template` / `testSphereMatrix` | full wall fixture, `sphere_cap_778.step`, `sphere_cap_complex.step` | HARD |
| `WEFT_TORUS_MATRIX` | `torus_template` / `testTorusMatrix` | full wall fixture, `face_116_torus.step` | HARD |

Contracts: `omitDeferredResiduals=false`; non-vacuous
`certified.triangle_intersection`; CapWall refuses geometric ·N minorities
then hard UV-trim (`windingsMatch=1 relax=0`); `face_116` in torus matrix;
apex locked via single-face `cone_apex.step` (multi-face apex solid may be
host-non-meshable under OCCT 8.x — skipped, not soft-admitted).

CLI spot-check (fail-closed):

| Extract | EXIT | Notes |
|---|---|---|
| `cone_apex.step` | 0 | `WEFT_G4_CONE_UVTRIM_ORIENT … windingsMatch=1 relax=0` |
| `cone_frustum.step` | 0 | structured frustum band quads |
| `sphere_cap_778.step` | 0 | CapWall refuse → UV-trim hard |
| `face_116_torus.step` | 0 | `WEFT_G4_TORUS_WALL_ORIENT … windingsMatch=1 relax=0` |

Authority: `docs/governance/brep-consumer-matrix.md` Wave C rows all HARD.

## Commands

```
cmake --build --preset vs2022 --target weft_cone_template_tests `
  weft_sphere_template_tests weft_torus_template_tests `
  weft_brep_consumer_matrix_tests
ctest --preset vs2022 -R "cone_template|sphere_template|torus_template|brep_consumer_matrix" --output-on-failure
```

Result: 4/4 PASS.

Marker lines:

```
WEFT_CONE_MATRIX locked=3/3 fail_closed=1 omit=0 apex_uvtrim_hardOrient=1 frustum_band=1
WEFT_SPHERE_MATRIX locked=3/3 fail_closed=1 omit=0 capwall_refuse_not_drop=1 uvtrim_hardOrient=1
WEFT_TORUS_MATRIX locked=2/2 fail_closed=1 omit=0 face_116_in_matrix=1 not_fixture_only_lod=1
```

Authority: `docs/governance/brep-consumer-matrix.md` Wave C rows all HARD.
Scaffold `brep_consumer_matrix` presence checks still green (`cone_apex.step` added).

## Out of scope

Full MP9 body; Wave B cylinder_template; Wave D mapped; CapWall soft-drop;
multi-face apex solid import_not_meshable host debt (skipped, not soft-fixed).
