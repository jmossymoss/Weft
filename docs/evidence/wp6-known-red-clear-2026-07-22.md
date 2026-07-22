# WP6 — KNOWN_RED clear progress (2026-07-22)

## Cleared rows

| Case | Was | Fix class |
| --- | --- | --- |
| slitdrill cad/default `require_watertight` | CAD NM=2 from exact tangent pocket/bore | Fixture pocket cuts into bore (`x=25.5`); manifold closed solid |
| tan_slit cad/default `max_raw` | raw=1 on drum at multi-owner generator | Border-contract skips input NM edges; demote accepts floor when face touches input NM |
| tork cad/default `require_watertight` | open-shell broken source | `ValidationReport::brokenSource` + `formatReport` diagnostic (≥⅓ open-shell edges) |
| MP9_fillet_capsule `visual_class` | FilletStrip → RevolutionGrid at >24 edges | FilletStrip Coons budget 48/16; ortho fallback as Coons+`orthogonalFreeformComb` (0 unexplained on reducer) |
| MP9_grip_freeform `visual_class` | shallow Freeform → minimal n-gon | Freeform skips residual CAD minimal grab → Coons quad flow |
| MP9_bullet_body `visual_class` | tip/body rim mismatch folklore | `testBulletBodyTipRimContinuity`: tip quad-fill, body coons/ring-lattice, shared rim one count, 0 folds, watertight |

## Remaining

| Case | Observed | Notes |
| --- | --- | --- |
| MP9 `max_open_edges` | ~378–383 (ceiling 450) | #1805-family still leads; extract unexplained 73 |

## Reducers / tests

- `tests/regressions/mp9/fillet_capsule_iso_band.step` + `testMp9FilletCapsuleNotRevolution`
- `tests/regressions/mp9/grip_freeform_panels.step` + `testMp9GripFreeformCoons`
- `testBulletBodyTipRimContinuity` (same reducer as tip)
- `testTanSlitNoRawDemotion`, `testBrokenSourceDiagnostic`

## Sanity

foam / teleporter CAD remain watertight. No filename / face-ID specials.
