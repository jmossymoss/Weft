# WP5 — feature-class planning foundation (2026-07-21)

AD-5 / WP5 slice: classify B-rep faces once in `analyze()`, route and report
from those facts, stop rediscovering sphere charts inside `planFace`.

## Landed

| Item | Status |
| --- | --- |
| `ChartKind` / `FeatureClass` / `LoopSignature` / `priority` on `FaceInfo` | Done |
| Sphere UV pole chart moved into `analyze()` | Done |
| `planFace` sphere path reads `SphereCap` × chart (pole/full-period vs geometric-cap) | Done |
| Fold self-heal sphere rescue gated on pole/full-period `SphereCap` only | Done |
| Report + topology signature `face.N.feature` / `face.N.chart` | Done |
| CLI `inspect` + app selection roster show class/chart | Done |
| Tests: `testFeatureClassAnalyze`, `testCylindricalStackContinuity`, tip/dimple asserts | Done |
| Body-wide post-density continuity raise | Deferred — over-coupled foam/teleporter; keep AD-5 rule + existing drum/blend equalize |
| Full `planFace` ladder → pure priority table | Partial — sphere class wired; other classes still use existing ladder reading `isFillet` / surface type |

## Counterexamples kept green

- `bullet_tip_3728.step` → `sphere-cap` / `geometric-cap` → `quad-fill`
- `sphere_dimple_annulus.step` → `sphere-cap` / `pole` → `revolution-grid`
- foam + teleporter CAD/default → watertight
- cylinder fixture → one shared circumferential rim count; per-edge pin may diverge

## Follow-up (same WP5)

1. Class-keyed cylindrical continuity that does not open sealed corpus models.
2. Replace remaining `planFace` rediscovery (fillet-strip narrowness, hole/boss)
   with `FaceInfo` fields only.
3. Shrink auto ladder toward `featureClass × chartKind → MesherKind` table.