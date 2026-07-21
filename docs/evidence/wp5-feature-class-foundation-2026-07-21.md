# WP5 — feature-class planning foundation (2026-07-21)

AD-5 / WP5: classify B-rep faces once in `analyze()`, route and report from
those facts, stop rediscovering geometry inside every `planFace` escape hatch.

## Landed

| Item | Status |
| --- | --- |
| `ChartKind` / `FeatureClass` / `LoopSignature` / `priority` on `FaceInfo` | Done |
| Sphere UV pole chart in `analyze()` | Done |
| Narrow fillet vs wide false-fillet drum in `analyze()` | Done |
| Early `featureClass` table: BossJunction / HolePlate / PlanarPanel | Done |
| Sphere routing + fold heal from class | Done |
| Open-band drum gate uses `Drum` × chart (not live narrowness probe) | Done |
| Adjacency-limited cylindrical stack continuity (shared drum rim → co-length neighbor seams that also touch Drum/FilletStrip) | Done |
| Report + signature `face.N.feature` / `.chart`; inspect + roster | Done |
| Tests: feature-class analyze (incl. foam strip/drum), tip/dimple, stack continuity | Done |
| Full ladder → pure table for Freeform/Coons/Ribbon | Partial — capability probes remain |

## Counterexamples kept green

- `bullet_tip_3728.step` → `sphere-cap` / `geometric-cap` → `quad-fill`
- `sphere_dimple_annulus.step` → `sphere-cap` / `pole` → `revolution-grid`
- foam + teleporter CAD/default → watertight
- cylinder fixture → one shared circumferential count; per-edge pin may diverge
- fillet fixture → ≥1 `fillet-strip`; foam → both strips and drums

## Rejected approaches

- Solid-wide radius-bin continuity raise — opened foam/teleporter.
- BFS through every stack-class face in a solid — same failure mode.

## Remaining WP5 (optional polish)

- More Freeform early routes (dome/ribbon) keyed by class without new MesherKinds.
- Report stack continuity conflicts in density attribution when a pin blocks a raise.
