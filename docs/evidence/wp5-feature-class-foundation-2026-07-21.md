# WP5 — feature-class planning foundation (complete, 2026-07-21)

AD-5 / WP5 exit: classify B-rep faces once in `analyze()`, route / densify /
self-heal from those facts, expose class+chart for debugging.

## Exit checklist

| Criterion | Evidence |
| --- | --- |
| Sphere tip / dimple / foam / teleporter green under same rules | `testBulletTipNotContractFloor`, `testSphereDimpleNotContractFloor`, foam+teleporter in `testAllMesherStrategies` + corpus |
| Fillet strip, hole plate, drum zoo + counterexample | fillet + hole + boss fixtures; foam strips/drums; `bossfillet` in continuity test |
| Cylindrical stack continuity + pin exception | `testCylindricalStackContinuity` |
| No rediscovery of sphere chart / fillet narrowness in planFace | analyze-owned; planFace reads `FaceInfo` |
| inspect / signature expose feature+chart | CLI inspect; `face.N.feature` / `.chart` in signature test |
| AD-3: no new MesherKind | unchanged enum |
| Parked MP9 visuals | `tests/KNOWN_RED.tsv` WP6 rows |

## Landed

- `FaceInfo`: `chartKind`, `featureClass`, `loop`, `priority`
- analyze(): sphere UV chart, narrow fillet vs wide false-fillet drum, loop signature
- Early `featureClass` table: SphereCap, BossJunction, HolePlate, PlanarPanel, Freeform (dome/rail/ribbon)
- Drum open-band from class×chart; sliver fillet + fold heal class-scoped
- Adjacency-limited stack continuity; pin blocks → `densityConflicts` reason `stack-continuity-pin`
- Report / signature / inspect / app roster

## Active package

Advanced to **WP6 — validate real work**. Fresh Plasticity / MP9 visual sign-off
continues from parked KNOWN_RED rows using AD-5 class names, not face-id patches.
