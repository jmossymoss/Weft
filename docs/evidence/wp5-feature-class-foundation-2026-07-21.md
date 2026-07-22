# WP5 — feature-class planning foundation (reopened, 2026-07-22)

AD-5 / WP5 exit: classify B-rep faces once in `analyze()`, route / densify /
self-heal from those facts, expose class+chart for debugging.

Reopened after the 2026-07-21 advance left FilletStrip/Drum early rows empty
and the continuity test only proved drum+cap (not drum+blend).

## Exit checklist

| Criterion | Evidence |
| --- | --- |
| Sphere tip / dimple / foam / teleporter green under same rules | `testBulletTipNotContractFloor`, `testSphereDimpleNotContractFloor`, foam+teleporter CAD validate |
| Fillet strip, hole plate, drum zoo + counterexample | `testFillet` asserts FilletStrip→Coons; `bossfillet` strip→coons + shared circ |
| Cylindrical stack continuity + pin exception | `testCylindricalStackContinuity` (cylinder drum+cap; bossfillet drum+fillet+boss; pin sticks) |
| No rediscovery of sphere chart / fillet narrowness in planFace | analyze-owned; planning uses `featureClass` for fillet authority |
| inspect / signature expose feature+chart | CLI inspect; `face.N.feature` / `.chart` in signature test |
| AD-3: no new MesherKind | unchanged enum |
| Parked MP9 visuals | `tests/KNOWN_RED.tsv` WP6 rows; #1805 reducer 83→73 unexplained, 0 folds |

## Landed

- `FaceInfo`: `chartKind`, `featureClass`, `loop`, `priority`
- analyze(): sphere UV chart, narrow fillet vs wide false-fillet drum, loop signature
- Early `featureClass` table: SphereCap, Drum×FullPeriod→revolution, FilletStrip→coons,
  BossJunction, HolePlate, PlanarPanel, Freeform→dome only (early ribbon/rail opened
  foam — kept in late ladder)
- Fillet authority from `FeatureClass::FilletStrip` (not dual `isFillet` signal)
- Closed-revolution loft excludes FilletStrip
- Drum open-band from class×chart; sliver fillet + fold heal class-scoped
- Adjacency-limited stack continuity; pin blocks → `densityConflicts` reason `stack-continuity-pin`
- Report / signature / inspect / app roster

## Rejected / still open inside WP5

- Early Freeform ribbon/rail: opened foam (87 ribbons) — late ladder only
- Body-wide radius-binned cylindrical continuity: broke foam/teleporter — adjacency-limited kept
- FreeformComb residual #1805 cracks: stitch deadlock repair landed (comb↔floor
  + comb→MinimalNGon inserts, border sample snap, second fuse+stitch,
  freer comb twin fuse). Reducer unexplained 83→73 with 0 folds; residual
  T-junction topology on open-shell extracts stays `KNOWN_RED` / WP6
- Early Freeform ribbon/rail: rejected (foam 87 ribbons) — late ladder +
  `MP9_grip_freeform` WP6 park is enough for WP5 exit
- Stack pin: drum↔fillet perEdge pin sticks (allowed mismatch); raise also
  records `stack-continuity-pin` when a non-rim co-length edge is blocked

## Active package

**WP5 (reopened)** until fillet/drum table + drum+blend continuity exit rows are
green at one revision; then WP6.
