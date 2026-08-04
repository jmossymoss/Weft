# Accuracy path — real CAD corpus (Medium)

Primitive fixtures alone are not enough. Corpus + follow-up hardening:

## Hardening landed

1. **Freeform UV cap** — max 64×64, sag-only refine (angle thrash removed).
   Was ~65k tris/face (`canrev`/`Holes`/`bullet_tip`); now ≤8k tris/face.
2. **Analytic rim seeding** — shared `edgeSlot` samples mapped onto UV grid
   nodes (parametric), not loose 3D nearest-neighbour.

## Latest key-model check

| model | validate | verts | quads | tris | open edges | slivers |
|---|---|---:|---:|---:|---:|---:|
| weft_cyl | PASS | 36 | 18 | 0 | 0 | 0 |
| filletslot | PASS | 56 | 28 | 0 | 0 | 0 |
| angle1 | fail | 186 | 89 | 0 | 8 | 0 |
| slitdrill | fail | 127 | 62 | 0 | 58 | 0 |
| 4pinplug | fail | 510 | 295 | 0 | 106 | 0 |
| as1_pe | fail | 2668 | 1718 | 0 | 0 | 4 |
| canrev | fail | 4232 | 0 | 8192 | 204 | 0 |
| Holes | fail | 6541 | 1496 | 8233 | 332 | 5589 |
| foam | fail | 30177 | 4118 | 40098 | 9173 | 562 |
| bullet_tip_3728 | fail | 1136 | 45 | 2048 | 79 | 480 |

## Still open

- Residual open borders on multi-cylinder parts (slitdrill / 4pinplug)
- Freeform still emits tris; slivers on large freeform patches
- Full `MP9.stp` integration pass after open-border rate drops further

CSV from first wide pass: `corpus_accuracy.csv`
