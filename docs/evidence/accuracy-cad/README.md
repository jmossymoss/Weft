# Accuracy path — real CAD corpus (Medium)

## Hardening

1. Freeform UV cap (64×64) — stopped 256² blow-ups
2. Boundary-loop meshing for trimmed cylinders/freeform — shared `edgeSlot`
   verts only (no UV-grid staircases)
3. Full-period drums still emit rim-to-rim quads; open fillet arcs earclip in UV

## Validate opens (Medium) — after boundary fill

| model | status | open | slivers | nm | notes |
|---|---|---:|---:|---:|---|
| weft_cyl | PASS | 0 | 0 | 0 | drum quads |
| filletslot | PASS | 0 | 36 | 0 | fillet arcs earclipped |
| slitdrill | PASS | 0 | 2 | 0 | was 58 opens |
| angle1 | PASS | 0 | 16 | 0 | was 8–24 opens |
| bullet_tip_3728 | PASS | 0 | 18 | 0 | was 79 opens |
| 4pinplug | fail | 0 | 37 | 2 | opens cleared |
| as1_pe | fail | 0 | 4 | 786 | opens cleared; NM assembly |
| canrev | fail | 0 | 41 | 4 | opens cleared |
| Holes | fail | 254 | 175 | 0 | still leaky freeform/holes |
| foam | fail | 896 | 1597 | 45 | improved from ~9k–14k opens |

## Next

- Reduce earclip slivers on fillets (wire-ordered quad strips)
- Holes/foam residual opens
- Full MP9 when open rate on medium CAD is stable
