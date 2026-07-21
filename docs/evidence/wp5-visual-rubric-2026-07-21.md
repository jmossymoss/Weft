# WP5 — target-asset visual rubric (Weft viewport, 2026-07-21)

Section 7 review of the release hard-surface set after WP4 close. Plasticity
side-by-side Blender compare still blocked on missing Plasticity mesh
exports for these STEPs.

## Method

```sh
tools/visual_check.sh <release-step-dir> docs/evidence/wp5-visual
```

CLI mesh uses `--profile cad --validate`; screenshots via `weft_app`
`--finalize` (same path as WP3 visual evidence).

## Results

| Model | Watertight | Degenerate / slivers | Viewport |
| --- | --- | --- | --- |
| flaregun | yes | 0 / 34 slivers | `wp5-visual/flaregun_v*.png` |
| foam | yes | 5 / 177 slivers | `wp5-visual/foam_v*.png` |
| iso14649-demo | yes | 0 / 9 slivers | `wp5-visual/iso14649-demo_v*.png` |
| teleporter | yes | 1 / 228 slivers | `wp5-visual/teleporter_v*.png` |

Report: `docs/evidence/wp5-visual/report.html`.

## Rubric notes

- Solids read as closed and editable; no raw-triangulation takeover on these
  four.
- Sliver counts on foam/teleporter remain a visual/editability debt (not a
  watertightness failure). Track for WP5/WP6 polish, not as a new open-edge
  class.
- Without Plasticity comparison OBJs, “useful advantage vs Plasticity
  export” cannot be scored yet. Need artist exports of the same STEP
  sessions into Blender for the remaining exit item.

## Fresh Plasticity set gap

In-repo target assets are `tests/STEP_Examples/{foam,teleporter,flaregun,
iso14649-demo,MP9,…}`. No versioned private Plasticity export bundle is
present. To finish WP5:

1. Drop current Plasticity STEP (+ optional Plasticity mesh) into a
   documented fresh-set folder (or extend STEP_Examples with provenance).
2. Re-run release geometry + workflow gates.
3. Blender compare Weft finalized OBJ vs Plasticity mesh per section 7.
