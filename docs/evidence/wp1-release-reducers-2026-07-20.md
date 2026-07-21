# WP1 evidence — release watertight reducer capture (2026-07-20)

## Goal

Capture reduced reproducers for the current release known-red watertight
failures (`foam`, `teleporter` in `tests/KNOWN_RED.tsv`) before any mesher or
routing change. Meshers were not modified.

## Full-model reproduction (`weft mesh --validate`)

| model | profile | open edges | non-manifold | failure class |
| --- | --- | --- | --- | --- |
| `STEP_Examples/foam.stp` | default | 6 | 2 | plane / fillet / B-spline junction open + NM |
| `STEP_Examples/foam.stp` | cad | 12 | 2 | same + fillet / B-spline cluster |
| `STEP_Examples/teleporter.stp` | default | 18 | 2 | multi-neighbor planar n-gon border opens (+ NM) |
| `STEP_Examples/teleporter.stp` | cad | 12 | 0 | multi-neighbor planar n-gon border opens |

Source B-reps are closed solids (`inputBoundaryEdges = 0`); all reported open
edges are unexplained mesh cracks.

### Foam class (not a face-ID fix)

Leak attribution concentrates on a plane meeting a fillet strip and B-spline
patches (mesher plan: `minimal-ngon` + `coons-grid`, with a `revolution-grid`
fillet in the CAD-profile cluster). Failure class name:
`plane_fillet_bspline_junction` (open edges + non-manifold).

### Teleporter class (not a face-ID fix)

Leak attribution concentrates on a large multi-neighbor planar face and its
planar pads / cylinder junctions (mesher plan: `minimal-ngon` +
`revolution-grid`). Failure class name: `planar_ngon_border_contract`
(open edges; NM on default profile only).

## `weft extract` neighborhood attempts

`weft extract <in> --faces … --rings N -o <out.step>` writes a face compound
(open shell). Neighborhoods seeded from the leakiest closed-solid faces:

| extract | faces (approx) | rings | mesh result |
| --- | --- | --- | --- |
| `regressions/release/foam_plane_fillet_bspline_r1.step` | 11 | 1 | all opens on cut boundary; 0 unexplained cracks; 0 NM |
| `regressions/release/foam_fillet_bspline_r1.step` | 11 | 1 | all opens on cut boundary; 0 unexplained cracks; 0 NM |
| `regressions/release/teleporter_planar_ngon_r1.step` | 25 | 1 | all opens on cut boundary; 0 unexplained cracks; 0 NM |

Larger rings (≥2) either still show 0 unexplained cracks while inventing
non-manifold edges on the open shell, or hang / explode cost on teleporter.
Isolating the seed faces removes the closed-solid interior-border context that
produces the real cracks, so the extract path cannot currently shrink the
failure to a smaller **closed solid**.

## Conclusion

- Authoritative closed-solid reproducers remain the full release STEP files.
- Extracted neighborhoods under `tests/regressions/release/` are open-shell
  diagnostics for the local surface stacks only; they do not stand in for the
  watertight known-red until a closed neighborhood (or fixture) can reproduce
  unexplained opens / NM.
- Corpus rows: `tier=regression`, `layer=release`, `require_watertight=0`
  matching known-red. Closed-solid rows point at the full models; extract rows
  use `validity=open`.
- No mesher or routing changes in this revision.
