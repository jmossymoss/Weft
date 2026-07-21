# WP3 — protect contract-exact MinimalNGon from stitch insertions (2026-07-20)

## Class

`planar_ngon_border_contract` (teleporter).

## Root cause

High-degree planar `minimal-ngon` faces already emit the shared border
contract. When `stitchSeams` runs (armed globally by any
`orthogonalTrimGrid` plan), pitch-scaled `fullEdge` chords on those n-gons
admit foreign fillet/pad corner verts. Insertions rewrite the authority
polygon and leave unexplained opens on the plane.

Conform exclusion of successful MinimalNGon (`isFreeform`) and the shared
`n=1` zero-solved floor in `samplePlanarRings` are correct but insufficient:
preview / `WEFT_NO_STITCH` leave the plane sealed; full finalize opens it.

## Fix (class-level)

In `stitchSeams` (`core/src/meshers.cpp`):

- Pass `plans` + `fellBack`.
- Skip insertion into faces with `kind == MinimalNGon` and
  `fellBack == 0` or `2`.
- Neighbors may still stitch toward the plane; the n-gon is not rewritten.

No filename, model-name, or face-ID special cases.

## Metrics

| model | profile | before | after |
| --- | --- | --- | --- |
| teleporter | cad | open 12, NM 0 | watertight (0 / 0) |
| teleporter | default | open 18, NM 2; leakiest plane | open 14, NM 2; plane off leak list |
| foam | default / cad | unchanged | unchanged (`plane_fillet_bspline_junction`) |

Default residual opens concentrate on revolution / pad neighbors
(`bspline_contract_floor_overweld` secondary class in
`tests/RELEASE_FAILURE_CLASSES.tsv`).
