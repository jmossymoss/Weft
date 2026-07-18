# M3 repeated wire occurrences evidence - 2026-07-18

## Proven increment

Meshing coedges are projected from the authoritative `TopologyAccount` so every
face-owned wire use keeps its topology wire occurrence identity. Face and edge
IDs in the temporary ShapeMap meshing view remain evaluator indices. Shared
edges still contribute one canonical sample sequence with distinct coedge uses
and orientation variants.

`validateMeshingCompatibilityView` requires a one-to-one topology-face to
ShapeMap-face projection, complete face-owned wire/coedge coverage, and unique
meshing coedge IDs. Colocated `IsSame` compound children that collapse under
`TopExp::MapShapes` remain distinct in the topology account and refuse meshing
with `topology.meshing_view.face_alias_collapse`. Planar trim resolves the outer
wire through the unique topology face occurrence rather than a global wire map.

## Verification

Commands from `D:\Weft` at revision with this increment (working tree; prior
committed tip `a1718d6`):

```text
cmake --build --preset vs2022 --target weft_secure_core_tests weft_planar_trim_assembly_tests weft_canonical_boundary_tests weft_secure_meshing_tests --config Release
ctest --preset vs2022 -C Release -R "secure_core|planar_trim_assembly|canonical_boundary|secure_meshing" --output-on-failure
```

All four tests passed.

Fixtures and adversaries:

- single box: 6 distinct topology wire IDs in meshing coedges, 24 coedges, 12
  edges each used twice, at least one opposite-orientation shared edge, meshing
  view complete and meshable;
- two distinctly located boxes: 12 wires, 48 coedges, meshing view complete and
  meshable;
- colocated `IsSame` duplicate compound children: topology retains 2 solids / 12
  faces while ShapeMap has 6 faces; named `topology.meshing_view.face_alias_collapse`
  refusal; non-meshable;
- dropped meshing coedge: `topology.meshing_view.coedge_count_mismatch`.

## Status boundary

WP-011 is closed. M3 remains open for critical segmentation, template sum
consumers, coupled/aliased sums, and Linux boundary determinism.
