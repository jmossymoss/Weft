# Independent mesh: MP9 (`mp9_Edited.stp`)

Release `weft mesh --independent --profile cad --validate`.
3052 faces / 8406 edges.

MP9 is not a T0 coverage oracle. These numbers are A/B evidence only.

## Revisions

| | OCCT-snap (prior) | shared-sample UV fill | drum loft (no UV fan) | exact-sample UV web |
| --- | --- | --- | --- | --- |
| meshIndependent | ~18 s | 8985 ms | 7853 ms | 7290 ms |
| vertices | 69569 | 28859 | 39220 | 30586 |
| polygons | 86425 | 33680 (9457 quads, 23435 tris, 788 n-gons) | 47523 (13553 quads, 33182 tris, 788 n-gons) | 36617 (9949 quads, 25880 tris, 788 n-gons) |
| unexplained opens | 19677 | 2413 | 8053 | 2509 |
| non-manifold | 153 | 195 | 142 | 161 |
| folded | 2048 | 3954 | 2179 | 965 |
| winding conflicts | — | — | 775 | 63 |
| empty faces | 3 | 3 (`557`, `1371`, `1372`) | 3 (same) | 3 (same) |
| contract-floor (OCCT) | — | 3 | 75 | 3 |

Mesher mix at exact-sample UV web: 1382 minimal-ngon, 910 plate-web, 675 coons-grid, 82 revolution-grid, 3 contract-floor.

## Shared class (not an MP9 special)

Drum-loft + “no fan on ear-clip failure” sent trimmed drums to OCCT. Face 133 (cylinder, 107 edges, many hole wires) owned 551 opens. Neighbors lost the shared samples.

Fix: Delaunay-web the exact UV sample loops; non-crossing hole bridges; force-clip only unholed rings; loft/Coons refuse unequal opposite counts. Face 133 is now plate-web (279 tris) and its 1-ring extract has 0 unexplained opens.

## What remains (T4 / later)

- ~2509 unexplained opens, similar to the UV-fill baseline. Leakiest faces now top out at 32 (plates `#561`/`#563`, 3-edge drum `#1768`).
- Torus fillet rings `#292`/`#293` still ear-clip to slivers (100 tris, folds) instead of an across-span lattice.
- Two-edge planar slivers `#557`, `#1371`, `#1372` still emit nothing.
- Chord max ~16 on some UV triangles. Numeric watertightness of this path is not the T0 gate.

Zoo still closed at CAD defaults after the fill change: cylinder, torus (1024 quads), ellipse_plate, bezier_slab, slotted, hole, fillet.

Default `weft mesh` still calls `generate()`.
