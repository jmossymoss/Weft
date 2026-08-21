# Independent mesh: MP9 (`mp9_Edited.stp`)

Release `weft mesh --independent --profile cad --validate`.
3052 faces / 8406 edges. Commit series on `cursor/independent-tessellate-b00c`.

| | OCCT-snap fallback (prior) | shared-sample UV fill |
| --- | --- | --- |
| time | ~18 s | 8985 ms |
| vertices | 69569 | 28859 |
| polygons | 86425 | 33680 (9457 quads, 23435 tris, 788 n-gons) |
| unexplained open edges | 19677 | 2413 |
| non-manifold | 153 | 195 |
| folded | 2048 | 3954 |
| empty faces | 3 | 3 (`557`, `1371`, `1372` — 2-edge planar slivers) |

Mesher mix: 1382 minimal-ngon, 900 coons-grid, 691 plate-web, 76 revolution-grid, 3 contract-floor.

Zoo (`ctest -R independent_mesh`): box, cylinder, hole, fillet, sphere closed.

Default `weft mesh` still calls `generate()`.
