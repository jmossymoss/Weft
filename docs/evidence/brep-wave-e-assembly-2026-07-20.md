# Wave E — Assembly certificate — 2026-07-20

## Goal

Per-solid closed-manifold when the B-rep shell is closed; incidence /
orientation (edge winding) / triangle-intersection coverage non-vacuous;
unit adversaries still refuse (orientation / position tamper). Light Track M:
distinguish within-solid leaks vs inter-solid free borders when multi-solid.

## Product changes

- `core/src/certified_mesh.cpp`: after global incidence/Euler, each closed
  solid shell (every non-degenerate edge used by ≥2 faces inside the solid)
  re-validates closed-manifold on its compacted triangle subset.
- `tests/test_secure_meshing.cpp`: `WEFT_ASSEMBLY_MATRIX` locks on box
  (incidence + winding + intersection + euler) and a two-component closed
  mesh (C=2, boundaryEdges=0). Free-solid compound BREP import remains
  non-meshable today (`repair.shared_geometry_immutable`) — not in this wave.
- `app/main.cpp` (Track M light): per-solid open-edge counts; UI names
  within-solid leaks vs inter-solid free borders / coincident skins.
- Matrix Wave E rows → HARD (adversaries remain REFUSE).

Preserved: Wave 0 `hardOrient*`, Wave F refuse codes, Wave D revolution UV
promote. No MP9 full-body gate.

## Commands

```
cmake --build --preset vs2022 --target weft_core weft_certified_mesh_tests `
  weft_secure_meshing_tests weft_brep_consumer_matrix_tests -j 8
ctest --preset vs2022 -R "certified_mesh|secure_meshing|brep_consumer_matrix" --output-on-failure
```

## Results (vs2022 Release)

```
1/3 Test  #8: certified_mesh ......... Passed
2/3 Test #14: secure_meshing ......... Passed
3/3 Test #15: brep_consumer_matrix ... Passed
100% tests passed, 0 tests failed
```

Markers include `WEFT_G5 box closed-manifold`, `WEFT_ASSEMBLY_MATRIX` closed_solid_manifold
(incidence/winding/intersection/euler), and multi_solid_closed (C=2).
certified_mesh still refuses orientation/position tamper adversaries.

## Authority

Matrix: `docs/governance/brep-consumer-matrix.md` Wave E.
Prior G5 note: `docs/evidence/mp9-g5-hard-assembly-2026-07-20.md`.
