# M1 exact occurrence account evidence - 2026-07-17

## Proven increment

Commit `33ceff6` adds an authoritative `TopologyAccount` beside the temporary
map-based application compatibility view. It records:

- XDE assembly definitions and every expanded instance with deterministic
  label identity, parentage, local/world transforms, and exact topology roots;
- recursive compound, compsolid, solid, shell, face, wire, edge, and vertex
  occurrences with exact located shapes, orientation, tolerance, transforms,
  parent/children, instance ownership, and canonical underlying identity;
- one ordered coedge per wire edge use, including owning face/instance and only
  stored p-curve representation references;
- exact identity correspondence between source and working occurrences;
- repair-certificate coverage for assemblies, instances, occurrences, coedges,
  and source/working occurrence correspondence.

Exact XDE leaf uses replace the earlier body-only ownership route. There is no
solid-ID or nearest-geometry fallback. This accounts free compounds, wires,
edges, and vertices and causes an unmapped repaired/deep working copy to refuse
meshing rather than claim provenance.

Lookup indices use exact OCCT `IsSame` shape identity, TShape identity, and
stable-ID maps; stable ordinals still come only from deterministic traversal.
The geometry evaluator receives a deliberately stripped lookup snapshot so it
does not duplicate the authoritative occurrence account on large models.

## Fixture proof

The nested repeated-box XDE fixture proves:

```text
assembly definitions       2
expanded instances         8
aggregate topology roots   1
compound occurrences       1   (1 canonical)
solid occurrences          6   (1 canonical)
shell occurrences          6   (1 canonical)
face occurrences          36   (6 canonical)
wire occurrences          36   (6 canonical)
edge occurrences         144  (12 canonical)
vertex occurrences       288   (8 canonical)
coedge uses              144
total occurrences        517
```

The six repeated/co-located leaves resolve to six distinct occurrence owners
while sharing the correct definition-level canonical identities. The generated
`solid.fillet.junction_t_y_x.step` fixture proves exact instance roots for XDE
leaves with no solid IDs. A dedicated two-box XDE document proves two free
top-level part instances parented by the model, each with its own exact root.

All 77 generated STEP fixtures retain conservative identity and complete
source/working occurrence correspondence. The existing accounting result stays
59 meshable, 18 inspectable-only, and 1,559 classified edge/face subjects.

## Independent negative proof

`validateTopologyAccount` reports `expected`, `checked`, `skipped`, and
`failed` for assemblies, instances, occurrences, and coedges. Tests mutate a
valid account and require these named failures:

- `topology.occurrence_account.empty`;
- `topology.occurrence.id_duplicate`;
- `topology.occurrence.shape_missing`;
- `topology.coedge.wire_coverage_invalid`;
- `topology.instance.topology_roots_missing`.

The flat cylinder legitimately reports zero expected assembly and instance
records while retaining non-zero, complete occurrence/coedge/correspondence
coverage. This distinguishes an absent evidence class from a vacuous validator.

The explicit compatibility profile retains a valid source and working topology
account for the flat cylinder but fails exact source/working occurrence
correspondence after the geometry-deep copy. It is non-meshable and emits
`import.topology_correspondence.incomplete` until a complete modified-occurrence
certificate exists.

## Windows commands and outcomes

Run from `D:\Weft` on branch `WIP/secure-core-rewrite`:

```text
cmake --build build\m0-windows-clean-20260717 --config Release -- /m:1 /nr:false
ctest --test-dir build\m0-windows-clean-20260717 -C Release --output-on-failure --no-tests=error
cmake --preset vs2022-static-analysis
cmake --build --preset vs2022-static-analysis --config Release -- /m:1 /nr:false
ctest --preset vs2022-static-analysis -C Release --output-on-failure --no-tests=error
git diff --check
```

Outcomes:

- strict MSVC compiled the core, CLI, desktop app, and all tests;
- strict CTest passed 14/14 in 8.74 seconds;
- MSVC `/analyze` compiled the same complete product with warnings as errors;
- analyzer-lane CTest passed 14/14 in 8.61 seconds;
- `git diff --check` passed;
- no Linux build was attempted or changed in this increment, following the
  Windows-first development priority.

## Status boundary

This closes the exact identity occurrence-account slice of M1 and removes the
"total occurrence hierarchy" dependency from M2. M1 remains `IN_PROGRESS`:
bounded conservative repair operations and history-backed non-identity
occurrence correspondence are not yet implemented, and no compatibility repair
may become meshable without those proofs. This evidence does not pass M1, M2,
or M5 as a whole.
