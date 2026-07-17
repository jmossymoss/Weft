# M1 face-adjacency orientation repair evidence - 2026-07-17

## Proven increment

This working tree adds the second bounded conservative repair. After the
topology-isolated working copy and optional SameParameter/SameRange pass, an
already-invalid single closed root solid may be repaired by:

- absorbing a reversed shell occurrence into solid-relative face orientations;
- proving two-manifold face-edge adjacency with opposite shared-edge traversal;
- rebuilding only shell/solid occurrence topology with solved face orientations;
- applying independent polarity (positive finite volume, infinite point out);
- committing only when `BRepCheck_Analyzer` reports the rebuilt solid valid.

`BRepLib::OrientClosedSolid` is never used as the sole fix. Geometry handles,
tolerances, stored p-curves, and topology cardinality remain unchanged. Face
TShapes stay partners; shell/solid TShapes rebind through the exact derivation
map. Occurrence correspondence may differ in orientation while remaining
one-to-one `Modified`.

## Positive and refusal witnesses

The reviewed native fixture
`corrupt.orientation.inverted_shell_face.brep` proves:

- immutable source remains invalid;
- working becomes valid and meshable;
- non-empty `OrientationChange` evidence and
  `repair.orientation_face_adjacency`;
- complete non-vacuous `repair.orientation_reconciliation`;
- zero tolerance, representation, and topology-cardinality changes;
- complete one-to-one occurrence correspondence.

Adversarial lanes prove:

- a tampered `manifoldEdgesChecked=0` certificate is non-meshable by
  `import.repair.validation_incomplete`;
- `corrupt.wire.inconsistent_orientation.brep` is not claimed by this repair;
- non-orientation edge pathologies keep identity correspondence when a solver
  proposal would not produce a valid solid.

## Windows commands and outcomes

Run from `D:\Weft` on branch `WIP/secure-core-rewrite`:

```text
cmake --build build\vs2022 --config Release --target weft_secure_core_tests -- /m:1 /nr:false
ctest --test-dir build\vs2022 -C Release --output-on-failure --no-tests=error
cmake --build --preset vs2022-static-analysis --config Release -- /m:1 /nr:false
ctest --preset vs2022-static-analysis -C Release --output-on-failure --no-tests=error
git diff --check
```

Outcomes:

- strict MSVC compiled the changed core and test targets;
- strict CTest passed 14/14 in 8.75 seconds;
- MSVC `/analyze` rebuilt the complete product with warnings as errors;
- analyzer-lane CTest passed 14/14 in 8.85 seconds;
- `git diff --check` reported only CRLF normalization warnings on touched
  sources;
- no Linux build, preset, or source change was attempted.

## Status boundary

This closes the PAT-009 face-adjacency orientation slice of M1. It does not
authorize whole-solid reversal, repair multi-body compounds, open shells,
INTERNAL/EXTERNAL policy, or wire-only orientation defects. BR-015 remains
enforced. M1 remains `IN_PROGRESS`.
