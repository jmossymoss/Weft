# M1 multi-body face-adjacency orientation repair evidence - 2026-07-17

## Proven increment

Face-adjacency orientation repair now applies to each already-invalid closed
solid under a solid, compound, or compsolid working root. Valid sibling solids
remain untouched. Compound/compsolid roots are rebound through the exact
derivation map after a solid is replaced so correspondence stays complete.

## Witness

A temporary compound of
`corrupt.orientation.inverted_shell_face.brep` and
`baseline.pathology.box.brep` imports as non-identity, working-valid, and
meshable with `repair.orientation_face_adjacency` and shared-geometry
immutability evidence. The single-solid PAT-009 witness remains green.

## Windows commands and outcomes

```text
cmake --build build\vs2022 --config Release --target weft_secure_core_tests -- /m:1 /nr:false
ctest --test-dir build\vs2022 -C Release -R "^secure_core$" --output-on-failure --no-tests=error
```

Outcomes:

- `secure_core` passed after the multi-body install/rebind fix;
- open shells, nested assemblies, and sewing remain out of scope.
