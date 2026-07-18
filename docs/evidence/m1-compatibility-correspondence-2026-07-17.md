# M1 compatibility correspondence evidence - 2026-07-17

## Proven increment

Compatibility imports bind a full TShape derivation map from the
geometry-deep copy. Already-valid copies skip sew/ShapeFix so occurrence
StableIds stay aligned. Invalid copies run the historical heal pipeline and
rebind faces/edges through history, wires through face-local pairing, shells
through sole-shell demotion, and vertices through nearest-neighbour snap.
Topology assembly comparison allows p-curve representation deltas when the
account is not representation-identical.

## Witnesses

- generated cylinder STEP under Compatibility: correspondence complete,
  meshable, non-identity;
- native box B-rep under Compatibility: correspondence complete and meshable;
- `corrupt.orientation.inverted_shell_face.brep` under Compatibility: heal
  lane (`historical` pipeline detail), correspondence complete, working
  valid, meshable.

## Windows commands and outcomes

```text
cmake --build build\vs2022 --config Release --target weft_secure_core_tests -- /m:4 /p:TreatWarningAsError=false
.\build\vs2022\bin\Release\weft_secure_core_tests.exe
ctest --test-dir build\vs2022 -C Release --output-on-failure --no-tests=error
```

Outcomes:

- `weft_secure_core_tests` reports `secure-core contract checks passed` with
  both valid-copy and heal-lane Compatibility correspondence complete.
- strict MSVC CTest passed 14/14 in 8.44 seconds.
