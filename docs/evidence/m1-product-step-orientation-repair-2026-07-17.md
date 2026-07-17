# M1 product STEP orientation repair witness evidence - 2026-07-17

## Proven increment

A processing-disabled STEP round-trip of the reviewed native orientation
pathology `corrupt.orientation.inverted_shell_face.brep` retains an invalid
source and becomes meshable through
`repair.orientation_face_adjacency` under `importStepSecure`. SameParameter
flag pathologies do not survive STEP export/import, so orientation is the
first product-format repair witness beyond the generated identity corpus.

## Witness

`testProductStepOrientationRepairWitness` in `tests/test_secure_core.cpp`
writes the native inverted-shell solid to a temporary STEP file, imports it
conservatively, and asserts non-identity meshable orientation repair.

## Windows commands and outcomes

```text
cmake --build build\vs2022 --config Release --target weft_secure_core_tests -- /m:1 /nr:false
ctest --test-dir build\vs2022 -C Release -R "^secure_core$" --output-on-failure --no-tests=error
```

Outcomes:

- `secure_core` passed with the product STEP orientation witness.
