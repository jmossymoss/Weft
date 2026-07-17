# M1 compatibility correspondence evidence - 2026-07-17

## Proven increment

Compatibility imports now bind a full TShape derivation map from the
geometry-deep copy. Already-valid copies skip sew/ShapeFix so occurrence
StableIds stay aligned. Topology assembly comparison allows p-curve
representation deltas when the account is not representation-identical.

## Witnesses

- generated cylinder STEP under Compatibility: correspondence complete,
  meshable, non-identity;
- native box B-rep under Compatibility: correspondence complete and meshable.

## Windows commands and outcomes

```text
cmake --build build\vs2022 --config Release --target weft_secure_core_tests -- /m:1 /nr:false
ctest --test-dir build\vs2022 -C Release --output-on-failure --no-tests=error
```

Outcomes:

- strict MSVC CTest passed 14/14 in 8.60 seconds after this increment.
