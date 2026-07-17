# M1 bounded sewing evidence - 2026-07-17

## Proven increment

Conservative import may sew open face compounds/shells at `1e-4` mm when face
count stays constant and free-edge count decreases. The operation is
`repair.sewing_one_to_one`. Reviewed wire-gap fixtures are not sew targets.

## Witnesses

- procedural two-face compound with a `1e-5` mm gap: sew runs, edges drop,
  faces preserved, correspondence/meshable still incomplete for absorbed
  edges/vertices;
- `corrupt.wire.gap_within_beyond.brep`: no `repair.sewing_one_to_one`.

## Windows commands and outcomes

```text
cmake --build build\vs2022 --config Release --target weft_secure_core_tests -- /m:1 /nr:false
ctest --test-dir build\vs2022 -C Release --output-on-failure --no-tests=error
```

Outcomes:

- strict MSVC CTest passed 14/14 in 9.31 seconds after the sewing operation
  witness landed.
