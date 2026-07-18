# M1 bounded sewing evidence - 2026-07-17

## Proven increment

Conservative import may sew open face compounds/shells at `1e-4` mm when face
count stays constant and free-edge count decreases. The operation is
`repair.sewing_one_to_one`. Absorbed free edges and vertices complete
correspondence through contiguous-edge couples and partner-aware topology
matching; sew-owned representation deltas are certified so the certificate is
meshable. Reviewed wire-gap fixtures are not sew targets.

## Witnesses

- procedural two-face compound with a `1e-5` mm gap: sew runs, edges drop,
  faces preserved, correspondence complete, meshable;
- `corrupt.wire.gap_within_beyond.brep`: no `repair.sewing_one_to_one`.

## Windows commands and outcomes

```text
cmake --build build\vs2022 --config Release --target weft_secure_core_tests -- /m:4 /p:TreatWarningAsError=false
.\build\vs2022\bin\Release\weft_secure_core_tests.exe
ctest --test-dir build\vs2022 -C Release --output-on-failure --no-tests=error
```

Outcomes:

- `weft_secure_core_tests` reports `secure-core contract checks passed` with
  meshable bounded sewing after contiguous-edge / partner-duplicate /
  sew-COW certification landed.
- strict MSVC CTest passed 14/14 in 8.44 seconds.
