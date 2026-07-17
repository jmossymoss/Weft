# M1 copy-on-write representation rules evidence - 2026-07-17

## Proven increment

Conservative certificate validation now requires corresponding edge 3D curves
and face surfaces to remain identical OCCT handles unless a certified
copy-on-write representation replacement is recorded. Unexplained
`RepresentationChange` entries fail
`repair.representation_copy_on_write_reconciliation` and cannot be meshable.
Compatibility remains exempt from the shared-handle gate.

## Witnesses

- identity native box import: non-zero complete
  `repair.shared_geometry_immutable`;
- SameParameter/SameRange repair: shared handles retained;
- tolerance envelope and orientation repairs remain meshable under the same
  gate;
- empty unexplained representation changes on those witnesses.

## Windows commands and outcomes

Run from `D:\Weft` on branch `WIP/secure-core-rewrite`:

```text
cmake --build build\vs2022 --config Release --target weft_secure_core_tests -- /m:1 /nr:false
ctest --test-dir build\vs2022 -C Release --output-on-failure --no-tests=error
```

Outcomes:

- strict MSVC CTest passed 14/14 in 8.25 seconds for the unit/COW increment;
- `repair.shared_geometry_immutable` is non-zero and complete on identity and
  repaired native witnesses;
- no Linux build was attempted.
