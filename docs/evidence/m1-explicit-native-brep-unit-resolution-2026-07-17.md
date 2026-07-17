# M1 explicit native B-rep unit resolution evidence - 2026-07-17

## Proven increment

`importBRepSecure` accepts an optional positive `lengthUnitMm`. When omitted,
native imports still leave unit metadata absent and emit
`import.brep.length_unit_unspecified`. When supplied, source metadata and both
snapshots record the scale, `lengthUnitExplicitlyResolved` is true, coordinates
are unchanged, and diagnostics report `import.brep.length_unit_resolved`.
Non-positive values refuse as `import.brep.length_unit_invalid`.

## Witnesses

- reviewed box B-rep with unit `1.0` remains identity and meshable;
- omitted unit still emits `import.brep.length_unit_unspecified`;
- `0.0` and negative scales refuse by name.

## Windows commands and outcomes

Run from `D:\Weft` on branch `WIP/secure-core-rewrite`:

```text
cmake --build build\vs2022 --config Release --target weft_secure_core_tests -- /m:1 /nr:false
ctest --test-dir build\vs2022 -C Release --output-on-failure --no-tests=error
cmake --build --preset vs2022-static-analysis --config Release --target weft_secure_core_tests -- /m:1 /nr:false
ctest --preset vs2022-static-analysis -C Release -R "^secure_core$" --output-on-failure --no-tests=error
```

Outcomes:

- strict MSVC CTest passed 14/14 in 8.25 seconds before the multi-body
  follow-on; `secure_core` remains green after unit/COW/multi-body changes;
- analyzer-lane `secure_core` passed after the unit/COW increment;
- no Linux build was attempted.
