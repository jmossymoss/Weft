# M1 certificate-or-named-refusal gate evidence - 2026-07-17

## Proven increment

M1 gate closure: every reviewed working model is either meshable with a
complete repair/correspondence certificate, or non-meshable with at least one
named `import.*` Error/Fatal diagnostic.

## Named refusals added

- `import.heal.multi_way_split` - Compatibility heal multi-way
  Modified/Generated without a unique partner image
- `import.heal.correspondence_unbound` - historical heal left subjects unbound
- `import.repair.orientation_open_shell` - face-adjacency refuses open shells
- `import.repair.orientation_non_manifold` - refuses non-manifold incidence
- `import.repair.orientation_non_orientable` / `orientation_unsupported`

## Witnesses

- open-shell solid procedural fixture: `orientation_open_shell`, non-meshable,
  no `repair.orientation_face_adjacency`
- non-manifold three-face solid: named orientation refusal family, no false
  orientation repair operation
- Compatibility self-intersecting shell: complete certificate or named heal
  refusal when correspondence is incomplete
- table-driven `testM1CertificateOrNamedRefusalGate` across Conservative and
  Compatibility corrupt fixtures

## Windows commands and outcomes

```text
cmake --build build\vs2022 --config Release --target weft_secure_core_tests -- /m:4 /p:TreatWarningAsError=false
ctest --test-dir build\vs2022 -C Release --output-on-failure --no-tests=error
```

Outcomes:

- `weft_secure_core_tests` reports `secure-core contract checks passed`
- strict MSVC CTest passed 14/14 in 8.59 seconds
