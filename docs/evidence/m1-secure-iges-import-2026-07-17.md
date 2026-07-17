# M1 secure IGES import evidence - 2026-07-17

## Proven increment

IGES secure import now binds SHA-256 provenance to an immutable byte snapshot,
parses through a private temp filled from those bytes (caller path never
reopened), disables shape processing, and derives the working copy through
`cafToImportedModel`. Public entry is `importIgesSecure`.

## Witnesses

- generated B-rep-mode millimetre box `.igs` is identity and meshable;
- unread IGES reader refuses as `import.iges.no_source_shape`;
- path replacement after `readFile` retains the original digest.

## Windows commands and outcomes

```text
cmake --build build\vs2022 --config Release --target weft_secure_core_tests -- /m:1 /nr:false
ctest --test-dir build\vs2022 -C Release -R "^secure_core$" --output-on-failure --no-tests=error
```

Outcomes:

- `secure_core` and the full strict 14/14 suite passed after IGES identity and
  path-replacement witnesses landed (~8.4s);
- unread IGES readers refuse as `import.iges.no_source_shape`.
