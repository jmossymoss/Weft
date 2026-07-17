# M0 parallel test isolation evidence - 2026-07-17

## Observed failure

Running the focused strict and static-analysis CTest lanes concurrently caused
`secure_core` to fail while parsing its generated cylinder STEP file. The two
test processes used the same fixed file in `%TEMP%`; one process could truncate
or replace it while the other process imported it. A sequential rerun passed in
both build trees, isolating the failure to the shared test artifact rather than
geometry or import logic.

## Change

Secure-import, canonical-boundary, and generated-corpus tests now allocate
temporary paths containing both the process ID and a high-resolution time
nonce. The helper is portable across Windows and POSIX test builds. Existing
cleanup remains scoped to the exact allocated file or directory.

## Verification

The affected targets built with warnings-as-errors and `/analyze`. These two
commands were then run concurrently from `D:\Weft`:

```text
ctest --preset vs2022 -R "secure_core|canonical_boundary|secure_generated_step_corpus" --output-on-failure
ctest --preset vs2022-static-analysis -R "secure_core|canonical_boundary|secure_generated_step_corpus" --output-on-failure
```

Both lanes passed all three tests. Each generated-corpus process independently
generated and securely imported all 77 procedural STEP fixtures.

## Status boundary

This proves test-artifact isolation for the three affected targets. It does not
prove the unavailable Linux lane or change any geometry milestone status.
