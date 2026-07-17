# M1/M2 frozen-corpus evidence - 2026-07-17

## Frozen evidence input

Weft owns a byte snapshot of `D:\weftocct\fixtures` from commit
`6eaa0708d5313210779b6d26c839306028db5f98`. It contains 308 committed files
(19,157,865 bytes). The dirty donor working tree was not copied; in particular,
the unreviewed twisted-cylinder work remains untouched.

The canonical manifest digest matches its sidecar:

```text
aade67d71a316a3df8585c8b8039dadceb9750fd715d782922c7bf85d10bb3f8
```

`secure_fixture_catalog` verifies 106 unique reviewed records, all expectation
and review hashes, committed artifact/baseline/transform/licence/provenance
hashes, and the exact lane accounting:

- 77 OCCT-procedural;
- 21 derived-corruption;
- 5 native-fault;
- 3 audited-external.

## Committed STEP lane

The nine committed STEP sources run twice. Exact outcome accounting is:

- six deterministic imports;
- `corrupt.step.syntax_failure.step` refuses as
  `import.step.transfer_failed`;
- mirrored and scaled mapped-item witnesses refuse as
  `import.transform.unresolved_representation_loss`.

An import success is distinct from mesh eligibility. Raw invalid topology stays
inspectable with `import.source.invalid`, `import.working.invalid`, and
`import.working.non_meshable`; it is not relabelled as a meshing success.

## Generated 77-fixture lane

The frozen upstream generator is compiled as a test-only target and recreates
all 77 procedural STEP artifacts in an isolated temporary directory. Every
artifact passes processing-disabled secure import and total edge/face
reconnaissance:

```text
77 imported
59 meshable
18 inspectable-only with named import errors
1,559 edge/face subjects classified and accounted
0 import refusals
```

Temporary generated artifacts are removed by the test and are never staged.

## Verification

Commands run from `D:\Weft`:

```text
cmake --build --preset vs2022 --target weft_secure_corpus_tests weft_generated_secure_corpus_tests
ctest --preset vs2022 -R 'secure_fixture_catalog|secure_committed_step_corpus|secure_generated_step_corpus' --output-on-failure
cmake --build --preset vs2022-static-analysis --target weft_secure_corpus_tests weft_generated_secure_corpus_tests
ctest --preset vs2022-static-analysis -R 'secure_fixture_catalog|secure_committed_step_corpus|secure_generated_step_corpus' --output-on-failure
```

All three tests pass in both lanes.

## Status boundary

This closes the fixture import/accounting foundation, not the M1 or M2 gate.
Assembly occurrence/transform accounts, bounded conservative repair witnesses,
oracle-level classification comparisons, full trim taxonomy, and merged logical
regions remain open. Generated fixture observations do not replace the reviewed
expectation documents.
