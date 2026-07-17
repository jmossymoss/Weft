# Fixture corpus

This directory is the machine-readable entry point for the M1 geometry corpus. Corpus revision 7
contains 106 reviewed fixtures: 77 OCCT-procedural, 21 deterministically derived/corrupted, 5
native fault-injection, and 3 audited external STEP fixtures. One aggregate provenance record
binds the external matrix. Every one of the 152 source and Section 3 obligations now has an
evidence route; all 152 are `PARTIAL`/fixture-ready, none is `BLOCKED` or `MISSING`, and none is yet
`COVERED` by its responsible later consumer.

The owner-approved M1 source gate combines representative audited Plasticity AP203
analytic/freeform evidence, exhaustive reviewed procedural/derived/native taxonomy routes, and
genuine AutoCAD AP214 and NIST AP242 assets. The Plasticity asset is not claimed to contain every
geometry family. Similarly named Plasticity `_214` and `_242` files still declare AP203 and have
duplicate DATA sections, so they remain rejected audit evidence and receive no schema credit.
Additional Plasticity family exports are future corpus expansion rather than an M1 blocker; see
[ADR-0004](../docs/adr/0004-m1-external-source-sufficiency.md).

## Layout

- [`manifest.json`](manifest.json) is canonical sorted JSON and locks fixture IDs, construction
  parameters, generator sources, OCCT version, STEP profile, artifact hashes, expectation hashes,
  obligation links, and exact blockers.
- [`schema`](schema/) defines strict catalog, oracle, review, later-verification, and external
  provenance contracts.
- [`reviews`](reviews/) contains canonical, hashed records binding each reviewed expectation to
  the exact generated STEP artifact, XDE summary, construction revision, and independent reviewer.
- [`expected`](expected/) contains hand-derived topology, geometry, trim, periodicity, singularity,
  semantic, support-state, unit, tolerance, and validation expectations.
- [`generator`](generator/) contains the OCCT AP242 fixture factory. Generated procedural STEP,
  B-rep, and summary artifacts belong under ignored `out/`; they are reproducible outputs rather
  than source files.
- [`derived`](derived/) contains source-bound corruption artifacts produced by registered,
  deterministic transforms.
- [`external`](external/) and [`provenance`](provenance/) contain accepted/rejected external
  assets, immutable audit data, inspection summaries, and per-source provenance.
- [`lanes`](lanes/) contains the registered derived and native execution routes used by catalog
  validation.
- The complete obligation ledger remains
  [`docs/governance/fixture-coverage.md`](../docs/governance/fixture-coverage.md).

## Generate and validate

Configure the OCCT preset, then build and run the corpus tests:

```powershell
$env:OCCT_DIR = 'C:\OCCT'
cmake --preset vs2022-occt
cmake --build --preset vs2022-occt-release
ctest --preset vs2022-occt-release --output-on-failure
python tests\fixtures\test_fixture_catalog.py `
  --native-runner out\build\vs2022-occt\tests\Release\cad_mesher_native_fault_fixture.exe `
  --jsonschema
```

The generator also supports direct development use:

```powershell
out\build\vs2022-occt\fixtures\generator\Release\cad_mesher_generate_fixtures.exe `
  --all --output out\generated-fixtures
```

Procedural fixture validation disables OCCT STEP shape processing and SameParameter repair, writes
AP242DIS in millimetres without tessellation or duplicate cleaning, fixes mutable metadata, and
reads each artifact back through `STEPCAFControl_Reader` into XDE with processing disabled before
transfer. It compares the artifact-measurable portion of the independent oracle and requires
byte-identical output from `--all` and fresh per-fixture processes. Catalog validation separately
executes each registered derived transform and native fault route twice, verifies source and result
bindings, and audits external hashes, declared schemas, ownership/licence, and review records.
Detailed semantic judgments and later consumer support decisions remain reviewed oracle inputs;
they are not claimed as M2/M3 verification results.

## Evidence rule

A manifest entry with a complete artifact recipe, oracle, and hashed independent review is
`fixture_ready`; its coverage row is only `PARTIAL` until the responsible M2/M3 implementation
matches that oracle and emits a hashed verification record. `COVERED` is never awarded from
generator-side output or a screenshot. The four allowed evidence lanes and their trust boundaries
are defined by [ADR-0002](../docs/adr/0002-fixture-corpus-oracles.md).

External files require immutable hashes, source application/version, export settings, verified
STEP schema, ownership/licence, acquisition record, and a reason OCCT authoring is insufficient.
The accepted Plasticity re-export contains presentation styles and colours but no STEP material
entity; no material entity is claimed or required to close M1. Its observed geometry families
remain recorded exactly, while family-by-family Plasticity expansion is additive future work.
Derived and native lanes fail closed unless their transformation or fault-test IDs have a
registered deterministic callable runner. Likewise, `verified` promotion rejects every
unregistered consumer and requires a reachable commit plus strict, committed result evidence.
