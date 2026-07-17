# M1 secure import evidence - 2026-07-17

## Proven increment

`weft_secure_core_tests` writes an OCCT cylinder STEP fixture, imports it with
both repair profiles, and proves that conservative import:

- verifies empty XDE and base STEP shape-processing policies before transfer;
- hashes and transfers one immutable byte snapshot even when the source path is
  replaced between `readFile` and `transferSecure`;
- retains immutable source and working snapshots of the processing-disabled
  transfer;
- hashes the source bytes and exact native B-rep serialization;
- produces an identity source-to-working face/edge correspondence;
- reports no topology-cardinality, tolerance, or stored-representation delta;
- emits non-vacuous repair-validation coverage;
- evaluates an exact 3D curve, stored p-curve, owning surface, and their
  measured discrepancy;
- fails a deliberately invalid stable ID with `geometry.edge_not_found`.
- reports an inaccessible source as `import.step.read_failed` and checks STEP
  schema plus one-result-per-requested-root accounting.

The compatibility profile is a separate post-capture stage and emits the
operation code `repair.compatibility_pipeline`. It operates on a geometry-deep
working copy, composes copy-plus-repair history back to source IDs, and the test
proves the retained source shape is not the working shape while its digest still
matches a fresh conservative import.

## Commands and outcomes

- `cmake --build --preset vs2022 --target weft_secure_core_tests` - passed;
- `ctest --preset vs2022 -R secure_core` - passed;
- `cmake --build --preset vs2022-static-analysis --target weft_secure_core_tests` - passed;
- `ctest --preset vs2022-static-analysis -R secure_core` - passed.

## Adversarial observation

Processing-disabled planar STEP faces may expose only an OCCT-computed p-curve.
The source snapshot records that representation as missing instead of treating
the computed value as authored evidence. The exact evaluator witness therefore
uses a stored cylindrical p-curve. Conservative handling of such planar inputs
remains an explicit M1/M3 design obligation.
