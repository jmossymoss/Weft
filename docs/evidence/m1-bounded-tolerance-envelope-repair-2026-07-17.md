# M1 bounded tolerance envelope repair evidence - 2026-07-17

## Proven increment

This working tree adds a bounded conservative tolerance repair. After topology
isolation and optional SameParameter/SameRange reconciliation, a working edge
tolerance may rise to the measured curve-on-surface maximum only when every
stored p-curve use is checked, ranges match exactly, and that maximum exceeds
the immutable source tolerance. The raise commits only when the resulting
working body is BRepCheck-valid.

## Positive and refusal witnesses

`corrupt.edge.pcurve_disagreement_beyond_tolerance.brep` proves:

- source remains invalid; working becomes valid and meshable;
- exactly one certified `ToleranceChange` with matching
  `repair.tolerance_envelope`;
- non-vacuous `repair.tolerance_envelope_reconciliation`;
- no parameterization flag changes on this witness.

`corrupt.edge.pcurve_disagreement_within_tolerance.brep` remains a valid
identity import with empty tolerance changes.

A tampered certificate with `checkedPcurveUses=0` is non-meshable by
`import.repair.validation_incomplete`. Range-mismatch and orientation lanes
remain separately gated.

## Windows commands and outcomes

Run from `D:\Weft` on branch `WIP/secure-core-rewrite`:

```text
cmake --build build\vs2022 --config Release --target weft_secure_core_tests -- /m:1 /nr:false
ctest --test-dir build\vs2022 -C Release --output-on-failure --no-tests=error
cmake --build --preset vs2022-static-analysis --config Release -- /m:1 /nr:false
ctest --preset vs2022-static-analysis -C Release --output-on-failure --no-tests=error
git diff --check
```

Outcomes:

- strict MSVC CTest passed 14/14 in 9.37 seconds;
- MSVC `/analyze` rebuilt changed targets with warnings as errors;
- analyzer-lane CTest passed 14/14 in 206.11 seconds (committed STEP corpus
  dominated wall time);
- no Linux build, preset, or source change was attempted.

## Status boundary

This closes the CRV-024/PAT-011 tolerance-envelope slice of M1. It does not
authorize sewing, p-curve synthesis, open-ended healing, native-unit
inference, IGES import, or compatibility meshing. M1 remains `IN_PROGRESS`.
