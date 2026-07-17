# M1 bounded parameterization repair evidence - 2026-07-17

## Proven increment

Commit `97a2113` adds the first bounded conservative repair operation. It may
set `SameRange` and `SameParameter` on an isolated working edge only after:

- finding a retained bounded 3D curve and every owning face;
- requiring every p-curve representation to be stored already;
- requiring exact equality between every p-curve range and the 3D range;
- checking both p-curves on a seam;
- running deterministic OCCT curve-on-surface maximum-distance evaluation for
  every expected use;
- proving the finite maximum is no greater than the source edge tolerance.

Only the two working TEdge flags change. The immutable source flags, geometry
handles, p-curves, parameter ranges, tolerances, and topology remain unchanged.
The certificate records non-zero expected/checked coverage and numeric
discrepancy evidence. An exact TShape-derivation map now resolves every
source/working shape family; digest equality is no longer a mapping fallback.

## Positive and refusal witnesses

The reviewed native fixture
`corrupt.edge.sameparameter_samerange_false.brep` proves:

- source invalid and unchanged; working valid and meshable;
- exactly one parameterization flag change and matching repair operation;
- zero measured maximum discrepancy within the retained tolerance envelope;
- no tolerance, stored-p-curve, or topology-cardinality changes;
- distinct source/working TShapes at every authoritative occurrence;
- complete one-to-one `Modified` correspondence for every occurrence;
- source curve-on-surface evaluation refuses the unproven flag state while the
  corresponding working evaluation succeeds within tolerance.

The committed cylinder seam baseline, with only its working-copy source flags
cleared by the test, requires `expected=2` and `checked=2` for its dual
p-curves. This prevents the repeated seam owner from being omitted or counted
twice accidentally.

The adversarial lanes prove:

- `corrupt.edge.range_mismatch.brep` is unchanged, named unproven, and
  non-meshable;
- `corrupt.edge.pcurve_disagreement_beyond_tolerance.brep` is unchanged,
  invalid, and non-meshable;
- a structurally valid working B-rep with a tampered `checked=0` certificate
  fails `import.repair.validation_incomplete` and cannot be meshed.

No `BRepLib_CheckCurveOnSurface`, unrestricted healing, p-curve synthesis,
fixed-point-only proof, digest/order fallback, or authoritative weld was added.

## Corpus and Windows outcomes

Run from `D:\Weft` on branch `WIP/secure-core-rewrite`:

```text
cmake --build build\m0-windows-clean-20260717 --config Release -- /m:1 /nr:false
ctest --test-dir build\m0-windows-clean-20260717 -C Release --output-on-failure --no-tests=error
cmake --build --preset vs2022-static-analysis --config Release -- /m:1 /nr:false
ctest --preset vs2022-static-analysis -C Release --output-on-failure --no-tests=error
build\m0-windows-clean-20260717\bin\Release\weft_generated_secure_corpus_tests.exe
git diff --check
```

Outcomes:

- strict MSVC compiled the core, CLI, desktop app, and every test target;
- strict CTest passed 14/14 in 8.28 seconds;
- MSVC `/analyze` compiled the same full product with warnings as errors;
- analyzer-lane CTest passed 14/14 in 8.07 seconds;
- the direct generated-corpus rerun passed: 77 imported, 59 meshable, 18
  inspectable-only, and 1,559 classified subjects;
- all generated imports retained identity certificates; none happened to
  trigger this native-B-rep repair operation;
- `git diff --check` and the adversarial source scan passed;
- no Linux build, preset, or source change was attempted; Windows remains the
  active development gate.

## Status boundary

This closes only the bounded false-flag reconciliation slice of M1. M1 remains
`IN_PROGRESS`. Orientation normalization, bounded tolerance reconciliation,
provable one-to-one sewing, copy-on-write replacement rules, production STEP
fixtures that exercise a repair, and complete compatibility correspondence are
still open. This evidence does not authorize compatibility meshing or mark M1
passed.
