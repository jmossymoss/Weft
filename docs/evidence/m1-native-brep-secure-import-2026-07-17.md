# M1 native B-rep secure-import evidence - 2026-07-17

## Proven increment

Commit `43e62ed` adds a public secure import for native OCCT ASCII B-rep. The
reader captures source bytes once, parses from a stream over that snapshot,
and hashes the same retained bytes. Commit `94d2cf3` adds direct stale-state
refusal proof. Conservative and compatibility results use the same repair
derivations as STEP; no test-only geometry path owns different repair
behaviour.

The base secure-reader implementation now refuses. A format without an
explicit immutable snapshot and topology-isolated working derivation cannot
claim secure identity by returning one materialized model twice.

`SourceMetadata::lengthUnitMm` now distinguishes a declared STEP unit scale
from the absence of physical-unit metadata in native B-rep. Native coordinates
remain unchanged and the import emits
`import.brep.length_unit_unspecified`; no millimetre declaration is invented.

## Adversarial proof

The public conservative API imports the reviewed artifact
`corrupt.edge.sameparameter_samerange_false.brep` and proves:

- exact source-byte SHA-256
  `c400ae0bb2102165a07673bb0df8ba28a17e998c84daf1ade7b4bc95a963ba1e`;
- byte length equal to the frozen artifact;
- three immutable source faces and distinct source/working TShapes;
- one complete bounded parameterization repair;
- valid, meshable working B-rep with complete modified correspondence;
- absent declared physical unit plus a visible warning.

The snapshot replacement witness copies that artifact to a temporary path,
calls `readFile`, overwrites the path with
`baseline.pathology.box.brep`, and transfers the retained reader. The retained
result still has the original SHA-256 and three faces. A fresh
`importBRepSecure` sees the box SHA-256
`bdf9bdd95e8b40373f26c929e855528deadd4808756537c4803e3d78062be461`
and six faces.

Additional refusal lanes prove:

- malformed and missing native B-reps fail as `import.brep.read_failed`;
- failed parsing nulls reader shape state;
- the IGES reader, which has no immutable secure contract yet, fails as
  `import.secure.reader_unsupported` instead of aliasing source and working;
- native compatibility uses a geometry-deep working copy, records
  `repair.compatibility_pipeline`, and remains non-meshable while modified
  occurrence correspondence is incomplete;
- existing STEP compatibility and all 77 generated STEP corpus imports remain
  unchanged and green.

## Windows commands and outcomes

Run from `D:\Weft` on branch `WIP/secure-core-rewrite`:

```text
cmake --build build\m0-windows-clean-20260717 --config Release -- /m:1 /nr:false
ctest --test-dir build\m0-windows-clean-20260717 -C Release --output-on-failure --no-tests=error
cmake --build --preset vs2022-static-analysis --config Release -- /m:1 /nr:false
ctest --preset vs2022-static-analysis -C Release --output-on-failure --no-tests=error
git diff --check
```

Outcomes after the final unit-provenance refinement:

- strict MSVC compiled the core, CLI, desktop app, and every test target;
- strict CTest passed 14/14 in 8.09 seconds;
- MSVC `/analyze` compiled the same complete product with warnings as errors;
- analyzer-lane CTest passed 14/14 in 8.05 seconds;
- `git diff --check` and the source scan passed;
- no Linux build, preset, or source change was attempted.

## Status boundary

This closes the immutable native-ASCII-BRep snapshot and public repair-admission
slice of M1. It does not route native B-rep through CLI/app workflows, infer a
physical unit, implement secure IGES import, add another repair operation, or
pass M1. Those dependencies remain explicit in the milestone ledger.
