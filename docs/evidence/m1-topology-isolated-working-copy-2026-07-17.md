# M1 topology-isolated working-copy evidence - 2026-07-17

## Proven increment

Commit `a62e6af` replaces the conservative source/working TShape alias with a
faithful topology clone. The clone:

- creates one new TShape for every unique source TShape;
- preserves the source child-sharing graph, occurrence orientation/location,
  exact geometry representations, and all TShape flags;
- rebinds XDE leaf exact uses through the complete clone map;
- emits supported OCCT history for vertices, edges, faces, and solids;
- proves higher-order identity through the byte-identical native B-rep digest
  and independently validated stable occurrence paths;
- keeps compatibility repair on its separate geometry-deep mutation lane.

No mesher-facing fallback, weld, healing operation, or p-curve synthesis was
introduced.

## Rejected copy routes

Both geometry-deep and geometry-shallow `BRepBuilderAPI_Copy` prototypes were
run against processing-disabled source shapes. They changed the native B-rep
digest because computed planar p-curves became stored representations. Those
routes are recorded as blocked for conservative identity in BR-012.

The accepted clone keeps exact curve/surface representation handles read-only
and isolates every topology carrier. Future conservative representation
replacement must be copy-on-write and separately certified.

## Corpus and adversarial proof

For every one of the 77 generated STEP imports, the test lane now requires:

- a valid conservative identity certificate;
- complete source and working topology accounts;
- complete source-to-working occurrence correspondence;
- equal 64-character native B-rep SHA-256 digests;
- empty tolerance, stored-p-curve, and topology-cardinality change sets;
- non-partner source and working root TShapes;
- non-partner source and working TShapes for every authoritative occurrence.

The first full-corpus attempt failed 18 fixtures with
`topology.instance.topology_roots_missing`. The independent validator exposed
that `BRepTools_History` cannot carry compound, compsolid, shell, or wire
identity. XDE exact uses are now rebound directly through the clone map, while
history calls are restricted to OCCT-supported entity families. The complete
77-fixture lane then passed without weakening validation.

The focused flat-cylinder test additionally exercises exact curve, vertex,
p-curve, surface, and curve-on-surface evaluation against the immutable source
and working snapshots. Compatibility remains explicitly non-meshable because
its modified occurrence correspondence is incomplete.

## Windows commands and outcomes

Run from `D:\Weft` on branch `WIP/secure-core-rewrite`:

```text
cmake --build build\m0-windows-clean-20260717 --config Release -- /m:1 /nr:false
ctest --test-dir build\m0-windows-clean-20260717 -C Release --output-on-failure --no-tests=error
cmake --build --preset vs2022-static-analysis --config Release -- /m:1 /nr:false
ctest --preset vs2022-static-analysis -C Release --output-on-failure --no-tests=error
git diff --check
```

Outcomes:

- strict MSVC compiled the core, CLI, desktop app, and every test target;
- strict CTest passed 14/14 in 8.29 seconds;
- MSVC `/analyze` compiled the same complete product with warnings as errors;
- analyzer-lane CTest passed 14/14 in 8.10 seconds;
- `git diff --check` passed;
- no Linux build, preset, or source change was attempted; Windows remains the
  active development gate.

## Status boundary

This closes the topology-isolated conservative identity-copy slice of M1. M1
remains `IN_PROGRESS`: the allowed bounded repairs, their tolerance envelopes,
copy-on-write representation changes, and complete history-backed modified
occurrence correspondence are still open. This evidence does not pass M1 or
authorize compatibility output.
