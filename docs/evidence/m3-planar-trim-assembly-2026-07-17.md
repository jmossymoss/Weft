# M3 planar trim assembly evidence - 2026-07-17

## Proven increment

Secure planar faces can now be assembled from ordered working coedges and
canonical edge sequences into independently validated trim domains. Shared
topological corners merge by zero-based canonical vertex index and exact lifted
UV equality while retaining every incident sample and source/working edge
record. Closed circular edges retain their non-repeated canonical sequence.

OCCT face topology identifies the outer wire. Complete loop reversal is
recorded when required by the canonical outer-CCW/hole-CW CDT convention; no UV
or boundary sample is modified.

## Verification

Commands run from `D:\Weft`:

```text
cmake --build --preset vs2022 --target weft_planar_trim_assembly_tests --config Release
ctest --preset vs2022 -R planar_trim_assembly --output-on-failure
cmake --build --preset vs2022-static-analysis --target weft_planar_trim_assembly_tests --config Release
ctest --preset vs2022-static-analysis -R planar_trim_assembly --output-on-failure
```

Both lanes passed. End-to-end fixture evidence includes six box faces, two
closed-circle cylinder caps, a named non-planar cylinder-wall refusal, a real
perforated through-hole face whose exact trim validation succeeds, and an exact
shared-corner UV tamper refusal. Every successful assembly has complete face,
wire, coedge, sample, junction, and validation coverage.

Both complete MSVC build presets then passed. Strict and static-analysis CTest
trees were run concurrently across all ten secure/corpus tests; both passed all
ten, including the frozen 106-record catalogue and 77 generated imports.

## Status boundary

M3 remains in progress. Planar face/coedge assembly is now proven for these
connected fixtures, but general periodic/singular loop assembly, repeated wire
occurrences, critical/monotone segmentation, broader corpus coverage, and Linux
determinism remain open. Hole-capable triangulation and certified surface/body
assembly remain M4 work.
