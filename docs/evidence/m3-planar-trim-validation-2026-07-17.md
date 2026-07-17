# M3 planar trim validation evidence - 2026-07-17

## Proven increment

The secure core now has an atomic pre-CDT planar-domain validator. It consumes
canonical sample IDs, shared canonical vertex indices, lifted UV coordinates,
and an explicit topology-closure witness. It checks complete intra-loop and
inter-loop edge-pair sets with the exact dyadic predicate backend, then proves
containment, direct nesting, declared roles, and canonical loop orientation.

Successful output retains the exact input order and coordinates. Failure emits
no validated domain and records stable diagnostics plus per-family
`expected/checked/skipped/failed` coverage.

## Verification

Commands run from `D:\Weft`:

```text
cmake --build --preset vs2022 --target weft_planar_trim_validation_tests --config Release
ctest --preset vs2022 -R planar_trim_validation --output-on-failure
cmake --build --preset vs2022-static-analysis --target weft_planar_trim_validation_tests --config Release
ctest --preset vs2022-static-analysis -R planar_trim_validation --output-on-failure
```

Both lanes passed. The battery proves:

- one outer plus one hole with 12 checked intra-loop edge pairs, 16 checked
  inter-loop edge pairs, both directed containment queries, and a valid
  zero-valued canonical vertex index;
- a triangle still checks all three edge relations and its orientation;
- exact validity for a square whose side is one subnormal double;
- named refusal of empty/open loops, missing provenance, repeated canonical and
  UV vertices, non-finite UV, bow-tie crossing, adjacent overlap, loop crossing,
  an external declared hole, nested holes, wrong orientation, and no predicate
  backend;
- deterministic evidence counts over repeated runs;
- dependent evidence is explicitly skipped, never presented as checked, after
  a prerequisite failure.

## Status boundary

M3 remains in progress. This increment validates already assembled planar trim
records. Face/coedge-to-loop assembly, critical/monotone segmentation, broader
periodic singularities, sum constraints, a CDT backend, and Linux determinism
remain open. No triangle mesh or M4 gate is claimed.
