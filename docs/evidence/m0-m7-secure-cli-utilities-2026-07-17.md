# M0/M7 secure CLI utility evidence - 2026-07-17

## Proven increment

Every reachable CLI route that turns a STEP B-rep into a mesh now delegates to
the secure pipeline. Unsupported or not-yet-proven workflows fail by name.
Source inspection and extraction use the immutable processing-disabled import.

## Build proof

Commands run from `D:\Weft`:

```text
cmake --build --preset vs2022 --config Release --parallel
cmake --build --preset vs2022-static-analysis --config Release --parallel
```

Both complete-product lanes pass with warnings as errors; the static lane also
passes MSVC analysis.

## Command proof

| Command | Result |
|---|---|
| `convert box.step -o box.stl` | 8 vertices, 12 certified triangles, 684-byte STL |
| `sweep cylinder.step --profile cad --radials 8,16,32 --verbose` | 3 completed, 0 failures; repeated fingerprints agree |
| `cache-check box.step --face 1:radial=24` | exit 1, `secure_cache.incremental_dependency_unimplemented` |
| `convert sphere.step -o sphere.obj` | exit 1, named unsupported-family refusal, output absent |
| `inspect box.step` | processing-disabled source model reports 6 faces and 12 edges |
| `extract box.step --faces 1 --rings 1` | source-backed STEP with source IDs 1,3,4,5,6 |

The profile warning explicitly states that the old profile is ignored. Sweep
minima 8 and 16 both resolve to the analytic demand and share fingerprint
`66a1da9c00968370`; minimum 32 resolves a denser mesh with fingerprint
`b360ebfeaa2ba913`.

## Status boundary

This closes runtime selection of the historical generators; it does not remove
their dormant source or frozen baseline tests. Secure cache dependency closure,
recipe-v2 per-entity sweeps, non-STEP audited import, and Linux execution remain
open.
