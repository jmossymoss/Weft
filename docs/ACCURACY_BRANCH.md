# Accuracy tessellator branch

This branch strips structured retopology so we can build a Pixyz-inspired
sag/angle/length tessellator from a clean base.

## What changed

- `core/src/legacy/meshers_structured.cpp` — archived RevolutionGrid/Coons/… (not built)
- `core/src/meshers.cpp` — new `generate()`: shared-edge sampling, planar n-gons, adaptive UV tris on curves
- Density UX: `QualityPreset` + `chordTolerance` (maxSag), `angleToleranceDeg` (−1 = off), `maxLength` (−1 = off)
- `weft_app` off by default (`-DWEFT_BUILD_APP=ON` to try); old `test_pipeline` replaced by `test_accuracy_smoke`

## Build

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

Mesh a fixture:

```sh
build/cli/weft fixture /tmp/cyl.step --shape cylinder
build/cli/weft mesh /tmp/cyl.step -o /tmp/cyl.obj --chord 0.2
```

Recipe keys: `chord`/`maxsag`, `angle`/`maxangle`, `maxlength`.

## Next

1. Shared topological-edge sample map (true seam contract)
2. Trim-aware curved tessellation (holes, outer wire clip)
3. Hole bridging on planar faces
4. Pixyz Medium/High calibration
