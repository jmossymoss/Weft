# M0 Linux GCC strict and static-analysis evidence - 2026-07-18

## Proven increment

Both supported Linux presets build and test cleanly under GCC 15.2.0 with
system OCCT 7.9.2. Temporary debug prints were removed from the committed STEP
corpus and secure-meshing harnesses. The generated 77-fixture corpus now
reconciles OCCT-version transfer refusals as named `import.*` codes instead of
hard failures, so Linux and Windows totals both close at 77 without false
passes.

## Environment

- Host path: `/mnt/d/Weft` via WSL
- Compiler: `g++ (Ubuntu 15.2.0-16ubuntu1) 15.2.0`
- CMake: 4.2.3
- OpenCASCADE: 7.9.2 (`libTKernel.so.7.9.2`)
- Base revision before this package: `0b9ab85`

## Analyzer scope

`-fanalyzer` is applied to `weft_core` only through `weft_enable_gcc_analyzer`.
App and large fixture-generator translation units remain outside the analyzer
pass because GCC 15 `cc1plus` ICEs or hangs on those files.

Known false-positive analyzer classes are surfaced as warnings but do not fail
the build (`-Wno-error=` for malloc-leak, uninitialized-value, null-*, and
out-of-bounds). MSVC `/analyze:external-` remains the Windows counterpart.

Translation units that still ICE under GCC 15 `-fanalyzer` keep a narrow
`-fno-analyzer` exclusion while remaining in the strict `-Werror` lane:

- `core/src/secure_topology.cpp`
- `core/src/cylinder_template.cpp`
- `core/src/model.cpp`
- `core/src/planar_trim_assembly.cpp`

## Commands

```bash
cmake --preset linux-gcc
cmake --build --preset linux-gcc -j 4
ctest --preset linux-gcc --output-on-failure

cmake --preset linux-gcc-static-analysis
cmake --build --preset linux-gcc-static-analysis -j 4
ctest --preset linux-gcc-static-analysis --output-on-failure
```

## Outcomes

- `linux-gcc`: build succeeded; CTest 14/14 passed (183.15 s).
- `linux-gcc-static-analysis`: build succeeded with analyzer warnings only as
  non-fatal classes above; CTest 14/14 passed (179.96 s).
- Committed STEP corpus: 6 success + 3 named refusals on Linux.
- Generated corpus accounting on OCCT 7.9.2:

```text
30 meshable
43 inspectable-only
4 named-import-refusals (import.step.transfer_failed)
1460 classified subjects
30 + 43 + 4 = 77
```

The four transfer refusals are OCCT-version capability differences under
processing-disabled STEP transfer (`TopoDS::Solid`). They are stable named
`import.*` refusals, not missing occurrence accounts or silent skips.

## Status boundary

This closes WP-001 and the deferred Linux analysis half of the M0 gate. Windows
clean build, legacy removal, and single-owner production routing were already
proven in `m0-windows-clean-legacy-removal-2026-07-17.md`. Baseline exception
debt (MP9/flaregun/foam) remains frozen and is not accepted product behaviour.
