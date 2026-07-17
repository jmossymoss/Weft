# M0 Windows clean build and legacy removal evidence - 2026-07-17

## Proven increment

Commit `9ec3f3f` removes 32,849 lines of retired compiler, mesher-strategy,
fallback/weld, internal-header, disabled CLI benchmark, and frozen pipeline-test
source. Commit `427975a` had already removed those translation units from every
production/test target and hardened the secure import/build boundary.

The retained `mesher_trace.cpp` and `mesher_utils.cpp` provide only product
debug logging and enum naming. Retained settings/report/recipe headers are
migration and adapter contracts; there is no retained legacy generator
implementation to link or select.

## Clean Windows proof

Commands run from `D:\Weft` with MSVC 19.44 and OCCT 8:

```text
cmake --preset vs2022 -B build/m0-windows-clean-20260717 --fresh
cmake --build build/m0-windows-clean-20260717 --config Release -- /m:1 /nr:false
ctest --test-dir build/m0-windows-clean-20260717 -C Release --output-on-failure --no-tests=error
cmake --build --preset vs2022-static-analysis --config Release -- /m:1 /nr:false
ctest --preset vs2022-static-analysis -C Release --output-on-failure --no-tests=error
```

Outcomes:

- the new build directory compiled `weft_core`, CLI, desktop app, fixture
  generator, and every secure/corpus test with `/W4 /WX`;
- strict CTest passed 14/14 in 8.07 seconds;
- MSVC `/analyze` built the complete product with warnings as errors;
- static-analysis CTest passed 14/14 in 7.40 seconds;
- generated strict and static Visual Studio projects contain zero references
  to the deleted compiler/mesher strategy translation units;
- a source search finds no `weft::generate(...)`, `legacyCacheCheck`, or
  `legacySweep` call/body in app, CLI, core, or active tests.

The clean directory matters: no stale object or project could satisfy a symbol
formerly supplied by deleted code.

## Windows product-route proof

The clean CLI meshed `tests/secure_core_fixtures/baselines/solid.box.step` with
`--validate` and a complete mesh report:

- conservative identity repair;
- 8 vertices and 12 certified triangles;
- zero open edges and zero non-manifold edges;
- consistent winding and zero degenerate/sliver polygons;
- report `complete 1`, with non-zero repair, provenance, boundary, trim, CDT,
  body, and workflow coverage where evidence was expected.

Artifact SHA-256 digests:

```text
8AA96DC3083C02B8BFD7A5E4E66FE31632D274A32086918227A4CFE892A31565  box.obj
B0160AB24F85F9E4589CA822F94B537DF9699E33858F84081192C4BE23B44D23  box.mesh-report.txt
```

`sweep ... --radials 8,16 --verbose` completed both requests with zero
failures and repeated topology fingerprint `5585232f0b19f621`.
`cache-check ... --face 1:radial=24` exited non-zero with
`secure_cache.incremental_dependency_unimplemented`; it did not route to the
removed cache/generator implementation.

## Status boundary

This closes dormant legacy source removal and proves the primary Windows lane
from a new build directory. A Linux strict build and 14/14 test run were also
observed during portability reconnaissance, but the GCC static-analysis run was
stopped when work was reprioritized to establish Windows first. M0 therefore
remains `IN_PROGRESS`; this evidence does not claim the cross-platform gate.
