# AGENTS.md

## Cursor Cloud specific instructions

Weft is a native C++17 toolchain (no web services, no database). It builds three
binaries over one shared `weft_core` library: `build/cli/weft` (CLI),
`build/app/weft_app` (GLFW/OpenGL + Dear ImGui GUI), and `build/tests/weft_tests`
(CTest). Standard build/test/run commands live in `README.md` ("Build" / "Try the
loop"), `build.sh`, and `.github/workflows/ci.yml` — use those; only the
non-obvious caveats are recorded here.

### Build / test / run

- Configure + build (Ninja, matches CI):
  `cmake -B build -G Ninja -DCMAKE_BUILD_TYPE=Release` then `cmake --build build -j"$(nproc)"`.
- The first CMake configure fetches Dear ImGui via `FetchContent` (needs network);
  it is cached in `build/_deps` afterward.
- Run tests: `ctest --test-dir build --output-on-failure`.
- CLI end-to-end loop: `build/cli/weft fixture demo.step --shape demo`,
  then `inspect`, then `mesh ... -o demo.obj` (see README "Try the loop").

### Non-obvious caveats

- Default compiler: this VM's `cc`/`c++` alternatives point at clang, and clang
  selects the gcc-14 toolchain dir, which has no `libstdc++.so` (only
  `libstdc++-13-dev` is installed) — configure fails with `cannot find -lstdc++`.
  The environment fixes this by setting the `cc`/`c++` alternatives to
  `gcc`/`g++` (gcc-13, matching CI on ubuntu-24.04). If you hit that link error,
  re-run `sudo update-alternatives --set c++ /usr/bin/g++` and
  `sudo update-alternatives --set cc /usr/bin/gcc`.
- The GUI has no display in cloud; render it headlessly:
  `xvfb-run -a -s "-screen 0 1280x1024x24" build/app/weft_app --fixture boss --screenshot out.png`.
- `ctest` and `tools/corpus_gate.sh` are currently RED on `main` (reproduced
  identically in CI on both Linux and Windows). The `pipeline` test asserts
  watertightness/topology-count invariants and reads `tests/CAD_CORPUS.tsv`,
  which references `tests/fixtures/generated/*.step` and `tests/regressions/**`
  fixtures that are not committed (`*.step` is gitignored except
  `tests/fixtures/*.step`). Treat these failures as pre-existing repo state, not
  an environment problem, unless your change is meant to fix them.
