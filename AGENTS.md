# AGENTS.md

## Secure-core agent directive

Before changing anything:

1. Work from the repository root and inspect `git status`. Preserve all
   existing uncommitted and user-owned changes.
2. Read `docs/governance/milestones.md`; it is the sole status authority.
3. Read only the authority/session sections of
   `docs/governance/agent-execution-playbook.md`.
4. Search the playbook for the first `Status: OPEN`, then read that work
   package and its named prerequisites only.
5. Read `docs/governance/blocked-routes.md`.
6. Inspect only the package-specific implementation, tests, ADRs, and evidence.

Before implementation, report:

- selected package and goal;
- prerequisites and whether they are complete;
- relevant working-tree state;
- exact first action;
- existing changes that must not be overwritten.

Execute one work package completely. Follow its tests, evidence requirements,
and binary exit gate. Update `milestones.md` only when the full milestone gate
is proven.

Do not:

- create additional plans, handoff files, investigation diaries, or summaries;
- treat chat, commits, plans, or test output as status authority;
- stage attachments, build output, generated STEP files, dumps, or unrelated
  user changes;
- add OCCT triangle-soup fallback, authoritative welding, missing faces, silent
  healing, or fail-open validation.

When switching agents, use the `RESUME` block defined in the playbook. Chat is
transport only; verify every handoff against the repository and evidence.

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
