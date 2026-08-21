# Agent instructions

## Source of truth

Read these files before changing Weft:

1. `docs/EXECUTION_PLAN.md` — sole product roadmap, completion definition, and
   work-package order for this fork.
2. `README.md` — current product and usage overview.
3. `tests/CAD_CORPUS.md` — corpus operation and regression-reduction guide.
4. `WINDOWS_BUILD.md` when changing or testing Windows behavior.

This branch is an independent-tessellation fork. Old border-contract plans,
pull requests, and generate()-only handoffs do not override the execution
plan on this branch.

## Work order

- Work on the first incomplete work package in `docs/EXECUTION_PLAN.md`
  (T0, then T1, …).
- The plan's `Active work package` line is the current state marker; update it
  only after all exit criteria pass at one revision.
- Product mesh on this fork is `weft::meshIndependent()`. Do not route new
  interactive or export work through `weft::generate()` except for explicit
  A/B comparison (T4).
- Do not change legacy `generate()` results, goldens, or corpus gates to
  accommodate the new path.
- `--stitch` remains a generate()-only diagnostic.

## Required engineering discipline

- Never special-case a filename, model name, or face ID.
- Do not add a `MesherKind` to `generate()` for this experiment. Independent
  tessellation is a separate function.
- Add or identify a deterministic reproducer before changing non-trivial
  meshing behavior.
- Promote useful probes to maintained tests and remove obsolete probes.
- Do not weaken validity checks, add exemptions, or refresh golden counts
  merely to obtain a passing result.
- Every intentional topology-count change on the legacy path requires a
  root-cause explanation and visual inspection. Independent-mesh counts are
  not golden-locked until T4.
- Keep generated status in machine-readable reports. Do not append session
  diaries or transient pass claims to roadmap documents.
- Treat `tests/CAD_CORPUS.tsv` as the sole case inventory. A runner must not
  create a competing hardcoded model list.

## Testing

Use focused tests while iterating, then verify in proportion to the change:

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

For independent-mesh work, also run:

- `build/cli/weft mesh <fixture.step> --independent --validate`
- the smallest zoo reproducer (box, cylinder, hole);
- timing vs `weft mesh` without `--independent` when claiming a speedup.

Do not run `tools/corpus_gate.sh` as a pass requirement for `--independent`
until T4. That gate still measures `generate()`.

Compile success or application startup alone is not sufficient evidence.

## Documentation responsibilities

- `docs/EXECUTION_PLAN.md` owns product direction and work order.
- `README.md` owns current capabilities and basic usage.
- `tests/CAD_CORPUS.md` owns corpus mechanics.
- `WINDOWS_BUILD.md` owns Windows setup.
- `docs/PRODUCTION_PATH.md` owns which entry points call which mesher.

Do not duplicate roadmaps between these files. Update links and commands when
paths or interfaces change.
