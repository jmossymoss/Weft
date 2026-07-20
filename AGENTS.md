# Agent instructions

## Source of truth

Read these files before changing Weft:

1. `docs/EXECUTION_PLAN.md` — sole product roadmap, completion definition, and
   work-package order.
2. `README.md` — current product and usage overview.
3. `tests/CAD_CORPUS.md` — corpus operation and regression-reduction guide.
4. `WINDOWS_BUILD.md` when changing or testing Windows behavior.

Old branches, pull requests, commit messages, probe comments, and external
handoffs are historical evidence only. They do not override the execution plan.

## Work order

- Work on the first incomplete work package in `docs/EXECUTION_PLAN.md`.
- The plan's `Active work package` line is the current state marker; update it
  only after all exit criteria pass at one revision.
- Choose a failing exit criterion, reproduce it, and address its shared topology
  class rather than one model.
- Do not begin deferred or post-MVP work while an earlier gate is red.
- Keep `weft::generate()` and the border-contract path authoritative for MVP.
  `--stitch` is a quarantined diagnostic experiment, not a second production
  path.

## Required engineering discipline

- Never special-case a filename, model name, or face ID.
- Do not add a `MesherKind` or parallel meshing architecture during
  stabilization without an evidence-backed revision to the execution plan.
- Add or identify a deterministic reproducer before changing non-trivial
  meshing behavior.
- Promote useful probes to maintained tests and remove obsolete probes.
- Do not weaken validity checks, add exemptions, or refresh golden counts merely
  to obtain a passing result.
- Every intentional topology-count change requires a root-cause explanation and
  visual inspection of affected output.
- Keep generated status in machine-readable reports. Do not append session
  diaries or transient pass claims to roadmap documents.
- Treat `tests/CAD_CORPUS.tsv` as the sole case inventory. A runner must not
  create a competing hardcoded model list.
- During stabilization, known release failures belong in
  `tests/KNOWN_RED.tsv`; the strict release gate never consumes those
  allowances.

## Testing

Use focused tests while iterating, then verify in proportion to the change:

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
tools/corpus_gate.sh
```

For a topology change, also run:

- the smallest reproducer;
- affected geometry-zoo fixtures;
- every release model;
- relevant density and override sweeps using `build/cli/weft sweep <model>`;
- visual inspection in the app and Blender when output quality is affected.

Compile success or application startup alone is not sufficient evidence.

## Documentation responsibilities

- `docs/EXECUTION_PLAN.md` owns product direction and work order.
- `README.md` owns current capabilities and basic usage.
- `tests/CAD_CORPUS.md` owns corpus mechanics.
- `WINDOWS_BUILD.md` owns Windows setup.

Do not duplicate roadmaps between these files. Update links and commands when
paths or interfaces change.
