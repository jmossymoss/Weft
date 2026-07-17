# M0 legacy baseline - 2026-07-17

## Revision and environment

- repository: `D:/Weft`
- revision: `18e1d28e9cc5302028d5bc2f915b35ddc2c514f3`
- branch before rewrite: `WIP/primitive-aware-compiler`
- command: `ctest --test-dir build -C Release --output-on-failure`
- elapsed CTest time: 70.06 seconds

Untracked `.codex-remote-attachments`, `tests/STEP_Examples/MP9.fbx`, and
`tests/STEP_Examples/MP9_Plasticity.mtl` were present before the run and are
excluded from rewrite scope.

## Outcome

CTest completed; the single aggregate `pipeline` test failed with eight
assertions. The generated primitive/editing/recipe/cache checks and layered
CAD corpus continued through completion.

Pre-existing assertions:

1. four MP9 faces expected all-quads but contained 1-2 non-quads;
2. MP9 face 1310 contained 32,306 polygons against a maximum of 40;
3. flaregun was not watertight;
4. flaregun contained a folded polygon;
5. foam was not watertight.

The aggregate printed `8 FAILURE(S)`. These are baseline defects, not accepted
exceptions. New core work must either preserve them as known failures or close
them with proof; it may not suppress, weaken, or relabel the assertions.

## Corpus observations

- 22 generated/committed entries reported watertight output.
- `slitdrill` remained an explicitly known open issue.
- the five reduced MP9 regressions retained their recorded raw/empty/open
  conditions.
- no test was skipped because of the rewrite; this run precedes rewrite edits.
