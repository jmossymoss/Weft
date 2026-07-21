# WP5 — public corpus smoke + target-asset check (2026-07-21)

Active package: WP5 — validate real work.

## Public smoke (declared validity expectations)

| Layer | Command | Result |
| --- | --- | --- |
| NIST / CAx-IF | `WEFT_RUN_PUBLIC=1 tools/public_corpus_gate.sh nist` | PASS (6/6) |
| MAMBO | `WEFT_RUN_PUBLIC=1 tools/public_corpus_gate.sh mambo` | PASS (13; B1 research_stress exit allowed) |
| ABC nightly | `WEFT_RUN_PUBLIC=1 tools/public_corpus_gate.sh abc` | PASS (6/6) after resampling |

ABC sampling:

- Downloaded `abc_0000_step_v00.7z` into gitignored `_cache/abc/archives/`.
- `tools/sample_abc_nightly.py` with `WEFT_ABC_ROOT=…/unpacked` (seed 42).
- Seed-42 mid-band `00000792_…_step_017.step` hung meshing (>20 min, empty
  log) under `--profile cad`. Replaced with
  `00004457_50a5c011c1d44d7ca5c90fbf_step_011.step` (meshes in <1 s,
  watertight). Manifest updated in `tests/public_corpus/abc_nightly.tsv`.
- Hang class: record as import/mesh stall on a specific ABC STEP — not yet
  reduced to a deterministic zoo case (needs a shareable reproducer smaller
  than the full STEP).

## Optional Fusion Gallery smoke (not public authority)

Cached 20-case smoke under `_cache/fusion360/smoke/`:

- 18/20 watertight under `--profile cad --validate`.
- `74287_800ffc81_0`: non-manifold edges=38, open=0, 16 contract-floor.
- `94008_7f295f63_0`: open edges=401, 1 raw demotion (`border contract failed`
  face 91).

Both map to existing failure-class vocabulary (non-manifold weld / border
contract → raw). Not release-blocking; optional layer only.

## Target hard-surface / Plasticity-class assets

`tools/release_gate.sh` PASS on this machine (includes foam, teleporter,
flaregun, iso14649-demo). Spot validate:

| Model | Watertight | Notes |
| --- | --- | --- |
| foam | yes | contract-floor demotions, 0 raw |
| teleporter | yes | contract-floor demotions, 0 raw |
| flaregun | yes | contract-floor demotions, 0 raw |
| iso14649-demo | yes | clean |

No separate private “fresh Plasticity export” bundle is in-repo. Release
`STEP_Examples` remain the available target-asset stand-in. Blender ↔
Plasticity side-by-side (section 7) still needs artist Plasticity mesh
exports alongside these STEPs.

## Visual (Weft-only)

See `docs/evidence/wp5-visual-rubric-2026-07-21.md` and
`docs/evidence/wp5-visual/` — release hard-surface set still watertight under
CAD profile; Plasticity side-by-side still pending artist mesh exports.

## Remaining for WP5 exit

- Reduce the ABC hang (`00000792`) and/or Fusion open/NM cases into
  deterministic zoo/regression fixtures when shareable.
- Fresh Plasticity export set + Blender visual compare vs Plasticity output.
- Confirm no new common failure class remains outside the zoo / failure TSV.
