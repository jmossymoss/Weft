# WP6 ABC nightly visual check (2026-07-23)

Screenshots from `tools/visual_check.sh` with CLI `--profile cad` (app
rebuild required so screenshots match the linked core).

| Model | CAD WT | Slivers | Floors | Visual notes |
| --- | --- | --- | --- | --- |
| 00005908 | yes | 0 | 0 | Clean small solid |
| 00004457 | yes | 42 | 0 | Acceptable hard-surface |
| 00009645 | yes | 0 | 0 | Acceptable (slivers cleared vs prior 36) |
| 00006051 | yes | 56 | 2 | Residual floors remain — next class |
| 00008536 | yes | 136 | 0 | Inner drum RevolutionGrid; 32 local web folds at teeth |
| 00002324 | yes | 264 | 0 | Tall FreeTrim drums → RevolutionGrid (were MinimalNGon needles); source shape is tall towers |

Regenerate:

```sh
WEFT_RUN_PUBLIC=1 tools/public_corpus_gate.sh abc   # fetch STEPs if needed
mkdir -p /tmp/abc_visual_src
# link nightly STEPs into /tmp/abc_visual_src as <id>.step
cmake --build build -j --target weft weft_app
bash tools/visual_check.sh /tmp/abc_visual_src docs/evidence/wp6-abc-visual
```

Open `report.html` for the full gallery.
