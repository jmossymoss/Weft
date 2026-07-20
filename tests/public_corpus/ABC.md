# ABC Dataset nightly sample

Role: public broad-diversity / nightly robustness authority (surface mix,
face-count bands). Not CI-blocking geometry coverage.

Upstream: [ABC Dataset](https://deep-geometry.github.io/abc-dataset/)

ABC is large and licensed for research use; it is **not** committed here.
Point `WEFT_ABC_ROOT` at a local STEP tree, then sample.

## Sample by face-count bands

```sh
export WEFT_ABC_ROOT=/path/to/abc/step   # directory tree of .step / .stp
tools/sample_abc_nightly.sh
# or:
tools/fetch_public_corpus.sh abc-nightly
```

`tools/sample_abc_nightly.sh` picks one file per band from
`abc_nightly.tsv` when `WEFT_ABC_ROOT` exists:

| Band | Face count |
| --- | --- |
| `1-100` | 1 ≤ faces ≤ 100 |
| `100-500` | 100 < faces ≤ 500 |
| `500-2000` | 500 < faces ≤ 2000 |

Face counts use `weft info` / mesh preamble when available; otherwise a cheap
STEP entity heuristic. Copied files land under
`tests/public_corpus/_cache/abc/` at the `local_path` from the manifest.

## Gate

`WEFT_RUN_PUBLIC=1` enables `tools/public_corpus_gate.sh`. Missing cache files
are skipped with a note (ABC is optional on developer machines).
