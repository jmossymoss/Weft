# WP2 evidence — stitch A/B (AD-2) (2026-07-20)

## Goal

A/B `GenerationSettings::decoupleSeams` / CLI `--stitch` across the release set
at default and CAD profiles. Decide whether stitch may leave quarantine under
[AD-2](../EXECUTION_PLAN.md#ad-2-stitch-experiment).

Promotion criteria (AD-2): consistent advantage **and** no new correctness
failures on the release set. Otherwise keep quarantined.

## Method

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release && cmake --build build -j
tools/stitch_ab.sh
```

- Inventory: `tests/CAD_CORPUS.tsv` tier=`release` (torture, flaregun,
  iso14649-demo, foam, teleporter).
- Binary: `build/cli/weft validate` with and without `--stitch`; CAD via
  `--profile cad`.
- Metrics from validate output: open edges, non-manifold, raw demotions, empty
  faces, quad / tri / n-gon counts.
- Corpus / release gates were not modified and do not pass `--stitch`.

Reproduce: `OUT=/tmp/stitch-ab tools/stitch_ab.sh` (writes TSV + per-run logs).

## Results

Counts are `open / non-manifold / raw / empty` and `quads / tris / n-gons`.
Delta is stitch − baseline (negative open/NM/raw/empty is better).

| model | profile | baseline | stitch | delta | recommendation |
| --- | --- | --- | --- | --- | --- |
| torture | default | o=0 nm=0 raw=0 empty=0; 403/102/48 | o=8 nm=0 raw=0 empty=0; 325/36/80 | Δo=+8 Δnm=0 Δraw=0 Δempty=0; Δq/t/n=−78/−66/+32 | keep quarantined — **new opens on a previously watertight model** |
| torture | cad | o=0 nm=0 raw=0 empty=0; 471/47/35 | o=0 nm=0 raw=0 empty=0; 416/47/60 | Δo=0 Δnm=0 Δraw=0 Δempty=0; Δq/t/n=−55/0/+25 | keep quarantined — neutral correctness; no advantage |
| flaregun | default | o=0 nm=0 raw=0 empty=0; 4284/967/170 | o=9 nm=0 raw=0 empty=0; 4305/965/178 | Δo=+9 Δnm=0 Δraw=0 Δempty=0; Δq/t/n=+21/−2/+8 | keep quarantined — **new opens on a previously watertight model** |
| flaregun | cad | o=0 nm=0 raw=0 empty=0; 4930/46/142 | o=0 nm=0 raw=0 empty=0; 4387/218/203 | Δo=0 Δnm=0 Δraw=0 Δempty=0; Δq/t/n=−543/+172/+61 | keep quarantined — neutral correctness; fewer quads, more tris/n-gons |
| iso14649-demo | default | o=0 nm=0 raw=0 empty=0; 602/2/75 | o=0 nm=0 raw=0 empty=0; 602/2/75 | all Δ=0 | keep quarantined — identical; no advantage |
| iso14649-demo | cad | o=0 nm=0 raw=0 empty=0; 594/2/75 | o=0 nm=0 raw=0 empty=0; 594/2/75 | all Δ=0 | keep quarantined — identical; no advantage |
| foam | default | o=6 nm=2 raw=3 empty=0; 3618/1193/334 | o=6 nm=2 raw=3 empty=0; 3582/1146/408 | Δo=0 Δnm=0 Δraw=0 Δempty=0; Δq/t/n=−36/−47/+74 | keep quarantined — known-red unchanged; no watertight gain |
| foam | cad | o=12 nm=2 raw=0 empty=0; 5048/693/298 | o=12 nm=2 raw=0 empty=0; 5264/691/418 | Δo=0 Δnm=0 Δraw=0 Δempty=0; Δq/t/n=+216/−2/+120 | keep quarantined — known-red unchanged; no watertight gain |
| teleporter | default | o=18 nm=2 raw=3 empty=0; 3795/5500/433 | o=44 nm=2 raw=2 empty=0; 3617/3317/745 | Δo=+26 Δnm=0 Δraw=−1 Δempty=0; Δq/t/n=−178/−2183/+312 | keep quarantined — **more opens**; raw −1 does not offset |
| teleporter | cad | o=12 nm=0 raw=0 empty=0; 6996/2764/404 | o=24 nm=0 raw=0 empty=0; 5839/1820/702 | Δo=+12 Δnm=0 Δraw=0 Δempty=0; Δq/t/n=−1157/−944/+298 | keep quarantined — **more opens** |

## AD-2 decision

**Keep `--stitch` / `decoupleSeams` quarantined. Do not promote.**

Evidence against promotion:

1. **New correctness failures** on models that are watertight on the default
   path: torture default (+8 open), flaregun default (+9 open).
2. **Worse leaks** on teleporter (default +26 open, cad +12 open) with only a
   trivial raw improvement (−1) on default.
3. **No consistent advantage**: foam correctness metrics unchanged; iso14649
   identical; CAD torture/flaregun stay watertight but show no open/NM/raw/empty
   win and mixed arity tradeoffs.
4. Failures are not confined to known-red models — stitch breaks green release
   cases.

Defaults remain stitch-off (`GenerationSettings::decoupleSeams = false`; CLI/app
opt-in only). `tools/corpus_gate.sh` / `tools/release_gate.sh` do not pass
`--stitch`.

## Follow-ups

- Leave stitch as a diagnostic flag for probes and manual A/B only.
- WP3 watertight work stays on the border-contract default path, not stitch.
- Re-run `tools/stitch_ab.sh` only if stitch internals change enough to revisit
  AD-2; do not refresh this report to chase counts without a root-cause change.
