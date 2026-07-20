# MAMBO stress supplement

Role: public meshing-topology stress authority. Complements ABC (broad
diversity) and NIST/CAx-IF (interop); not a geometry-coverage oracle.

Upstream: [franck.ledoux/mambo](https://gitlab.com/franck.ledoux/mambo)
(Apache-2.0). Models live under `Basic/`, `Simple/`, `Medium/`.

## Fetch

```sh
tools/fetch_public_corpus.sh mambo
```

This shallow-clones MAMBO into the cache (or reuses an existing clone) and
copies the manifest cases into paths listed by `mambo_stress.tsv`.

Environment:

- `WEFT_PUBLIC_CORPUS_ROOT` — cache root (default `tests/public_corpus/_cache`)
- `WEFT_MAMBO_ROOT` — optional existing checkout; skips clone when set

## Manifest

`mambo_stress.tsv` paths are relative to the cache root. Placeholders are
replaced by the fetch script; do not commit MAMBO STEP files into this repo.

## Gate

Enabled only with `WEFT_RUN_PUBLIC=1` via `tools/public_corpus_gate.sh`.
