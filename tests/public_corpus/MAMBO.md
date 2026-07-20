# MAMBO stress supplement

Role: public meshing / hex-blocking topology stress authority. Complements ABC
(breadth) and NIST/CAx-IF (interop); not a geometry-coverage oracle.

Upstream: [franck.ledoux/mambo](https://gitlab.com/franck.ledoux/mambo)
(Apache-2.0). Models live under `Basic/`, `Simple/`, `Medium/`.

## Fetch

```sh
tools/fetch_mambo_corpus.sh
# equivalent:
tools/fetch_public_corpus.sh mambo
```

Shallow sparse-clones MAMBO into `tests/public_corpus/_cache/mambo/src`
(Basic/Simple/Medium only; skips bulky Misc/Evolutive trees) and copies the
manifest cases to the cache paths listed in `mambo_stress.tsv`.

Environment:

- `WEFT_PUBLIC_CORPUS_ROOT` — cache root (default `tests/public_corpus/_cache`)
- `WEFT_MAMBO_ROOT` — optional existing checkout; skips clone when set
- `WEFT_MAMBO_REPO` — override clone URL

Cache is gitignored. Commit only manifests, fetch scripts, and docs.

## Manifest

`mambo_stress.tsv` paths are relative to the cache root. Smoke set is intentionally
small (about a dozen models across Basic / Simple / Medium).

| validity | Gate expectation |
| --- | --- |
| `closed_solid` | Mesh must succeed and be watertight (open/non-manifold edges fail) |
| `research_stress` | Import/mesh may fail or be non-watertight; recorded as a note, not a gate failure |

Medium rows with dense holes / high face counts are marked `research_stress`
because MAMBO targets difficult hex-blocking configurations, not release solids.

## Gate

```sh
WEFT_RUN_PUBLIC=1 tools/public_corpus_gate.sh mambo
# or include with other public layers:
WEFT_RUN_PUBLIC=1 tools/public_corpus_gate.sh all
```

Requires a built `weft` binary and fetched cache files. Missing cache skips
cleanly (exit 0). Not part of the default corpus / release gates.
