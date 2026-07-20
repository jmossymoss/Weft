# NIST / CAx-IF interoperability corpus

Role: public STEP import interoperability authority for AP203/AP242-style
files, units, and kernel-produced geometry. Not a geometry-coverage oracle
(that is the deterministic zoo + `COVERAGE_MATRIX.tsv`).

## License

NIST CAD models and STEP files from the MBE PMI / CAx-IF program may be used
without restriction (see the NIST download page). The two small fixtures
committed under `tests/fixtures/interop/` come from the public-domain
[usnistgov/engineering-design-models](https://github.com/usnistgov/engineering-design-models)
repository (NIST Terms of Use / public domain in the United States).

## Committed smoke fixtures

| File | Upstream path | Notes |
| --- | --- | --- |
| `tests/fixtures/interop/nist_pivot_pin.step` | `models/CMU/scanner-2/pivot-pin.hh.sat.stp` | ~12 KB solid |
| `tests/fixtures/interop/nist_yaw_shaft.step` | `models/CMU/seeker/seeker/yaw_shaft.hh.sat.stp` | ~9 KB solid |

These are redistributed for offline CI. Manifest rows in `nist_interop.tsv`
point at cache copies populated by the fetch script (which prefers the
committed fixtures, then falls back to GitHub raw URLs).

## Upstream indexes (broader set)

- [NIST MBE PMI CAD models and STEP files](https://www.nist.gov/ctl/smart-connected-systems-division/smart-connected-manufacturing-systems-group/mbe-pmi-0)
  — FTC/CTC/STC AP203 and AP242 with PMI; CAx-IF interoperability suite.
  Direct curl of individual FTC/STC archives is not stable from this repo;
  download manually into `tests/public_corpus/_cache/nist/` when expanding.
- [NIST Public Data Repository — CAD models with PMI](https://data.nist.gov/od/id/mds2-1452)
- [MBx-IF / CAx resources](https://www.mbx-if.org/home/cax/resources/)
- [engineering-design-models](https://github.com/usnistgov/engineering-design-models)

## Fetch

```sh
tools/fetch_public_corpus.sh nist
tools/fetch_public_corpus.sh status
```

Cache root: `WEFT_PUBLIC_CORPUS_ROOT` or `tests/public_corpus/_cache`.
Paths in `nist_interop.tsv` are relative to that cache root.

## Gate

Public rows run only when `WEFT_RUN_PUBLIC=1` (see `tools/public_corpus_gate.sh`).
Default CI does not require the cache; coverage comes from the zoo.
