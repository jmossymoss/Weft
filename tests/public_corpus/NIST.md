# NIST / CAx-IF interoperability corpus

Role: STEP import smoke for AP203/AP242, units, assembly-mate parts, and
kernel-produced metadata/PMI. Not a geometry-coverage oracle (that is the
deterministic zoo + `COVERAGE_MATRIX.tsv`) and not a retopology-quality set.

## License

NIST CAD models and STEP files from the MBE PMI / CAx-IF program may be used
without restriction (NIST disclaimer on the download page). Acknowledgement is
appreciated; do not use the NIST logo in promotional materials.

## Upstream

- [Download Free CAD Models, STEP Files, and Test Results](https://www.nist.gov/ctl/smart-connected-systems-division/smart-connected-manufacturing-systems-group/mbe-pmi-0)
- Archives used by the fetcher:
  - `NIST-PMI-STEP-Files.zip` — AP203 geometry-only, AP203+PMI, AP242 CTC/FTC/STC
  - `NIST-D2MI-Models.zip` — Design-to-Manufacturing-and-Inspection STEP samples
- Mirror of the PMI STEP zip also ships with
  [usnistgov/SFA](https://github.com/usnistgov/SFA/tree/master/Release)
  (`NIST-PMI-STEP-Files.zip`); the fetcher prefers the official NIST URL.

## Curated smoke set

Paths below are relative to `tests/public_corpus/_cache/` (gitignored).

| id | path | purpose tags |
| --- | --- | --- |
| `NIST_CTC01_AP203` | `nist/ap203/nist_ctc_01_asme1_rd.stp` | ap203, import, units_mm |
| `NIST_CTC01_AP242` | `nist/ap242/nist_ctc_01_asme1_ap242-e1.stp` | ap242, metadata, pmi |
| `NIST_FTC07_AP203` | `nist/ap203/nist_ftc_07_asme1_rd.stp` | ap203, units_inch, assembly_mate (box) |
| `NIST_FTC08_AP203` | `nist/ap203/nist_ftc_08_asme1_rc.stp` | ap203, units_inch, assembly_mate (lid) |
| `NIST_STC06_AP242` | `nist/ap242/nist_stc_06_asme1_ap242-e3.stp` | ap242-e3, metadata, pmi |
| `NIST_D2MI_905` | `nist/d2mi/827-9999-905.stp` | ap203, units_inch, cax-if |

FTC-07/08/09/10 are documented by NIST as parts that fit together in an
assembly; this smoke set keeps the box+lid pair as assembly-mate coverage.
Official NIST STEP files may report syntax warnings in SFA; geometry validity
for Weft is recorded per row in `nist_interop.tsv`.

## Fetch

```sh
tools/fetch_nist_corpus.sh
# or
tools/fetch_public_corpus.sh nist
tools/fetch_public_corpus.sh status
```

Re-download and re-extract:

```sh
WEFT_NIST_FORCE=1 tools/fetch_nist_corpus.sh
```

Archives land under `_cache/nist/archives/` with SHA-256 sidecars. Expected
digests are pinned in `tools/fetch_nist_corpus.sh`.

Cache root: `WEFT_PUBLIC_CORPUS_ROOT` or `tests/public_corpus/_cache`.

## Gate

Public rows run only when `WEFT_RUN_PUBLIC=1` (see `tools/public_corpus_gate.sh`).
Default CI does not require the cache; coverage comes from the zoo.
