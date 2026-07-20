# WP1 evidence — NIST / CAx-IF public interop corpus (2026-07-20)

## Goal

Wire NIST / CAx-IF STEP models as the interoperability public corpus layer
(import, assemblies, units, metadata). Not retopology quality. Not Fusion 360.
Not local artist `STEP_Examples`.

## Upstream

Index:
https://www.nist.gov/ctl/smart-connected-systems-division/smart-connected-manufacturing-systems-group/mbe-pmi-0

Archives (official NIST URLs):

| archive | URL | SHA-256 |
| --- | --- | --- |
| `NIST-PMI-STEP-Files.zip` | https://www.nist.gov/system/files/documents/noindex/2024/06/19/NIST-PMI-STEP-Files.zip | `8fa78429e6d8d9b0d7681d223b6aa9ec98c3772185c55b1a0e3679b21c181911` |
| `NIST-D2MI-Models.zip` | https://www.nist.gov/system/files/documents/el/msid/infotest/NIST-D2MI-Models.zip | `f20e36fb68633129dfedadf209ba4836bb0e42e18f12e7a2c28140ecdfb26cf3` |

## Curated on-disk set (`tests/public_corpus/_cache/nist/`, gitignored)

| id | relative path | tags |
| --- | --- | --- |
| `NIST_CTC01_AP203` | `nist/ap203/nist_ctc_01_asme1_rd.stp` | ap203, import, units_mm |
| `NIST_CTC01_AP242` | `nist/ap242/nist_ctc_01_asme1_ap242-e1.stp` | ap242, metadata, pmi |
| `NIST_FTC07_AP203` | `nist/ap203/nist_ftc_07_asme1_rd.stp` | ap203, units_inch, assembly_mate |
| `NIST_FTC08_AP203` | `nist/ap203/nist_ftc_08_asme1_rc.stp` | ap203, units_inch, assembly_mate |
| `NIST_STC06_AP242` | `nist/ap242/nist_stc_06_asme1_ap242-e3.stp` | ap242-e3, metadata, pmi |
| `NIST_D2MI_905` | `nist/d2mi/827-9999-905.stp` | ap203, units_inch, cax-if |

All six files contain manifold solid / advanced B-rep entities. Validity in
`nist_interop.tsv` is `closed_solid` for import smoke. Official NIST README
notes that PMI STEP files are not guaranteed syntax-clean under SFA.

## Wiring

- Manifest: `tests/public_corpus/nist_interop.tsv`
- Docs: `tests/public_corpus/NIST.md`
- Fetcher: `tools/fetch_nist_corpus.sh` (`curl -L`, pinned SHA-256, curated extract)
- Thin wrapper: `tools/fetch_public_corpus.sh nist` → exec dedicated script

## Re-fetch

```sh
tools/fetch_nist_corpus.sh
# or
tools/fetch_public_corpus.sh nist
# force re-download:
WEFT_NIST_FORCE=1 tools/fetch_nist_corpus.sh
tools/fetch_public_corpus.sh status
```
