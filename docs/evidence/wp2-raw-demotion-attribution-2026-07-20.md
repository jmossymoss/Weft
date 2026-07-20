# WP2 evidence — raw-demotion attribution (2026-07-20)

## Goal

Audit where `generate()` marks raw OCCT triangulation (`faceBuild==1`) and
empty faces (`faceBuild==-1`), and make unsupported cases explicit by face id
and cause string in the report / CLI validate path. No new `MesherKind`, no
filename special cases, no routing changes.

## Where `faceBuild` is set

Authoritative write site: `weft::generate()` in `core/src/meshers.cpp`.

| Value | Meaning | How it is produced |
| --- | --- | --- |
| `0` | Planned mesher built | Default `fellBack[fid]` |
| `2` | Contract floor (exact borders) | `demote()` when floor verifies; planned `Fallback`/`QuadDominant` floor; fold self-heal swap |
| `1` | Raw OCCT triangulation | `demote()` when floor unavailable; planned Fallback when floor cannot express the face |
| `-1` | Empty face (no polygons) | `parts[fid].polygons.empty()` at report time (including fallback-threw) |

Report assignment (after merge):

```text
faceBuild[fid] = polygons.empty() ? -1 : int(fellBack[fid])
```

Excluded faces are omitted. Cache hits restore `fellBack` and the cause string
from `GenerationCache::CachedFace`.

## Single demotion funnel

Structured mesher failures share one lambda:

```text
demote(fid, face, surf, settings, why)
  → try meshContractFallback (verified → faceBuild=2)
  → else meshFallback / OCCT (faceBuild=1)
```

Cause strings passed into `demote()` (and now stored on the report):

- `orthogonal revolution grid failed`
- `castellated insert failed`
- `rim notch failed`
- `open band insert failed`
- `revolution insert failed`
- `tapered revolution failed` (and related revolution failures)
- `open band failed`
- `revolution grid failed`
- `coons failed`
- `minimal planar failed`
- `dome cap failed`
- `annulus c-ring failed` / `annulus ring failed`
- `plate web failed`
- `ribbon sweep failed`
- `quad fill failed`
- `rail ladder failed`
- `emitted nothing`
- `self-check failed`
- `border contract failed`
- `fold check failed`
- `mesher threw`

Non-`demote()` attributions added for explicit unsupported cases:

- `planned contract floor` — planned Fallback/QuadDominant verified floor
- `contract floor unavailable` — planned Fallback last-resort raw OCCT
  (previously left `fellBack==0`, under-reporting raw status)
- `fold self-heal → contract floor` — fold tournament kept the floor
- `fallback threw` / `emitted nothing` — empty leftovers

## Report / CLI surface

- `GenerationReport::faceBuildCause` — `FaceId → cause` for every non-zero
  `faceBuild` entry.
- `formatBuildDemotions(report)` — shared formatter used by
  `weft mesh` / `weft validate`.
- Output lists demotion counts plus `floor` / `raw` / `empty` face ids with
  causes, e.g. `1(planned contract floor)`.

Routing, mesher selection, and mesh geometry are unchanged; only attribution
and the planned-Fallback raw status bit were corrected to match the documented
`faceBuild==1` contract.

## Focused test

`testDemotionAttribution` in `tests/test_pipeline.cpp`:

- Fixture: `boss` (existing interaction-zoo solid).
- Forces `MesherKind::Fallback` on face 1 (generic forceMesher, not a
  filename/face-ID special case in production code).
- Asserts demotion count ≥ 1, `faceBuildCause[1]` non-empty, formatter text
  includes face id + cause, and a cache hit re-emits the same attribution.

Observed at this revision:

```text
face 1 build=2 cause="planned contract floor"
demotions floor=1 raw=0 empty=0 (cached ok)
```

## Conclusion

Raw / empty / contract-floor outcomes are attributed by face and cause through
`GenerationReport` and the CLI validate path. Unsupported planned-Fallback
faces that land on raw OCCT are no longer silent `faceBuild==0` builds.
