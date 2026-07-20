# WP2 evidence — probe retirement (2026-07-20)

## Goal

Convert useful `tools/probes` diagnostics into maintained fixture assertions and
retire superseded one-offs. No validity-check weakening, no mesher-routing
changes for foam/teleporter (WP3), no face-ID product routing.

## Inventory

Searched for:

- `tools/probes/probe*.cpp` (53 sources + relink scripts)
- `WEFT_PROBE` macros (none found)
- Env-gated dbg prints in core (`WEFT_*_DEBUG`, stitch diagnosis) — left in
  place; they are opt-in diagnostics, not a second production path
- Obsolete hard-coded face IDs in tests (flaregun density-edit face `81`)

`core/src/io/probes.cpp` is the IO format registry, not a diagnostic probe tree.

## Promoted

| Probe theme | Assertion | Where |
| --- | --- | --- |
| probe101 folded-poly census | CAD closed zoo (`cylinder`, `box`, `boss`, `fillet`) has 0 `foldedPolys` | `testPromotedProbeInvariants` |
| probe78 demotion census | Same fixtures: `faceBuild` never empty/raw; demotions have `faceBuildCause` | `testPromotedProbeInvariants` (+ existing `testDemotionAttribution`) |
| probe85 conform A/B | Cylinder unexplained opens/NM = 0 with `conformBorders` true and false | `testPromotedProbeInvariants` |
| probe89 stitch faceBuild | Default (coupled) path emits no empty faces; stitch remains A/B-only | `testConcurrentGenerationSettings` |
| probe103 `faceAcross` | Fillet strip discovered via `isFillet` reports across axis 1 or 2 | `testFillet` |
| probe95 / probe99 | Already covered | `testFillet`, `testAdaptiveDensity` |

Also: flaregun density-edit regression now selects a `RailLadder` face from the
generation report instead of hard-coding face id `81`.

## Retired (deleted)

All `tools/probes/probe*.cpp` and `relink*.sh` removed. Themes not promoted:

- Face-ID / model-specific dumps (foam 294/275/481, demo face 18, stitch edge
  pairs, UV/polygon dumps): superseded by corpus, `KNOWN_RED.tsv`, and
  `RELEASE_FAILURE_CLASSES.tsv` reducers.
- Stitch residual classifiers (probe86–91, 87–88): AD-2 quarantine; default
  path assertions above; no stitch promotion.
- One-off surface/adaptor/coons ranking dumps (probe92–94, 98, 102): exploration
  only; zoo classification tests already cover surface/curve/mesher families.
- Open-shell twin-edge / sew forensics (probe65–70, 74–75): dirty-step corpus +
  `testDirtyStepFixtures` own validity policy; not product face routing.

Directory now holds only `README.md` pointing at maintained tests.

## Explicit non-goals

- No changes to `weft::generate()` routing for foam/teleporter classes.
- No new `MesherKind`, no filename special cases.
- Validity ceilings (`max_raw`, `max_empty`, watertight requirements) unchanged.
- `tests/fixtures/interop/` left untouched.

## Conclusion

Useful probe invariants are CI assertions. The ad-hoc probe tree is retired;
further diagnosis should add fixture/corpus coverage, not revive `probeNN.cpp`.
