# Probes (retired)

Ad-hoc `probeNN.cpp` diagnostics under this directory were retired on
2026-07-20 as part of WP2.

Durable invariants they encoded live in maintained tests:

| Former probe theme | Maintained coverage |
| --- | --- |
| Folded-poly census (probe101) | `testPromotedProbeInvariants`, `testAllMesherStrategies` |
| Demotion / faceBuild census (probe78) | `testDemotionAttribution`, `testPromotedProbeInvariants`, corpus `max_raw`/`max_empty` |
| Stitch vs default faceBuild (probe89) | `testConcurrentGenerationSettings` (quarantined A/B only) |
| `faceAcross` blend axis (probe103) | `testFillet` |
| Conform on/off opens (probe85) | `testPromotedProbeInvariants` |
| Adaptive radius density (probe99) | `testAdaptiveDensity` |
| Fillet analysis flags (probe95) | `testFillet` |

Face-ID one-offs (foam 294/481/183, demo face 18, stitch residual dumps) were
deleted without promotion — they are not product routing and are superseded by
corpus / `KNOWN_RED` / release-failure class fixtures.

Do not add new probes here during stabilization. Prefer a fixture assertion in
`tests/test_pipeline.cpp` or a corpus row in `tests/CAD_CORPUS.tsv`.

Evidence: `docs/evidence/wp2-probe-retirement-2026-07-20.md`.
