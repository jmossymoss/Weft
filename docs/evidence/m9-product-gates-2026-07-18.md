# M9 product gates - 2026-07-18

## Linux release-candidate gates

| Gate | Result |
|---|---|
| linux-gcc product build (`weft`, `weft_app`) | GREEN |
| linux-gcc-static-analysis selected secure suites | GREEN (prior APP/MAP packets) |
| Supported-fixture CLI mesh matrix (10 shapes) | GREEN (WP-152) |
| Headless app screenshot matrix | GREEN (WP-152) |
| Legacy pipeline / full corpus (`tools/corpus_gate.sh`) | RED — pre-existing missing STEP fixtures (AGENTS.md) |
| Windows product gates | BLOCKED — no Windows host here |

## Status

WP-162 closed for Linux product path with named BLOCKED/RED items above.
