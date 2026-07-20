# WP3 — release set sealed on Linux (2026-07-20)

## Scope

Five release models (default + CAD): flaregun, iso14649-demo, torture,
foam, teleporter.

## §3.1 baseline

| model | profiles | open | NM | folds | raw | empty | watertight |
| --- | --- | --- | --- | --- | --- | --- | --- |
| flaregun | default, cad | 0 | 0 | 0 | 0 | 0 | yes |
| iso14649-demo | default, cad | 0 | 0 | 0 | 0 | 0 | yes |
| torture | default, cad | 0 | 0 | 0 | 0 | 0 | yes |
| foam | default, cad | 0 | 0 | 0 | 0 | 0 | yes |
| teleporter | default, cad | 0 | 0 | 0 | 0 | 0 | yes |

Contract-floor demotions remain where structured meshers cannot express a
face; they are watertight, local, and reported (allowed by §3.1).

`bash tools/release_gate.sh` → PASS (KNOWN_RED ignored).

## §3.2 density sweeps (CAD profile)

| model | runs | failures |
| --- | ---: | ---: |
| flaregun | 138 | 0 |
| iso14649-demo | 144 | 0 |
| torture | 84 | 0 |
| foam | 786 | 0 |
| teleporter | 438 | 0 |

## Class-level fixes (this train)

1. MinimalNGon / Coons←MinimalNGon stitch insertion protect.
2. Open-curve stitch midpoint accept when union-chain sample intervenes.
3. Density-cap wrap-scaled revolution shares via relativeDeviation.
4. Digon-chord floor (count ≥ 2) for micro-edge collapse NM.
5. Exact-border floor preferred over raw; fold repair / self-heal.
6. Radial-raise propagation through blend/density groups; adaptive-path
   override demotes edited revolution-family face to contract floor.
7. Face-cache pin fingerprint for warm sweep correctness.

No filename / model-name / face-ID product specials.

## KNOWN_RED

No release-model rows remain. Residual allowances are stress/dirty only:

- slitdrill (input non-manifold mirrored on CAD)
- tan_slit (dirty-step max_raw)
- tork (broken / open-shell source)

## Still open for full WP3 exit

- Visual rubric evidence (section 7) with saved region reviews.
- Windows cross-platform confirmation of §3.2 policy (topology signature
  artifact exists; Windows CI run still required).

Active package remains WP3 until those exit items land at one revision.
