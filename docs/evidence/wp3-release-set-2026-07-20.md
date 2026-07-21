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

## Visual + cross-platform

- Section 7 viewport review: `docs/evidence/wp3-visual-rubric-2026-07-20.md`
  and `docs/evidence/wp3-visual/`.
- Cross-platform: CI emits Linux release signatures
  (`tools/topology_signature.sh release-set`) and Windows compares them
  with `weft signature-compare` (job `windows` needs `linux`).

## Active package

WP3 exit criteria pass on this revision train. Active work package advanced
to WP4 — complete the artist correction loop.
