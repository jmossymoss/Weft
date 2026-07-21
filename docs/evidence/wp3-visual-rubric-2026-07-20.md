# WP3 — visual rubric review, release set (2026-07-20)

## Method

Headless Weft viewport captures via `tools/visual_check.sh` with
`--finalize` so screenshots show the production export mesh (not the
interactive preview). CLI `weft mesh --profile cad --validate` logs sit
beside each model in `docs/evidence/wp3-visual/report.html`.

Artifacts: `docs/evidence/wp3-visual/<model>_v{0,1,2}.png` (three camera
angles). Same files under `/opt/cursor/artifacts/wp3-visual/`.

Plasticity side-by-side Blender compare is deferred to WP5 (fresh
Plasticity set); this review covers Weft viewport rubric items for the
five release models.

## Rubric checklist (CAD / adaptive finalize)

| Item | flaregun | foam | teleporter | torture | iso14649-demo |
| --- | --- | --- | --- | --- | --- |
| Silhouette follows CAD at selected density | pass | pass | pass | pass | pass |
| Cylinder / revolution columns intentional | pass | pass | pass | pass | pass |
| Fillet strips readable | pass | pass | pass | n/a | n/a |
| Holes/slots get local collars (no global fans) | pass | pass | pass | pass | pass |
| Flat regions stay sparse (n-gon panels) | pass | pass | pass | pass | pass |
| Poles / tris / n-gons look deliberate | pass | pass | pass | pass | pass |
| No folded / overlapping / spiraling / hair-thin | pass | pass | pass | pass | pass |
| Normals / hard-surface shading stable | pass | pass | pass | pass | pass |
| Density transitions without visible seam tears | pass | pass | pass | pass | pass |
| UI reports watertight (finalize) | pass | pass | pass | pass | pass |

## Notes

- App `--finalize` was added so visual QA matches CLI export. Preview
  previously showed open border loops while finalize was already sealed.
- Foam / teleporter retain contract-floor regions (reported, watertight);
  wireframe shows denser tris there, not raw OCCT fans.
- Iso14649-demo and torture show clean bore collars and sparse flats.
- Flaregun fillet body (`Fillet26`) carries coherent coons/revolution flow.

## Verdict

Relevant section 7 items for the release set pass with saved viewport
evidence. Blender ↔ Plasticity comparison remains a WP5 real-work item.
