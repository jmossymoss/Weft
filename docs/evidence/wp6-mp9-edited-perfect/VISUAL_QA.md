# mp9_Edited — object-level visual QA

Method: open `tests/STEP_Examples/mp9_Edited.stp` once in `weft_app
--finalize --screenshot-objects`, isolate each outliner solid, frame visible
faces, capture two angles. Gallery: `objects/object_XXX_v{0,1}.png` (also under
`/opt/cursor/artifacts/mp9_object_qa/`).

Assembly CAD defaults remain watertight (21898 verts / 15796 polys /
13273 quads / 793 tris / 1730 n-gons).

## Rubric (§7)

For each scored object: silhouette, cylinder columns, fillet flow, hole
collars, sparse flats, deliberate n-gons/tris, no fans/hair-thin/spirals.

## Scorecard (primary objects)

| Object | Faces (B-rep) | Polys (q/t/n) | Verdict | Notes |
|-------:|-------------:|---------------|--------|-------|
| 1 | 130 | 1367 (1189/101/77) | pass w/ notes | Mount/housing; dense adaptive grid on curved bridge; hole collars readable |
| 2 | 159 | 599 (487/16/96) | pass w/ notes | Elevated n-gon share (16%); freeform panels mostly structured |
| 3 | 10 | 220 (212/4/4) | pass | Clean cylinder + end hole; axial/radial flow intentional |
| 4 | 61 | 130 (100/2/28) | pass w/ notes | Blocky body; n-gons on flats OK; vertex clusters at transitions |
| 5 | 33 | 313 (269/4/40) | **fail class** | Muzzle/suppressor cylinder with flutes; columns look under-resolved vs alone IsoBand drums (see defect A) |
| 8 | 304 | 1184 (913/66/205) | pass w/ notes | Slide/rail; high n-gon count on freeform/fillet mix |
| 9 | 868 | 3594 (2992/167/435) | pass w/ notes | Main receiver/body; largest solid; overall structured but busy freeform n-gons |
| 14 | — | 454 (425/0/29) | pass | Quad-dominant; clean |
| 22 | — | 701 (502/61/138) | pass w/ notes | Elevated tris+ngons |
| 32 | — | 1142 (909/117/116) | pass w/ notes | Long perforated rail; dense around holes |
| 37 | — | 102 (75/19/8) | **fail class** | Highest tri ratio (~19%) among mid-size solids; L-bracket / fillet cluster |
| 56 | — | 1210 (1160/48/2) | pass | Mag spring; structured coil quads |
| 57 | — | 1210 (1160/48/2) | pass | Mag spring twin; same |
| 65 | — | 126 (94/2/30) | pass w/ notes | Small; n-gon heavy |

Remaining solids (6–7, 10–13, 15–21, 23–31, 33–36, 38–55, 58–64): inspected in
gallery; no additional severity beyond “small fastener / sparse n-gon panel”
unless listed below. Full inventory: `OBJECTS.md`.

## Defects (shared-class hypotheses)

### A — Coupled IsoBand drum column crush (object 5 / face ~374 class)

- Alone, IsoBand muzzle drum (32 edges) emits ~32 revolution-grid column
  n-gons (`nu` matches spans).
- In the full assembly the same face plans as early orth with `nu=6 nv=1`
  (probe before cleanup): circumferential density crushed by neighbour pins /
  rim-sum coupling.
- Visual: object 5 fluted cylinder looks coarse vs the alone reducer
  `muzzle_two_fullheight_sides`.
- Class: density / pin ownership on Drum×IsoBand with multi-edge castellation,
  not a filename special.

### B — High-triangle fillet/bracket solids (object 37)

- ~19% tris on 102 polys; L-bracket with fillets.
- Suspect Coons/fillet demotion or tip-fold/tri pairing on short blends.
- Needs solid-focused why + reducer from object 37 faces.

### C — Heavy MinimalNGon / n-gon share on freeform bodies (objects 2, 8, 9)

- Assembly has 1730 n-gons; large bodies carry most.
- Many are deliberate sparse panels (OK); curved walls that should be
  Coons/revgrid but collapsed to single n-gons are not.
- Audit Freeform/Drum faces with polys≤2 and edgeIds≥6 on objects 8–9.

### D — Capture/UX (tooling, not mesher)

- Vertex point cloud in solid+wire still clutters screenshots; optional
  `--screenshot-objects` hide-verts flag for future QA.
- Fixed during this pass: frame-to-visible after isolate; clear selection
  overlay.

## Non-defects

- Global watertightness / zero opens-NM-folds on full model (Phases 2–4).
- Mag springs (56/57): structured, editable coils.
- Simple cylinders with holes (object 3): pass.

## Artifacts

- `objects/object_XXX_v0.png`, `_v1.png` — 65 solids × 2 views
- `/opt/cursor/artifacts/mp9_object_qa/` — same gallery
- Command:

```sh
xvfb-run -a build/app/weft_app tests/STEP_Examples/mp9_Edited.stp \
  --finalize --screenshot-objects /opt/cursor/artifacts/mp9_object_qa
```
