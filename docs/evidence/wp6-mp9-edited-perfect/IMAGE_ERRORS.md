# mp9_Edited — solid+wire image error analysis

Source gallery: `objects/object_XXX_v{0,1}.png` (deselected, solid+wire).
Captured from one `--finalize` load of `tests/STEP_Examples/mp9_Edited.stp`.

Assembly remains watertight (opens/NM/folds/floors = 0). This document lists
**visual / topology-quality** defects only — what to correct next.

Severity:
- **P0** — clearly wrong edge flow vs CAD intent on a major part
- **P1** — bad local class (tris/fans/crush) on a visible part
- **P2** — density imbalance or sparse n-gon panels; lower visual cost
- **OK** — acceptable for MVP (deliberate sparse flats / structured coils)

---

## Prioritized defects (from images + mesh arity)

### D1 — P0 — Object 5 muzzle/suppressor: under-columned IsoBand drum

**Images:** `object_005_v0.png`, `object_005_v1.png`

**Seen:** Outer cylinder (teal) uses very long, sparse circumferential cells;
cutout “teeth” and orange capsule protrusions look denser than the main wall.
Wall does not read as a clean iso-azimuth column lattice.

**Measured:** Face 374 `drum/iso-band` edges=37 → `revolution-grid` **u=6 v=1**,
6 n-gons. Alone (rings=0 extract) the same face emits **~32** column n-gons.
`tryOpenBand=1` but `bandDriver=0`, so early orth wins with crushed `nu`.

**Class:** Coupled IsoBand castellation: open-band accepted but declined without
plain-rim `bandDriver`; orth density then collapses circumference.

**Verify after fix:** Re-shot object 5; expect denser, more even columns on the
teal wall; face 374 `u`/polys near alone spans; watertight retained.

**Verification (2026-08-06):** FIXED / improved.
- Before: face 374 `u=6`, 6 n-gons; wall looked coarse.
- After: open-band route (`tryOpenBand` even when `bandDriver=0`); face 374
  `u=14`, **170 polys** (109 quads + n-gons). Screenshots
  `before/object_005_*.png` vs `after_d1/object_005_*.png` show denser
  circumferential edge flow on the teal wall. Watertight retained.
- Residual: still below alone `nu≈32`; thin spanning quads remain on some
  outer panels — acceptable vs prior crush; further densify optional.

---

### D2 — P1 — Object 19: collapsed / blocky freeform body

**Images:** `object_019_v0.png` (and v1)

**Seen:** Part looks “melted” / faceted lumps; silhouette does not read as
clean CAD hard-surface. Wire is extremely sparse.

**Measured:** Only **3 polygons**, all n-gons (100% n-gon). Entire solid
effectively MinimalNGon-collapsed.

**Class:** Over-aggressive MinimalNGon / failed structured claim on a
multi-face freeform body.

**Verify after fix:** Object 19 must show structured face flow (Coons/revgrid/
planar panels), not 3 blobs; poly count ≫ 3.

**Verification (attempted):** PARKED — tradeoff.
- Tip-fold → MinimalNGon is fold-free but shades as a melted blob (sparse
  single n-gon on a helical freeform).
- Keeping Coons yields a readable coil (~224 quads) in solid+wire shots but
  leaves **3 foldedPolys** on face 1828 (fails `testMp9EditedWatertight`).
- Cell winding flips cleared folds locally but opened 6 seam edges against
  neighbouring MinimalNGon planes.
- Needs a non-planar edge-exact n-gon or Coons geoheal that preserves seams.
  Screenshots: `before_d2/` (blob) vs `after_d2/` (coil + 3 folded cells).

---

### D3 — P1 — Object 37: high-triangle fillet bracket

**Images:** `object_037_v0.png`

**Seen:** L-bracket; dense thin cells on tan fillet bands; flat grey faces
OK but blend zones look triangulated.

**Measured:** 102 polys, **18.6% tris** (highest among mid-size solids).

**Class:** Fillet-strip / Coons tip→tri pairing or fold overlay on short blends.

**Verify after fix:** Object 37 tri% drops substantially; fillet bands read as
across/along strips.

---

### D4 — P1 — Object 2: radial n-gon plate (notched disk)

**Images:** `object_002_v0.png`

**Seen:** Flat disk with perimeter notches + hex bore; large irregular n-gons
with long spokes from hex to rim; slivery cells at hex corners.

**Measured:** 599 polys, **16% n-gons**.

**Class:** Hole-plate / planar panel web should collar the hex and notch
pockets without spanning the disk in one fan of n-gons.

**Verify after fix:** Cleaner collar + web; fewer spanning n-gons on the flat.

---

### D5 — P1 — Objects 8 & 9: freeform density extremes

**Images:** `object_008_v0.png`, `object_009_v0.png`

**Seen (8):** Slide/rail — dense flats, orange patch faces, thin tris at nose
cone transitions.
**Seen (9):** Main body — grips extremely dense horizontal loops; receiver
sides sparse large n-gons; uneven artist-edit density.

**Measured:** Obj8 1184 polys (17% n); Obj9 3594 polys (12% n).

**Class:** Mix of (a) adaptive over-density on grip freeform and (b) curved /
trimmed walls left as MinimalNGon or under-gridded Coons.

**Verify after fix:** Grip density closer to neighbouring body; receiver
sidewalls show readable Coons/rev flow where curved.

---

### D6 — P2 — Object 1 optic mount: curvature over-density

**Images:** `object_001_v0.png`

**Seen:** Bridge arch extremely dense vs base flats; holes OK; some n-gons at
transitions.

**Measured:** 1367 polys, mostly quads (87%) — structured but uneven.

**Class:** Adaptive curvature floor too aggressive on shallow arches.

**Verify:** Arch segment count closer to base without losing silhouette.

---

### D7 — P2 — Objects 3, 14, 24: cylinder density steps

**Images:** `object_003_v0.png`, `object_014_v0.png`, `object_024_v0.png`

**Seen:** Abrupt density rings (teal band on 3; base ring on 14; coarse teal
barrel on 24 vs denser orange). Faceting on coarse cylinders.

**Class:** Circumferential count mismatch across co-axial stack / annulus
floors.

**Verify:** Softer density transitions; co-axial counts more continuous.

---

### D8 — P2 — Objects 22, 32: elevated tris on long rails

**Images:** `object_022_v0.png`, `object_032_v0.png`

**Measured:** Obj22 8.7% tris; Obj32 10.2% tris on 700–1100 polys.

**Class:** Serration / hole cutouts → local tri fans.

**Verify:** Tri% down; serration loops stay readable.

---

### D9 — P2 — Small high-n-gon / high-tri fasteners

**Objects:** 6, 7 (33% n); 34 (27% t, small); 41–43 (high tri, small);
60–61 (43% n); 15, 19 already covered.

**Seen:** Hex nuts (6/7) may hide wire on tiny meshes; mostly acceptable as
sparse panels unless silhouette breaks.

**Action:** Fix only if after D1–D5 still ugly; otherwise park.

---

## Explicit OK (no fix this pass)

| Objects | Why OK |
|--------:|--------|
| 56, 57 | Mag springs — structured coil quads |
| 21, 27, 52, 59 | High quad ratio / trivial |
| 11, 13, 16, 17, 25, 26, 28, 36, 38, 47, 48, 51 | Small/simple; no image red flags beyond sparse n-gons |
| Watertight assembly | Validity already green |

---

## Fix order (implement only after this analysis)

1. **D1** Object 5 IsoBand column crush  
2. **D2** Object 19 collapsed solid  
3. **D3** Object 37 high-tri fillet  
4. **D4** Object 2 plate web  
5. **D5** Objects 8/9 freeform density / MinimalNGon  
6. **D6–D8** as time allows  

For each: implement shared-class fix → `--screenshot-object N` before/after →
assess in `IMAGE_ERRORS.md` “Verification” section → commit.
