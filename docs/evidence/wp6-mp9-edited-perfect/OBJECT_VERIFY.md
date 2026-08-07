# mp9_Edited — per-object mesh verification

Tip after D1+D3+D10. Gallery: `objects/object_XXX_v{0,1}.png` (solid+wire, deselected).

## Assembly

| Metric | Value |
|--------|------:|
| open / NM | 0 / 0 |
| foldedPolys | 0 |
| failed-floor / raw | 0 / 0 |
| watertight | yes |

## Per-object verdicts

Automated from CAD `generate()`: FAIL = floor/raw/folds; WARN = high-tri ≥15% (p≥40), high-ngon ≥35%, or collapsed (≤3 polys on ≥3 faces).

| Obj | Faces | Polys | q/t/n | Verdict | Notes |
|----:|------:|------:|-------|---------|-------|
| 1–18, 20–36, 38–65 | — | — | — | **PASS** | See `/tmp/verify_all.out` full table |
| 19 | 3 | 3 | 0/0/3 | **WARN** | D2 parked — MinimalNGon blob vs Coons+folds |
| 37 | 26 | 60 | 41/9/10 | **WARN** | D3 partial — tri% 15% (was 18.6%) |

**Summary: 63 PASS, 2 WARN, 0 FAIL.**

## Fix status linked to objects

| Defect | Objects | Status |
|--------|---------|--------|
| D1 IsoBand columns | 5 | DONE |
| D2 collapsed freeform | 19 | PARKED |
| D3 fillet tris | 37 | PARTIAL |
| D10 cylinder support rings | 5 (+ drums) | DONE |
| D4 plate web | 2 | OPEN (PASS metrics; visual n-gons remain) |
| D5 freeform density | 8, 9 | OPEN (PASS metrics; density uneven) |
| D6–D8 | 1, 3, 14, 24, 22, 32 | OPEN (PASS metrics) |

## Re-run

```sh
xvfb-run -a build/app/weft_app tests/STEP_Examples/mp9_Edited.stp \
  --finalize --screenshot-objects /opt/cursor/artifacts/mp9_object_qa
# + /tmp/verify_all (see tools recipe in IMAGE_ERRORS)
```
