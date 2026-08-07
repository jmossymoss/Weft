# mp9_Edited — per-object mesh verification

Tip (D1+D3+D10+D11 scale spans). Gallery: `objects/object_XXX_v{0,1}.png`.

## Assembly

| Metric | Value |
|--------|------:|
| open / NM | 0 / 0 |
| foldedPolys | 0 |
| failed-floor / raw | 0 / 0 |
| watertight | yes |

## Per-object (65 solids)

| Verdict | Count | Objects |
|---------|------:|---------|
| PASS | 63 | all except 19, 37 |
| WARN | 2 | 19 (D2 parked collapsed MinimalNGon), 37 (D3 partial high-tri 15%) |
| FAIL | 0 | — |

Full machine table: `verify_all.out`.

## Completed fixes this campaign

| ID | Change | Verify |
|----|--------|--------|
| D1 | IsoBand open-band without bandDriver (≥32 edges) | object 5 before_d1/after_d1 |
| D3 | Small iso-band cylinder fillets → MinimalNGon | object 37 before_d3/after_d3 |
| D10 | nv==1 notch lips snap to top strip (no support rings) | object 5 before_d10/after_d10 |

## Remaining WARN (accepted / parked)

| Obj | Issue | Next step |
|----:|-------|-----------|
| 19 | 3-face solid as 3 n-gons (melted) vs Coons+3 folds | seam-safe Coons geoheal |
| 37 | 15% tris on remaining freeform Coons | further fillet/freeform tip cleanup |

## Re-verify

```sh
xvfb-run -a build/app/weft_app tests/STEP_Examples/mp9_Edited.stp \
  --finalize --screenshot-objects OUTDIR
# build /tmp/verify_all against weft_core (see session notes)
```
