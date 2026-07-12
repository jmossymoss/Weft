# CAD regression corpus

The corpus is deliberately layered:

- `fixtures/generated` contains small OCCT-authored feature fixtures. These
  isolate surface and topology families and run in the fast CTest gate.
- `regressions/mp9` contains real faces reduced from MP9 with one ring of
  adjacent CAD faces. They preserve the failure context while running in less
  than a second each.
- `STEP_Examples/MP9.stp` is the hero integration/performance model. It is not
  part of the fast gate.
- `visual_baselines` contains viewport captures with wireframe and diagnostic
  overlays. They are reviewed alongside numeric results; they are not treated
  as pixel-perfect golden images.

`CAD_CORPUS.tsv` is the machine-readable inventory. Failure limits are maxima:
fixing a raw or empty face is allowed, while introducing additional failures is
not. Cases marked `require_watertight=0` are either intentionally extracted
open neighborhoods or known topology defects.

## Reducing a new real-world failure

```powershell
weft extract model.step --faces 42 --rings 1 -o tests/regressions/model_face_42.step
```

Keep the failing face and the smallest adjacency radius that reproduces the
problem. Add the result to `CAD_CORPUS.tsv`, generate a viewport screenshot,
and record what is visibly wrong rather than only its polygon counts.

## Current visual findings

- MP9 127 and the 243-247 cluster are circular/annular trimmed surfaces whose
  fallback triangulation produces dense fans and visible folded cells.
- MP9 1692 is a sharply folded extrusion junction with overlapping triangular
  sheets and non-manifold edges.
- MP9 1189 is an offset face that emits no surface inside its boundary.
- MP9 2128 is a missing curved extrusion patch between otherwise structured
  neighboring faces.
- `slitdrill` looks closed by silhouette but the diagnostic overlay reports
  four non-manifold edges along the slit. This is exactly why visual and
  topological checks are both required.
- The remaining reviewed generated fixtures are visually coherent and their
  structured flow follows the intended analytic features. `barrel2` retains
  long transition diagonals around its chained cut and remains worth watching.
