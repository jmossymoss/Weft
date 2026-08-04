# Accuracy tessellator branch

This branch strips structured retopology so we can build a Pixyz-inspired
sag/angle/length tessellator from a clean base.

## Status

Working now:

- Shared topological-edge sample map (finest adjacent-face sag wins)
- Planar faces → boundary n-gons; holes keyhole-bridged into one n-gon
- Analytic cylinder/cone/sphere/torus as UV **quads** (closed-form circle sag)
- Analytic rims seeded from shared edge samples via UV grid slots
- Freeform UV hard-capped (64×64, sag refine only) — stops 256² blow-ups
- Presets hone with maxSag + maxAngle (Pixyz-style fillet densify)
- Torus uses major+minor radii for U/V density
- Normal-aware polygon winding

Still open:

- Residual open borders on real multi-cylinder CAD (slitdrill, plugs)
- Multi-hole pathological bridges (fallback leaves hole unmerged)
- Pixyz side-by-side calibration on MP9
- App UI for accuracy knobs (`-DWEFT_BUILD_APP=ON` still parked)

## What changed

- `core/src/legacy/meshers_structured.cpp` — archived RevolutionGrid/Coons/… (not built)
- `core/src/meshers.cpp` — accuracy `generate()`
- Density UX: `QualityPreset` + `chordTolerance` (maxSag), `angleToleranceDeg`
  (maxAngle; −1 = off), `maxLength` (−1 = off). Presets set both sag and angle.
- `weft_app` off by default; smoke tests in `tests/test_accuracy_smoke.cpp`

## Build

```sh
cmake -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
ctest --test-dir build --output-on-failure
```

```sh
build/cli/weft fixture /tmp/cyl.step --shape cylinder
build/cli/weft mesh /tmp/cyl.step -o /tmp/cyl.obj --profile accuracy   # Medium
build/cli/weft mesh /tmp/cyl.step -o /tmp/cyl_h.obj --profile high
```

Recipe keys: `chord`/`maxsag`, `angle`/`maxangle`, `maxlength`.

## Push (from your terminal)

Agent sandbox cannot reach GitHub DNS. Use SSH:

```sh
cd ~/Documents/GitHub/Weft
unset GIT_ASKPASS SSH_ASKPASS
export GIT_SSH_COMMAND='ssh -F /dev/null -o IdentitiesOnly=yes -o IdentityFile=~/.ssh/id_ed25519'
git remote set-url origin git@github.com:jmossymoss/Weft.git
git push -u origin accuracy-tessellator
```
