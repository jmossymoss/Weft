# Density raise / lower verification (object 5)

Baseline tip: `6935e53` (watertight CAD defaults). Solid-bbox densify WIP
was reverted after it left `failed-floor≥1`; this page records what happens
when artists change segment counts instead.

Screenshots: `objects/density_sweep/` (solid+wire, object 5 isolated).

## Validity matrix (`weft mesh --profile cad --validate`)

| Case | Setting | watertight | open / NM | structured | failed-floor | folded |
|------|---------|:----------:|----------:|-----------:|-------------:|-------:|
| BASE | CAD defaults | yes | 0 / 0 | 3052/3052 | 0 | 0 |
| UP | `--face 374:radial=24` | yes* | 0 / 0 | 3050/3052 | **2** | 0 |
| UP | `--face 374:radial=32` | yes* | 0 / 0 | 3050/3052 | **2** | 0 |
| DOWN | `--face 374:radial=8` | yes | 0 / 0 | 3052/3052 | 0 | 0 |
| UP | `--face 362:radial=32` | yes | 0 / 0 | 3052/3052 | 0 | **4** |
| DOWN | `--face 362:radial=8` | yes | 0 / 0 | 3052/3052 | 0 | 0 |
| UP | `--radial 24` (global) | yes | 0 / 0 | 3052/3052 | 0 | 0 |
| DOWN | `--radial 8` (global) | yes | 0 / 0 | 3052/3052 | 0 | 0 |

\*Assembly can still report watertight while faces demote to contract floor.

## Face 374 (IsoBand wall) — raise breaks the face

| Setting | Built columns (debug) | Outcome |
|---------|----------------------:|---------|
| default | 11 spans | structured orth |
| radial=8 | 11 spans | unchanged (adaptive floor wins) |
| radial=24 / 32 | — | **orth failed → contract floor**; neighbour **f375 self-check failed** |

Floor causes at `374:radial=24`:
- `374` — `orthogonal revolution grid failed`
- `375` — `self-check failed`

UI on UP screenshot shows yellow **“2 face(s) on contract floor”**; orange
flute pockets pinches into floor webs.

## Screenshots

| Tag | File | Notes |
|-----|------|-------|
| BASE | `base_v0.png` / `base_v1.png` | Clean, watertight, no floor banner |
| UP f374×24 | `f374_up24_v0.png` / `_v1.png` | Contract-floor banner; denser + broken pockets |
| DOWN f374×8 | `f374_dn8_v0.png` / `_v1.png` | Matches BASE (override ignored under adaptive) |
| UP f362×32 | `f362_up32_v0.png` / `_v1.png` | Visually denser cone; 4 folded polys in metrics |
| multi-up | `global_hint_up_v0.png` | 374+362+364+368 radial=24 |

## Findings

1. Raising a single IsoBand drum’s radial is unsafe on this assembly: the
   face and a FullPeriod neighbour demote even though the solid stays
   “watertight.”
2. Lowering radial on f374 does nothing under CAD adaptive — curvature /
   drum floors keep ~11 spans.
3. Global `--radial 8/24` stayed structured on this tip; per-face override
   on the coupled wall is the sharp edge.
4. Any automatic “object bbox → more segments” path must be treated as a
   density raise and pass this same UP screenshot + floor gate before
   merge.

## Next

- Harden orth / rim-sum so `face N:radial=up` cannot floor the edited face
  or its drum neighbours (root cause of the solid-bbox attempt failures).
- Re-run this gallery after that fix; only then revisit per-solid scale.

## Slotted drum → Coons fillet coupling (fixed)

On `tests/fixtures/demo.step`, raising the stacked slotted cylinders
(`--face 7:radial=22,adapt=0 --face 27:radial=22,adapt=0`) used to stamp
`radial` onto neighbour Coons fillet strips via `propagateBandRadialToBlendGroup`.
That tripped `curCountOverride`, killed along-axis adaptive, and squared
slot fillets (e.g. faces 42/43: **6×3 → 1×3**; tip fillets 2/4 grew with
the drum radial).

Fix: do not stamp `radial` onto Coons / rail / ribbon blend faces; shared
rim contracts still densify through density groups / edge pins.

| Fillet | BASE | radial 22 (before) | radial 22 (after) |
|--------|------|--------------------|-------------------|
| #42 / #43 | 6×3 | 1×3 | 6×3 |
| #2 / #4 | 1×3 / 3×1 | 1×6 / 6×1 | 1×3 / 3×1 |

Lock: `testSlottedDrumRadialPreservesCoonsFillets`.
Screenshots: `objects/slotted_fillet_fix/base_v0.png`, `radial22_v0.png`.
