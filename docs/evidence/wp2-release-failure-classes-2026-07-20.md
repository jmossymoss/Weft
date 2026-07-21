# WP2 evidence — release failure classes (2026-07-20)

## Goal

Classify the foam / teleporter known-red watertight failures by topology class
and shared root cause so WP3 can fix by class (not filename or face ID). Builds
on WP1 reducers in `docs/evidence/wp1-release-reducers-2026-07-20.md` and
`tests/regressions/release/`.

Machine-readable index: `tests/RELEASE_FAILURE_CLASSES.tsv`.

Meshers were not changed. CLI only: `weft validate` / `weft mesh --validate` now
prints mesher-plan and demotion attribution even when watertightness fails
(needed for class evidence via `WEFT_FACE_KINDS`).

## Reproduction snapshot (`weft validate`, closed solids)

| model | profile | open | NM | dominant leak surfaces (types) | planned meshers on leakiest |
| --- | --- | --- | --- | --- | --- |
| foam | default | 6 | 2 | plane + cylinder fillet + bspline | minimal-ngon, coons-grid |
| foam | cad | 12 | 2 | same + second fillet/bspline cluster | + revolution-grid fillet, coons-grid bsplines |
| teleporter | default | 18 | 2 | large multi-neighbor plane (+ pads) | minimal-ngon (plane); revolution-grid on side cylinders |
| teleporter | cad | 12 | 0 | large multi-neighbor plane (+ pads) | minimal-ngon |

Source B-reps remain closed (`inputBoundaryEdges = 0`). Opens are unexplained
mesh cracks. Face IDs below are diagnostic only — not fix targets.

## Class A — `plane_fillet_bspline_junction`

### Symptom

Open edges and non-manifold edges where a planar face, a fillet strip
(cylinder/torus marked fillet), and B-spline patches meet.

### Shared root-cause hypothesis

Border-contract / weld disagreement at a three-family junction:

1. Planar face emits a `minimal-ngon` (exact border samples).
2. Fillet strip is planned as `coons-grid` (default) or `revolution-grid` (CAD
   cluster).
3. Adjacent B-spline patches are planned as `coons-grid`.

Along shared B-rep edges the three sides do not produce a single manifold weld:
sample count or position drift leaves open segments; at T-junction / triple
incidence the same mesh edge is claimed by three faces (NM). CAD profile adds a
second fillet→bspline cluster of the same shape (more opens, same NM pair).

NM attribution (OBJ edge incidence) lands on the same junction neighborhood
(large plane + fillet/bspline), not on unrelated demotions.

### Fixtures that prove the class

| role | path | notes |
| --- | --- | --- |
| authoritative closed-solid repro | `tests/STEP_Examples/foam.stp` | corpus `foam` / `foam_wt_open_nm` |
| open-shell surface-stack diagnostic | `tests/regressions/release/foam_plane_fillet_bspline_r1.step` | plane+fillet+bspline ring-1; 0 unexplained cracks (cut removes closed interior borders) |
| open-shell fillet/bspline arm | `tests/regressions/release/foam_fillet_bspline_r1.step` | default: 0 unexplained; **cad: 4 unexplained** on fillet + bspline faces — partial crack repro for the curved arm |

Full foam remains the smallest **closed** reproducer (WP1 conclusion unchanged).

### What WP3 should fix (class-level)

- Enforce identical shared-border samples (count + 3D positions) across plane /
  fillet / B-spline junctions under both default and CAD density ownership.
- Guarantee manifold welding at triple face meetings of this surface stack
  (eliminate NM without suppressing opens by over-weld elsewhere).
- Prefer a deterministic zoo / closed neighborhood fixture of the same stack
  once one can be authored; until then drive fixes from foam + the CAD
  unexplained cracks on `foam_fillet_bspline_r1`.
- No filename, model-name, or face-ID special cases.

## Class B — `planar_ngon_border_contract`

### Symptom

Open edges concentrated on a large multi-neighbor planar face meshed as
`minimal-ngon`, plus smaller planar pads at cylinder/cone/fillet junctions.
CAD: opens only. Default: same opens plus a separate NM pair (see secondary).

### Shared root-cause hypothesis

Border-contract failure on a high-degree planar n-gon:

- One plane with many neighbors (planar pads, cones, fillet cylinders) is filled
  as a single `minimal-ngon`.
- Adjacent faces solve different edge divisions / sample placements along the
  shared wires (pads are also `minimal-ngon`; some side cylinders
  `revolution-grid`).
- Weld cannot pair every border segment → unexplained opens. Count scales with
  that plane (dominant leak share on both profiles).

This is a density-ownership / shared-border sampling problem for multi-neighbor
planar webs, not a single bad face ID.

### Fixtures that prove the class

| role | path | notes |
| --- | --- | --- |
| authoritative closed-solid repro | `tests/STEP_Examples/teleporter.stp` | corpus `teleporter` / `teleporter_wt_open` |
| open-shell diagnostic | `tests/regressions/release/teleporter_planar_ngon_r1.step` | planar multi-neighbor ring-1; all opens on cut boundary; 0 unexplained |

### What WP3 should fix (class-level)

- Make multi-neighbor planar `minimal-ngon` (and adjacent pads) share one border
  contract: edge division ownership, even-arc samples, and weld pairing.
- Preserve watertightness across default and CAD profiles and supported density
  sweeps without demoting the plane to raw triangulation.
- No filename / face-ID exemptions.

## Secondary (teleporter default only) — `bspline_contract_floor_overweld`

### Symptom

Two non-manifold edges (use count 4) among B-spline patches where at least one
face demoted to `contract-floor`, co-occurring with class B on default only.
CAD teleporter: 0 NM.

### Hypothesis

Contract-floor demotion on a B-spline patch re-emits border samples that collide
with neighboring `coons-grid` B-splines, producing multi-incidence edges.

### WP3 note

Treat as a follow-on after class B opens are closed, or fix in the same pass if
the shared border/demotion path is touched. Do not conflate with class B’s
planar n-gon opens (different surface stack).

## Out of scope for this note (still in `KNOWN_RED.tsv`)

| name | brief class tag (from known-red notes) | WP |
| --- | --- | --- |
| slitdrill | tangent-contact pocket/bore NM | WP3 |
| ribbonnotch | default-profile opens on ribbon+notch | WP3 |
| tork | broken / invalid source | WP3 (diagnostic / replace) |

## Classifier discipline

- Classes name surface/mesher **stacks**, not models.
- Face IDs in validate output are for attribution only.
- Open-shell extracts diagnose stacks; closed-solid release STEP files remain
  the watertight gate until a closed reduced solid reproduces unexplained opens.
