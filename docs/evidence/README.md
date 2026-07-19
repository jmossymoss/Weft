# Secure-core evidence index

Evidence files record commands, exact revisions, outcomes, failures, and
artifacts for milestone gates. Evidence is append-only in meaning: a later run
may supersede a result but must not rewrite what an earlier run proved.
Use `docs/governance/agent-execution-playbook.md` for sequencing. Evidence files
prove completed work; they are not plans or handoff diaries.

- `m0-legacy-baseline-2026-07-17.md` - pre-rewrite Release test baseline.
- `m0-strict-build-2026-07-17.md` - MSVC warnings-as-errors and static-analysis build evidence.
- `m0-parallel-test-isolation-2026-07-17.md` - process-isolated temporary artifacts and concurrent strict/static test evidence.
- `m0-windows-clean-legacy-removal-2026-07-17.md` - clean MSVC build/analyzer,
  retired source deletion, single-owner build graph, and secure CLI proof.
- `m0-linux-gcc-analysis-2026-07-18.md` - Linux GCC strict and `-fanalyzer`
  lanes, scoped analyzer exclusions, and OCCT 7.9 named-refusal corpus
  reconciliation.
- `m1-secure-import-2026-07-17.md` - processing-disabled source/working import contract evidence.
- `m1-exact-occurrence-account-2026-07-17.md` - exact XDE
  definition/instance ownership, recursive B-rep occurrences, coedge uses,
  non-vacuous validation, and identity correspondence evidence.
- `m1-topology-isolated-working-copy-2026-07-17.md` - byte-identical but
  TShape-distinct conservative working topology, rejected OCCT copy routes,
  exact-use rebinding, and full Windows corpus proof.
- `m1-bounded-parameterization-repair-2026-07-17.md` - flag-only
  SameParameter/SameRange repair, exact modified correspondence, complete
  p-curve-use evidence, and adversarial refusals.
- `m1-native-brep-secure-import-2026-07-17.md` - immutable-byte native B-rep
  provenance, public repair admission, path-replacement proof, missing-unit
  visibility, and fail-closed unsupported readers.
- `m1-orientation-reconnaissance-2026-07-17.md` - experimental rejection of
  whole-solid reversal as a general shell/face orientation repair.
- `m1-face-adjacency-orientation-repair-2026-07-17.md` - two-manifold
  face-adjacency orientation repair, polarity check, and PAT-009 proof.
- `m1-multi-body-orientation-repair-2026-07-17.md` - orientation repair of
  invalid free solids inside compound/compsolid roots.
- `m1-bounded-tolerance-envelope-repair-2026-07-17.md` - raise working edge
  tolerance to proven curve-on-surface maximum for beyond-threshold witnesses.
- `m1-copy-on-write-representation-rules-2026-07-17.md` - shared geometry
  handle immutability and unexplained representation refusal.
- `m1-explicit-native-brep-unit-resolution-2026-07-17.md` - caller-supplied
  native millimetre scale without invented units or coordinate rescaling.
- `m1-secure-iges-import-2026-07-17.md` - immutable-byte IGES secure import,
  processing-disabled transfer, and path-replacement proof.
- `m1-compatibility-correspondence-2026-07-17.md` - valid Compatibility deep
  copies and heal-lane inverted-shell correspondence.
- `m1-product-step-orientation-repair-2026-07-17.md` - STEP round-trip of the
  PAT-009 orientation pathology as a product-format repair witness.
- `m1-bounded-sewing-2026-07-17.md` - meshable face-preserving free-edge sew
  with gap-fixture refusal.
- `m1-certificate-or-named-refusal-2026-07-17.md` - M1 gate sweep: meshable
  certificate or named `import.*` Error for every reviewed working model.
- `m2-reconnaissance-2026-07-17.md` - total face/edge classification and planar projection evidence.
- `m2-total-reconnaissance-2026-07-17.md` - M2 gate close: schema trim taxonomy, merged coplanar regions, multidomain deferral, adversarial unknown-family probes, and oracle slices.
- `m3-interval-solver-2026-07-17.md` - exact equality/minimum/parity count assignment evidence.
- `m3-canonical-boundaries-2026-07-17.md` - atomic shared-edge samples, UV uses, and azimuth registration evidence.
- `m3-periodic-closure-2026-07-18.md` - closed coedge periodic UV lift/closure witnesses and ambiguous-lift refusals.
- `m3-repeated-wire-occurrences-2026-07-18.md` - topology-backed meshing wire/coedge identity and ShapeMap face alias-collapse refusal.
- `m3-cross-platform-determinism-2026-07-18.md` - box/cylinder/hole count-boundary-lift-report digests matched on Windows and Linux.
- `m3-critical-segmentation-2026-07-18.md` - line/circle/plane/cylinder critical parameter events, non-vacuous coverage, and unsupported-family refusal before CDT/templates.
- `m3-canonical-endpoint-identity-2026-07-17.md` - exact topological vertex normalization and bounded curve/vertex evidence.
- `m3-exact-predicates-2026-07-17.md` - exact dyadic orientation, incircle, and segment-relation evidence.
- `m3-planar-trim-validation-2026-07-17.md` - exact pre-CDT loop, intersection, containment, orientation, and coverage evidence.
- `m3-planar-trim-assembly-2026-07-17.md` - real B-rep planar coedge/wire assembly and shared-corner provenance evidence.
- `m4-single-loop-reference-cdt-2026-07-17.md` - exact-predicate boundary-preserving reference CDT and independent certificate evidence.
- `m4-certified-planar-box-2026-07-17.md` - no-weld global canonical assembly, closed-box incidence, provenance, and MeshingResult evidence.
- `m4-planar-hole-cdt-2026-07-17.md` - exact visibility bridges, constrained planar holes, and real perforated-face certification evidence.
- `m4-m6-certified-cylinder-2026-07-17.md` - registered periodic wall, analytic curved error checks, and closed capped-cylinder certification evidence.
- `m4-m5-triangle-intersections-2026-07-18.md` - exact 3D triangle intersection
  predicates, non-vacuous body coverage, topology-legal contacts, and named
  refusal adversaries on Linux strict/static-analysis lanes.
- `m4-m6-axial-cylinder-samples-2026-07-18.md` - certified cylinder interior
  axial rings with CylinderInteriorStation provenance and seam-sample
  consumption on Linux strict/static-analysis lanes.
- `m4-m6-partial-cylinder-2026-07-18.md` - open cylindrical bands
  (`PeriodicBandCrossingSeam`) with two rim arcs and two rails certified
  end to end on Linux strict/static-analysis lanes.
- `m6-cylinder-frame-registration-2026-07-18.md` - distinct azimuth
  reflection/twist/incompatibility refusals and compatible phase/origin/axis
  registrations on Linux strict/static-analysis lanes.
- `m5-incidence-euler-validation-2026-07-18.md` - certified incidence/Euler
  coverage for closed and open bodies, including through-hole χ=0, on Linux
  strict/static-analysis lanes.
- `m5-modelling-provenance-2026-07-18.md` - explicit Absent/FloorAlias/Independent
  modelling provenance with adapter selected-output reporting on Linux
  strict/static-analysis lanes.
- `m6-independent-modelling-topology-2026-07-18.md` - independent modelling
  quads paired from certified triangle strips with non-vacuous independent
  certificates on Linux strict/static-analysis lanes.
- `m7-admission-path-audit-2026-07-18.md` - centralized MeshingResult admission
  for CLI/app regenerate/export routes with incomplete/stale/provenance
  refusals on Linux strict/static-analysis lanes.
- `m7-certified-per-face-recipe-2026-07-18.md` - axial-only per-face cylinder
  consumer with save/reload deterministic replay on Linux strict/static-analysis
  lanes.
- `m7-secure-cache-invalidation-2026-07-18.md` - SecureCacheKey lookup with
  named mismatch/corruption refusals and app generation-epoch stale admission
  on Linux strict/static-analysis lanes.
- `m7-certified-editing-overlays-2026-07-18.md` - named LOD/density effect
  reporting and refusal-gated editing/admission completing the M7 workflow
  gate on Linux strict/static-analysis lanes.
- `m4-m8-certified-coverage-baseline-2026-07-18.md` - frozen committed STEP plus
  built-in fixture classification into certified / named-refusal /
  inspectable-only terminals with reconciled totals on Linux
  strict/static-analysis lanes.
- `m8-cone-a-reconnaissance-2026-07-18.md` - cone fixture exact classification,
  deferred residual support, and named pre-template mesh refusal on Linux
  strict/static-analysis lanes.
- `m8-cone-b-boundaries-2026-07-18.md` - cone singular apex stations, critical
  segmentation on `touches_one_singularity`, and named degenerate-count
  adversaries on Linux strict/static-analysis lanes.
- `m8-cone-c-certified-floor-2026-07-18.md` - apex-cone wall template, secure
  pipeline routing, and CLI OBJ export on Linux strict/static-analysis lanes.
- `m8-cone-d-modelling-2026-07-18.md` - independent modelling polygons and
  tampered-provenance refusal for apex-cone results on Linux
  strict/static-analysis lanes.
- `m8-cone-e-product-2026-07-18.md` - CLI OBJ and app screenshot admission for
  cone with sphere refusal regression on Linux.
- `m8-cone-f-proof-2026-07-18.md` - apex-cone density sweep, corpus impact, and
  family-complete proof (fingerprint hex is OCCT-version evidence).
- `m8-sphere-a-reconnaissance-2026-07-18.md` through
  `m8-sphere-f-proof-2026-07-18.md` - sphere family packet.
- `m8-torus-a-reconnaissance-2026-07-18.md` through
  `m8-torus-f-proof-2026-07-18.md` - torus dual-periodic family packet.
- `m8-fillet-a-reconnaissance-2026-07-18.md` through
  `m8-fillet-f-proof-2026-07-18.md` and `m8-gate-closure-2026-07-18.md` -
  analytic fillet packet and M8 selected-scope gate.
- `m4-gate-closure-2026-07-18.md` - M4 tessellation floor gate closure.
- `m5-gate-closure-2026-07-18.md` - M5 non-vacuous validation gate closure.
- `m6-gate-closure-2026-07-18.md` - M6 planar/cylinder template gate closure.
- `m8-map-a-reconnaissance-2026-07-18.md` - four-sided mapped/Coons candidate
  tagging on ribbon/ribbonnotch fixtures.
- `m8-map-b-boundaries-2026-07-18.md` - bounded bspline/bezier edge sampling
  and four-rail ribbon boundaries for mapped candidates.
- `m8-map-c-certified-floor-2026-07-18.md` through `m8-map-f-proof-2026-07-18.md` -
  mapped four-sided UV-grid floor, product, and family proof.
- `m8-cut-a-reconnaissance-2026-07-18.md` through `m8-cut-f-proof-2026-07-18.md` -
  circular through-hole cutout subclass (hole fixture).
- `m8-free-a-reconnaissance-2026-07-18.md` through `m8-free-f-proof-2026-07-18.md` -
  freeform UV-grid patch subclass.
- `m8-app-family-matrix-2026-07-18.md`, `m8-app-editing-overlays-2026-07-18.md`,
  `m8-app-gate-2026-07-18.md` - running-app hardening before M9.
- `m9-dependency-br006-2026-07-18.md` through `m9-gate-closure-2026-07-18.md` -
  M9 distribution progress (Linux package; Plasticity↔Blender blocked).
- `packet-audit-2026-07-18.md` - reopen under-scoped CUT/FREE/MAP-D-F/APP-151.
- `m6-secure-orchestration-2026-07-17.md` - atomic plane/cylinder/through-hole secure pipeline and combined certificate evidence.
- `m7-secure-cli-routing-2026-07-17.md` - lossless certified export adapter, secure CLI routing, report coverage, and named workflow refusals.
- `m7-secure-app-live-link-2026-07-17.md` - secure app preview/export/hot-reload routing, visual proof, and deterministic atomic Blender link.
- `m0-m7-secure-cli-utilities-2026-07-17.md` - secure conversion/sweep routing, source inspection/extraction, and cache refusal evidence.
- `m7-secure-recipe-v2-2026-07-17.md` - source-referenced v2 persistence, v1 migration, correspondence-aware app capture, deterministic replay, and named application conflicts.
- `m3-m7-certified-edge-counts-2026-07-17.md` - exact per-edge recipe constraints, equality propagation, shared canonical samples, deterministic CLI replay, and active app controls.
- `m3-exact-chain-sums-2026-07-17.md` - bounded exact independent chain sums, exhaustive-oracle properties, and named coupling/complexity refusals.
- `m3-template-chain-sum-consumer-2026-07-18.md` - planar template consumption of independent chain sums with non-vacuous requested/solved/consumed coverage and named adversaries.
- `m3-bounded-coupled-counts-2026-07-18.md` - one bounded exact coupled component, exhaustive-oracle equivalence, planar template consumption, and named scope/complexity refusals.
- `m1-m2-frozen-corpus-2026-07-17.md` - frozen 106-record catalogue, committed STEP outcomes, and 77-fixture import/accounting evidence.
- `mp9-inventory-2026-07-18.md` - MP9 face/curve blocker inventory and meshable=0 root cause.
- `m8-ellipse-b-boundaries-2026-07-18.md` - ellipse critical segmentation + interval demand.
- `m8-ellipse-f-proof-2026-07-18.md` - ellipse_hole fixture end-to-end mesh proof.
- `mp9-mesher-phase1-2026-07-18.md` - MP9 extract meshes + analytic UV without p-curve.
- `mp9-freeform-expansion-2026-07-18.md` - freeform ≤5-edge UV-grid vs high-edge deferral.
- `mp9-extrusion-offset-2026-07-18.md` - extrusion/offset certify or named refuse.
- `mp9-analytic-residuals-2026-07-18.md` - named cone/sphere/cylinder residual demotions.
- `mp9-scale-perf-2026-07-18.md` - MP9 import/recon wall times and compat scale refuse.
- `mp9-cone-frustum-consumer-2026-07-18.md` - truncated-cone revolved-band consumer + quads.
- `mp9-sphere-cap-consumer-2026-07-18.md` - spherical-cap consumer + complex_cap demote.
- `mp9-uv-trim-consumers-2026-07-18.md` - UV-trim CDT for complex cylinders + freeform n-gons.
- `mp9-sphere-cap-seam-fix-2026-07-18.md` - Plasticity sphere-cap CapWall / seam fix.
- `mp9-full-mesh-gate-2026-07-18.md` - MP9 full-body mesh gate (408342 tris).
- `mp9-quads-density-progress-2026-07-19.md` - modelling quads + radial-32 density progress.
- `mp9-import-perf-2026-07-19.md` - MP9 import timing after large-repair skips.
- `mp9-preview-density-2026-07-19.md` - preview density ~82k polys at radial UI 32.
- `mp9-mesh-progress-speed-2026-07-19.md` - stage progress + interval O(n) speedup.
- [mp9-failclosed-g0g1-2026-07-19.md](mp9-failclosed-g0g1-2026-07-19.md) — G0/G1 fail-closed MP9 mesh without face omission
- [mp9-g2-cylinder-no-uv-relax-2026-07-19.md](mp9-g2-cylinder-no-uv-relax-2026-07-19.md) — G2 complex cylinder UV-trim without relax
