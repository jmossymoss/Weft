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
- `m3-canonical-endpoint-identity-2026-07-17.md` - exact topological vertex normalization and bounded curve/vertex evidence.
- `m3-exact-predicates-2026-07-17.md` - exact dyadic orientation, incircle, and segment-relation evidence.
- `m3-planar-trim-validation-2026-07-17.md` - exact pre-CDT loop, intersection, containment, orientation, and coverage evidence.
- `m3-planar-trim-assembly-2026-07-17.md` - real B-rep planar coedge/wire assembly and shared-corner provenance evidence.
- `m4-single-loop-reference-cdt-2026-07-17.md` - exact-predicate boundary-preserving reference CDT and independent certificate evidence.
- `m4-certified-planar-box-2026-07-17.md` - no-weld global canonical assembly, closed-box incidence, provenance, and MeshingResult evidence.
- `m4-planar-hole-cdt-2026-07-17.md` - exact visibility bridges, constrained planar holes, and real perforated-face certification evidence.
- `m4-m6-certified-cylinder-2026-07-17.md` - registered periodic wall, analytic curved error checks, and closed capped-cylinder certification evidence.
- `m6-secure-orchestration-2026-07-17.md` - atomic plane/cylinder/through-hole secure pipeline and combined certificate evidence.
- `m7-secure-cli-routing-2026-07-17.md` - lossless certified export adapter, secure CLI routing, report coverage, and named workflow refusals.
- `m7-secure-app-live-link-2026-07-17.md` - secure app preview/export/hot-reload routing, visual proof, and deterministic atomic Blender link.
- `m0-m7-secure-cli-utilities-2026-07-17.md` - secure conversion/sweep routing, source inspection/extraction, and cache refusal evidence.
- `m7-secure-recipe-v2-2026-07-17.md` - source-referenced v2 persistence, v1 migration, correspondence-aware app capture, deterministic replay, and named application conflicts.
- `m3-m7-certified-edge-counts-2026-07-17.md` - exact per-edge recipe constraints, equality propagation, shared canonical samples, deterministic CLI replay, and active app controls.
- `m3-exact-chain-sums-2026-07-17.md` - bounded exact independent chain sums, exhaustive-oracle properties, and named coupling/complexity refusals.
- `m1-m2-frozen-corpus-2026-07-17.md` - frozen 106-record catalogue, committed STEP outcomes, and 77-fixture import/accounting evidence.
