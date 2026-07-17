# Secure-core gate checklist

Single source of truth for gate progress. `docs/governance/milestones.md`
holds the formal status rows; details for the first four repair increments
are in ADR-0026..0029 and their evidence docs. Everything after those is
recorded HERE only, by decision, to avoid duplicating context.

Verification state for every ticked item: full 14-test battery green on the
Linux dev vehicle (OCCT 8.0 from source, warnings-as-errors off) after each
commit. Windows strict MSVC + `/analyze` lanes NOT run this session and
remain the acceptance gate. Frozen fixture snapshot untouched throughout.

## M0 — baseline and build matrix

- [x] Governance, build matrix, legacy production routing removed (pre-session)
- [ ] Linux GCC static-analysis proof — deferred by standing decision (ignore Linux)

## M1 — immutable source and working B-rep

- [x] Face-adjacency orientation repair: two-manifold parity proof, independent signed-volume/infinite-point polarity, occurrence-only flips, certificate re-audit, named refusals (ADR-0026, `bb98ac8`)
- [x] Coherent-shell polarity normalization: inside-out detection by infinite-point classification (BRepCheck accepts inverted solids); corpus fixture that was silently meshable inward now certified-repaired (ADR-0027, `472956a`)
- [x] Caller-side native unit resolution with named authority; byte-identical digests; STEP units not overridable (ADR-0028, `a9ef529`)
- [x] Secure IGES import: snapshot-verified scratch-file parse (OCCT cannot stream-parse IGES), roots-gated refusals, shared XDE certificate chain, committed box/cylinder witnesses (ADR-0029, `c5da7b4`)
- [x] Public-STEP repair witness: certified repair fires on `importStepSecure`, non-vacuous (`c0a070b`)
- [x] Source-immutability row: digest captured before derivation on all import paths; drift = Fatal `import.source.mutated_after_capture`, non-meshable (`c0a070b`)
- [x] Bounded vertex tolerance reconciliation: working-only raise to the measured incidence gap within the incident-edge evidence envelope; independent re-audit row; `repair.tolerance.gap_beyond_envelope` refusals; frozen gap witness behaves as specified (`caf99c5`)
- [x] Sewing reconnaissance: same-wire duplicate vertex/edge definitions refuse by name with measured distances; slits/seams/touching bodies exempt; frozen duplicate witness fires both codes; fixes the future merge contract (1:1 `Merged` + certified cardinality change; `BRepBuilderAPI_Sewing` stays blocked) (`dc3efc7`)
- [x] Hollow-solid orientation defect detection: per-shell parity + trial volumes; inverted outer or outward cavity refuses as `repair.orientation.multi_shell_solid` (`45fa31a`)
- [ ] Hollow-solid occurrence repair (cavity nesting proofs)
- [ ] Reversed-solid-occurrence normalization (compound child-list flips)
- [ ] Shared-shell-definition repair scope
- [ ] Provable one-to-one sewing merges
- [ ] Complete compatibility-profile correspondence

## M2 — total reconnaissance

- [x] Merged coplanar region evidence: bit-exact stored-plane equality across shared two-use edges; named near-miss refusals (`plane_mismatch`/`orientation_mismatch`/`nonmanifold_interface`); additive, per-face regions untouched (`e6b7af9`)
- [x] Adversarial unknown-family injection: foreign-RTTI surface survives import (fixed a crash in the snapshot wire walk), classifies `kernel_specific`, never folded into a supported family, named diagnostic (`ccc12a4`)
- [ ] Oracle-level taxonomy matching against the frozen expectation records
- [ ] Full trim taxonomy coverage

## M3 — predicates, counts, canonical boundaries

- [x] Repeated wire occurrences: each seam coedge occurrence owns exactly its oriented p-curve; occurrences must pair bijectively or refuse `boundary.seam_occurrence_unresolved` (`68efd26`)
- [x] Exact `orient3d` in the dyadic predicate backend (`4b3869b`)
- [ ] Full periodic closure proof
- [ ] General periodic/singular assembly
- [ ] Template sum consumer; coupled/aliased sum systems

## M4 — certified tessellation floor

- [x] Exact cross-face triangle disjointness gate `certified.triangle_intersection`: every non-adjacent pair from different faces proven disjoint by exact predicates (plane separation, piercing with boundary-inclusive containment, projected on-plane contact, coplanar 2D overlap); contact or unprovable pair fails closed (`4b3869b`)
- [ ] General curved-surface certified floors (today: planes + full cylinders)
- [ ] Axial interior provenance/refinement; adaptive patch bounds; critical regions
- [ ] Corpus-wide certified meshing

## M5 — non-vacuous validation

- [x] Euler-characteristic preservation gate `certified.euler_characteristic`: certified complex must equal the source face-complex characteristic; faces contribute `2 - wireCount` (holed faces are not disks); box χ=2, seam-cut cylinder χ=2, genus-1 through-hole χ=0, perforated open face χ=0 (`d17d7c7`)
- [ ] General curved chord/normal bounds
- [ ] Modelling/mesh provenance-vacuity gates

## M6 — first usable templates

- [ ] Partial (non-full-periodic) cylinder walls — unblocked by the M3 seam item
- [ ] Interior axial rings
- [ ] Independently certified modelling polygons/quads
- [ ] Mismatched-reference fixtures

## M7 — workflow restoration

- [ ] Certified editing; overlays; secure caching/proxy updates; LOD report naming
- [ ] Certified consumers for per-face/manual recipe records

## M8 — research-backed expansion

- [ ] Not started (requires M7)

## M9 — packaging and release gate

- [ ] Not started (licensing, reproducible packaging, release)

## Deferred by decision (test/doc consolidation)

- [ ] Positive tolerance-raise witness (gap above vertex tolerance, inside edge envelope)
- [ ] Hollow-box orientation witnesses (coherent-outward cavity; flipped cavity face; two outers)
- [ ] Merged-region positive witness; seam-pairing refusal witness
- [ ] Triangle-gate adversarial witness (deliberate pierce); Euler tamper witness
- [ ] Windows strict + `/analyze` reruns for everything after ADR-0029
