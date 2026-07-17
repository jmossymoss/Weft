# Secure-core blocked routes

| ID | Route | Why blocked | Proof required to unblock | Status |
|---|---|---|---|---|
| BR-001 | Legacy/experimental generator in production | The primary `mesh` CLI and desktop app are secure-only, but B-rep-to-mesh `convert`, `sweep`, and `cache-check` still expose historical generation paths | Route or explicitly block every remaining CLI workflow, then remove dormant generator code | OPEN |
| BR-002 | Default unrestricted OCCT healing | It mutates the only retained shape and cannot prove source correspondence | Immutable source import plus bounded repair profile and complete certificate | OPEN |
| BR-003 | P-curve synthesis in conservative repair | New representation data is not source evidence | Explicit compatibility profile, ADR, discrepancy proof, and source-to-working mapping | OPEN |
| BR-004 | Face-local edge sampling | Equal coordinates/counts do not prove shared identity or phase | Immutable canonical boundary with per-coedge UV mappings and registration witnesses | OPEN |
| BR-005 | Authoritative weld/fallback triangulation | Post-hoc repair can conceal gaps, missing faces, and unrelated topology | Boundary-exact CDT floor assembled by shared indices and validated before export | OPEN |
| BR-006 | CGAL-backed distribution | Prototype policy permits GPL/commercial packages only for local research | Commercial license or permissive replacement plus license audit | OPEN |
| BR-007 | Unreviewed twisted-cylinder fixture import | The current fixture work in `D:/weftocct` is uncommitted user work | Independent review and committed evidence before copying | OPEN |
| BR-008 | Non-rigid STEP transform without a typed account | XDE may drop scale/reflection unless retention or baking is independently proven | Retained-affine or baked-geometry account with composition and parity validation | ENFORCED |
| BR-009 | Fail-open or tolerance-only trim validation | A triangulator cannot repair ambiguous crossings, contacts, nesting, or missing provenance without changing meaning | Exact full-pair loop/domain evidence with non-vacuous coverage before CDT | ENFORCED |
