# Secure-core blocked routes

| ID | Route | Why blocked | Proof required to unblock | Status |
|---|---|---|---|---|
| BR-001 | Legacy/experimental generator in production | The rewrite requires one authoritative core; selectable legacy behavior hides correctness gaps | Capture baseline, remove runtime selection, and route generation through the secure result contract | OPEN |
| BR-002 | Default unrestricted OCCT healing | It mutates the only retained shape and cannot prove source correspondence | Immutable source import plus bounded repair profile and complete certificate | OPEN |
| BR-003 | P-curve synthesis in conservative repair | New representation data is not source evidence | Explicit compatibility profile, ADR, discrepancy proof, and source-to-working mapping | OPEN |
| BR-004 | Face-local edge sampling | Equal coordinates/counts do not prove shared identity or phase | Immutable canonical boundary with per-coedge UV mappings and registration witnesses | OPEN |
| BR-005 | Authoritative weld/fallback triangulation | Post-hoc repair can conceal gaps, missing faces, and unrelated topology | Boundary-exact CDT floor assembled by shared indices and validated before export | OPEN |
| BR-006 | CGAL-backed distribution | Prototype policy permits GPL/commercial packages only for local research | Commercial license or permissive replacement plus license audit | OPEN |
| BR-007 | Unreviewed twisted-cylinder fixture import | The current fixture work in `D:/weftocct` is uncommitted user work | Independent review and committed evidence before copying | OPEN |
