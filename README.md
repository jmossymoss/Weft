# Weft

Weft is a STEP-to-Blender bridge for generating modelable hard-surface
topology from CAD B-reps. It preserves source geometry and provenance, builds a
separate working representation, and exports only meshes that pass the
secure-core certificate gates.

## Current status

The secure-core rewrite is in progress. The sole implementation-status
authority is [the milestone ledger](docs/governance/milestones.md).

The certified automatic baseline currently covers:

- planar faces, including bounded holes;
- full periodic cylindrical walls;
- composed planar boxes, capped cylinders, and connected through-holes;
- certified OBJ, GLB, and FBX routing through the CLI;
- desktop display/export and Blender live-link for supported results.

Other geometry remains inspectable but is not exportable unless its certified
floor can be proved. Weft does not use OCCT triangle soup, welding, dropped
faces, or silent healing as evidence that a result is correct.

## Build

Windows prerequisites and bootstrap instructions are in
[WINDOWS_BUILD.md](WINDOWS_BUILD.md).

The maintained CMake presets are:

```text
vs2022
vs2022-static-analysis
linux-gcc
linux-gcc-static-analysis
```

Example Windows strict lane:

```powershell
cmake --preset vs2022
cmake --build --preset vs2022
ctest --preset vs2022
```

## Documentation

- [Agent startup directive](AGENTS.md)
- [Secure-core architecture](docs/SECURE_CORE_REWRITE_PLAN.md)
- [Agent execution playbook](docs/governance/agent-execution-playbook.md)
- [Milestone status](docs/governance/milestones.md)
- [Blocked routes](docs/governance/blocked-routes.md)
- [Architecture decisions](docs/adr/README.md)
- [Verification evidence](docs/evidence/README.md)
- [Meshing research reference](docs/MESHING_RESEARCH_PLAN.md)

Agents should not create extra roadmaps or handoff documents. Continue work
through the execution playbook and record only durable decisions or completed
verification in the existing ADR/evidence indexes.
