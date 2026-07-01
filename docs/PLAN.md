# B-Rep Retopology Tool — Technical Plan & Spec

**Working name:** *(placeholder)* "Weft" — a dedicated bridge between CAD (STEP from Plasticity) and game-ready topology in Blender.

**Purpose of this document:** a plan detailed enough to hand to an implementer. It defines the problem, the core architectural bet, the module breakdown, the feature set, a technical stack, a phased roadmap, and the genuinely hard research problems so they aren't discovered late.

---

## 1. Problem statement

CAD models are defined by exact surfaces — planes, cylinders, cones, spheres, tori, and NURBS/B-spline patches — stitched together by a boundary-representation (B-rep) topology of faces, edges, and vertices. Games need *polygonal* topology: quad-dominant where it matters, even density where needed, sparse where flat, with controlled edge loops, deliberate pole placement, clean seams, and support loops that bake well.

Current tools sit at two unsatisfying extremes:

- **Global auto-tessellation** (Plasticity's Blender Bridge, MOI-style mesh export, the faceting exporters). These convert B-rep to mesh using *global* tolerances (chord, angle, min/max face width). Fast, but you get one dial for the whole model. You cannot say "12 divisions around this cylinder, 3 support loops across that fillet, and leave that flat panel as two triangles." Hard edges soften or over-tessellate.
- **Auto-retopology** (Quad Remesher, QuadriFlow, Instant Meshes, ZRemesher). These operate on an already-tessellated mesh and infer flow. They're strong on organic forms and weak exactly where hard-surface game assets need precision: crisp edges, exact cylinder divisions, deterministic loops.
- **Manual retopo in Blender** (RetopoFlow, Poly Build, shrinkwrap). Full control, but you're working on a *dead mesh* — the CAD surface math is already gone, snapping is approximate, Python is the performance bottleneck, and none of the B-rep structure (which face is a cylinder, which edge is a fillet) is available to help you.

**The gap:** nothing operates *on the B-rep itself* with per-feature, manually-controllable topology generation. That is the tool.

### The core bet

> Do not retopologize a mesh. Ingest the B-rep, keep the surface math live, and generate topology from it — feature-aware, per-face controllable, with manual override always available. The mesh is a *view* of decisions made against the B-rep, so it can be regenerated at any density without losing manual work.

This is the one design decision everything else follows from, and it's the reason this is a standalone app with a C++ geometry core rather than another Blender add-on.

---

## 2. Pipeline position

```
Plasticity  ──(STEP / IGES / Parasolid X_T)──►  [ THIS TOOL ]  ──(FBX / OBJ / glTF / USD / live bridge)──►  Blender
   B-rep + face/edge IDs                     B-rep retopology              game-ready mesh + UVs + IDs
```

- **Ingest:** STEP is the pragmatic primary format (universal, carries full B-rep, free to parse). Parasolid X_T is Plasticity's *native* format and the highest-fidelity option, but reading it requires a Parasolid or HOOPS Exchange license. Support STEP first; add X_T behind an optional licensed backend.
- **Deliver:** static export (FBX/OBJ/glTF/USD) for the baseline, plus an optional **live bridge** Blender add-on mirroring Plasticity's own connect/refresh UX, so the tool drops into an existing Plasticity→Blender habit rather than replacing it.

### 2.1 Architecture decision (resolved)

Build this as a **standalone native application on top of a headless C++ geometry + meshing core**, with a **thin Blender companion add-on used only for delivery and the live bridge**. This is a deliberate choice, not a default:

- **Why not a Blender add-on** (the approach that hit walls): Blender discards the B-rep on import. An add-on would have to maintain a shadow B-rep inside a native extension and perpetually sync it against Blender's mesh, fighting the host's data model and its Python performance ceiling. The central design idea — *the mesh is a view of decisions made against the live B-rep* — only works when the app owns its data model end to end, which means owning the surface-constrained editing loop, the viewport, and regeneration. That is incompatible with living inside Blender.
- **Why headless-core-first:** the geometry/meshing engine is a reusable library; the standalone shell is its primary client and the Blender bridge is a secondary client. Studios get a scripting/batch API for free, and the tool is never locked to a single host.
- **Accepted cost:** a standalone app must rebuild table-stakes 3D infrastructure (viewport, selection, gizmos, undo, file I/O) that Blender would provide. This is bounded, well-trodden work and is the price of the fluid, purpose-built tooling that motivated leaving the add-on behind.
- **Friction mitigation:** make Plasticity→tool ingestion one-click and make the Blender bridge mirror Plasticity's connect/refresh UX, so the pipeline *feels* like one flow rather than three separate tools.
- **Language:** C++ for the core, because OpenCASCADE and HOOPS Exchange are C++ and binding friction is the deciding factor. Rust is viable but buys nothing here that offsets less mature kernel bindings.

---

## 3. Architecture (modules)

Seven layers. Keep the geometry/meshing core in a fast native language and expose it to the UI and to a scripting API.

### 3.1 Ingestion & healing
- STEP/IGES import → B-rep (OpenCASCADE `TopoDS` shapes).
- Sew, heal, remove duplicate/degenerate edges, fix orientation, unify coincident vertices.
- **Preserve persistent CAD IDs.** Plasticity assigns stable face/edge IDs and its bridge round-trips them; carry these through so re-exports and upstream CAD edits can be reconciled. Where IDs are absent (foreign STEP), synthesize stable IDs from geometric hashing.
- Optional licensed backend: HOOPS Exchange or Parasolid for X_T and dirtier foreign CAD.

### 3.2 B-rep analysis / feature recognition
- Classify each face: plane, cylinder, cone, sphere, torus, extrusion/revolution, general NURBS.
- Detect **fillets/blends** (constant- and variable-radius), **chamfers**, **holes/bosses**, and rib/pocket patterns.
- Classify each edge as **convex / concave / smooth (tangent) / sharp**, with dihedral angle.
- Build the **face-adjacency graph** (which faces share which edges) — this graph is the substrate for density propagation and quad-flow.

### 3.3 Topology generation engine *(the heart)*
Per-surface-type meshers, each with explicit controls, unified by a density-matching pass:

- **Analytic primitives** get parametric grid meshers with *named* controls:
  - Cylinder/cone: radial divisions, axial divisions, cap style (n-gon / triangle fan / quad grid / dome), seam placement.
  - Sphere/torus: two independent division counts, pole handling (fan vs. quad-cap).
  - Plane / general extrusion: quad grid respecting the trim boundary.
- **Freeform NURBS** get either a controllable UV-grid mesh or field-guided quad meshing (§3.5).
- **Fillets/blends** get **support-loop generation**: N loops across the fillet width, density matched to the neighbours, with controls for "hold" loops near the crease for baking.
- **Trimmed-face boundary handling:** conform the interior grid to arbitrary trim curves without collapsing to a triangle soup at the border. (This is the hard part — see §7.)
- **Density propagation:** edge subdivision counts are shared constraints across the adjacency graph, so neighbouring faces meet with matching vertices instead of T-junctions. Solved as an integer constraint problem over the graph.

Every generated element stores a back-reference to the B-rep face/edge it came from, so manual edits and regeneration coexist.

### 3.4 Manual editing layer
Operates on a half-edge mesh whose vertices are *constrained to the live B-rep surface*, so every manual move re-projects exactly onto the CAD surface (true snapping, not shrinkwrap approximation):
- Draw/insert quads and loops directly on a surface (projected geodesically).
- **Cutting:** surface knife (projected path), loop cut, ring cut, radial cut for cylinders, spin cut.
- Insert / remove / slide edge loops; redirect flow; dissolve to n-gon; triangulate/quadrangulate regions.
- Deliberate **pole (3-/5-star) placement** and junction patterns from a pattern library (cylinder-to-plane, 3-way corner, dome cap, T-junction resolutions).
- **Density painting:** brush higher/lower target density onto regions; brush guide directions for flow.

### 3.5 Quad-flow / cross-field system
- A cross-field (à la *Instant Field-Aligned Meshes*, QuadriFlow) for freeform regions — but **guided, not fully automatic**: initialized from B-rep feature edges and constrained by user strokes and painted directions. Auto is an *assist* that the artist seeds and corrects, never a black box.

### 3.6 UV & seams
- B-rep faces are natural UV islands; propose seams automatically from feature edges (convex/concave transitions), fully overridable.
- Straighten seams along cylinder axes / planar borders; pack; carry seam+sharp flags to export.

### 3.7 Export & round-trip
- FBX / OBJ / glTF / USD with UVs, smoothing groups / sharp edges, material IDs, and preserved CAD face/edge IDs as custom attributes.
- Optional **live Blender bridge** add-on (connect/refresh, mirroring the Plasticity Bridge UX).
- **Non-destructive recipe (see §5):** the set of retopo decisions is serialized so that if the source CAD changes, the recipe re-applies to the updated B-rep.

---

## 4. Feature set

### 4.1 Requested (must-have)
- Mixed **tris / quads / n-gons**, with per-region control over which are allowed.
- **Manual-first**, auto-as-assist — no reliance on a single global auto pass.
- **Explicit density control** for NURBS sheets and cylinder/cone divisions (independent radial vs. axial).
- **Snapping** (to surface, feature edge, existing vertex, symmetry plane, grid) and **cutting** (knife, loop, ring, radial, spin).
- **Deliberate, legible visuals and UI** built for flow (see §6).

### 4.2 Additions worth building in
- **Non-destructive re-tessellation:** change a face's density and keep your manual edits elsewhere. The killer feature — it's what a mesh-based add-on can never do.
- **Density matching across shared edges** — no T-junctions between adjacent faces (integer constraint solve).
- **Curvature-adaptive density** as an optional per-region toggle (not a global default).
- **Feature-edge auto support loops** with per-edge tuning, for hard-surface bake fidelity.
- **Diagnostic overlays:** quad-ratio heatmap, skew/aspect-ratio shading, pole finder, n-gon/tri finder, non-manifold + open-edge finder, and **deviation shading** (max chord error vs. the original B-rep, so you see where the mesh strays from the true surface).
- **Symmetry / mirror** workflows with a live mirror plane.
- **LOD generation:** emit several density tiers from one control setup.
- **Bake-ready validation:** watertightness, consistent normals, thin/sliver faces, self-intersection.
- **Junction/pattern library** for common hard-surface topology transitions.
- **Scripting / automation API** for batch processing and pipeline integration (studios will want headless runs).
- **Presets per asset class** (weapon, vehicle panel, small prop, environment piece).
- **Robust history / undo-redo**, GPU viewport able to handle dense assemblies.

---

## 5. Data model: the "recipe"

Persist decisions, not just the output mesh. A recipe is an ordered, mostly-declarative record keyed to **stable CAD IDs**:

```
Recipe
 ├─ source: { file, kernel, unit_scale, id_scheme }
 ├─ per_face_settings:  faceID → { mesher_type, density_params, cap_style, allow_ngon, uv_policy }
 ├─ per_edge_settings:  edgeID → { subdivisions, support_loops, seam, sharp }
 ├─ global_constraints: { symmetry_plane, density_matching: on/off, target_polycount }
 └─ manual_ops:         ordered list of edits, each anchored to CAD IDs or to
                        surface-parametric coordinates (u,v on faceID) so they
                        survive re-tessellation and reasonable upstream CAD edits.
```

Because manual edits anchor to `(faceID, u, v)` rather than to absolute mesh vertices, re-tessellating a face or re-importing a lightly-changed CAD model re-lands them instead of destroying them. Edits that can't be re-anchored (topology changed underneath them) are flagged for review rather than silently dropped.

---

## 6. UI / UX principles

The complaint about existing tools is friction, so treat UX as a first-class engineering target:

- **Two-panel focus:** live B-rep with feature colouring on one side, generated topology on the other, with a synced overlay mode showing both.
- **Modal, gesture-light tooling** (à la Plasticity/MOI): a small set of verbs (draw, cut, loop, slide, dissolve, snap, paint) with strong hotkeys and a radial/pie menu; number-typed precision (type `12` for divisions).
- **Everything numeric is also draggable**, and every drag shows the live count/measurement.
- **Selection follows the B-rep:** click a cylinder, get the whole cylindrical face; select-by-feature (all fillets, all holes of radius r).
- **Overlays are toggles, not modes** — heatmaps, deviation shading, pole/n-gon finders layer on without leaving your current tool.
- **Non-blocking regeneration:** density changes recompute incrementally and asynchronously; the viewport never freezes on a dense model.
- **Legibility:** deliberate colour language for convex/concave/smooth edges, poles, n-gons, seams — the visual system *is* the feedback loop.

---

## 7. Hard problems (call these out early)

These are the genuine research/engineering risks. Budget for them.

1. **Quad meshing of trimmed surfaces with clean boundaries.** Filling a NURBS/planar face bounded by arbitrary trim curves with a well-behaved quad grid that conforms to the border is the central difficulty. Relevant literature: integer-grid maps, mixed-integer quadrangulation (Bommes et al. 2009), *Instant Field-Aligned Meshes* (Jakob et al. 2015), QuadriFlow, and quad-layout / motorcycle-graph methods.
2. **Global density matching as an integer problem.** Making all shared edges agree on subdivision counts while keeping quad-dominance is a constrained integer optimization over the face-adjacency graph. Expect to solve it incrementally and tolerate local tris/n-gons where the constraints can't all be met.
3. **Fillet/blend handling and pole placement at junctions.** Where three or more surfaces of different curvature meet, clean flow requires deliberate poles; automating good defaults here is hard and high-value.
4. **STEP healing robustness.** Real-world CAD is dirty — gaps, slivers, inverted normals, tolerance mismatches. A licensed backend (HOOPS Exchange) buys robustness if the OCCT path proves fragile on production files.
5. **Performance on dense assemblies.** Incremental recompute, spatial acceleration structures, and a GPU viewport are prerequisites, not polish.

---

## 8. Technical stack (recommended)

- **Geometry / B-rep kernel:** OpenCASCADE (OCCT) — free, reads STEP/IGES, exposes full B-rep. Add an optional **HOOPS Exchange** (or Parasolid) backend for X_T and difficult foreign CAD.
- **Core language:** C++ for the geometry/meshing core (or Rust if the team prefers — the constraint is native performance and direct kernel bindings). This is the primary reason it's not a Blender add-on.
- **Mesh data structure:** half-edge (OpenMesh, CGAL, or custom) with per-element back-references to CAD IDs.
- **Freeform quad assist:** integrate/adapt open-source Instant Meshes / QuadriFlow rather than writing a cross-field solver from scratch.
- **App shell & UI:** native for viewport performance — Qt or Dear ImGui over a Vulkan/OpenGL (or `wgpu`) renderer. Avoid Electron for the main app; the geometry viewport needs native perf.
- **Scripting API:** embed Python or Lua for automation/batch and pipeline hooks.
- **Blender integration:** a companion add-on (Python) speaking to the core over a local socket, mirroring the Plasticity Bridge's connect/refresh model.

---

## 9. Phased roadmap

**Phase 0 — Prove the loop (spike).** STEP import via OCCT → view B-rep → tessellate one cylinder with live radial/axial division controls → export OBJ → open in Blender. Validates the entire thesis end to end.

**Phase 1 — Per-surface meshers + density.** All analytic primitives with named controls; freeform UV-grid mesher; edge-density propagation / matching across shared edges.

**Phase 2 — Manual editing.** Surface-constrained half-edge editing: snapping, cutting (knife/loop/ring/radial), loop insert/slide/remove, dissolve/quadrangulate, poles.

**Phase 3 — Feature recognition + fillets.** Edge classification, fillet/hole detection, auto support loops with per-edge tuning, junction pattern library.

**Phase 4 — Guided freeform quad flow.** Cross-field assist seeded by feature edges and user strokes; density painting.

**Phase 5 — UV, overlays, validation.** Seam proposal + straightening, packing; diagnostic overlays and deviation shading; bake-ready checks.

**Phase 6 — Round-trip & pipeline.** Recipe serialization + re-apply on CAD change; ID-preserving export; LODs; live Blender bridge; scripting API.

**Phase 7 — Polish.** Presets, performance passes on dense assemblies, docs, onboarding.

*Suggested slice for an early usable release:* Phases 0–2 already beat the status quo for hard-surface props (exact cylinder divisions + real snapping/cutting on live surfaces + non-destructive density). Ship that, then layer 3–6.

---

## 10. What makes this different from what already exists

| | Plasticity Bridge / MOI export | Auto-retopo (Quad Remesher etc.) | Manual retopo (RetopoFlow) | **This tool** |
|---|---|---|---|---|
| Works on live B-rep | partial (mesh out) | no | no | **yes** |
| Per-feature density control | global only | limited | manual | **yes** |
| Exact cylinder/NURBS divisions | no | no | by hand | **yes** |
| True surface snapping | n/a | n/a | approximate | **exact (re-project)** |
| Manual + generative combined | no | no | manual only | **yes** |
| Non-destructive re-tessellation | no | no | no | **yes (recipe)** |
| Hard-surface edge fidelity | ok | weak | good | **strong** |

---

## 11. Open questions to settle before build

*Resolved: standalone native app on a headless C++ core, with a thin Blender companion for delivery — see §2.1.*

1. Is a HOOPS Exchange / Parasolid license in budget, or is OCCT-only acceptable for v1?
2. Target polycount ranges and asset classes for the initial presets (weapons? vehicles? environment?).
3. How far to push automatic quad-flow before it becomes the "auto solution" you explicitly want to avoid — where's the line between *assist* and *autopilot*?
4. Blender live-bridge parity: match Plasticity's connect/refresh UX, or a simpler file-watch round-trip for v1?
