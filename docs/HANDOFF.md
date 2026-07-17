# HANDOFF — secure-core gate drive (branch `claude/m1-face-adjacency-orientation-4ya4uc`)

Read this first. It is the continuation guide for the secure-core rewrite
gate work on THIS branch. (The previous contents of this file described a
legacy visual-meshing campaign on other branches; that context lives in git
history and is irrelevant here.)

## 1. What you are working on

Weft is replacing its production geometry core with a certified,
fail-closed pipeline. The architecture and non-negotiables are in
`docs/SECURE_CORE_REWRITE_PLAN.md`. Progress is governed by milestone gates
M0-M9:

- `docs/governance/milestones.md` — formal status rows (the authority).
- `docs/governance/GATES.md` — THE working checklist. One line per gate
  item, ticked with its commit hash when landed, unticked when open. Your
  job is to pick the next unticked box, land it, tick it, and keep both
  files honest.
- `docs/governance/blocked-routes.md` — routes you must NOT take
  (unrestricted healing, authoritative welds, whole-solid reversal, etc.).
- `docs/adr/` — decision records for the first repair increments
  (ADR-0026..0029 are the most recent; later increments are recorded in
  GATES.md only, by the owner's decision to avoid doc sprawl).

## 2. Non-negotiable working rules

1. Small increments. One gate item per commit where possible; the full
   test battery must be green after every commit.
2. Fail closed, by name. A defect that cannot be repaired with a complete
   proof gets a stable named refusal code and forces non-meshable. Never
   guess, never heal silently, never let a defect pass because BRepCheck
   accepts it (BRepCheck accepts inside-out solids — proven here).
3. The frozen fixture snapshot `tests/secure_core_fixtures/**` is
   digest-pinned. NEVER modify anything under it. Author new witnesses
   in-test from the committed baselines, or as plain files under
   `tests/fixtures/` (not pinned).
4. The 77-fixture generated corpus must keep identity certificates except
   where a contract change is intentional and asserted exactly (see
   `tests/test_generated_secure_corpus.cpp` for the one certified-repair
   carve-out).
5. Determinism: no recorded output (refusal codes, evidence counts,
   operation order) may depend on unordered-container iteration or locale.
   Count defects first, then name them in a fixed priority; format numbers
   with `std::locale::classic()` and `max_digits10`.
6. Commits are authored as `Jordan Moss <jordan.moss@live.co.uk>` with NO
   AI attribution of any kind (no Co-Authored-By, no tool names). The repo
   git config is already set; do not override it.
7. Windows is the acceptance gate (strict MSVC + `/analyze`, see the
   command block in `docs/evidence/m1-bounded-parameterization-repair-2026-07-17.md`).
   Linux is a dev vehicle only — do not chase Linux-only warnings; the
   `-Werror` lane failure on `-Wmissing-field-initializers` is a known
   deferred item.

## 3. Build and test (Linux dev vehicle)

Ubuntu's OCCT 7.6.3 CANNOT compile this branch (needs 7.8+/8.0 APIs).
Build OCCT 8.0.0 from source once:

```bash
sudo apt-get install -y libwayland-dev wayland-protocols libxkbcommon-dev \
  xorg-dev libgl1-mesa-dev libtbb-dev rapidjson-dev ninja-build
git clone --depth 1 --branch V8_0_0 https://github.com/Open-Cascade-SAS/OCCT.git /root/occt-src
cmake -S /root/occt-src -B /root/occt-build -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_MODULE_Draw=OFF -DBUILD_MODULE_Visualization=OFF -DUSE_TK=OFF \
  -DUSE_FREETYPE=OFF -DUSE_TBB=OFF -DUSE_RAPIDJSON=ON -DINSTALL_DIR=/opt/occt8
ninja -C /root/occt-build && ninja -C /root/occt-build install
```

Then Weft:

```bash
cmake -B build/linux-dev -G Ninja -DCMAKE_BUILD_TYPE=Release \
  -DWEFT_WARNINGS_AS_ERRORS=OFF -DOpenCASCADE_DIR=/opt/occt8/lib/cmake/opencascade
cmake --build build/linux-dev -j4
LD_LIBRARY_PATH=/opt/occt8/lib ctest --test-dir build/linux-dev --output-on-failure
```

Expect 14/14. `LD_LIBRARY_PATH` is required (no RPATH). Piping through
`tail` eats exit codes — check `${PIPESTATUS[0]}`.

## 4. Where the code lives

- Import + certificates: `core/src/secure_core.cpp` (`buildImportedModel`
  is the audit heart: correspondence, evidence rows, diagnostics),
  `core/src/io/{step_reader,brep_reader,iges_reader,xcaf}.cpp`.
- Conservative repairs: `core/src/secure_repair.cpp` (derivation order:
  identity copy → parameterization flags → orientation → tolerance →
  sewing scan), `core/src/secure_orientation.cpp`,
  `core/src/secure_sewing.cpp`.
- Reconnaissance/regions: `core/src/secure_reconnaissance.cpp`.
- Canonical boundaries/intervals: `core/src/canonical_boundary.cpp`,
  `core/src/interval_solver.cpp`.
- Certified floor + gates: `core/src/planar_cdt.cpp`,
  `core/src/cylinder_template.cpp`, `core/src/certified_mesh.cpp`
  (triangle-disjointness and Euler gates live here),
  `core/src/secure_meshing.cpp`, exact predicates in
  `core/src/geometric_predicates.cpp` (orient2d/orient3d/incircle/segments,
  exact over finite doubles).
- Tests: `tests/test_secure_core.cpp` is the big battery; corpus contracts
  in `tests/test_generated_secure_corpus.cpp` and
  `tests/test_secure_corpus.cpp`.

## 5. Hard-won facts (do not rediscover these)

- `BRepCheck_Analyzer` accepts inside-out coherent solids; polarity must be
  proven with independent volume + infinite-point classification.
- OCCT cannot stream-parse IGES (`IFSelect_WorkLibrary::ReadStream` fails
  unconditionally); the IGES reader materializes its byte snapshot to a
  scratch file and re-verifies the bytes after parsing.
- The IGES file parser accepts ANY bytes; malformed input only surfaces at
  the transfer-roots gate.
- `BRep_Builder::Add/Remove` re-express the child against the parent
  HANDLE's orientation/location — normalize the handle to FORWARD/identity
  before rebuilding stored child lists.
- `TopoDS_Iterator(shape, false, false)` gives stored (relative) children;
  the topology account records RELATIVE orientations, so an occurrence
  flip changes exactly one record.
- In `orient3d`-based piercing tests, a zero sign about a triangle edge
  means "on that edge's line", NOT contact; only mixed strict signs prove
  the piercing point outside.
- A B-rep face with w wires contributes `2 - w` to the Euler
  characteristic (a holed face is not a disk).
- Seam coedge occurrences each own representationIndex 0 (their own
  oriented p-curve) by construction.
- `tests/test_certified_mesh.cpp` pins the certified check-row count
  (currently 10); adding a validation row means updating that pin.
- The product IGES writer (`occ_writers.cpp`) crashed nondeterministically
  on this dev vehicle inside `IGESControl_Writer::AddShape`; the committed
  IGES witnesses were authored with a plain writer in an isolated process.
  Verify writer stability on Windows before trusting it.

## 6. How to continue

1. Open `docs/governance/GATES.md`. Pick the first unticked item you can
   land completely (suggested order: M6 partial cylinders — unblocked by
   the M3 seam item — or the deferred witness list, or M1 hollow-solid
   repair building on the detection in `secure_orientation.cpp`).
2. Read the neighbouring code and any named ADR before writing anything.
3. Implement with the rules in §2; add/extend witnesses per the deferred
   list where they prove the increment.
4. Run the full battery; fix regressions honestly (update pinned contracts
   only when the contract change is the intent).
5. Tick the item in GATES.md with the commit hash, keep
   `milestones.md` rows truthful, commit as the owner, push to this
   branch with `git push -u origin claude/m1-face-adjacency-orientation-4ya4uc`.
6. Windows lanes: when run, record outcomes; until then every new item is
   partial evidence by definition.
