# M9 dependency / BR-006 audit - 2026-07-18

## Inventory (distributable path)

| Component | Source | License | Linked by | Distributable? |
|---|---|---|---|---|
| Open CASCADE Technology | system `find_package(OpenCASCADE)` | LGPL-2.1 + OCCT exception | weft_core | Yes with LGPL obligations |
| GLFW 3.3+/3.4 | system or FetchContent | Zlib | weft_app | Yes |
| Dear ImGui v1.90.9-docking | FetchContent | MIT | weft_app | Yes |
| OpenGL / Mesa | system | various | weft_app | Yes |
| Python3 Interpreter | host | PSF | tests only | N/A for runtime package |
| CGAL | not present | GPL/commercial | none | Must remain absent (BR-006) |

## BR-006 decision

- Current product builds do **not** link CGAL.
- Exact predicates / CDT / assembly use in-tree Weft code (see M3–M4 evidence).
- BR-006 status set to `SCHEDULED — no CGAL in current tree; keep distribution CGAL-free`.
- Reintroduction of CGAL requires commercial license or permissive replacement plus a new audit before any distribution package may link it.

## Next actions

1. Keep CI/package scripts free of CGAL.
2. When packaging Windows, repeat this inventory against the OCCT third-party DLL set.
