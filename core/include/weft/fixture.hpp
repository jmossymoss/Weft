#pragma once

#include <TopoDS_Shape.hxx>

#include <string>

namespace weft {

// Test/demo geometry built with OCCT primitives so the pipeline can be
// exercised without external CAD files.
//
//   "cylinder" - a single solid cylinder (r=10, h=30)
//   "partial_cylinder" - solid cylinder sector (r=10, h=30, 270 deg)
//   "box"      - a single solid box (20 x 30 x 15)
//   "cone"     - solid cone to an apex (r=10, h=20)
//   "truncated_cone" - solid frustum (r1=10, r2=4, h=20)
//   "sphere"   - solid sphere (r=10)
//   "sphere_cap" - spherical segment to the equator (single-pole cap)
//   "torus"    - solid torus (R=10, r=3)
//   "fillet"   - box with one long edge blended r=4 (a fillet strip with
//                two tangent-smooth joins, for support-loop testing)
//   "hole"     - plate with a through-bore r=8 (two ring junctions and a
//                bore wall, for hole detection and junction testing)
//   "ellipse_hole" - plate with an elliptical through-cut (major=10,
//                minor=5) for ellipse boundary / interval consumers
//   "demo"     - compound of cylinder and box side by side
//   "boss"     - box fused with a cylindrical boss (exercises trimmed
//                planar faces, which must fall back to triangulation)
//   "curve_hyperbola" / "curve_parabola" / "curve_offset" — planar faces
//                whose boundary retains an unsupported curve family for
//                Wave F named-refuse locks (offset prefers native .brep)
//   "extrusion_quad" — four-sided SurfaceOfLinearExtrusion patch (Wave D)
//   "revolution_ngon" — five-sided SurfaceOfRevolution UV-trim (Wave D)
TopoDS_Shape makeFixture(const std::string& name);

}  // namespace weft
