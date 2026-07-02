#pragma once

#include <TopoDS_Shape.hxx>

#include <string>

namespace weft {

// Test/demo geometry built with OCCT primitives so the pipeline can be
// exercised without external CAD files.
//
//   "cylinder" - a single solid cylinder (r=10, h=30)
//   "box"      - a single solid box (20 x 30 x 15)
//   "cone"     - solid cone to an apex (r=10, h=20)
//   "sphere"   - solid sphere (r=10)
//   "torus"    - solid torus (R=10, r=3)
//   "demo"     - compound of cylinder and box side by side
//   "boss"     - box fused with a cylindrical boss (exercises trimmed
//                planar faces, which must fall back to triangulation)
TopoDS_Shape makeFixture(const std::string& name);

}  // namespace weft
