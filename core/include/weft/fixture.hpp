#pragma once

#include <TopoDS_Shape.hxx>

#include <string>

namespace weft {

// Test/demo geometry built with OCCT primitives so the pipeline can be
// exercised without external CAD files.
//
//   "cylinder" - a single solid cylinder (r=10, h=30)
//   "box"      - a single solid box (20 x 30 x 15)
//   "demo"     - compound of the two side by side
//   "boss"     - box fused with a cylindrical boss (exercises trimmed
//                planar faces, which must fall back to triangulation)
TopoDS_Shape makeFixture(const std::string& name);

}  // namespace weft
