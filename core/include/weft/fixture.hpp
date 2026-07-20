#pragma once

#include <TopoDS_Shape.hxx>

#include <string>

namespace weft {

// Deterministic OpenCASCADE fixtures for the geometry / interaction / dirty
// zoo. Names are the CAD_CORPUS fixture keys; see tests/CAD_CORPUS.tsv.
//
// Geometry zoo: cylinder, box, cone, sphere, torus, extrusion, canrev,
//   bspline_slab, bezier_slab, bezier_face, offset_slab, ellipse_plate,
//   fillet, microedge
// Interaction zoo: hole, plate_holes, slotted, barrel, barrel2, drilled,
//   notched, boss, bossfillet, ribbon, ribbonnotch, filletslot, hairline,
//   slitdrill, torture, compound2
// Dirty / adversarial: open_shell, dirty_gap, bezier_face
// Aliases: demo -> torture
TopoDS_Shape makeFixture(const std::string& name);

}  // namespace weft
