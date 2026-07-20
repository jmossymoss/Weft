#pragma once

#include <TopoDS_Shape.hxx>

#include <string>

namespace weft {

// Deterministic OpenCASCADE fixtures for the geometry / interaction / dirty
// zoo. Names are the CAD_CORPUS fixture keys; see tests/CAD_CORPUS.tsv.
//
// Geometry zoo: cylinder, box, cone, sphere, torus, extrusion, canrev,
//   bspline_slab, bezier_slab, bezier_face, offset_slab, ellipse_plate,
//   parabola_plate, hyperbola_plate, bezier_curve, bspline_curve,
//   offset_curve, rev_wire, unequal_rims, fillet, microedge
// Interaction zoo: hole, plate_holes, slotted, barrel, barrel2, drilled,
//   notched, boss, bossfillet, ribbon, ribbonnotch, filletslot, hairline,
//   slitdrill, torture, compound2
// Dirty / adversarial (§4.2): open_shell, dirty_gap, bezier_face,
//   sliver, near_dup, gap_lo, gap_at, rev_orient, dup_trim, bowtie,
//   tan_slit, seam_cut, hi_aspect, tiny_big
// Aliases: demo -> torture
TopoDS_Shape makeFixture(const std::string& name);

}  // namespace weft
