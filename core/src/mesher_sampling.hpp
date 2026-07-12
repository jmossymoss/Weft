#pragma once

class Adaptor3d_Curve;

namespace weft::mesher_detail {

// Deterministic subdivision count for the uniform-parameter border contract.
int stableDeflectionCount(const Adaptor3d_Curve& curve, double angleTolerance,
                          double chordTolerance);

}  // namespace weft::mesher_detail
