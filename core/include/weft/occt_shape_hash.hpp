#pragma once

#include <Standard_Integer.hxx>
#include <Standard_Version.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS_Shape.hxx>

#include <cstddef>
#include <functional>

namespace weft {

// OCCT 7.8+ exposes TopTools_ShapeMapHasher as a std-compatible hasher/equal
// functor. OCCT 7.6 provides only static HashCode/IsEqual helpers.
#if OCC_VERSION_HEX >= 0x070800
using OcctShapeHash = TopTools_ShapeMapHasher;
using OcctShapeEqual = TopTools_ShapeMapHasher;
#else
struct OcctShapeHash {
    std::size_t operator()(const TopoDS_Shape& shape) const {
        return static_cast<std::size_t>(
            TopTools_ShapeMapHasher::HashCode(shape, IntegerLast()));
    }
};

struct OcctShapeEqual {
    bool operator()(const TopoDS_Shape& left,
                    const TopoDS_Shape& right) const {
        return TopTools_ShapeMapHasher::IsEqual(left, right);
    }
};
#endif

}  // namespace weft
