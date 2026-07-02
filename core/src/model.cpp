#include "weft/model.hpp"

#include <BRepBuilderAPI_Sewing.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <STEPControl_Reader.hxx>
#include <STEPControl_Writer.hxx>
#include <BRepBuilderAPI_Sewing.hxx>
#include <ShapeFix_Shape.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>

#include <stdexcept>

namespace weft {

static Model indexShape(const TopoDS_Shape& shape) {
    Model m;
    m.shape = shape;
    TopExp::MapShapes(shape, TopAbs_FACE, m.faces);
    TopExp::MapShapes(shape, TopAbs_EDGE, m.edges);
    TopExp::MapShapesAndAncestors(shape, TopAbs_EDGE, TopAbs_FACE, m.edgeToFaces);
    return m;
}

Model loadStep(const std::string& path) {
    STEPControl_Reader reader;
    IFSelect_ReturnStatus status = reader.ReadFile(path.c_str());
    if (status != IFSelect_RetDone) {
        throw std::runtime_error("failed to read STEP file: " + path);
    }
    reader.TransferRoots();
    TopoDS_Shape shape = reader.OneShape();
    if (shape.IsNull()) {
        throw std::runtime_error("STEP file contained no transferable shapes: " + path);
    }

    // Sew faces that arrive with their own duplicate copies of shared
    // edges (common in some exporters): unshared edges can't take part in
    // density matching or welding, leaving open seams through the model.
    BRepBuilderAPI_Sewing sewing(1e-4);
    sewing.Add(shape);
    sewing.Perform();
    if (!sewing.SewedShape().IsNull()) shape = sewing.SewedShape();

    ShapeFix_Shape fixer(shape);
    fixer.Perform();
    shape = fixer.Shape();

    return indexShape(shape);
}

void writeStep(const TopoDS_Shape& shape, const std::string& path) {
    STEPControl_Writer writer;
    writer.Transfer(shape, STEPControl_AsIs);
    if (writer.Write(path.c_str()) != IFSelect_RetDone) {
        throw std::runtime_error("failed to write STEP file: " + path);
    }
}

}  // namespace weft
