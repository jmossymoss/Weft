// B-rep-format writers for the weft::io registry. These serialize
// Model::shape (the healed CAD B-rep), NOT the retopo PolyMesh. Global
// Interface_Static writer knobs are wrapped in OccStaticVariablesRollback.
//
// Also hosts tessellate(): a default BRepMesh triangulation of Model::shape,
// used by `weft convert <brep> -o <mesh>` where there is no retopo step.

#include "occ_rollback.hpp"

#include "weft/io/system.hpp"
#include "weft/io/writer.hpp"
#include "weft/mesh.hpp"
#include "weft/model.hpp"

#include <BRepMesh_IncrementalMesh.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <IGESControl_Controller.hxx>
#include <IGESControl_Writer.hxx>
#include <Poly_Triangulation.hxx>
#include <STEPControl_Writer.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

#include <algorithm>
#include <memory>
#include <stdexcept>
#include <string>

namespace weft::io {

namespace {

const Model* requireBRep(const WriteInput& in) {
    if (!in.model || in.model->shape.IsNull()) return nullptr;
    return in.model;
}

// STEP schema wire-name -> Interface_Static "write.step.schema" integer.
int stepSchemaCode(const std::string& s) {
    if (s == "ap203") return 3;
    if (s == "ap242") return 4;  // AP242DIS
    return 2;                    // ap214 -> AP214DIS
}

const char* unitCode(const std::string& s) {
    if (s == "m") return "M";
    if (s == "in") return "INCH";
    if (s == "ft") return "FOOT";
    return "MM";
}

// -------------------------------------------------------------------- STEP ---
class StepWriter final : public Writer {
public:
    void applyParams(const ParamGroup& g) override {
        m_schema = g.getEnum("schema");
        m_unit = g.getEnum("unit");
    }
    bool transfer(const WriteInput& in) override {
        m_model = requireBRep(in);
        return m_model != nullptr;
    }
    bool writeFile(const std::string& path) override {
        OccStaticVariablesRollback rb;  // schema must be set before Writer ctor
        rb.change("write.step.schema", stepSchemaCode(m_schema));
        rb.change("write.step.unit", unitCode(m_unit));
        STEPControl_Writer writer;
        if (writer.Transfer(m_model->shape, STEPControl_AsIs) != IFSelect_RetDone)
            throw std::runtime_error("STEP transfer failed: " + path);
        if (writer.Write(path.c_str()) != IFSelect_RetDone)
            throw std::runtime_error("failed to write STEP: " + path);
        return true;
    }

private:
    const Model* m_model = nullptr;
    std::string m_schema = "ap214";
    std::string m_unit = "mm";
};

// -------------------------------------------------------------------- IGES ---
class IgesWriter final : public Writer {
public:
    void applyParams(const ParamGroup& g) override {
        m_brepMode = g.getEnum("brepMode");
        m_unit = g.getEnum("unit");
    }
    bool transfer(const WriteInput& in) override {
        m_model = requireBRep(in);
        return m_model != nullptr;
    }
    bool writeFile(const std::string& path) override {
        OccStaticVariablesRollback rb;
        IGESControl_Controller::Init();
        const int mode = m_brepMode == "brep" ? 1 : 0;  // 0=faces, 1=solids/brep
        IGESControl_Writer writer(unitCode(m_unit), mode);
        if (!writer.AddShape(m_model->shape))
            throw std::runtime_error("IGES AddShape failed: " + path);
        writer.ComputeModel();
        if (!writer.Write(path.c_str()))
            throw std::runtime_error("failed to write IGES: " + path);
        return true;
    }

private:
    const Model* m_model = nullptr;
    std::string m_brepMode = "faces";
    std::string m_unit = "mm";
};

// -------------------------------------------------------------------- BREP ---
class BrepWriter final : public Writer {
public:
    bool transfer(const WriteInput& in) override {
        m_model = requireBRep(in);
        return m_model != nullptr;
    }
    bool writeFile(const std::string& path) override {
        if (!BRepTools::Write(m_model->shape, path.c_str()))
            throw std::runtime_error("failed to write BREP: " + path);
        return true;
    }

private:
    const Model* m_model = nullptr;
};

// ------------------------------------------------------------------ factory ---
class OccFactoryWriter final : public FactoryWriter {
public:
    std::span<const Format> formats() const override {
        static constexpr Format kF[] = {Format::Step, Format::Iges, Format::Brep};
        return kF;
    }

    std::unique_ptr<Writer> create(Format f) const override {
        switch (f) {
            case Format::Step: return std::make_unique<StepWriter>();
            case Format::Iges: return std::make_unique<IgesWriter>();
            case Format::Brep: return std::make_unique<BrepWriter>();
            default: return nullptr;
        }
    }

    ParamGroup createParams(Format f) const override {
        ParamGroup g;
        switch (f) {
            case Format::Step:
                g.specs = {
                    {"schema", ParamSpec::Enum, {"ap203", "ap214", "ap242"}, "ap214",
                     "STEP application protocol"},
                    {"unit", ParamSpec::Enum, {"mm", "m", "in", "ft"}, "mm", "length unit"},
                    {"assembly", ParamSpec::Bool, {}, "true", "write assembly structure"},
                };
                break;
            case Format::Iges:
                g.specs = {
                    {"brepMode", ParamSpec::Enum, {"faces", "brep"}, "faces", "IGES BRep mode"},
                    {"unit", ParamSpec::Enum, {"mm", "m", "in"}, "mm", "length unit"},
                };
                break;
            default: break;  // Brep has no params
        }
        g.resetToDefaults();
        return g;
    }
};

}  // namespace

std::unique_ptr<FactoryWriter> makeOccFactoryWriter() {
    return std::make_unique<OccFactoryWriter>();
}

// Default tessellation of Model::shape for B-rep -> mesh conversion. FaceId
// (the model's face-map index) rides along as polygonFaceId so downstream
// grouping/coloring still works.
PolyMesh tessellate(const Model& model) {
    PolyMesh mesh;
    if (model.shape.IsNull()) return mesh;
    BRepMesh_IncrementalMesh mesher(model.shape, 0.1, Standard_False, 0.5, Standard_True);
    mesher.Perform();
    int triangulated = 0;
    for (int fid = 1; fid <= model.faces.Extent(); ++fid) {
        TopoDS_Face face = TopoDS::Face(model.faces(fid));
        TopLoc_Location loc;
        Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
        if (tri.IsNull()) continue;
        ++triangulated;
        const gp_Trsf trsf = loc.Transformation();
        const uint32_t base = static_cast<uint32_t>(mesh.vertices.size());
        for (int i = 1; i <= tri->NbNodes(); ++i) {
            gp_Pnt p = tri->Node(i).Transformed(trsf);
            mesh.vertices.push_back({p.X(), p.Y(), p.Z()});
            mesh.anchors.push_back(Anchor{fid, 0.0, 0.0});
        }
        const bool rev = face.Orientation() == TopAbs_REVERSED;
        for (int i = 1; i <= tri->NbTriangles(); ++i) {
            int a = 0, b = 0, c = 0;
            tri->Triangle(i).Get(a, b, c);
            if (rev) std::swap(b, c);
            mesh.polygons.push_back({base + static_cast<uint32_t>(a - 1),
                                     base + static_cast<uint32_t>(b - 1),
                                     base + static_cast<uint32_t>(c - 1)});
            mesh.polygonFaceId.push_back(fid);
        }
    }
    // BRepMesh reports per-face failure only by leaving the face without a
    // triangulation. Skipping a few faces is a lossy-but-usable conversion;
    // producing nothing at all is a failed one, and writing that as an empty
    // mesh would report success for a file with no geometry in it.
    if (triangulated == 0 && model.faces.Extent() > 0) {
        throw std::runtime_error("tessellation produced no triangles for any "
                                 "of the model's " +
                                 std::to_string(model.faces.Extent()) +
                                 " face(s)");
    }
    return mesh;
}

}  // namespace weft::io
