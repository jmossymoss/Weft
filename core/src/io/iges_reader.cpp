#include "occ_rollback.hpp"
#include "xcaf.hpp"

#include "weft/io/reader.hpp"

#include <IFSelect_ReturnStatus.hxx>
#include <IGESCAFControl_Reader.hxx>
#include <TDocStd_Document.hxx>
#include <TopoDS_Shape.hxx>
#include <XCAFApp_Application.hxx>

#include <memory>
#include <stdexcept>
#include <string>

namespace weft::io {

namespace {

class IgesReader final : public Reader {
public:
    IgesReader() {
        m_reader.SetColorMode(true);  // same trap as STEP: set before Transfer
        m_reader.SetNameMode(true);
        m_reader.SetLayerMode(true);
    }

    bool readFile(const std::string& path) override {
        OccStaticVariablesRollback rb;
        m_path = path;
        return m_reader.ReadFile(path.c_str()) == IFSelect_RetDone;
    }

    Model transfer() override {
        OccStaticVariablesRollback rb;
        Handle(TDocStd_Document) doc;
        XCAFApp_Application::GetApplication()->NewDocument("BinXCAF", doc);
        m_reader.Transfer(doc);

        TopoDS_Shape oneShape = m_reader.OneShape();  // inherited from IGESControl_Reader
        if (oneShape.IsNull())
            throw std::runtime_error("IGES file contained no transferable shapes: " + m_path);

        // IGES carries no STEP-entity product names, so keep the XCAF names
        // cafToModel filled (nothing to override here).
        Model m = cafToModel(oneShape, doc);
        XCAFApp_Application::GetApplication()->Close(doc);  // release the XDE document
        return m;
    }

private:
    IGESCAFControl_Reader m_reader;
    std::string m_path;
};

}  // namespace

std::unique_ptr<Reader> makeIgesReader() { return std::make_unique<IgesReader>(); }

}  // namespace weft::io
