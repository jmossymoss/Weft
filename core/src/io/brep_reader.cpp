#include "xcaf.hpp"  // weft::healWithHistory / weft::indexShape

#include "weft/io/reader.hpp"

#include <BRepTools.hxx>
#include <BRepTools_History.hxx>
#include <BRep_Builder.hxx>
#include <TopoDS_Shape.hxx>

#include <memory>
#include <stdexcept>
#include <string>

namespace weft::io {

namespace {

// BREP carries only geometry — no color/name/layer/material/hierarchy — so it
// bypasses XCAF and runs straight through the shared heal + index path.
class BrepReader final : public Reader {
public:
    bool readFile(const std::string& path) override {
        m_path = path;
        BRep_Builder builder;
        return BRepTools::Read(m_shape, path.c_str(), builder) && !m_shape.IsNull();
    }

    Model transfer() override {
        if (m_shape.IsNull())
            throw std::runtime_error("BREP file contained no shape: " + m_path);
        Handle(BRepTools_History) hist;
        TopoDS_Shape healed = weft::healWithHistory(m_shape, hist);
        return weft::indexShape(healed);
    }

private:
    TopoDS_Shape m_shape;
    std::string m_path;
};

}  // namespace

std::unique_ptr<Reader> makeBrepReader() { return std::make_unique<BrepReader>(); }

}  // namespace weft::io
