#include "occ_rollback.hpp"
#include "xcaf.hpp"

#include "weft/io/reader.hpp"

#include <Standard_Version.hxx>

#include <IFSelect_ReturnStatus.hxx>
#include <Interface_InterfaceModel.hxx>
#include <Resource_FormatType.hxx>
#include <STEPCAFControl_Controller.hxx>
#include <STEPCAFControl_Reader.hxx>
#include <STEPControl_Reader.hxx>
#include <StepBasic_Product.hxx>
#include <StepBasic_ProductDefinition.hxx>
#include <StepBasic_ProductDefinitionFormation.hxx>
#include <StepRepr_ProductDefinitionShape.hxx>
#include <StepRepr_Representation.hxx>
#include <StepRepr_RepresentationItem.hxx>
#include <StepShape_ShapeDefinitionRepresentation.hxx>
#include <TCollection_HAsciiString.hxx>
#include <TDocStd_Document.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS_Iterator.hxx>
#include <TransferBRep.hxx>
#include <Transfer_TransientProcess.hxx>
#include <XCAFApp_Application.hxx>
#include <XSControl_TransferReader.hxx>
#include <XSControl_WorkSession.hxx>

#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace weft::io {

// --------------------------------------------------------------------------
// Legacy STEP-entity name walk, preserved verbatim from the old loadStep.
// STEP solid names are sourced from this walk (not XCAF) so re-exported OBJ
// object blocks stay byte-identical to the pre-io-subsystem output.
// --------------------------------------------------------------------------

static std::map<const void*, std::string> shapeNames(const STEPControl_Reader& reader) {
    std::map<const void*, std::string> names;
    const Handle(XSControl_TransferReader)& tr = reader.WS()->TransferReader();
    if (tr.IsNull()) return names;
    Handle(Transfer_TransientProcess) tp = tr->TransientProcess();
    if (tp.IsNull()) return names;
    for (int i = 1; i <= tp->NbMapped(); ++i) {
        Handle(StepRepr_RepresentationItem) item =
            Handle(StepRepr_RepresentationItem)::DownCast(tp->Mapped(i));
        if (item.IsNull() || item->Name().IsNull() || item->Name()->Length() == 0) {
            continue;
        }
        TopoDS_Shape sh = TransferBRep::ShapeResult(tp, tp->Mapped(i));
        if (sh.IsNull()) continue;
        names.try_emplace(sh.TShape().get(), item->Name()->ToCString());
    }
    Handle(Interface_InterfaceModel) im = reader.WS()->Model();
    if (im.IsNull()) return names;
    for (int e = 1; e <= im->NbEntities(); ++e) {
        Handle(StepShape_ShapeDefinitionRepresentation) sdr =
            Handle(StepShape_ShapeDefinitionRepresentation)::DownCast(im->Value(e));
        if (sdr.IsNull() || sdr->UsedRepresentation().IsNull()) continue;
        Handle(StepRepr_ProductDefinitionShape) pds =
            Handle(StepRepr_ProductDefinitionShape)::DownCast(
                sdr->Definition().PropertyDefinition());
        if (pds.IsNull()) continue;
        Handle(StepBasic_ProductDefinition) pd = pds->Definition().ProductDefinition();
        if (pd.IsNull() || pd->Formation().IsNull() || pd->Formation()->OfProduct().IsNull()) {
            continue;
        }
        Handle(TCollection_HAsciiString) pname = pd->Formation()->OfProduct()->Name();
        if (pname.IsNull() || pname->Length() == 0) continue;
        const Handle(StepRepr_Representation)& rep = sdr->UsedRepresentation();
        for (int k = 1; k <= rep->NbItems(); ++k) {
            if (rep->ItemsValue(k).IsNull()) continue;
            TopoDS_Shape sh = TransferBRep::ShapeResult(tp, rep->ItemsValue(k));
            if (sh.IsNull()) continue;
            names.try_emplace(sh.TShape().get(), pname->ToCString());
            for (TopoDS_Iterator it(sh); it.More(); it.Next()) {
                names.try_emplace(it.Value().TShape().get(), pname->ToCString());
            }
        }
    }
    return names;
}

static std::string solidName(const std::map<const void*, std::string>& names,
                             const TopoDS_Shape& solid) {
    auto it = names.find(solid.TShape().get());
    if (it != names.end()) return it->second;
    for (TopExp_Explorer sx(solid, TopAbs_SHELL); sx.More(); sx.Next()) {
        it = names.find(sx.Current().TShape().get());
        if (it != names.end()) return it->second;
    }
    return {};
}

// Reproduce the old loadStep name assignment EXACTLY: reset to empty, read
// names off the ORIGINAL (pre-heal) shapes, then pair with the healed solids
// by traversal order (suffixing instanced duplicates). Overwrites any XCAF
// names cafToModel filled, preserving byte-identical output.
static void applyLegacySolidNames(Model& m, const TopoDS_Shape& oneShape,
                                  const STEPControl_Reader& reader) {
    m.solidNames.assign(m.solids.Extent(), std::string());
    const std::map<const void*, std::string> names = shapeNames(reader);
    std::vector<std::string> rawNames;
    for (TopExp_Explorer sx(oneShape, TopAbs_SOLID); sx.More(); sx.Next())
        rawNames.push_back(solidName(names, sx.Current()));
    for (TopExp_Explorer sx(oneShape, TopAbs_SHELL, TopAbs_SOLID); sx.More(); sx.Next())
        rawNames.push_back(solidName(names, sx.Current()));
    if (static_cast<int>(rawNames.size()) == m.solids.Extent()) {
        std::map<std::string, int> used;
        for (int sid = 1; sid <= m.solids.Extent(); ++sid) {
            std::string name = rawNames[sid - 1];
            if (name.empty()) continue;
            int n = ++used[name];
            if (n > 1) name += "_" + std::to_string(n);
            m.solidNames[sid - 1] = std::move(name);
        }
    }
}

// ------------------------------------------------------------- StepReader ---

namespace {

void applyStepReadStatics(OccStaticVariablesRollback& rb) {
    // These OCCT defaults are already 1 (all products/levels/representations),
    // so setting them changes nothing about the geometry transfer; the guard
    // just makes the process-global state explicit and restores it after.
    rb.change("read.step.product.context", 1);
    rb.change("read.step.assembly.level", 1);
    rb.change("read.step.shape.repr", 1);
    rb.change("read.stepcaf.subshapes.name", 1);
#if OCC_VERSION_HEX >= 0x070500
    rb.change("read.step.codepage", static_cast<int>(Resource_FormatType_UTF8));
#endif
}

class StepReader final : public Reader {
public:
    StepReader() {
        STEPCAFControl_Controller::Init();  // idempotent
        m_reader.SetColorMode(true);        // without these the tools stay empty
        m_reader.SetNameMode(true);         // and you silently get geometry only
        m_reader.SetLayerMode(true);
        m_reader.SetMatMode(true);
        m_reader.SetPropsMode(true);
    }

    bool readFile(const std::string& path) override {
        OccStaticVariablesRollback rb;
        applyStepReadStatics(rb);
        m_path = path;
        return m_reader.ReadFile(path.c_str()) == IFSelect_RetDone;
    }

    Model transfer() override {
        OccStaticVariablesRollback rb;
        applyStepReadStatics(rb);

        Handle(TDocStd_Document) doc;
        XCAFApp_Application::GetApplication()->NewDocument("BinXCAF", doc);
        m_reader.Transfer(doc);

        // Heal the SAME compound the old loadStep did (byte-identical
        // geometry) while XCAF carries the metadata off the same transfer.
        TopoDS_Shape oneShape = m_reader.Reader().OneShape();
        if (oneShape.IsNull())
            throw std::runtime_error("STEP file contained no transferable shapes: " + m_path);

        Model m = cafToModel(oneShape, doc);
        applyLegacySolidNames(m, oneShape, m_reader.Reader());
        XCAFApp_Application::GetApplication()->Close(doc);  // release the XDE document
        return m;
    }

private:
    STEPCAFControl_Reader m_reader;
    std::string m_path;
};

}  // namespace

std::unique_ptr<Reader> makeStepReader() { return std::make_unique<StepReader>(); }

}  // namespace weft::io
