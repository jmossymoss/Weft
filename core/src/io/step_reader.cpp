#include "occ_rollback.hpp"
#include "xcaf.hpp"
#include "../secure_core_internal.hpp"

#include "weft/io/reader.hpp"

#include <Standard_Version.hxx>

#include <APIHeaderSection_MakeHeader.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Interface_InterfaceModel.hxx>
#include <Resource_FormatType.hxx>
#include <ShapeProcess.hxx>
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

#include <algorithm>
#include <map>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <iterator>
#include <memory>
#include <optional>
#include <sstream>
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

std::optional<std::string> stepSchema(const STEPCAFControl_Reader& reader) {
    const auto model = reader.Reader().StepModel();
    if (model.IsNull()) return std::nullopt;
    APIHeaderSection_MakeHeader header(model);
    if (!header.HasFs() || header.NbSchemaIdentifiers() <= 0) {
        return std::nullopt;
    }
    const auto identifier = header.SchemaIdentifiersValue(1);
    if (identifier.IsNull() || identifier->Length() == 0) {
        return std::nullopt;
    }
    std::string schema = identifier->ToCString();
    std::transform(schema.begin(), schema.end(), schema.begin(), [](char value) {
        return value >= 'a' && value <= 'z'
                   ? static_cast<char>(value - 'a' + 'A')
                   : value;
    });
    if (schema.find("AP242") != std::string::npos ||
        schema.find("MANAGED_MODEL_BASED_3D_ENGINEERING") != std::string::npos) {
        return "AP242";
    }
    if (schema.find("AP214") != std::string::npos ||
        schema.find("AUTOMOTIVE_DESIGN") != std::string::npos) {
        return "AP214";
    }
    if (schema.find("AP203") != std::string::npos ||
        schema.find("CONFIG_CONTROL_DESIGN") != std::string::npos) {
        return "AP203";
    }
    return schema;
}

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
        const auto begin = std::chrono::steady_clock::now();
        OccStaticVariablesRollback rb;
        applyStepReadStatics(rb);
        m_path = path;
        // Bind provenance and translation to one immutable byte snapshot.
        // Reopening the path after hashing would permit a concurrent replace
        // to make the recorded digest describe different source bytes.
        std::ifstream input(path, std::ios::binary);
        if (!input) return false;
        m_sourceBytes.assign(std::istreambuf_iterator<char>(input),
                             std::istreambuf_iterator<char>());
        if (input.bad()) return false;
        std::istringstream source(
            m_sourceBytes, std::ios::in | std::ios::binary);
        const bool ok =
            m_reader.ReadStream(path.c_str(), source) == IFSelect_RetDone;
        if (std::getenv("WEFT_PROFILE_IMPORT")) {
            std::fprintf(stderr, "import profile: %-24s %8lld ms\n", "STEP parse",
                         static_cast<long long>(
                             std::chrono::duration_cast<std::chrono::milliseconds>(
                                 std::chrono::steady_clock::now() - begin)
                                 .count()));
        }
        return ok;
    }

    Model transfer() override {
        const bool profile = std::getenv("WEFT_PROFILE_IMPORT") != nullptr;
        auto last = std::chrono::steady_clock::now();
        auto mark = [&](const char* stage) {
            if (!profile) return;
            const auto now = std::chrono::steady_clock::now();
            std::fprintf(stderr, "import profile: %-24s %8lld ms\n", stage,
                         static_cast<long long>(
                             std::chrono::duration_cast<std::chrono::milliseconds>(
                                 now - last)
                                 .count()));
            last = now;
        };
        OccStaticVariablesRollback rb;
        applyStepReadStatics(rb);

        Handle(TDocStd_Document) doc;
        XCAFApp_Application::GetApplication()->NewDocument("BinXCAF", doc);
        m_reader.Transfer(doc);
        mark("STEP transfer/XCAF");

        // Heal the SAME compound the old loadStep did (byte-identical
        // geometry) while XCAF carries the metadata off the same transfer.
        TopoDS_Shape oneShape = m_reader.Reader().OneShape();
        if (oneShape.IsNull())
            throw std::runtime_error("STEP file contained no transferable shapes: " + m_path);

        Model m = cafToModel(oneShape, doc);
        mark("heal/index/metadata");
        applyLegacySolidNames(m, oneShape, m_reader.Reader());
        mark("legacy names");
        XCAFApp_Application::GetApplication()->Close(doc);  // release the XDE document
        return m;
    }

    ImportedModel transferSecure(RepairProfile repairProfile) override {
        OccStaticVariablesRollback rb;
        applyStepReadStatics(rb);

        // The transfer result below is evidence, not a convenience shape.
        // Freeze both the XDE reader and its base STEP reader to an empty
        // processing policy and verify OCCT retained that request.
        const ShapeProcess::OperationsFlags noShapeProcessing;
        m_reader.SetShapeProcessFlags(noShapeProcessing);
        m_reader.ChangeReader().SetShapeProcessFlags(noShapeProcessing);
        const auto xdeProcessing = m_reader.GetShapeProcessFlags();
        const auto baseProcessing = m_reader.ChangeReader().GetShapeProcessFlags();
        if (!xdeProcessing.second || xdeProcessing.first.any() ||
            !baseProcessing.second || baseProcessing.first.any()) {
            throw SecureImportError(
                "import.processing.not_disabled",
                "OCCT did not retain the processing-disabled STEP policy: " + m_path);
        }

        const int requestedRootCount = m_reader.NbRootsForTransfer();
        if (requestedRootCount <= 0) {
            throw SecureImportError(
                "import.step.no_transfer_roots",
                "STEP source contains no transferable product roots: " + m_path);
        }

        Handle(TDocStd_Document) doc;
        XCAFApp_Application::GetApplication()->NewDocument("BinXCAF", doc);
        if (!m_reader.Transfer(doc)) {
            XCAFApp_Application::GetApplication()->Close(doc);
            throw SecureImportError("import.step.transfer_failed",
                                    "STEP/XDE secure transfer failed: " + m_path);
        }

        if (m_reader.Reader().NbShapes() != requestedRootCount) {
            XCAFApp_Application::GetApplication()->Close(doc);
            throw SecureImportError(
                "import.step.partial_transfer",
                "STEP/XDE did not retain one source result for every requested root: " +
                    m_path);
        }

        TopoDS_Shape sourceShape = m_reader.Reader().OneShape();
        if (sourceShape.IsNull()) {
            XCAFApp_Application::GetApplication()->Close(doc);
            throw SecureImportError(
                "import.step.no_source_shape",
                "STEP file contained no processing-disabled source shape: " + m_path);
        }

        SourceMetadata metadata =
            secure_detail::readSourceMetadata(m_path, m_sourceBytes);
        metadata.stepSchema = stepSchema(m_reader);
        metadata.effectiveTranslatorConfiguration = {
            "STEP/XDE transfer",
            "XDE shape processing: disabled",
            "base STEP shape processing: disabled",
            "requested roots: " + std::to_string(requestedRootCount),
            "repair profile: " + std::string(repairProfileName(repairProfile)),
        };

        try {
            ImportedModel imported = cafToImportedModel(
                sourceShape, doc, std::move(metadata), repairProfile);
            XCAFApp_Application::GetApplication()->Close(doc);
            return imported;
        } catch (...) {
            XCAFApp_Application::GetApplication()->Close(doc);
            throw;
        }
    }

private:
    STEPCAFControl_Reader m_reader;
    std::string m_path;
    std::string m_sourceBytes;
};

}  // namespace

std::unique_ptr<Reader> makeStepReader() { return std::make_unique<StepReader>(); }

}  // namespace weft::io
