#include "xcaf.hpp"  // weft::healWithHistory / weft::indexShape

#include "../secure_core_internal.hpp"

#include "weft/io/reader.hpp"

#include <BRepTools.hxx>
#include <BRepTools_History.hxx>
#include <BRep_Builder.hxx>
#include <TopoDS_Shape.hxx>

#include <fstream>
#include <iterator>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace weft::io {

namespace {

// BREP carries only geometry — no color/name/layer/material/hierarchy — so it
// bypasses XCAF and runs straight through the shared heal + index path.
class BrepReader final : public Reader {
public:
    bool readFile(const std::string& path) override {
        m_path = path;
        m_shape.Nullify();
        m_sourceBytes.clear();

        // Bind the byte digest and parsed source shape to the same immutable
        // snapshot. Reopening the path would permit replacement between those
        // two operations, exactly as on the secure STEP path.
        std::ifstream input(path, std::ios::binary);
        if (!input) return false;
        m_sourceBytes.assign(std::istreambuf_iterator<char>(input),
                             std::istreambuf_iterator<char>());
        if (input.bad()) {
            m_sourceBytes.clear();
            return false;
        }

        std::istringstream source(
            m_sourceBytes, std::ios::in | std::ios::binary);
        BRep_Builder builder;
        try {
            BRepTools::Read(m_shape, source, builder);
        } catch (...) {
            m_shape.Nullify();
            throw;
        }
        if (m_shape.IsNull() || source.bad()) {
            m_shape.Nullify();
            return false;
        }
        return true;
    }

    Model transfer() override {
        if (m_shape.IsNull())
            throw std::runtime_error("BREP file contained no shape: " + m_path);
        Handle(BRepTools_History) hist;
        TopoDS_Shape healed = weft::healWithHistory(m_shape, hist);
        return weft::indexShape(healed);
    }

    ImportedModel transferSecure(RepairProfile repairProfile) override {
        if (m_shape.IsNull()) {
            throw SecureImportError(
                "import.brep.no_source_shape",
                "native B-rep snapshot contained no source shape: " + m_path);
        }

        SourceMetadata metadata = secure_detail::readSourceMetadata(
            m_path, m_sourceBytes);
        metadata.importerVersion = "weft-secure-brep-0.1";
        metadata.effectiveTranslatorConfiguration = {
            "OCCT ASCII B-rep stream parse from immutable byte snapshot",
            "shape processing: none",
            "length unit: unspecified native model units",
            "repair profile: " +
                std::string(repairProfileName(repairProfile)),
        };

        Model source = weft::indexShape(m_shape);
        TopoDS_Shape workingShape;
        Handle(BRepTools_History) workingHistory;
        secure_detail::ExactShapeDerivationMap exactShapeDerivation;
        std::vector<RepairOperation> operations;
        std::vector<ParameterizationFlagChange> parameterizationFlagChanges;
        if (repairProfile == RepairProfile::Conservative) {
            secure_detail::ConservativeWorkingDerivation derivation =
                secure_detail::deriveConservativeWorking(source);
            workingShape = std::move(derivation.shape);
            workingHistory = std::move(derivation.history);
            exactShapeDerivation = std::move(derivation.exactShapes);
            operations = std::move(derivation.operations);
            parameterizationFlagChanges =
                std::move(derivation.parameterizationFlagChanges);
        } else {
            secure_detail::CompatibilityWorkingDerivation derivation =
                secure_detail::deriveCompatibilityWorking(source);
            workingShape = std::move(derivation.shape);
            workingHistory = std::move(derivation.history);
            operations = std::move(derivation.operations);
        }

        Model working = weft::indexShape(workingShape);
        ImportedModel imported = secure_detail::buildImportedModel(
            std::move(source), std::move(working), std::move(metadata),
            repairProfile, workingHistory, exactShapeDerivation,
            std::move(operations),
            std::move(parameterizationFlagChanges));
        imported.diagnostics.events.push_back({
            {StableIdKind::Diagnostic,
             static_cast<std::uint64_t>(imported.diagnostics.events.size() + 1)},
            "import.brep.length_unit_unspecified",
            DiagnosticSeverity::Warning,
            {{StableIdKind::Model, 1}},
            "native OCCT ASCII B-rep declares no physical length unit; coordinates remain unchanged in source model units"});
        return imported;
    }

private:
    TopoDS_Shape m_shape;
    std::string m_path;
    std::string m_sourceBytes;
};

}  // namespace

std::unique_ptr<Reader> makeBrepReader() { return std::make_unique<BrepReader>(); }

}  // namespace weft::io
