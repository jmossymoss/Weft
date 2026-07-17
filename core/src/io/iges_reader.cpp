#include "occ_rollback.hpp"
#include "xcaf.hpp"
#include "../secure_core_internal.hpp"

#include "weft/io/reader.hpp"

#include <IFSelect_ReturnStatus.hxx>
#include <IGESCAFControl_Reader.hxx>
#include <IGESControl_Controller.hxx>
#include <ShapeProcess.hxx>
#include <TDocStd_Document.hxx>
#include <TopoDS_Shape.hxx>
#include <XCAFApp_Application.hxx>

#include <cstdio>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <string>
#include <utility>

namespace weft::io {

namespace {

class IgesReader final : public Reader {
public:
    IgesReader() {
        IGESControl_Controller::Init();
        m_reader.SetColorMode(true);
        m_reader.SetNameMode(true);
        m_reader.SetLayerMode(true);
    }

    bool readFile(const std::string& path) override {
        OccStaticVariablesRollback rb;
        m_path = path;
        m_sourceBytes.clear();

        // Bind provenance and parsing to one immutable byte snapshot. Reopening
        // the path after hashing would permit a concurrent replace to make the
        // recorded digest describe different source bytes.
        std::ifstream input(path, std::ios::binary);
        if (!input) return false;
        m_sourceBytes.assign(std::istreambuf_iterator<char>(input),
                             std::istreambuf_iterator<char>());
        if (input.bad()) {
            m_sourceBytes.clear();
            return false;
        }
        // IGESCAFControl_Reader's stream parse is not reliable across OCCT
        // builds. Parse only from a private temp filled with the retained
        // snapshot so the caller path is never reopened and the digest still
        // describes the exact bytes transferred into OCCT.
        const std::filesystem::path tempPath =
            std::filesystem::temp_directory_path() /
            ("weft_iges_snapshot_" +
             std::to_string(static_cast<unsigned long long>(
                 reinterpret_cast<std::uintptr_t>(this))) +
             ".igs");
        {
            std::ofstream output(tempPath, std::ios::binary | std::ios::trunc);
            if (!output) {
                m_sourceBytes.clear();
                return false;
            }
            output.write(m_sourceBytes.data(),
                         static_cast<std::streamsize>(m_sourceBytes.size()));
            if (!output) {
                m_sourceBytes.clear();
                std::error_code ignored;
                std::filesystem::remove(tempPath, ignored);
                return false;
            }
        }
        const IFSelect_ReturnStatus status =
            m_reader.ReadFile(tempPath.string().c_str());
        std::error_code ignored;
        std::filesystem::remove(tempPath, ignored);
        if (status != IFSelect_RetDone) {
            m_sourceBytes.clear();
            return false;
        }
        return true;
    }

    Model transfer() override {
        OccStaticVariablesRollback rb;
        Handle(TDocStd_Document) doc;
        XCAFApp_Application::GetApplication()->NewDocument("BinXCAF", doc);
        m_reader.Transfer(doc);

        TopoDS_Shape oneShape = m_reader.OneShape();
        if (oneShape.IsNull())
            throw std::runtime_error(
                "IGES file contained no transferable shapes: " + m_path);

        Model m = cafToModel(oneShape, doc);
        XCAFApp_Application::GetApplication()->Close(doc);
        return m;
    }

    ImportedModel transferSecure(RepairProfile repairProfile) override {
        OccStaticVariablesRollback rb;

        if (m_sourceBytes.empty()) {
            throw SecureImportError(
                "import.iges.no_source_shape",
                "IGES reader has no immutable source snapshot: " + m_path);
        }

        const ShapeProcess::OperationsFlags noShapeProcessing;
        m_reader.SetShapeProcessFlags(noShapeProcessing);
        const auto processing = m_reader.GetShapeProcessFlags();
        if (!processing.second || processing.first.any()) {
            throw SecureImportError(
                "import.processing.not_disabled",
                "OCCT did not retain the processing-disabled IGES policy: " +
                    m_path);
        }

        const int requestedRootCount = m_reader.NbRootsForTransfer();
        if (requestedRootCount <= 0) {
            throw SecureImportError(
                "import.iges.no_transfer_roots",
                "IGES source contains no transferable roots: " + m_path);
        }

        Handle(TDocStd_Document) doc;
        XCAFApp_Application::GetApplication()->NewDocument("BinXCAF", doc);
        if (!m_reader.Transfer(doc)) {
            XCAFApp_Application::GetApplication()->Close(doc);
            throw SecureImportError(
                "import.iges.transfer_failed",
                "IGES/XDE secure transfer failed: " + m_path);
        }

        if (m_reader.NbShapes() != requestedRootCount) {
            XCAFApp_Application::GetApplication()->Close(doc);
            throw SecureImportError(
                "import.iges.partial_transfer",
                "IGES/XDE did not retain one source result for every requested "
                "root: " +
                    m_path);
        }

        TopoDS_Shape sourceShape = m_reader.OneShape();
        if (sourceShape.IsNull()) {
            XCAFApp_Application::GetApplication()->Close(doc);
            throw SecureImportError(
                "import.iges.no_source_shape",
                "IGES file contained no processing-disabled source shape: " +
                    m_path);
        }

        SourceMetadata metadata =
            secure_detail::readSourceMetadata(m_path, m_sourceBytes);
        metadata.importerVersion = "weft-secure-iges-0.1";
        metadata.effectiveTranslatorConfiguration = {
            "IGES/XDE transfer",
            "IGES shape processing: disabled",
            "immutable byte snapshot parsed via private temp",
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
    IGESCAFControl_Reader m_reader;
    std::string m_path;
    std::string m_sourceBytes;
};

}  // namespace

std::unique_ptr<Reader> makeIgesReader() {
    return std::make_unique<IgesReader>();
}

}  // namespace weft::io
