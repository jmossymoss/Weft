#include "occ_rollback.hpp"
#include "xcaf.hpp"

#include "../secure_core_internal.hpp"

#include "weft/io/reader.hpp"

#include <IFSelect_ReturnStatus.hxx>
#include <IGESCAFControl_Reader.hxx>
#include <ShapeProcess.hxx>
#include <TDocStd_Document.hxx>
#include <TopoDS_Shape.hxx>
#include <XCAFApp_Application.hxx>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <memory>
#include <sstream>
#include <stdexcept>
#include <string>
#include <system_error>
#include <utility>

#if defined(_WIN32)
#include <process.h>
#else
#include <unistd.h>
#endif

namespace weft::io {

namespace {

std::uint64_t currentProcessId() noexcept {
#if defined(_WIN32)
    return static_cast<std::uint64_t>(_getpid());
#else
    return static_cast<std::uint64_t>(getpid());
#endif
}

// A private scratch path for materializing one retained byte snapshot.
// OCCT's IGES work library cannot parse from a stream (its ReadStream
// default fails), so the snapshot is written here, parsed, and re-verified.
std::filesystem::path snapshotScratchPath() {
    static std::atomic<std::uint64_t> ordinal{0};
    const auto ticks = static_cast<std::uint64_t>(
        std::chrono::steady_clock::now().time_since_epoch().count());
    return std::filesystem::temp_directory_path() /
        ("weft-iges-snapshot-" + std::to_string(currentProcessId()) + "-" +
         std::to_string(ticks) + "-" +
         std::to_string(ordinal.fetch_add(1)) + ".igs");
}

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
        m_sourceBytes.clear();
        m_snapshotValid = false;

        // Bind the byte digest and parsed IGES model to the same retained
        // snapshot. The original path is read exactly once; replacing it
        // afterwards cannot change what was hashed or parsed.
        std::ifstream input(path, std::ios::binary);
        if (!input) return false;
        m_sourceBytes.assign(std::istreambuf_iterator<char>(input),
                             std::istreambuf_iterator<char>());
        if (input.bad()) {
            m_sourceBytes.clear();
            return false;
        }

        // OCCT's IGES work library has no stream parsing (the base
        // ReadStream unconditionally fails), so materialize the retained
        // bytes to a private scratch file, parse that, and then re-read the
        // scratch bytes: parsing is accepted only when they still equal the
        // retained snapshot, so hashing and parsing provably consumed the
        // same bytes.
        const std::filesystem::path scratch = snapshotScratchPath();
        bool parsed = false;
        try {
            {
                std::ofstream output(scratch, std::ios::binary |
                                                  std::ios::trunc);
                if (!output) return false;
                output.write(m_sourceBytes.data(),
                             static_cast<std::streamsize>(
                                 m_sourceBytes.size()));
                if (!output) {
                    m_sourceBytes.clear();
                    return false;
                }
            }
            parsed = m_reader.ReadFile(scratch.string().c_str()) ==
                     IFSelect_RetDone;
            if (parsed) {
                std::ifstream verify(scratch, std::ios::binary);
                std::string parsedBytes(
                    (std::istreambuf_iterator<char>(verify)),
                    std::istreambuf_iterator<char>());
                parsed = !verify.bad() && parsedBytes == m_sourceBytes;
            }
        } catch (...) {
            std::error_code ignored;
            std::filesystem::remove(scratch, ignored);
            m_sourceBytes.clear();
            throw;
        }
        std::error_code ignored;
        std::filesystem::remove(scratch, ignored);
        if (!parsed) {
            m_sourceBytes.clear();
            return false;
        }
        m_snapshotValid = true;
        return true;
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

    ImportedModel transferSecure(RepairProfile repairProfile) override {
        OccStaticVariablesRollback rb;
        if (!m_snapshotValid) {
            throw SecureImportError(
                "import.iges.no_source_snapshot",
                "IGES reader retains no successfully parsed byte snapshot: " +
                    m_path);
        }

        // OCCT's IGES parser accepts arbitrary bytes and only materializes
        // a translator actor for real models, so gate on transfer roots
        // before the processing freeze can be verified meaningfully.
        const int requestedRootCount = m_reader.NbRootsForTransfer();
        if (requestedRootCount <= 0) {
            throw SecureImportError(
                "import.iges.no_transfer_roots",
                "IGES source contains no transferable roots: " + m_path);
        }

        // The transferred B-rep is evidence of the deterministic IGES
        // translation. Freeze the post-transfer shape-processing policy to
        // empty and verify OCCT retained the request; the intrinsic
        // IGES-to-BRep conversion itself is translator-defined and is
        // recorded as such in the configuration.
        const ShapeProcess::OperationsFlags noShapeProcessing;
        m_reader.SetShapeProcessFlags(noShapeProcessing);
        const auto processing = m_reader.GetShapeProcessFlags();
        if (!processing.second || processing.first.any()) {
            throw SecureImportError(
                "import.processing.not_disabled",
                "OCCT did not retain the processing-disabled IGES policy: " +
                    m_path);
        }

        Handle(TDocStd_Document) doc;
        XCAFApp_Application::GetApplication()->NewDocument("BinXCAF", doc);
        if (!m_reader.Transfer(doc)) {
            XCAFApp_Application::GetApplication()->Close(doc);
            throw SecureImportError(
                "import.iges.transfer_failed",
                "IGES/XDE secure transfer failed: " + m_path);
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
            "IGES/XDE transfer from immutable byte snapshot",
            "XDE shape processing: disabled",
            "intrinsic IGES-to-BRep conversion: translator-defined",
            "requested roots: " + std::to_string(requestedRootCount),
            "repair profile: " +
                std::string(repairProfileName(repairProfile)),
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
    bool m_snapshotValid = false;
};

}  // namespace

std::unique_ptr<Reader> makeIgesReader() { return std::make_unique<IgesReader>(); }

}  // namespace weft::io
