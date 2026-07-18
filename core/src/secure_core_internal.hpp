#pragma once

#include "weft/secure_core.hpp"

#include <BRepTools_History.hxx>
#include <Standard_Handle.hxx>
#include <TopoDS_Shape.hxx>

#include <stdexcept>
#include <string_view>
#include <unordered_map>

namespace weft::secure_detail {

class ExactShapeDerivationMap {
public:
    void bind(const TopoDS_Shape& source, const TopoDS_Shape& working) {
        if (source.IsNull() || working.IsNull()) return;
        const auto [found, inserted] = workingBySourceTShape_.try_emplace(
            source.TShape().get(), working);
        if (!inserted && !found->second.IsPartner(working)) {
            throw std::logic_error(
                "one source TShape mapped to multiple working TShapes");
        }
    }

    void rebind(const TopoDS_Shape& source, const TopoDS_Shape& working) {
        if (source.IsNull() || working.IsNull()) {
            throw std::logic_error(
                "exact-shape rebind requires non-null source and working");
        }
        workingBySourceTShape_[source.TShape().get()] = working;
    }

    TopoDS_Shape mapped(const TopoDS_Shape& source) const {
        if (source.IsNull()) return {};
        const auto found =
            workingBySourceTShape_.find(source.TShape().get());
        if (found == workingBySourceTShape_.end()) return {};
        // Return the stored working occurrence (TShape + location). Copying
        // only the TShape onto the source wrapper drops sew/heal locations and
        // breaks IsSame/Contains checks used by correspondence rebinds.
        return found->second;
    }

    bool maps(const TopoDS_Shape& source,
              const TopoDS_Shape& working) const {
        const TopoDS_Shape candidate = mapped(source);
        return !candidate.IsNull() && candidate.IsSame(working);
    }

    bool mapsPartner(const TopoDS_Shape& source,
                     const TopoDS_Shape& working) const {
        const TopoDS_Shape candidate = mapped(source);
        return !candidate.IsNull() && candidate.IsPartner(working);
    }

    std::size_t size() const noexcept {
        return workingBySourceTShape_.size();
    }

private:
    std::unordered_map<const void*, TopoDS_Shape> workingBySourceTShape_;
};

struct ConservativeWorkingDerivation {
    TopoDS_Shape shape;
    Handle(BRepTools_History) history;
    ExactShapeDerivationMap exactShapes;
    std::vector<RepairOperation> operations;
    std::vector<ParameterizationFlagChange> parameterizationFlagChanges;
    std::vector<OrientationChange> orientationChanges;
    std::vector<ToleranceChange> toleranceChanges;
    std::vector<ImportDiagnostic> namedRefusals;
};

struct CompatibilityWorkingDerivation {
    TopoDS_Shape shape;
    Handle(BRepTools_History) history;
    ExactShapeDerivationMap exactShapes;
    std::vector<RepairOperation> operations;
    std::vector<ImportDiagnostic> namedRefusals;
    std::size_t multiWaySplitCount = 0;
};

ConservativeWorkingDerivation deriveConservativeWorking(
    const Model& source);
CompatibilityWorkingDerivation deriveCompatibilityWorking(
    const Model& source);

TopologyAccount buildTopologyAccount(const Model& model);

SourceMetadata readSourceMetadata(const std::string& path,
                                  std::string_view sourceBytes);

ImportedModel buildImportedModel(
    Model source, Model working, SourceMetadata metadata,
    RepairProfile profile, const Handle(BRepTools_History)& history,
    const ExactShapeDerivationMap& exactShapeDerivation = {},
    std::vector<RepairOperation> operations = {},
    std::vector<ParameterizationFlagChange> parameterizationFlagChanges = {},
    std::vector<OrientationChange> orientationChanges = {},
    std::vector<ToleranceChange> toleranceChanges = {},
    std::vector<ImportDiagnostic> namedRefusals = {});

}  // namespace weft::secure_detail
