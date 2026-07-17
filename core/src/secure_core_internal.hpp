#pragma once

#include "weft/secure_core.hpp"

#include <BRepTools_History.hxx>
#include <Standard_Handle.hxx>

#include <string_view>

namespace weft::secure_detail {

TopologyAccount buildTopologyAccount(const Model& model);

SourceMetadata readSourceMetadata(const std::string& path,
                                  std::string_view sourceBytes);

ImportedModel buildImportedModel(
    Model source, Model working, SourceMetadata metadata,
    RepairProfile profile, const Handle(BRepTools_History)& history,
    std::vector<RepairOperation> operations = {});

}  // namespace weft::secure_detail
