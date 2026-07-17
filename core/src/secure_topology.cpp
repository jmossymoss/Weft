#include "secure_core_internal.hpp"

#include <BRep_Tool.hxx>
#include <Geom2d_Curve.hxx>
#include <Standard_Failure.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopLoc_Location.hxx>
#include <TopTools_ShapeMapHasher.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopExp_Explorer.hxx>
#include <gp_Trsf.hxx>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <map>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <utility>
#include <vector>

namespace weft {
namespace {

StableId id(StableIdKind kind, std::uint64_t ordinal) {
    return {kind, ordinal};
}

StableIdKind stableKind(TopAbs_ShapeEnum kind) {
    switch (kind) {
        case TopAbs_COMPOUND: return StableIdKind::Compound;
        case TopAbs_COMPSOLID: return StableIdKind::CompSolid;
        case TopAbs_SOLID: return StableIdKind::Solid;
        case TopAbs_SHELL: return StableIdKind::Shell;
        case TopAbs_FACE: return StableIdKind::Face;
        case TopAbs_WIRE: return StableIdKind::Wire;
        case TopAbs_EDGE: return StableIdKind::Edge;
        case TopAbs_VERTEX: return StableIdKind::Vertex;
        default: return StableIdKind::Invalid;
    }
}

TopologyOrientation topologyOrientation(TopAbs_Orientation orientation) {
    switch (orientation) {
        case TopAbs_FORWARD: return TopologyOrientation::Forward;
        case TopAbs_REVERSED: return TopologyOrientation::Reversed;
        case TopAbs_INTERNAL: return TopologyOrientation::Internal;
        case TopAbs_EXTERNAL: return TopologyOrientation::External;
    }
    return TopologyOrientation::Forward;
}

std::array<double, 16> transform(const TopLoc_Location& location) {
    const gp_Trsf value = location.Transformation();
    std::array<double, 16> matrix{};
    matrix[0] = value.Value(1, 1);
    matrix[1] = value.Value(2, 1);
    matrix[2] = value.Value(3, 1);
    matrix[4] = value.Value(1, 2);
    matrix[5] = value.Value(2, 2);
    matrix[6] = value.Value(3, 2);
    matrix[8] = value.Value(1, 3);
    matrix[9] = value.Value(2, 3);
    matrix[10] = value.Value(3, 3);
    matrix[12] = value.Value(1, 4);
    matrix[13] = value.Value(2, 4);
    matrix[14] = value.Value(3, 4);
    matrix[15] = 1.0;
    return matrix;
}

double tolerance(const TopoDS_Shape& shape) {
    switch (shape.ShapeType()) {
        case TopAbs_FACE: return BRep_Tool::Tolerance(TopoDS::Face(shape));
        case TopAbs_EDGE: return BRep_Tool::Tolerance(TopoDS::Edge(shape));
        case TopAbs_VERTEX: return BRep_Tool::Tolerance(TopoDS::Vertex(shape));
        default: return 0.0;
    }
}

bool validTransform(const std::array<double, 16>& matrix) {
    return std::all_of(matrix.begin(), matrix.end(),
                       [](double value) { return std::isfinite(value); }) &&
        matrix[3] == 0.0 && matrix[7] == 0.0 && matrix[11] == 0.0 &&
        matrix[15] == 1.0;
}

void appendFailure(TopologyAccountValidation& validation,
                   TopologyAccountCheck& check, std::string code) {
    ++check.failed;
    if (std::find(validation.failureCodes.begin(),
                  validation.failureCodes.end(), code) ==
        validation.failureCodes.end()) {
        validation.failureCodes.push_back(std::move(code));
    }
}

struct ExactUseBinding {
    StableId instance;
    std::string definition;
    TopoDS_Shape shape;
    bool consumed = false;
};

struct ExactUseBindingBucket {
    std::vector<std::size_t> indices;
    std::size_t next = 0;
};

class TopologyBuilder {
public:
    explicit TopologyBuilder(const Model& model) : model_(model) {}

    TopologyAccount build() {
        buildAssemblyGraph();
        if (model_.shape.IsNull()) {
            account_.constructionFailureCodes.push_back(
                "topology.authoritative_shape.null");
            return std::move(account_);
        }
        const StableId root = visit(model_.shape, model_.shape, std::nullopt,
                                    std::nullopt, std::nullopt, {}, {});
        if (root.valid()) account_.topologyRoots.push_back(root);
        std::sort(account_.uniqueEntityIds.begin(),
                  account_.uniqueEntityIds.end());
        return std::move(account_);
    }

private:
    void buildAssemblyGraph() {
        const std::vector<AssemblyNode>& nodes = model_.assembly;
        if (nodes.empty()) return;

        std::map<std::string, StableId> assemblyByDefinition;
        for (const AssemblyNode& node : nodes) {
            if (!node.isAssembly || node.sourceDefinition.empty() ||
                assemblyByDefinition.contains(node.sourceDefinition)) {
                continue;
            }
            const StableId assemblyId =
                id(StableIdKind::Assembly,
                   static_cast<std::uint64_t>(account_.assemblies.size()) + 1U);
            assemblyByDefinition.emplace(node.sourceDefinition, assemblyId);
            account_.assemblies.push_back(
                {assemblyId, node.sourceDefinition});
        }

        std::vector<std::optional<StableId>> nodeInstances(nodes.size());
        for (std::size_t index = 0; index < nodes.size(); ++index) {
            if (nodes[index].parent < 0 && nodes[index].isAssembly) continue;
            nodeInstances[index] =
                id(StableIdKind::Instance,
                   static_cast<std::uint64_t>(account_.instances.size()) + 1U);
            account_.instances.push_back({});
        }

        std::size_t instanceIndex = 0;
        for (std::size_t index = 0; index < nodes.size(); ++index) {
            const AssemblyNode& node = nodes[index];
            if (node.parent < 0 && node.isAssembly) {
                const auto found =
                    assemblyByDefinition.find(node.sourceDefinition);
                if (found != assemblyByDefinition.end() &&
                    std::find(account_.assemblyRoots.begin(),
                              account_.assemblyRoots.end(), found->second) ==
                        account_.assemblyRoots.end()) {
                    account_.assemblyRoots.push_back(found->second);
                }
                continue;
            }

            InstanceRecord& instance = account_.instances[instanceIndex++];
            instance.id = *nodeInstances[index];
            if (node.parent < 0) {
                instance.parentId = {StableIdKind::Model, 1};
            } else {
                const std::size_t parentIndex =
                    static_cast<std::size_t>(node.parent);
                if (parentIndex >= nodes.size()) {
                    instance.parentId = {StableIdKind::Model, 1};
                } else if (nodes[parentIndex].parent < 0 &&
                           nodes[parentIndex].isAssembly) {
                    const auto found = assemblyByDefinition.find(
                        nodes[parentIndex].sourceDefinition);
                    instance.parentId = found == assemblyByDefinition.end()
                        ? StableId{StableIdKind::Model, 1}
                        : found->second;
                } else if (nodeInstances[parentIndex]) {
                    instance.parentId = *nodeInstances[parentIndex];
                } else {
                    instance.parentId = {StableIdKind::Model, 1};
                }
            }
            if (node.isAssembly) {
                const auto found =
                    assemblyByDefinition.find(node.sourceDefinition);
                if (found != assemblyByDefinition.end()) {
                    instance.targetAssembly = found->second;
                }
            }
            instance.sourceDefinition = node.sourceDefinition;
            instance.sourceComponent = node.sourceComponent;
            instance.localTransform = node.localTransform;
            instance.worldTransform = node.transform;
            instanceIndices_.emplace(instance.id,
                                     static_cast<std::size_t>(instance.id.ordinal - 1U));

            if (!node.isAssembly) {
                if (!node.exactUse.IsNull()) {
                    const std::size_t bindingIndex =
                        exactUseBindings_.size();
                    exactUseBindings_.push_back(
                        {instance.id, node.sourceDefinition, node.exactUse});
                    exactUseBindingIndices_[node.exactUse].indices.push_back(
                        bindingIndex);
                }
            }
        }

        const std::function<void(std::size_t, std::set<int>&)> collectBodies =
            [&](std::size_t index, std::set<int>& bodies) {
                if (index >= nodes.size()) return;
                const AssemblyNode& node = nodes[index];
                bodies.insert(node.solidIds.begin(), node.solidIds.end());
                for (int child : node.children) {
                    if (child >= 0) {
                        collectBodies(static_cast<std::size_t>(child), bodies);
                    }
                }
            };
        for (std::size_t index = 0; index < nodes.size(); ++index) {
            if (!nodes[index].isAssembly || nodes[index].parent < 0) continue;
            std::set<int> bodies;
            collectBodies(index, bodies);
            bodies.erase(-1);
            if (!bodies.empty()) {
                assemblyContainerBodySets_.insert(std::move(bodies));
            }
        }
    }

    StableId nextId(StableIdKind kind) {
        std::uint64_t& ordinal = nextOrdinals_[kind];
        if (ordinal == std::numeric_limits<std::uint64_t>::max()) {
            throw std::runtime_error("topology stable-ID range exhausted");
        }
        return id(kind, ++ordinal);
    }

    std::string definitionPathKey(
        std::string_view scope, StableIdKind kind,
        const std::vector<std::uint32_t>& path) const {
        if (scope.empty()) return {};
        std::ostringstream stream;
        stream << scope << ':' << static_cast<int>(kind);
        for (std::uint32_t ordinal : path) stream << '/' << ordinal;
        return stream.str();
    }

    StableId underlyingId(const TopoDS_Shape& shape, StableId occurrenceId,
                          const std::string& pathKey) {
        const void* tshape = shape.TShape().get();
        if (!pathKey.empty()) {
            const auto known = definitionPathIds_.find(pathKey);
            if (known != definitionPathIds_.end()) {
                underlyingByTShape_.try_emplace(tshape, known->second);
                return known->second;
            }
        }
        const auto partner = underlyingByTShape_.find(tshape);
        const StableId representative = partner == underlyingByTShape_.end()
            ? occurrenceId
            : partner->second;
        if (partner == underlyingByTShape_.end()) {
            underlyingByTShape_.emplace(tshape, representative);
            account_.uniqueEntityIds.push_back(representative);
        }
        if (!pathKey.empty()) {
            definitionPathIds_.emplace(pathKey, representative);
        }
        return representative;
    }

    StableId visit(const TopoDS_Shape& worldShape,
                   const TopoDS_Shape& relativeShape,
                   std::optional<StableId> parentId,
                   std::optional<StableId> inheritedInstance,
                   std::optional<StableId> containingFace,
                   std::string definitionScope,
                   std::vector<std::uint32_t> definitionPath) {
        const StableIdKind kind = stableKind(worldShape.ShapeType());
        if (kind == StableIdKind::Invalid) {
            throw std::runtime_error(
                "authoritative topology contains an unsupported direct shape kind");
        }

        std::optional<StableId> instanceId = inheritedInstance;
        ExactUseBinding* exactBinding = nullptr;
        const auto bindingBucket = exactUseBindingIndices_.find(worldShape);
        if (bindingBucket != exactUseBindingIndices_.end()) {
            ExactUseBindingBucket& bucket = bindingBucket->second;
            while (bucket.next < bucket.indices.size() &&
                   exactUseBindings_[bucket.indices[bucket.next]].consumed) {
                ++bucket.next;
            }
            if (bucket.next < bucket.indices.size()) {
                exactBinding =
                    &exactUseBindings_[bucket.indices[bucket.next++]];
            }
        }
        if (exactBinding != nullptr) {
            exactBinding->consumed = true;
            instanceId = exactBinding->instance;
            definitionScope = exactBinding->definition;
            definitionPath.clear();
        }

        const StableId occurrenceId = nextId(kind);
        const StableId representative = underlyingId(
            worldShape, occurrenceId,
            definitionPathKey(definitionScope, kind, definitionPath));
        const double shapeTolerance = tolerance(worldShape);
        if (!std::isfinite(shapeTolerance) || shapeTolerance < 0.0) {
            throw std::runtime_error(
                "authoritative topology contains an invalid tolerance");
        }

        TopologyOccurrence occurrence;
        occurrence.id = occurrenceId;
        occurrence.underlyingId = representative;
        occurrence.parentId = parentId;
        occurrence.orientation =
            topologyOrientation(relativeShape.Orientation());
        occurrence.tolerance = shapeTolerance;
        occurrence.hasExactRepresentation = !worldShape.IsNull();
        occurrence.instanceId = instanceId;
        TopLoc_Location localLocation = worldShape.Location();
        if (parentId) {
            localLocation = account_.exactShapes.at(*parentId)
                                .Location()
                                .Inverted() *
                worldShape.Location();
        }
        occurrence.localTransform = transform(localLocation);
        occurrence.worldTransform = transform(worldShape.Location());
        const std::size_t occurrenceIndex = account_.occurrences.size();
        account_.occurrences.push_back(std::move(occurrence));
        account_.exactShapes.emplace(occurrenceId, worldShape);

        std::optional<StableId> parentInstance;
        if (parentId) {
            const auto parent = occurrenceIndices_.find(*parentId);
            if (parent != occurrenceIndices_.end()) {
                parentInstance = account_.occurrences[parent->second].instanceId;
            }
        }
        occurrenceIndices_.emplace(occurrenceId, occurrenceIndex);
        if (instanceId && parentInstance != instanceId) {
            const auto instance = instanceIndices_.find(*instanceId);
            if (instance != instanceIndices_.end()) {
                std::vector<StableId>& roots =
                    account_.instances[instance->second].topologyRoots;
                if (std::find(roots.begin(), roots.end(), occurrenceId) ==
                    roots.end()) {
                    roots.push_back(occurrenceId);
                }
            }
        }

        TopoDS_Iterator worldIterator(worldShape, true, true);
        TopoDS_Iterator relativeIterator(relativeShape, false, false);
        std::uint32_t childOrdinal = 0;
        while (worldIterator.More() && relativeIterator.More()) {
            const TopoDS_Shape childWorld = worldIterator.Value();
            const TopoDS_Shape childRelative = relativeIterator.Value();
            if (kind == StableIdKind::Compound &&
                childWorld.ShapeType() == TopAbs_COMPOUND &&
                isAssemblyContainer(childWorld)) {
                appendFlattenedAssemblyChildren(
                    childWorld, childRelative, occurrenceIndex, occurrenceId,
                    instanceId, containingFace, definitionScope,
                    definitionPath, childOrdinal);
            } else {
                appendChild(childWorld, childRelative, occurrenceIndex,
                            occurrenceId, kind, instanceId, containingFace,
                            definitionScope, definitionPath, childOrdinal);
            }
            worldIterator.Next();
            relativeIterator.Next();
        }
        if (worldIterator.More() || relativeIterator.More()) {
            throw std::runtime_error(
                "cumulative and relative topology traversals disagree");
        }
        return occurrenceId;
    }

    bool isAssemblyContainer(const TopoDS_Shape& shape) const {
        std::set<int> bodies;
        for (TopExp_Explorer explorer(shape, TopAbs_SOLID); explorer.More();
             explorer.Next()) {
            const int solidId = model_.solids.FindIndex(explorer.Current());
            if (solidId > 0) bodies.insert(solidId);
        }
        for (TopExp_Explorer explorer(shape, TopAbs_SHELL, TopAbs_SOLID);
             explorer.More(); explorer.Next()) {
            const int solidId = model_.solids.FindIndex(explorer.Current());
            if (solidId > 0) bodies.insert(solidId);
        }
        return assemblyContainerBodySets_.contains(bodies);
    }

    void incrementChildOrdinal(std::uint32_t& ordinal) const {
        if (ordinal == std::numeric_limits<std::uint32_t>::max()) {
            throw std::runtime_error("topology child ordinal range exhausted");
        }
        ++ordinal;
    }

    void appendChild(const TopoDS_Shape& childWorld,
                     const TopoDS_Shape& childRelative,
                     std::size_t parentOccurrenceIndex, StableId parentId,
                     StableIdKind parentKind,
                     std::optional<StableId> instanceId,
                     std::optional<StableId> containingFace,
                     const std::string& definitionScope,
                     const std::vector<std::uint32_t>& definitionPath,
                     std::uint32_t& childOrdinal) {
        std::vector<std::uint32_t> childPath = definitionPath;
        childPath.push_back(childOrdinal);
        const std::optional<StableId> childFace =
            parentKind == StableIdKind::Face
            ? std::optional<StableId>{parentId}
            : containingFace;
        const StableId childId = visit(
            childWorld, childRelative, parentId, instanceId, childFace,
            definitionScope, std::move(childPath));
        account_.occurrences[parentOccurrenceIndex].childIds.push_back(childId);
        if (parentKind == StableIdKind::Wire) {
            if (childId.kind != StableIdKind::Edge) {
                throw std::runtime_error(
                    "wire contains a non-edge direct child");
            }
            createCoedge(childId, parentId, containingFace, instanceId,
                         childOrdinal, childRelative.Orientation());
        }
        incrementChildOrdinal(childOrdinal);
    }

    void appendFlattenedAssemblyChildren(
        const TopoDS_Shape& worldContainer,
        const TopoDS_Shape& relativeContainer,
        std::size_t parentOccurrenceIndex, StableId parentId,
        std::optional<StableId> instanceId,
        std::optional<StableId> containingFace,
        const std::string& definitionScope,
        const std::vector<std::uint32_t>& definitionPath,
        std::uint32_t& childOrdinal) {
        TopoDS_Iterator worldIterator(worldContainer, true, true);
        TopoDS_Iterator relativeIterator(relativeContainer, false, false);
        while (worldIterator.More() && relativeIterator.More()) {
            const TopoDS_Shape childWorld = worldIterator.Value();
            const TopoDS_Shape childRelative = relativeIterator.Value();
            if (childWorld.ShapeType() == TopAbs_COMPOUND &&
                isAssemblyContainer(childWorld)) {
                appendFlattenedAssemblyChildren(
                    childWorld, childRelative, parentOccurrenceIndex, parentId,
                    instanceId, containingFace, definitionScope,
                    definitionPath, childOrdinal);
            } else {
                appendChild(childWorld, childRelative, parentOccurrenceIndex,
                            parentId, StableIdKind::Compound, instanceId,
                            containingFace, definitionScope, definitionPath,
                            childOrdinal);
            }
            worldIterator.Next();
            relativeIterator.Next();
        }
        if (worldIterator.More() || relativeIterator.More()) {
            throw std::runtime_error(
                "assembly container traversals disagree");
        }
    }

    void createCoedge(StableId edgeId, StableId wireId,
                      std::optional<StableId> faceId,
                      std::optional<StableId> instanceId,
                      std::uint32_t ordinal,
                      TopAbs_Orientation orientation) {
        if (nextCoedgeOrdinal_ ==
            std::numeric_limits<std::uint64_t>::max()) {
            throw std::runtime_error("coedge stable-ID range exhausted");
        }
        TopologyCoedgeRecord record;
        record.id = id(StableIdKind::Coedge, ++nextCoedgeOrdinal_);
        record.edgeId = edgeId;
        record.wireId = wireId;
        record.faceId = faceId;
        record.instanceId = instanceId;
        record.ordinalInWire = ordinal;
        record.orientation = topologyOrientation(orientation);

        const TopoDS_Edge edge = TopoDS::Edge(account_.exactShapes.at(edgeId));
        if (faceId) {
            const TopoDS_Face face =
                TopoDS::Face(account_.exactShapes.at(*faceId));
            const bool seam = BRep_Tool::IsClosed(edge, face);
            const auto appendPcurve = [&](TopoDS_Edge oriented,
                                          std::uint32_t index) {
                double first = 0.0;
                double last = 0.0;
                bool stored = false;
                const Handle(Geom2d_Curve) curve = BRep_Tool::CurveOnSurface(
                    oriented, face, first, last, &stored);
                if (curve.IsNull() || !stored) return false;
                record.pcurveRepresentations.push_back(
                    {record.id, *faceId, index});
                return true;
            };
            const bool firstStored = appendPcurve(edge, 0);
            bool secondStored = true;
            if (seam) {
                TopoDS_Edge reversed = edge;
                reversed.Reverse();
                secondStored = appendPcurve(reversed, 1);
            }
            if (!firstStored || !secondStored) {
                record.conditionCodes.push_back(
                    seam ? "pcurve.incomplete_seam" : "pcurve.missing");
            }
        }
        if (!BRep_Tool::SameParameter(edge)) {
            record.conditionCodes.push_back("pcurve.not_same_parameter");
        }
        if (!BRep_Tool::SameRange(edge)) {
            record.conditionCodes.push_back("pcurve.not_same_range");
        }
        account_.coedges.push_back(std::move(record));
    }

    const Model& model_;
    TopologyAccount account_;
    std::map<StableIdKind, std::uint64_t> nextOrdinals_;
    std::uint64_t nextCoedgeOrdinal_ = 0;
    std::vector<ExactUseBinding> exactUseBindings_;
    std::unordered_map<TopoDS_Shape, ExactUseBindingBucket,
                       TopTools_ShapeMapHasher, TopTools_ShapeMapHasher>
        exactUseBindingIndices_;
    std::set<std::set<int>> assemblyContainerBodySets_;
    std::map<StableId, std::size_t> instanceIndices_;
    std::map<StableId, std::size_t> occurrenceIndices_;
    std::unordered_map<const void*, StableId> underlyingByTShape_;
    std::unordered_map<std::string, StableId> definitionPathIds_;
};

}  // namespace

bool TopologyAccountValidation::complete() const noexcept {
    return failureCodes.empty() &&
        std::all_of(checks.begin(), checks.end(),
                    [](const TopologyAccountCheck& check) {
                        return check.complete();
                    });
}

const TopologyAccountCheck* TopologyAccountValidation::find(
    std::string_view code) const noexcept {
    const auto found = std::find_if(
        checks.begin(), checks.end(),
        [code](const TopologyAccountCheck& check) {
            return check.code == code;
        });
    return found == checks.end() ? nullptr : &*found;
}

TopologyAccountValidation validateTopologyAccount(
    const TopologyAccount& account) {
    TopologyAccountValidation validation;
    validation.checks = {
        {"topology.assemblies",
         account.assemblies.size() + account.assemblyRoots.size()},
        {"topology.instances", account.instances.size()},
        {"topology.occurrences",
         account.occurrences.size() + account.topologyRoots.size() +
             account.uniqueEntityIds.size()},
        {"topology.coedges", account.coedges.size()},
    };
    TopologyAccountCheck& assemblyCheck = validation.checks[0];
    TopologyAccountCheck& instanceCheck = validation.checks[1];
    TopologyAccountCheck& occurrenceCheck = validation.checks[2];
    TopologyAccountCheck& coedgeCheck = validation.checks[3];

    for (const std::string& code : account.constructionFailureCodes) {
        appendFailure(validation, occurrenceCheck, code);
    }

    std::map<StableId, const AssemblyRecord*> assemblyRecords;
    for (const AssemblyRecord& assembly : account.assemblies) {
        assemblyRecords.emplace(assembly.id, &assembly);
    }
    std::map<StableId, const InstanceRecord*> instanceRecords;
    for (const InstanceRecord& instance : account.instances) {
        instanceRecords.emplace(instance.id, &instance);
    }
    std::map<StableId, const TopologyOccurrence*> occurrences;
    for (const TopologyOccurrence& occurrence : account.occurrences) {
        occurrences.emplace(occurrence.id, &occurrence);
    }
    const auto assemblyRecord = [&](StableId subject) {
        const auto found = assemblyRecords.find(subject);
        return found == assemblyRecords.end() ? nullptr : found->second;
    };
    const auto instanceRecord = [&](StableId subject) {
        const auto found = instanceRecords.find(subject);
        return found == instanceRecords.end() ? nullptr : found->second;
    };
    const auto occurrenceRecord = [&](StableId subject) {
        const auto found = occurrences.find(subject);
        return found == occurrences.end() ? nullptr : found->second;
    };

    const std::set<StableId> declaredAssemblyRoots(
        account.assemblyRoots.begin(), account.assemblyRoots.end());
    std::set<StableId> targetedAssemblies;
    for (const InstanceRecord& instance : account.instances) {
        if (instance.targetAssembly) {
            targetedAssemblies.insert(*instance.targetAssembly);
        }
    }
    std::set<StableId> assemblyIds;
    for (const AssemblyRecord& assembly : account.assemblies) {
        ++assemblyCheck.checked;
        if (!assembly.id.valid() ||
            assembly.id.kind != StableIdKind::Assembly ||
            assembly.sourceLabel.empty() ||
            !assemblyIds.insert(assembly.id).second) {
            appendFailure(validation, assemblyCheck,
                          "topology.assembly.record_invalid");
        }
        if (!declaredAssemblyRoots.contains(assembly.id) &&
            !targetedAssemblies.contains(assembly.id)) {
            appendFailure(validation, assemblyCheck,
                          "topology.assembly.unreachable");
        }
    }
    std::set<StableId> checkedAssemblyRoots;
    for (StableId root : account.assemblyRoots) {
        ++assemblyCheck.checked;
        if (!root.valid() || root.kind != StableIdKind::Assembly ||
            assemblyRecord(root) == nullptr ||
            !checkedAssemblyRoots.insert(root).second) {
            appendFailure(validation, assemblyCheck,
                          "topology.assembly.root_missing");
        }
    }

    std::set<StableId> instanceIds;
    for (const InstanceRecord& instance : account.instances) {
        ++instanceCheck.checked;
        const auto fail = [&](std::string code) {
            appendFailure(validation, instanceCheck, std::move(code));
        };
        if (!instance.id.valid() ||
            instance.id.kind != StableIdKind::Instance) {
            fail("topology.instance.id_invalid");
        } else if (!instanceIds.insert(instance.id).second) {
            fail("topology.instance.id_duplicate");
        }
        if (instance.sourceDefinition.empty()) {
            fail("topology.instance.definition_missing");
        }
        if (instance.sourceComponent.empty() &&
            instance.parentId != StableId{StableIdKind::Model, 1}) {
            fail("topology.instance.component_missing");
        }
        if (!validTransform(instance.localTransform)) {
            fail("topology.instance.local_transform_invalid");
        }
        if (!validTransform(instance.worldTransform)) {
            fail("topology.instance.world_transform_invalid");
        }
        if (instance.parentId.kind == StableIdKind::Assembly) {
            if (assemblyRecord(instance.parentId) == nullptr) {
                fail("topology.instance.parent_assembly_missing");
            }
        } else if (instance.parentId.kind == StableIdKind::Instance) {
            const InstanceRecord* parent =
                instanceRecord(instance.parentId);
            if (parent == nullptr) {
                fail("topology.instance.parent_instance_missing");
            } else if (!parent->targetAssembly) {
                fail("topology.instance.parent_instance_not_assembly");
            }
        } else if (instance.parentId !=
                   StableId{StableIdKind::Model, 1}) {
            fail("topology.instance.parent_invalid");
        }
        if (instance.targetAssembly) {
            if (!instance.topologyRoots.empty()) {
                fail("topology.instance.assembly_has_topology_roots");
            }
            if (assemblyRecord(*instance.targetAssembly) == nullptr) {
                fail("topology.instance.target_assembly_missing");
            }
        } else if (instance.topologyRoots.empty()) {
            fail("topology.instance.topology_roots_missing");
        }
        for (StableId root : instance.topologyRoots) {
            const TopologyOccurrence* occurrence =
                occurrenceRecord(root);
            if (occurrence == nullptr) {
                fail("topology.instance.topology_root_missing");
            } else if (occurrence->instanceId != instance.id) {
                fail("topology.instance.topology_root_owner_mismatch");
            }
        }
    }

    for (const InstanceRecord& instance : account.instances) {
        std::set<StableId> path;
        const InstanceRecord* current = &instance;
        while (current != nullptr &&
               current->parentId.kind == StableIdKind::Instance) {
            if (!path.insert(current->id).second) {
                appendFailure(validation, instanceCheck,
                              "topology.instance.parent_cycle");
                break;
            }
            current = instanceRecord(current->parentId);
        }
        if (current != nullptr &&
            current->parentId.kind == StableIdKind::Assembly &&
            !declaredAssemblyRoots.contains(current->parentId)) {
            appendFailure(validation, instanceCheck,
                          "topology.instance.orphaned");
        } else if (current != nullptr &&
                   current->parentId.kind != StableIdKind::Assembly &&
                   current->parentId !=
                       StableId{StableIdKind::Model, 1}) {
            appendFailure(validation, instanceCheck,
                          "topology.instance.orphaned");
        }
    }

    if (occurrences.size() != account.occurrences.size()) {
        appendFailure(validation, occurrenceCheck,
                      "topology.occurrence.id_duplicate");
    }
    const std::set<StableId> declaredTopologyRoots(
        account.topologyRoots.begin(), account.topologyRoots.end());
    for (const TopologyOccurrence& occurrence : account.occurrences) {
        ++occurrenceCheck.checked;
        const auto exactShape = account.exactShapes.find(occurrence.id);
        bool valid = occurrence.id.valid() &&
            occurrence.underlyingId.valid() &&
            occurrence.id.kind == occurrence.underlyingId.kind &&
            occurrences.contains(occurrence.id) &&
            exactShape != account.exactShapes.end() &&
            occurrence.hasExactRepresentation &&
            validTransform(occurrence.localTransform) &&
            validTransform(occurrence.worldTransform);
        if (exactShape == account.exactShapes.end() ||
            exactShape->second.IsNull()) {
            appendFailure(validation, occurrenceCheck,
                          "topology.occurrence.shape_missing");
            valid = false;
        } else {
            const TopoDS_Shape& shape = exactShape->second;
            if (stableKind(shape.ShapeType()) != occurrence.id.kind) {
                appendFailure(validation, occurrenceCheck,
                              "topology.occurrence.shape_kind_mismatch");
                valid = false;
            }
            if (transform(shape.Location()) != occurrence.worldTransform) {
                appendFailure(validation, occurrenceCheck,
                              "topology.occurrence.world_transform_mismatch");
                valid = false;
            }
            if (tolerance(shape) != occurrence.tolerance) {
                appendFailure(validation, occurrenceCheck,
                              "topology.occurrence.tolerance_mismatch");
                valid = false;
            }
        }
        const auto underlying = occurrences.find(occurrence.underlyingId);
        valid = valid && underlying != occurrences.end() &&
            underlying->second->id == underlying->second->underlyingId;
        if (occurrence.instanceId) {
            valid = valid && instanceRecord(*occurrence.instanceId) != nullptr;
        }
        if (occurrence.parentId) {
            const auto parent = occurrences.find(*occurrence.parentId);
            valid = valid && parent != occurrences.end() &&
                std::find(parent->second->childIds.begin(),
                          parent->second->childIds.end(), occurrence.id) !=
                    parent->second->childIds.end();
            if (parent != occurrences.end() && parent->second->instanceId &&
                parent->second->instanceId != occurrence.instanceId) {
                appendFailure(validation, occurrenceCheck,
                              "topology.occurrence.instance_not_inherited");
                valid = false;
            }
            if (exactShape != account.exactShapes.end() &&
                !exactShape->second.IsNull()) {
                const auto parentShape =
                    account.exactShapes.find(*occurrence.parentId);
                if (parentShape != account.exactShapes.end() &&
                    !parentShape->second.IsNull()) {
                    const TopLoc_Location localLocation =
                        parentShape->second.Location().Inverted() *
                        exactShape->second.Location();
                    if (transform(localLocation) !=
                        occurrence.localTransform) {
                        appendFailure(
                            validation, occurrenceCheck,
                            "topology.occurrence.local_transform_mismatch");
                        valid = false;
                    }
                }
            }
        } else {
            if (!declaredTopologyRoots.contains(occurrence.id)) {
                appendFailure(validation, occurrenceCheck,
                              "topology.occurrence.root_undeclared");
                valid = false;
            }
            if (exactShape != account.exactShapes.end() &&
                !exactShape->second.IsNull() &&
                transform(exactShape->second.Location()) !=
                    occurrence.localTransform) {
                appendFailure(validation, occurrenceCheck,
                              "topology.occurrence.local_transform_mismatch");
                valid = false;
            }
        }
        std::set<StableId> childIds;
        for (StableId child : occurrence.childIds) {
            const auto found = occurrences.find(child);
            const bool childValid = found != occurrences.end() &&
                found->second->parentId == occurrence.id &&
                childIds.insert(child).second;
            valid = valid && childValid;
        }
        if (!valid) {
            appendFailure(validation, occurrenceCheck,
                          "topology.occurrence.record_invalid");
        }
    }

    if (account.occurrences.empty()) {
        appendFailure(validation, occurrenceCheck,
                      "topology.occurrence_account.empty");
    }
    std::set<StableId> checkedTopologyRoots;
    for (StableId root : account.topologyRoots) {
        ++occurrenceCheck.checked;
        const TopologyOccurrence* occurrence = occurrenceRecord(root);
        if (occurrence == nullptr || occurrence->parentId.has_value() ||
            !checkedTopologyRoots.insert(root).second) {
            appendFailure(validation, occurrenceCheck,
                          "topology.occurrence.root_invalid");
        }
    }

    std::vector<StableId> canonical;
    for (const TopologyOccurrence& occurrence : account.occurrences) {
        if (occurrence.id == occurrence.underlyingId) {
            canonical.push_back(occurrence.id);
        }
    }
    std::sort(canonical.begin(), canonical.end());
    std::vector<StableId> declared = account.uniqueEntityIds;
    std::sort(declared.begin(), declared.end());
    for (StableId unique : account.uniqueEntityIds) {
        ++occurrenceCheck.checked;
        const TopologyOccurrence* occurrence = occurrenceRecord(unique);
        if (occurrence == nullptr || occurrence->id != occurrence->underlyingId) {
            appendFailure(validation, occurrenceCheck,
                          "topology.occurrence.underlying_invalid");
        }
    }
    if (canonical != declared ||
        account.exactShapes.size() != account.occurrences.size()) {
        appendFailure(validation, occurrenceCheck,
                      "topology.occurrence.unique_set_mismatch");
    }

    std::set<StableId> coedgeIds;
    std::map<StableId, std::vector<std::uint32_t>> wireOrdinals;
    for (const TopologyCoedgeRecord& coedge : account.coedges) {
        ++coedgeCheck.checked;
        const bool uniqueId = coedgeIds.insert(coedge.id).second;
        bool valid = coedge.id.valid() &&
            coedge.id.kind == StableIdKind::Coedge &&
            uniqueId &&
            coedge.edgeId.kind == StableIdKind::Edge &&
            coedge.wireId.kind == StableIdKind::Wire;
        const TopologyOccurrence* edge =
            occurrenceRecord(coedge.edgeId);
        const TopologyOccurrence* wire =
            occurrenceRecord(coedge.wireId);
        valid = valid && edge != nullptr && wire != nullptr &&
            std::find(wire->childIds.begin(), wire->childIds.end(),
                      coedge.edgeId) != wire->childIds.end() &&
            edge->instanceId == coedge.instanceId &&
            wire->instanceId == coedge.instanceId;
        if (coedge.faceId) {
            const TopologyOccurrence* face =
                occurrenceRecord(*coedge.faceId);
            valid = valid && face != nullptr &&
                face->id.kind == StableIdKind::Face &&
                face->instanceId == coedge.instanceId &&
                wire != nullptr && wire->parentId == face->id;
        }
        for (const PcurveRef& representation :
             coedge.pcurveRepresentations) {
            valid = valid && representation.coedgeId == coedge.id &&
                coedge.faceId == representation.faceId;
        }
        wireOrdinals[coedge.wireId].push_back(coedge.ordinalInWire);
        if (!valid) {
            appendFailure(validation, coedgeCheck,
                          "topology.coedge.record_invalid");
        }
    }
    for (const TopologyOccurrence& wireOccurrence : account.occurrences) {
        if (wireOccurrence.id.kind != StableIdKind::Wire) continue;
        std::vector<std::uint32_t>& ordinals =
            wireOrdinals[wireOccurrence.id];
        std::sort(ordinals.begin(), ordinals.end());
        bool contiguous = true;
        for (std::size_t index = 0; index < ordinals.size(); ++index) {
            contiguous = contiguous &&
                ordinals[index] == static_cast<std::uint32_t>(index);
        }
        const std::size_t edgeChildren =
            static_cast<std::size_t>(std::count_if(
                wireOccurrence.childIds.begin(),
                wireOccurrence.childIds.end(), [](StableId child) {
                    return child.kind == StableIdKind::Edge;
                }));
        if (!contiguous || edgeChildren != ordinals.size()) {
            appendFailure(validation, coedgeCheck,
                          "topology.coedge.wire_coverage_invalid");
        }
    }

    std::sort(validation.failureCodes.begin(),
              validation.failureCodes.end());
    return validation;
}

namespace secure_detail {

TopologyAccount buildTopologyAccount(const Model& model) {
    try {
        return TopologyBuilder(model).build();
    } catch (const Standard_Failure&) {
        TopologyAccount account;
        account.constructionFailureCodes.push_back(
            "topology.occt_traversal_failure");
        return account;
    } catch (const std::exception&) {
        TopologyAccount account;
        account.constructionFailureCodes.push_back(
            "topology.traversal_failure");
        return account;
    }
}

}  // namespace secure_detail
}  // namespace weft
