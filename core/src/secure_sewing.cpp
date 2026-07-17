#include "secure_core_internal.hpp"

#include <BRep_Tool.hxx>
#include <Standard_Failure.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Pnt.hxx>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iomanip>
#include <limits>
#include <locale>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace weft::secure_detail {
namespace {

struct WireVertexUse {
    TopoDS_Vertex vertex;
    int mapIndex = 0;
    gp_Pnt position;
    double tolerance = 0.0;
};

struct WireEdgeUse {
    TopoDS_Edge edge;
    int mapIndex = 0;
    std::vector<std::size_t> vertexUses;  // indices into the wire vertex list
};

std::string coincidenceDetail(const char* subject, double distance) {
    std::ostringstream detail;
    detail.imbue(std::locale::classic());
    detail << std::setprecision(std::numeric_limits<double>::max_digits10)
           << "distinct " << subject
           << " definitions coincide within their stored tolerances "
              "(measured distance="
           << distance
           << "); merging them needs a one-to-one provenance proof that the "
              "conservative profile does not have";
    return detail.str();
}

}  // namespace

// Reconnaissance for unproven sewing candidates. Duplicate coincident
// boundary subjects inside ONE wire are exactly what sewing would merge;
// merging changes topology cardinality and needs 1:1 Merged correspondence
// proof, so the conservative profile refuses them by name instead.
// Coincident subjects across different wires (touching bodies, shared-face
// pairs, unsewn translated face soups) are legitimate representations and
// are never candidates.
void detectUnprovenSewing(const Model& source,
                          ConservativeWorkingDerivation& derivation) {
    ShapeMap wires;
    ShapeMap vertices;
    try {
        TopExp::MapShapes(source.shape, TopAbs_WIRE, wires);
        TopExp::MapShapes(source.shape, TopAbs_VERTEX, vertices);
    } catch (const Standard_Failure&) {
        return;
    }

    for (int wireIndex = 1; wireIndex <= wires.Extent(); ++wireIndex) {
        const StableId wireId{StableIdKind::Wire,
                              static_cast<std::uint64_t>(wireIndex)};
        std::vector<WireVertexUse> wireVertices;
        std::vector<WireEdgeUse> wireEdges;
        try {
            for (TopoDS_Iterator child(wires(wireIndex), false, false);
                 child.More(); child.Next()) {
                if (child.Value().ShapeType() != TopAbs_EDGE) continue;
                const TopoDS_Edge edge = TopoDS::Edge(child.Value());
                if (std::any_of(wireEdges.begin(), wireEdges.end(),
                                [&](const WireEdgeUse& use) {
                                    return use.edge.IsSame(edge);
                                })) {
                    continue;  // a seam/slit reuses one definition: not a
                               // duplicate
                }
                WireEdgeUse use;
                use.edge = edge;
                use.mapIndex = source.edges.FindIndex(edge);
                TopoDS_Vertex first;
                TopoDS_Vertex last;
                TopExp::Vertices(edge, first, last);
                for (const TopoDS_Vertex& vertex : {first, last}) {
                    if (vertex.IsNull()) continue;
                    std::size_t vertexUse = wireVertices.size();
                    for (std::size_t known = 0; known < wireVertices.size();
                         ++known) {
                        if (wireVertices[known].vertex.IsSame(vertex)) {
                            vertexUse = known;
                            break;
                        }
                    }
                    if (vertexUse == wireVertices.size()) {
                        wireVertices.push_back(
                            {vertex, vertices.FindIndex(vertex),
                             BRep_Tool::Pnt(vertex),
                             BRep_Tool::Tolerance(vertex)});
                    }
                    use.vertexUses.push_back(vertexUse);
                }
                wireEdges.push_back(std::move(use));
            }
        } catch (const Standard_Failure&) {
            continue;
        }

        // Distinct vertex definitions coinciding inside one wire.
        for (std::size_t first = 0; first < wireVertices.size(); ++first) {
            for (std::size_t second = first + 1;
                 second < wireVertices.size(); ++second) {
                const WireVertexUse& a = wireVertices[first];
                const WireVertexUse& b = wireVertices[second];
                const double distance = a.position.Distance(b.position);
                if (!std::isfinite(distance) ||
                    distance > a.tolerance + b.tolerance) {
                    continue;
                }
                derivation.refusals.push_back(
                    {"repair.sewing.duplicate_vertex_unproven",
                     {wireId,
                      {StableIdKind::Vertex,
                       static_cast<std::uint64_t>(a.mapIndex)},
                      {StableIdKind::Vertex,
                       static_cast<std::uint64_t>(b.mapIndex)}},
                     coincidenceDetail("vertex", distance)});
            }
        }

        // Distinct edge definitions whose endpoints coincide pairwise with
        // at least one duplicated (distinct-definition) endpoint. A slit
        // that reuses one edge definition or shares its vertex definitions
        // is a legitimate representation and never matches.
        for (std::size_t first = 0; first < wireEdges.size(); ++first) {
            for (std::size_t second = first + 1; second < wireEdges.size();
                 ++second) {
                const WireEdgeUse& a = wireEdges[first];
                const WireEdgeUse& b = wireEdges[second];
                if (a.vertexUses.size() != 2 || b.vertexUses.size() != 2) {
                    continue;
                }
                const auto endpointPairMatch =
                    [&](std::size_t aUse,
                        std::size_t bUse) -> std::optional<double> {
                    const WireVertexUse& av = wireVertices[aUse];
                    const WireVertexUse& bv = wireVertices[bUse];
                    if (aUse == bUse) return 0.0;  // shared definition
                    const double distance =
                        av.position.Distance(bv.position);
                    if (!std::isfinite(distance) ||
                        distance > av.tolerance + bv.tolerance) {
                        return std::nullopt;
                    }
                    return distance;
                };
                const auto pairing =
                    [&](std::size_t bFirst, std::size_t bSecond)
                    -> std::optional<double> {
                    const auto start = endpointPairMatch(
                        a.vertexUses[0], b.vertexUses[bFirst]);
                    const auto finish = endpointPairMatch(
                        a.vertexUses[1], b.vertexUses[bSecond]);
                    if (!start || !finish) return std::nullopt;
                    const bool duplicatedEndpoint =
                        a.vertexUses[0] != b.vertexUses[bFirst] ||
                        a.vertexUses[1] != b.vertexUses[bSecond];
                    if (!duplicatedEndpoint) return std::nullopt;
                    return std::max(*start, *finish);
                };
                std::optional<double> matched = pairing(0, 1);
                if (!matched) matched = pairing(1, 0);
                if (!matched) continue;
                derivation.refusals.push_back(
                    {"repair.sewing.duplicate_edge_unproven",
                     {wireId,
                      {StableIdKind::Edge,
                       static_cast<std::uint64_t>(a.mapIndex)},
                      {StableIdKind::Edge,
                       static_cast<std::uint64_t>(b.mapIndex)}},
                     coincidenceDetail("edge boundary", *matched)});
            }
        }
    }
}

}  // namespace weft::secure_detail
