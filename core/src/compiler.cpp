#include "weft/compiler.hpp"

#include "mesher_sampling.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepTools.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <BRep_Tool.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <GCPnts_UniformAbscissa.hxx>
#include <Geom2d_Curve.hxx>
#include <GeomAbs_CurveType.hxx>
#include <Standard_Failure.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Ax1.hxx>
#include <gp_Circ.hxx>
#include <gp_Cone.hxx>
#include <gp_Cylinder.hxx>
#include <gp_Pln.hxx>
#include <gp_Sphere.hxx>
#include <gp_Torus.hxx>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <limits>
#include <map>
#include <numeric>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <utility>

namespace weft {
namespace {

using mesher_detail::stableDeflectionCount;

constexpr double kPi = 3.1415926535897932384626433832795;

std::array<double, 3> pointArray(const gp_Pnt& p) {
    return {p.X(), p.Y(), p.Z()};
}

std::array<double, 3> dirArray(const gp_Dir& d) {
    return {d.X(), d.Y(), d.Z()};
}

CurveType classifyCurve(const BRepAdaptor_Curve& curve) {
    switch (curve.GetType()) {
        case GeomAbs_Line: return CurveType::Line;
        case GeomAbs_Circle: return CurveType::Circle;
        case GeomAbs_Ellipse: return CurveType::Ellipse;
        case GeomAbs_Hyperbola: return CurveType::Hyperbola;
        case GeomAbs_Parabola: return CurveType::Parabola;
        case GeomAbs_BezierCurve: return CurveType::Bezier;
        case GeomAbs_BSplineCurve: return CurveType::BSpline;
        case GeomAbs_OffsetCurve: return CurveType::Offset;
        default: return CurveType::Other;
    }
}

SurfaceDescriptor describeSurface(const TopoDS_Face& face,
                                  const FaceInfo& info) {
    SurfaceDescriptor out;
    out.type = info.type;
    out.radius = info.radius;
    try {
        BRepAdaptor_Surface surface(face, false);
        out.uPeriodic = surface.IsUPeriodic();
        out.vPeriodic = surface.IsVPeriodic();
        switch (surface.GetType()) {
            case GeomAbs_Plane: {
                const gp_Pln p = surface.Plane();
                out.origin = pointArray(p.Location());
                out.axis = dirArray(p.Axis().Direction());
                break;
            }
            case GeomAbs_Cylinder: {
                const gp_Cylinder c = surface.Cylinder();
                out.origin = pointArray(c.Location());
                out.axis = dirArray(c.Axis().Direction());
                out.radius = c.Radius();
                break;
            }
            case GeomAbs_Cone: {
                const gp_Cone c = surface.Cone();
                out.origin = pointArray(c.Apex());
                out.axis = dirArray(c.Axis().Direction());
                out.radius = c.RefRadius();
                out.semiAngle = c.SemiAngle();
                break;
            }
            case GeomAbs_Sphere: {
                const gp_Sphere s = surface.Sphere();
                out.origin = pointArray(s.Location());
                out.axis = dirArray(s.Position().Direction());
                out.radius = s.Radius();
                break;
            }
            case GeomAbs_Torus: {
                const gp_Torus t = surface.Torus();
                out.origin = pointArray(t.Location());
                out.axis = dirArray(t.Axis().Direction());
                out.radius = t.MajorRadius();
                out.secondaryRadius = t.MinorRadius();
                break;
            }
            default: break;
        }
    } catch (const Standard_Failure&) {
        // A descriptor is advisory.  The exact face remains in Model and is
        // still available to the fallback backend.
    }
    return out;
}

double dot(const std::array<double, 3>& a,
           const std::array<double, 3>& b) {
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

std::array<double, 3> sub(const std::array<double, 3>& a,
                          const std::array<double, 3>& b) {
    return {a[0] - b[0], a[1] - b[1], a[2] - b[2]};
}

std::array<double, 3> cross(const std::array<double, 3>& a,
                            const std::array<double, 3>& b) {
    return {a[1] * b[2] - a[2] * b[1],
            a[2] * b[0] - a[0] * b[2],
            a[0] * b[1] - a[1] * b[0]};
}

double norm(const std::array<double, 3>& a) {
    return std::sqrt(dot(a, a));
}

bool sameAxisLine(const SurfaceDescriptor& a, const SurfaceDescriptor& b,
                  double linearTolerance) {
    if (std::abs(dot(a.axis, b.axis)) < 1.0 - 1e-9) return false;
    return norm(cross(sub(b.origin, a.origin), a.axis)) <= linearTolerance;
}

bool sameDomain(const SurfaceDescriptor& a, const SurfaceDescriptor& b,
                double linearTolerance) {
    if (a.type != b.type) return false;
    const double radiusTolerance = std::max(linearTolerance, 1e-9);
    switch (a.type) {
        case SurfaceType::Plane:
            return std::abs(dot(a.axis, b.axis)) >= 1.0 - 1e-9 &&
                   std::abs(dot(sub(b.origin, a.origin), a.axis)) <=
                       linearTolerance;
        case SurfaceType::Cylinder:
            return sameAxisLine(a, b, linearTolerance) &&
                   std::abs(a.radius - b.radius) <= radiusTolerance;
        case SurfaceType::Cone:
            return sameAxisLine(a, b, linearTolerance) &&
                   std::abs(a.semiAngle - b.semiAngle) <= 1e-9 &&
                   norm(sub(a.origin, b.origin)) <= linearTolerance;
        case SurfaceType::Sphere:
            return norm(sub(a.origin, b.origin)) <= linearTolerance &&
                   std::abs(a.radius - b.radius) <= radiusTolerance;
        case SurfaceType::Torus:
            return sameAxisLine(a, b, linearTolerance) &&
                   norm(sub(a.origin, b.origin)) <= linearTolerance &&
                   std::abs(a.radius - b.radius) <= radiusTolerance &&
                   std::abs(a.secondaryRadius - b.secondaryRadius) <=
                       radiusTolerance;
        default:
            // Recovered or freeform same-domain grouping needs stronger
            // evidence than a type label, so keep those faces separate.
            return false;
    }
}

struct DisjointSet {
    explicit DisjointSet(int count) : parent(count + 1), rank(count + 1, 0) {
        std::iota(parent.begin(), parent.end(), 0);
    }
    int find(int x) {
        if (parent[x] != x) parent[x] = find(parent[x]);
        return parent[x];
    }
    void unite(int a, int b) {
        a = find(a);
        b = find(b);
        if (a == b) return;
        if (rank[a] < rank[b]) std::swap(a, b);
        parent[b] = a;
        if (rank[a] == rank[b]) ++rank[a];
    }
    std::vector<int> parent;
    std::vector<int> rank;
};

RegionType regionTypeFor(const BrepFaceNode& face) {
    if (face.isFillet) return RegionType::Fillet;
    switch (face.surface.type) {
        case SurfaceType::Plane: return RegionType::Planar;
        case SurfaceType::Cylinder: return RegionType::Cylinder;
        case SurfaceType::Cone: return RegionType::Cone;
        case SurfaceType::Sphere: return RegionType::Sphere;
        case SurfaceType::Torus: return RegionType::Torus;
        case SurfaceType::Revolution: return RegionType::Revolution;
        case SurfaceType::Extrusion: return RegionType::Extrusion;
        default: return RegionType::Freeform;
    }
}

bool structuredSurface(const BrepFaceNode& face) {
    if (face.isFillet) return true;
    switch (face.surface.type) {
        case SurfaceType::Cylinder:
        case SurfaceType::Cone:
        case SurfaceType::Revolution:
        case SurfaceType::Extrusion: return true;
        default: return false;
    }
}

BrepGraph buildGraph(const Model& model, const Analysis& analysis) {
    BrepGraph graph;

    ShapeMap vertexMap;
    TopExp::MapShapes(model.shape, TopAbs_VERTEX, vertexMap);
    graph.vertices.resize(vertexMap.Extent());
    for (int vid = 1; vid <= vertexMap.Extent(); ++vid) {
        const TopoDS_Vertex vertex = TopoDS::Vertex(vertexMap(vid));
        BrepVertexNode& node = graph.vertices[vid - 1];
        node.id = vid;
        node.position = pointArray(BRep_Tool::Pnt(vertex));
        node.tolerance = BRep_Tool::Tolerance(vertex);
        node.sampleId = static_cast<std::uint64_t>(vid);
    }

    graph.faces.resize(model.faceCount());
    for (int fid = 1; fid <= model.faceCount(); ++fid) {
        const TopoDS_Face face = TopoDS::Face(model.faces(fid));
        BrepFaceNode& node = graph.faces[fid - 1];
        node.id = fid;
        node.surface = describeSurface(face, analysis.faces[fid - 1]);
        node.tolerance = BRep_Tool::Tolerance(face);
        node.reversed = face.Orientation() == TopAbs_REVERSED;
        node.isFillet = analysis.faces[fid - 1].isFillet;

        const TopoDS_Wire outer = BRepTools::OuterWire(face);
        std::vector<TopoDS_Wire> orderedWires;
        if (!outer.IsNull()) orderedWires.push_back(outer);
        for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
            const TopoDS_Wire wire = TopoDS::Wire(wx.Current());
            if (!outer.IsNull() && wire.IsSame(outer)) continue;
            orderedWires.push_back(wire);
        }

        for (size_t wi = 0; wi < orderedWires.size(); ++wi) {
            std::vector<int> wireCoedges;
            for (BRepTools_WireExplorer ex(orderedWires[wi], face); ex.More();
                 ex.Next()) {
                const TopoDS_Edge edge = TopoDS::Edge(ex.Current());
                const int eid = model.edges.FindIndex(edge);
                if (eid <= 0) continue;
                BrepCoedgeNode coedge;
                coedge.id = static_cast<int>(graph.coedges.size()) + 1;
                coedge.edgeId = eid;
                coedge.faceId = fid;
                coedge.wireIndex = static_cast<int>(wi);
                coedge.reversed = edge.Orientation() == TopAbs_REVERSED;
                wireCoedges.push_back(coedge.id);
                graph.coedges.push_back(std::move(coedge));
            }
            if (!wireCoedges.empty()) node.wires.push_back(std::move(wireCoedges));
        }
    }

    graph.edges.resize(model.edgeCount());
    for (int eid = 1; eid <= model.edgeCount(); ++eid) {
        TopoDS_Edge edge = TopoDS::Edge(model.edges(eid));
        edge.Orientation(TopAbs_FORWARD);  // canonical curve direction
        BrepEdgeNode& node = graph.edges[eid - 1];
        node.id = eid;
        node.tolerance = BRep_Tool::Tolerance(edge);
        node.degenerate = BRep_Tool::Degenerated(edge);
        node.length = analysis.edges[eid - 1].length;
        TopoDS_Vertex first, last;
        TopExp::Vertices(edge, first, last, Standard_True);
        node.firstVertex = first.IsNull() ? 0 : vertexMap.FindIndex(first);
        node.lastVertex = last.IsNull() ? 0 : vertexMap.FindIndex(last);
        node.closed = node.firstVertex > 0 &&
                      node.firstVertex == node.lastVertex;
        if (!node.degenerate) {
            try {
                BRepAdaptor_Curve curve(edge);
                node.curve = classifyCurve(curve);
                if (!(node.length > 0.0)) {
                    node.length = GCPnts_AbscissaPoint::Length(curve);
                }
                node.closed = node.closed || curve.IsClosed();
            } catch (const Standard_Failure&) {
            }
        }
    }
    for (BrepCoedgeNode& coedge : graph.coedges) {
        graph.edges[coedge.edgeId - 1].coedges.push_back(coedge.id);
    }
    return graph;
}

std::vector<SemanticRegion> buildRegions(BrepGraph& graph,
                                         const Analysis& analysis) {
    DisjointSet groups(static_cast<int>(graph.faces.size()));
    for (const EdgeInfo& edge : analysis.edges) {
        if (edge.convexity != EdgeConvexity::Smooth ||
            edge.faceIds.size() != 2) {
            continue;
        }
        const int a = edge.faceIds[0];
        const int b = edge.faceIds[1];
        const BrepFaceNode& fa = graph.faces[a - 1];
        const BrepFaceNode& fb = graph.faces[b - 1];
        const double tol = std::max(
            1e-7, 4.0 * std::max({fa.tolerance, fb.tolerance,
                                  graph.edges[edge.id - 1].tolerance}));
        if (sameDomain(fa.surface, fb.surface, tol)) groups.unite(a, b);
    }

    std::map<int, std::vector<int>> facesByRoot;
    for (const BrepFaceNode& face : graph.faces) {
        facesByRoot[groups.find(face.id)].push_back(face.id);
    }

    std::vector<SemanticRegion> regions;
    std::map<int, int> regionOfFace;
    for (auto& [root, faces] : facesByRoot) {
        (void)root;
        SemanticRegion region;
        region.id = static_cast<int>(regions.size()) + 1;
        region.type = regionTypeFor(graph.faces[faces.front() - 1]);
        for (int fid : faces) {
            if (graph.faces[fid - 1].isFillet) region.type = RegionType::Fillet;
            regionOfFace[fid] = region.id;
        }
        region.faceIds = std::move(faces);
        regions.push_back(std::move(region));
    }

    for (SemanticRegion& region : regions) {
        std::set<int> boundary;
        for (int fid : region.faceIds) {
            for (const std::vector<int>& wire : graph.faces[fid - 1].wires) {
                for (int cid : wire) {
                    const int eid = graph.coedges[cid - 1].edgeId;
                    bool outside = false;
                    int owners = 0;
                    for (int ocid : graph.edges[eid - 1].coedges) {
                        const int ofid = graph.coedges[ocid - 1].faceId;
                        if (ofid == fid || regionOfFace[ofid] == region.id) {
                            ++owners;
                        } else {
                            outside = true;
                        }
                    }
                    if (outside || owners < 2) boundary.insert(eid);
                }
            }
        }
        region.boundaryEdgeIds.assign(boundary.begin(), boundary.end());
    }

    // Edge semantics are derived after virtual grouping so a smooth imported
    // face split is distinguishable from a meaningful tangent feature rail.
    for (BrepEdgeNode& edge : graph.edges) {
        if (edge.degenerate) {
            edge.semantic = SemanticEdgeType::Degenerate;
            continue;
        }
        std::map<int, int> coedgesPerFace;
        std::set<int> faces;
        for (int cid : edge.coedges) {
            const int fid = graph.coedges[cid - 1].faceId;
            ++coedgesPerFace[fid];
            faces.insert(fid);
        }
        bool periodicSeam = false;
        for (const auto& [fid, uses] : coedgesPerFace) {
            if (uses > 1 && (graph.faces[fid - 1].surface.uPeriodic ||
                             graph.faces[fid - 1].surface.vPeriodic)) {
                periodicSeam = true;
            }
        }
        if (periodicSeam) {
            edge.semantic = SemanticEdgeType::PeriodicSeam;
            continue;
        }
        if (faces.size() < 2) {
            edge.semantic = SemanticEdgeType::Boundary;
            continue;
        }
        const EdgeInfo& info = analysis.edges[edge.id - 1];
        if (info.convexity == EdgeConvexity::Convex) {
            edge.semantic = SemanticEdgeType::SharpConvex;
        } else if (info.convexity == EdgeConvexity::Concave) {
            edge.semantic = SemanticEdgeType::SharpConcave;
        } else if (info.convexity == EdgeConvexity::Smooth) {
            const std::vector<int> ids(faces.begin(), faces.end());
            const bool sameRegion = ids.size() == 2 &&
                                    regionOfFace[ids[0]] ==
                                        regionOfFace[ids[1]];
            const bool filletRail = std::any_of(
                ids.begin(), ids.end(), [&](int fid) {
                    return graph.faces[fid - 1].isFillet;
                });
            edge.semantic = sameRegion
                                ? SemanticEdgeType::SmoothFaceSplit
                                : filletRail ? SemanticEdgeType::FilletRail
                                             : SemanticEdgeType::Smooth;
        } else {
            edge.semantic = SemanticEdgeType::Boundary;
        }
    }
    return regions;
}

std::vector<CountConstraint> buildCountConstraints(const BrepGraph& graph,
                                                   DisjointSet& edgeGroups) {
    std::vector<CountConstraint> constraints;
    for (const BrepFaceNode& face : graph.faces) {
        if (!structuredSurface(face) || face.wires.empty() ||
            face.wires[0].size() != 4) {
            continue;
        }
        const std::vector<int>& wire = face.wires[0];
        for (int pair = 0; pair < 2; ++pair) {
            const int a = graph.coedges[wire[pair] - 1].edgeId;
            const int b = graph.coedges[wire[pair + 2] - 1].edgeId;
            if (a == b) continue;
            edgeGroups.unite(a, b);
            CountConstraint c;
            c.type = face.isFillet
                         ? CountConstraintType::FilletOppositeSides
                         : CountConstraintType::StructuredOppositeSides;
            c.faceId = face.id;
            c.edgeIds = {a, b};
            constraints.push_back(std::move(c));
        }
    }
    return constraints;
}

bool trimmedRevolutionSurface(const BrepFaceNode& face) {
    if (face.isFillet) return false;
    return face.surface.type == SurfaceType::Cylinder ||
           face.surface.type == SurfaceType::Cone ||
           face.surface.type == SurfaceType::Revolution;
}

std::vector<PatchPlan> buildPatchPlans(const CompilerPlan& plan,
                                       const CompilerSettings& settings) {
    std::vector<PatchPlan> patches(plan.graph.faces.size());
    for (const BrepFaceNode& face : plan.graph.faces) {
        PatchPlan& patch = patches[face.id - 1];
        patch.faceId = face.id;
        if (face.surface.type == SurfaceType::Plane &&
            face.wires.size() == 1 && !face.wires[0].empty()) {
            patch.kind = PatchKind::PlanarNGon;
            patch.fallbackReason = PatchFallbackReason::None;
            continue;
        }
        if (trimmedRevolutionSurface(face) && face.wires.size() == 1) {
            const std::vector<int>& wire = face.wires[0];
            std::vector<size_t> seams;
            for (size_t i = 0; i < wire.size(); ++i) {
                const int edgeId =
                    plan.graph.coedges[wire[i] - 1].edgeId;
                if (plan.graph.edges[edgeId - 1].semantic ==
                    SemanticEdgeType::PeriodicSeam) {
                    seams.push_back(i);
                }
            }
            if (seams.size() == 2) {
                auto appendBetween = [&](size_t first, size_t last,
                                         std::vector<int>& out) {
                    for (size_t i = (first + 1) % wire.size(); i != last;
                         i = (i + 1) % wire.size()) {
                        out.push_back(wire[i]);
                    }
                };
                appendBetween(seams[0], seams[1], patch.rims[0]);
                appendBetween(seams[1], seams[0], patch.rims[1]);
                if (!patch.rims[0].empty() && !patch.rims[1].empty()) {
                    auto chainSegments = [&](const std::vector<int>& chain) {
                        int count = 0;
                        for (int coedgeId : chain) {
                            const int edgeId =
                                plan.graph.coedges[coedgeId - 1].edgeId;
                            count += plan.edgePlans[edgeId - 1].segmentCount;
                        }
                        return count;
                    };
                    patch.kind = PatchKind::TrimmedRevolution;
                    patch.fallbackReason = PatchFallbackReason::None;
                    patch.uSegments = std::max(
                        {3, settings.radialSegments,
                         chainSegments(patch.rims[0]),
                         chainSegments(patch.rims[1])});
                    // Two narrow trim-transition strips plus at least two
                    // clean interior quad rows.  The boundary counts may be
                    // unrelated, but their mismatch must not consume the
                    // visible cylinder body.
                    patch.vSegments = std::max(4, settings.axialSegments);
                    continue;
                }
            }
        }
        if (face.wires.size() > 1) {
            patch.fallbackReason = PatchFallbackReason::MultipleTrimLoops;
            continue;
        }
        if (structuredSurface(face) && face.wires.size() == 1 &&
            face.wires[0].size() != 4) {
            patch.fallbackReason = PatchFallbackReason::NonFourSidedTrim;
            continue;
        }
        if (structuredSurface(face) && face.wires.size() == 1) {
            for (int i = 0; i < 4; ++i) patch.sides[i] = face.wires[0][i];
            const int e0 = plan.graph.coedges[patch.sides[0] - 1].edgeId;
            const int e1 = plan.graph.coedges[patch.sides[1] - 1].edgeId;
            const int e2 = plan.graph.coedges[patch.sides[2] - 1].edgeId;
            const int e3 = plan.graph.coedges[patch.sides[3] - 1].edgeId;
            const int n0 = plan.edgePlans[e0 - 1].segmentCount;
            const int n1 = plan.edgePlans[e1 - 1].segmentCount;
            const int n2 = plan.edgePlans[e2 - 1].segmentCount;
            const int n3 = plan.edgePlans[e3 - 1].segmentCount;
            if (n0 == n2 && n1 == n3) {
                patch.kind = face.isFillet ? PatchKind::FilletRibbon
                                           : PatchKind::StructuredSurface;
                patch.uSegments = n0;
                patch.vSegments = n1;
                patch.fallbackReason = PatchFallbackReason::None;
                if (face.isFillet) {
                    const double uLength =
                        plan.graph.edges[e0 - 1].length +
                        plan.graph.edges[e2 - 1].length;
                    const double vLength =
                        plan.graph.edges[e1 - 1].length +
                        plan.graph.edges[e3 - 1].length;
                    patch.longitudinalAxis = uLength >= vLength ? 0 : 1;
                }
                continue;
            }
            patch.fallbackReason =
                PatchFallbackReason::OppositeCountMismatch;
        }
        patch.kind = PatchKind::TriangleFallback;
    }
    return patches;
}

struct SampleRecord {
    std::array<double, 3> position{};
    MeshConstraint constraint;
};

class CompilerMeshBuilder {
public:
    explicit CompilerMeshBuilder(const CompilerPlan& plan) : m_plan(plan) {
        for (const BrepVertexNode& vertex : plan.graph.vertices) {
            SampleRecord record;
            record.position = vertex.position;
            record.constraint.type = MeshConstraintType::BrepVertex;
            record.constraint.ownerId = vertex.id;
            m_samples.emplace(vertex.sampleId, record);
        }
        for (const CanonicalEdgePlan& edge : plan.edgePlans) {
            for (const CanonicalEdgeSample& sample : edge.samples) {
                if (!sample.valid) continue;
                if (m_samples.count(sample.id)) continue;
                SampleRecord record;
                record.position = sample.position;
                record.constraint.type = MeshConstraintType::BrepEdge;
                record.constraint.ownerId = edge.edgeId;
                record.constraint.t = sample.curveParameter;
                m_samples.emplace(sample.id, record);
            }
        }
    }

    uint32_t canonical(std::uint64_t sampleId, int faceId,
                       const std::array<double, 2>& uv) {
        auto existing = m_vertexBySample.find(sampleId);
        if (existing != m_vertexBySample.end()) return existing->second;
        const auto sample = m_samples.find(sampleId);
        if (sample == m_samples.end()) return UINT32_MAX;
        const uint32_t index = static_cast<uint32_t>(mesh.vertices.size());
        mesh.vertices.push_back(sample->second.position);
        mesh.anchors.push_back({faceId, uv[0], uv[1]});
        mesh.constraints.push_back(sample->second.constraint);
        m_vertexBySample.emplace(sampleId, index);
        return index;
    }

    uint32_t facePoint(int faceId, const std::array<double, 2>& uv,
                       const std::array<double, 3>& position) {
        const uint32_t index = static_cast<uint32_t>(mesh.vertices.size());
        mesh.vertices.push_back(position);
        mesh.anchors.push_back({faceId, uv[0], uv[1]});
        MeshConstraint constraint;
        constraint.type = MeshConstraintType::BrepFace;
        constraint.ownerId = faceId;
        constraint.u = uv[0];
        constraint.v = uv[1];
        mesh.constraints.push_back(constraint);
        return index;
    }

    void polygon(std::vector<uint32_t> corners, int faceId,
                 std::vector<Anchor> cornerUv) {
        if (corners.size() < 3) return;
        mesh.polygons.push_back(std::move(corners));
        mesh.polygonFaceId.push_back(faceId);
        mesh.polygonCornerAnchors.push_back(std::move(cornerUv));
    }

    PolyMesh mesh;

private:
    const CompilerPlan& m_plan;
    std::unordered_map<std::uint64_t, SampleRecord> m_samples;
    std::unordered_map<std::uint64_t, uint32_t> m_vertexBySample;
};

bool finiteUv(const std::array<double, 2>& uv) {
    return std::isfinite(uv[0]) && std::isfinite(uv[1]);
}

std::array<double, 3> newellNormal(
    const PolyMesh& mesh, const std::vector<uint32_t>& polygon) {
    std::array<double, 3> n{};
    for (size_t i = 0; i < polygon.size(); ++i) {
        const auto& a = mesh.vertices[polygon[i]];
        const auto& b = mesh.vertices[polygon[(i + 1) % polygon.size()]];
        n[0] += (a[1] - b[1]) * (a[2] + b[2]);
        n[1] += (a[2] - b[2]) * (a[0] + b[0]);
        n[2] += (a[0] - b[0]) * (a[1] + b[1]);
    }
    return n;
}

std::array<double, 3> surfaceNormal(const TopoDS_Face& face,
                                    const std::array<double, 2>& uv) {
    std::array<double, 3> out{};
    try {
        BRepAdaptor_Surface surface(face, false);
        gp_Pnt point;
        gp_Vec du, dv;
        surface.D1(uv[0], uv[1], point, du, dv);
        gp_Vec normal = du.Crossed(dv);
        if (normal.SquareMagnitude() > 1e-24) {
            normal.Normalize();
            if (face.Orientation() == TopAbs_REVERSED) normal.Reverse();
            out = {normal.X(), normal.Y(), normal.Z()};
        }
    } catch (const Standard_Failure&) {
    }
    return out;
}

void orientToFace(const Model& model, int faceId, PolyMesh& mesh,
                  std::vector<uint32_t>& polygon,
                  std::vector<Anchor>& corners) {
    if (polygon.size() < 3 || corners.size() != polygon.size()) return;
    std::array<double, 2> uv{};
    int valid = 0;
    for (const Anchor& corner : corners) {
        if (!std::isfinite(corner.u) || !std::isfinite(corner.v)) continue;
        uv[0] += corner.u;
        uv[1] += corner.v;
        ++valid;
    }
    if (!valid) return;
    uv[0] /= valid;
    uv[1] /= valid;
    const std::array<double, 3> exact =
        surfaceNormal(TopoDS::Face(model.faces(faceId)), uv);
    const std::array<double, 3> polygonNormal = newellNormal(mesh, polygon);
    if (dot(exact, polygonNormal) < 0.0) {
        std::reverse(polygon.begin(), polygon.end());
        std::reverse(corners.begin(), corners.end());
    }
}

bool meshPlanarPatch(const Model& model, const CompilerPlan& plan,
                     const PatchPlan& patch, CompilerMeshBuilder& builder) {
    const BrepFaceNode& face = plan.graph.faces[patch.faceId - 1];
    if (face.wires.size() != 1 || face.wires[0].empty()) return false;
    std::vector<uint32_t> polygon;
    std::vector<Anchor> corners;
    for (int cid : face.wires[0]) {
        const BrepCoedgeNode& coedge = plan.graph.coedges[cid - 1];
        if (coedge.sampleIds.size() < 2 ||
            coedge.uv.size() != coedge.sampleIds.size()) {
            return false;
        }
        // Every coedge contributes its start and interior samples.  The next
        // coedge owns the shared corner; the polygon closes implicitly.
        for (size_t i = 0; i + 1 < coedge.sampleIds.size(); ++i) {
            if (!finiteUv(coedge.uv[i])) return false;
            const uint32_t vertex = builder.canonical(
                coedge.sampleIds[i], patch.faceId, coedge.uv[i]);
            if (vertex == UINT32_MAX) return false;
            if (!polygon.empty() && polygon.back() == vertex) continue;
            polygon.push_back(vertex);
            corners.push_back(
                {patch.faceId, coedge.uv[i][0], coedge.uv[i][1]});
        }
    }
    while (polygon.size() > 1 && polygon.front() == polygon.back()) {
        polygon.pop_back();
        corners.pop_back();
    }
    if (polygon.size() < 3) return false;
    orientToFace(model, patch.faceId, builder.mesh, polygon, corners);
    builder.polygon(std::move(polygon), patch.faceId, std::move(corners));
    return true;
}

struct SideSamples {
    std::vector<std::uint64_t> ids;
    std::vector<std::array<double, 2>> uv;
};

SideSamples sideSamples(const CompilerPlan& plan, int coedgeId,
                        bool reverse) {
    const BrepCoedgeNode& coedge = plan.graph.coedges[coedgeId - 1];
    SideSamples side{coedge.sampleIds, coedge.uv};
    if (reverse) {
        std::reverse(side.ids.begin(), side.ids.end());
        std::reverse(side.uv.begin(), side.uv.end());
    }
    return side;
}

void unwrapCoordinate(std::vector<std::array<double, 2>>& uv, int axis,
                      double period) {
    if (!(period > 0.0) || uv.empty()) return;
    for (size_t i = 1; i < uv.size(); ++i) {
        while (uv[i][axis] - uv[i - 1][axis] > 0.5 * period)
            uv[i][axis] -= period;
        while (uv[i][axis] - uv[i - 1][axis] < -0.5 * period)
            uv[i][axis] += period;
    }
}

void alignCoordinate(std::vector<std::array<double, 2>>& uv, int axis,
                     double target, double period, bool useBack = false) {
    if (!(period > 0.0) || uv.empty()) return;
    const double value = useBack ? uv.back()[axis] : uv.front()[axis];
    const double shift = std::round((target - value) / period) * period;
    for (auto& p : uv) p[axis] += shift;
}

std::array<double, 2> coonsUv(const std::array<double, 2>& bottom,
                              const std::array<double, 2>& top,
                              const std::array<double, 2>& left,
                              const std::array<double, 2>& right,
                              const std::array<double, 2>& p00,
                              const std::array<double, 2>& p10,
                              const std::array<double, 2>& p11,
                              const std::array<double, 2>& p01, double s,
                              double t) {
    std::array<double, 2> uv{};
    for (int axis = 0; axis < 2; ++axis) {
        const double boundary =
            (1.0 - t) * bottom[axis] + t * top[axis] +
            (1.0 - s) * left[axis] + s * right[axis];
        const double bilinear =
            (1.0 - s) * (1.0 - t) * p00[axis] +
            s * (1.0 - t) * p10[axis] + s * t * p11[axis] +
            (1.0 - s) * t * p01[axis];
        uv[axis] = boundary - bilinear;
    }
    return uv;
}

bool meshStructuredPatch(const Model& model, const CompilerPlan& plan,
                         const PatchPlan& patch,
                         CompilerMeshBuilder& builder) {
    auto fail = [&](const char* reason) {
        if (std::getenv("WEFT_COMPILER_TRACE")) {
            std::fprintf(stderr, "compiler face %d structured reject: %s\n",
                         patch.faceId, reason);
        }
        return false;
    };
    if (patch.uSegments < 1 || patch.vSegments < 1)
        return fail("empty grid");
    SideSamples bottom = sideSamples(plan, patch.sides[0], false);
    SideSamples right = sideSamples(plan, patch.sides[1], false);
    SideSamples top = sideSamples(plan, patch.sides[2], true);
    SideSamples left = sideSamples(plan, patch.sides[3], true);
    const size_t nu = static_cast<size_t>(patch.uSegments);
    const size_t nv = static_cast<size_t>(patch.vSegments);
    if (bottom.ids.size() != nu + 1 || top.ids.size() != nu + 1 ||
        right.ids.size() != nv + 1 || left.ids.size() != nv + 1 ||
        bottom.uv.size() != bottom.ids.size() ||
        top.uv.size() != top.ids.size() ||
        right.uv.size() != right.ids.size() ||
        left.uv.size() != left.ids.size()) {
        return fail("opposite side sample-count mismatch");
    }
    for (const SideSamples* side : {&bottom, &right, &top, &left}) {
        if (!std::all_of(side->uv.begin(), side->uv.end(), finiteUv)) {
            return fail("missing face pcurve UV");
        }
    }

    const TopoDS_Face face = TopoDS::Face(model.faces(patch.faceId));
    BRepAdaptor_Surface surface(face, false);
    const bool periodic[2] = {surface.IsUPeriodic(), surface.IsVPeriodic()};
    const double period[2] = {
        periodic[0] ? surface.UPeriod() : 0.0,
        periodic[1] ? surface.VPeriod() : 0.0};
    for (int axis = 0; axis < 2; ++axis) {
        if (!periodic[axis]) continue;
        unwrapCoordinate(bottom.uv, axis, period[axis]);
        unwrapCoordinate(right.uv, axis, period[axis]);
        unwrapCoordinate(top.uv, axis, period[axis]);
        unwrapCoordinate(left.uv, axis, period[axis]);
        alignCoordinate(right.uv, axis, bottom.uv.back()[axis], period[axis]);
        alignCoordinate(left.uv, axis, bottom.uv.front()[axis], period[axis]);
        alignCoordinate(top.uv, axis, right.uv.back()[axis], period[axis],
                        true);
    }

    std::vector<uint32_t> grid((nu + 1) * (nv + 1), UINT32_MAX);
    std::vector<std::array<double, 2>> gridUv(grid.size());
    auto index = [&](size_t i, size_t j) { return j * (nu + 1) + i; };
    for (size_t j = 0; j <= nv; ++j) {
        const double t = j / static_cast<double>(nv);
        for (size_t i = 0; i <= nu; ++i) {
            const double s = i / static_cast<double>(nu);
            const size_t k = index(i, j);
            std::uint64_t sampleId = 0;
            if (j == 0) {
                sampleId = bottom.ids[i];
                gridUv[k] = bottom.uv[i];
            } else if (i == nu) {
                sampleId = right.ids[j];
                gridUv[k] = right.uv[j];
            } else if (j == nv) {
                sampleId = top.ids[i];
                gridUv[k] = top.uv[i];
            } else if (i == 0) {
                sampleId = left.ids[j];
                gridUv[k] = left.uv[j];
            } else {
                gridUv[k] = coonsUv(
                    bottom.uv[i], top.uv[i], left.uv[j], right.uv[j],
                    bottom.uv.front(), bottom.uv.back(), top.uv.back(),
                    top.uv.front(), s, t);
            }
            if (sampleId) {
                grid[k] =
                    builder.canonical(sampleId, patch.faceId, gridUv[k]);
            } else {
                try {
                    const gp_Pnt p = surface.Value(gridUv[k][0], gridUv[k][1]);
                    grid[k] = builder.facePoint(
                        patch.faceId, gridUv[k], pointArray(p));
                } catch (const Standard_Failure&) {
                    return fail("surface evaluation failed");
                }
            }
            if (grid[k] == UINT32_MAX) return fail("canonical sample missing");
        }
    }

    for (size_t j = 0; j < nv; ++j) {
        for (size_t i = 0; i < nu; ++i) {
            std::vector<uint32_t> polygon = {
                grid[index(i, j)], grid[index(i + 1, j)],
                grid[index(i + 1, j + 1)], grid[index(i, j + 1)]};
            std::vector<Anchor> corners = {
                {patch.faceId, gridUv[index(i, j)][0],
                 gridUv[index(i, j)][1]},
                {patch.faceId, gridUv[index(i + 1, j)][0],
                 gridUv[index(i + 1, j)][1]},
                {patch.faceId, gridUv[index(i + 1, j + 1)][0],
                 gridUv[index(i + 1, j + 1)][1]},
                {patch.faceId, gridUv[index(i, j + 1)][0],
                 gridUv[index(i, j + 1)][1]}};
            orientToFace(model, patch.faceId, builder.mesh, polygon, corners);
            builder.polygon(std::move(polygon), patch.faceId,
                            std::move(corners));
        }
    }
    return true;
}

bool evaluateCanonicalEdgePoint(const Model& model, const CompilerPlan& plan,
                                int edgeId, const TopoDS_Edge& edge,
                                double parameter,
                                std::array<double, 3>& position);
void projectCanonicalSamples(const Model& model, CompilerPlan& plan);

struct RevolutionRim {
    std::vector<std::uint64_t> ids;
    std::vector<std::array<double, 2>> uv;
    std::vector<uint32_t> vertices;
};

bool collectRevolutionRim(const CompilerPlan& plan,
                          const std::vector<int>& chain, double period,
                          double chartStart, RevolutionRim& rim) {
    for (int coedgeId : chain) {
        const BrepCoedgeNode& coedge = plan.graph.coedges[coedgeId - 1];
        if (coedge.sampleIds.size() < 2 ||
            coedge.sampleIds.size() != coedge.uv.size()) {
            return false;
        }
        for (size_t i = 0; i < coedge.sampleIds.size(); ++i) {
            if (!rim.ids.empty() && i == 0 &&
                rim.ids.back() == coedge.sampleIds[i]) {
                continue;
            }
            if (!finiteUv(coedge.uv[i])) return false;
            rim.ids.push_back(coedge.sampleIds[i]);
            rim.uv.push_back(coedge.uv[i]);
        }
    }
    if (rim.ids.size() < 3) return false;
    unwrapCoordinate(rim.uv, 0, period);
    if (rim.uv.back()[0] < rim.uv.front()[0]) {
        std::reverse(rim.ids.begin(), rim.ids.end());
        std::reverse(rim.uv.begin(), rim.uv.end());
        unwrapCoordinate(rim.uv, 0, period);
    }
    const double shift =
        std::round((chartStart - rim.uv.front()[0]) / period) * period;
    for (auto& uv : rim.uv) uv[0] += shift;
    const double span = rim.uv.back()[0] - rim.uv.front()[0];
    return span > 0.8 * period && span < 1.2 * period;
}

bool insertRimStation(const Model& model, CompilerPlan& plan,
                      const PatchPlan& patch,
                      const std::vector<int>& chain, double targetU,
                      double period, std::uint64_t& nextSample) {
    const double epsilon = std::max(1e-10, period * 1e-8);
    for (int coedgeId : chain) {
        const BrepCoedgeNode& coedge = plan.graph.coedges[coedgeId - 1];
        std::vector<std::array<double, 2>> uv = coedge.uv;
        if (uv.size() < 2 ||
            !std::all_of(uv.begin(), uv.end(), finiteUv)) {
            continue;
        }
        unwrapCoordinate(uv, 0, period);
        double middle = 0.5 * (uv.front()[0] + uv.back()[0]);
        const double alignedTarget =
            targetU + std::round((middle - targetU) / period) * period;
        const double low = std::min(uv.front()[0], uv.back()[0]);
        const double high = std::max(uv.front()[0], uv.back()[0]);
        if (alignedTarget < low - epsilon ||
            alignedTarget > high + epsilon) {
            continue;
        }
        for (const auto& sampleUv : uv) {
            if (std::abs(sampleUv[0] - alignedTarget) <= epsilon) return true;
        }

        TopoDS_Edge edge = TopoDS::Edge(model.edges(coedge.edgeId));
        edge.Orientation(TopAbs_FORWARD);
        const TopoDS_Face face = TopoDS::Face(model.faces(patch.faceId));
        double first = 0.0, last = 1.0;
        Handle(Geom2d_Curve) pcurve =
            BRep_Tool::CurveOnSurface(edge, face, first, last);
        if (pcurve.IsNull()) continue;
        size_t bracket = uv.size();
        for (size_t i = 0; i + 1 < uv.size(); ++i) {
            if (alignedTarget >= std::min(uv[i][0], uv[i + 1][0]) - epsilon &&
                alignedTarget <= std::max(uv[i][0], uv[i + 1][0]) + epsilon) {
                bracket = i;
                break;
            }
        }
        if (bracket == uv.size()) continue;
        CanonicalEdgePlan& edgePlan = plan.edgePlans[coedge.edgeId - 1];
        auto parameterForId = [&](std::uint64_t id, double& parameter) {
            for (const CanonicalEdgeSample& sample : edgePlan.samples) {
                if (sample.id == id) {
                    parameter = sample.curveParameter;
                    return true;
                }
            }
            return false;
        };
        auto uAt = [&](double parameter) {
            const double raw = pcurve->Value(parameter).X();
            return raw + std::round((alignedTarget - raw) / period) * period;
        };
        try {
            double a = first, b = last;
            if (!parameterForId(coedge.sampleIds[bracket], a) ||
                !parameterForId(coedge.sampleIds[bracket + 1], b)) {
                continue;
            }
            double ua = uAt(a), ub = uAt(b);
            for (int iteration = 0; iteration < 60; ++iteration) {
                const double mid = 0.5 * (a + b);
                const double um = uAt(mid);
                if ((alignedTarget - ua) * (alignedTarget - um) <= 0.0 ||
                    std::abs(um - alignedTarget) <= epsilon) {
                    b = mid;
                    ub = um;
                } else {
                    a = mid;
                    ua = um;
                }
            }
            const double parameter = 0.5 * (a + b);
            for (const CanonicalEdgeSample& sample : edgePlan.samples) {
                if (std::abs(sample.curveParameter - parameter) < 1e-11) {
                    return true;
                }
            }
            CanonicalEdgeSample sample;
            sample.id = nextSample++;
            sample.curveParameter = parameter;
            sample.valid = evaluateCanonicalEdgePoint(
                model, plan, coedge.edgeId, edge, parameter, sample.position);
            if (!sample.valid) return false;
            edgePlan.samples.push_back(sample);
            std::sort(edgePlan.samples.begin(), edgePlan.samples.end(),
                      [](const CanonicalEdgeSample& lhs,
                         const CanonicalEdgeSample& rhs) {
                          return lhs.curveParameter < rhs.curveParameter;
                      });
            edgePlan.segmentCount =
                std::max(1, static_cast<int>(edgePlan.samples.size()) - 1);
            for (size_t i = 0; i < edgePlan.samples.size(); ++i) {
                edgePlan.samples[i].normalizedAbscissa =
                    i / static_cast<double>(edgePlan.samples.size() - 1);
            }
            return true;
        } catch (const Standard_Failure&) {
            continue;
        }
    }
    return false;
}

void synchronizeTrimmedRevolutionRims(const Model& model,
                                      CompilerPlan& plan) {
    std::uint64_t nextSample =
        static_cast<std::uint64_t>(plan.graph.vertices.size()) + 1;
    for (const CanonicalEdgePlan& edge : plan.edgePlans) {
        for (const CanonicalEdgeSample& sample : edge.samples) {
            nextSample = std::max(nextSample, sample.id + 1);
        }
    }
    for (PatchPlan& patch : plan.patches) {
        if (patch.kind != PatchKind::TrimmedRevolution) continue;
        const TopoDS_Face face = TopoDS::Face(model.faces(patch.faceId));
        BRepAdaptor_Surface surface(face, false);
        if (!surface.IsUPeriodic()) continue;
        const double period = surface.UPeriod();
        RevolutionRim rim[2];
        if (!collectRevolutionRim(plan, patch.rims[0], period,
                                  surface.FirstUParameter(), rim[0]) ||
            !collectRevolutionRim(plan, patch.rims[1], period,
                                  surface.FirstUParameter(), rim[1])) {
            continue;
        }
        const double start = std::max(rim[0].uv.front()[0],
                                      rim[1].uv.front()[0]);
        // Only the uniform cylinder stations are synchronized.  Imported
        // trim vertices remain exact border details, but they do not become
        // full-length columns and therefore cannot distort span spacing over
        // the body of the primitive.
        std::vector<double> stations;
        for (int i = 0; i < patch.uSegments; ++i) {
            stations.push_back(start + period * i / patch.uSegments);
        }
        std::sort(stations.begin(), stations.end());
        const double epsilon = std::max(1e-10, period * 1e-8);
        stations.erase(
            std::unique(stations.begin(), stations.end(),
                        [&](double a, double b) {
                            return std::abs(a - b) <= epsilon;
                        }),
            stations.end());
        bool complete = true;
        for (const std::vector<int>& chain : patch.rims) {
            for (double station : stations) {
                if (!insertRimStation(model, plan, patch, chain, station,
                                      period, nextSample)) {
                    complete = false;
                    break;
                }
            }
            if (!complete) break;
        }
        if (!complete) continue;
    }
    projectCanonicalSamples(model, plan);

}

double rimVAt(const RevolutionRim& rim, double u) {
    if (u <= rim.uv.front()[0]) return rim.uv.front()[1];
    if (u >= rim.uv.back()[0]) return rim.uv.back()[1];
    const auto it = std::upper_bound(
        rim.uv.begin(), rim.uv.end(), u,
        [](double value, const std::array<double, 2>& point) {
            return value < point[0];
        });
    const size_t hi = static_cast<size_t>(it - rim.uv.begin());
    const size_t lo = hi - 1;
    const double span = rim.uv[hi][0] - rim.uv[lo][0];
    if (std::abs(span) < 1e-12) return rim.uv[lo][1];
    const double t = std::clamp((u - rim.uv[lo][0]) / span, 0.0, 1.0);
    return rim.uv[lo][1] + (rim.uv[hi][1] - rim.uv[lo][1]) * t;
}

bool emitRevolutionZipper(const Model& model, const PatchPlan& patch,
                          CompilerMeshBuilder& builder,
                          const RevolutionRim& boundary,
                          const std::vector<uint32_t>& innerVertices,
                          const std::vector<std::array<double, 2>>& innerUv) {
    if (boundary.vertices.size() != boundary.uv.size() ||
        innerVertices.size() != innerUv.size() ||
        boundary.vertices.size() < 2 || innerVertices.size() < 2) {
        return false;
    }
    const double a0 = boundary.uv.front()[0];
    const double aSpan = boundary.uv.back()[0] - a0;
    const double b0 = innerUv.front()[0];
    const double bSpan = innerUv.back()[0] - b0;
    if (!(aSpan > 0.0) || !(bSpan > 0.0)) return false;
    size_t a = 0, b = 0;
    const double epsilon = 1e-8;
    while (a + 1 < boundary.vertices.size() ||
           b + 1 < innerVertices.size()) {
        const double nextA = a + 1 < boundary.vertices.size()
                                 ? (boundary.uv[a + 1][0] - a0) / aSpan
                                 : std::numeric_limits<double>::infinity();
        const double nextB = b + 1 < innerVertices.size()
                                 ? (innerUv[b + 1][0] - b0) / bSpan
                                 : std::numeric_limits<double>::infinity();
        std::vector<uint32_t> polygon;
        std::vector<Anchor> corners;
        auto addBoundary = [&](size_t index) {
            polygon.push_back(boundary.vertices[index]);
            corners.push_back({patch.faceId, boundary.uv[index][0],
                               boundary.uv[index][1]});
        };
        auto addInner = [&](size_t index) {
            polygon.push_back(innerVertices[index]);
            corners.push_back({patch.faceId, innerUv[index][0],
                               innerUv[index][1]});
        };
        if (std::abs(nextA - nextB) <= epsilon) {
            addBoundary(a);
            addBoundary(a + 1);
            addInner(b + 1);
            addInner(b);
            ++a;
            ++b;
        } else if (nextA < nextB) {
            addBoundary(a);
            addBoundary(a + 1);
            addInner(b);
            ++a;
        } else {
            addBoundary(a);
            addInner(b + 1);
            addInner(b);
            ++b;
        }
        polygon.erase(std::unique(polygon.begin(), polygon.end()),
                      polygon.end());
        if (polygon.size() < 3 || polygon.size() != corners.size()) continue;
        orientToFace(model, patch.faceId, builder.mesh, polygon, corners);
        builder.polygon(std::move(polygon), patch.faceId,
                        std::move(corners));
    }
    return true;
}

bool emitRevolutionCollar(const Model& model, const PatchPlan& patch,
                          CompilerMeshBuilder& builder,
                          const RevolutionRim& boundary,
                          const std::vector<uint32_t>& innerVertices,
                          const std::vector<std::array<double, 2>>& innerUv,
                          double period) {
    if (boundary.vertices.size() != boundary.uv.size() ||
        innerVertices.size() != innerUv.size() || innerUv.size() < 4) {
        return false;
    }
    const double tolerance = std::max(1e-10, period * 1e-7);
    std::vector<size_t> stationIndex(innerUv.size(), boundary.uv.size());
    size_t cursor = 0;
    for (size_t station = 0; station < innerUv.size(); ++station) {
        const double target = innerUv[station][0];
        double best = tolerance;
        size_t found = boundary.uv.size();
        for (size_t i = cursor; i < boundary.uv.size(); ++i) {
            const double distance = std::abs(boundary.uv[i][0] - target);
            if (distance <= best) {
                best = distance;
                found = i;
            }
            if (boundary.uv[i][0] > target + tolerance) break;
        }
        if (found == boundary.uv.size()) return false;
        stationIndex[station] = found;
        cursor = found;
    }
    for (size_t station = 0; station + 1 < stationIndex.size(); ++station) {
        if (stationIndex[station + 1] <= stationIndex[station]) return false;
    }
    for (size_t station = 0; station + 1 < stationIndex.size(); ++station) {
        const size_t first = stationIndex[station];
        const size_t last = stationIndex[station + 1];
        std::vector<uint32_t> polygon;
        std::vector<Anchor> corners;
        for (size_t i = first; i <= last; ++i) {
            polygon.push_back(boundary.vertices[i]);
            corners.push_back(
                {patch.faceId, boundary.uv[i][0], boundary.uv[i][1]});
        }
        polygon.push_back(innerVertices[station + 1]);
        corners.push_back({patch.faceId, innerUv[station + 1][0],
                           innerUv[station + 1][1]});
        polygon.push_back(innerVertices[station]);
        corners.push_back({patch.faceId, innerUv[station][0],
                           innerUv[station][1]});
        orientToFace(model, patch.faceId, builder.mesh, polygon, corners);
        builder.polygon(std::move(polygon), patch.faceId,
                        std::move(corners));
    }
    return true;
}

bool meshTrimmedRevolutionPatch(const Model& model,
                                const CompilerPlan& plan,
                                const PatchPlan& patch,
                                CompilerMeshBuilder& builder) {
    if (patch.uSegments < 3 || patch.vSegments < 3 ||
        patch.rims[0].empty() || patch.rims[1].empty()) {
        return false;
    }
    const TopoDS_Face face = TopoDS::Face(model.faces(patch.faceId));
    BRepAdaptor_Surface surface(face, false);
    if (!surface.IsUPeriodic()) return false;
    const double period = surface.UPeriod();
    const double chartStart = surface.FirstUParameter();
    RevolutionRim rims[2];
    if (!collectRevolutionRim(plan, patch.rims[0], period, chartStart,
                              rims[0]) ||
        !collectRevolutionRim(plan, patch.rims[1], period, chartStart,
                              rims[1])) {
        return false;
    }
    auto averageV = [](const RevolutionRim& rim) {
        double value = 0.0;
        for (const auto& uv : rim.uv) value += uv[1];
        return value / rim.uv.size();
    };
    if (averageV(rims[0]) > averageV(rims[1])) std::swap(rims[0], rims[1]);

    for (RevolutionRim& rim : rims) {
        rim.vertices.reserve(rim.ids.size());
        for (size_t i = 0; i < rim.ids.size(); ++i) {
            const uint32_t vertex =
                builder.canonical(rim.ids[i], patch.faceId, rim.uv[i]);
            if (vertex == UINT32_MAX) return false;
            rim.vertices.push_back(vertex);
        }
    }

    const int nu = patch.uSegments;
    const int nv = patch.vSegments;
    const double uStart = std::max(rims[0].uv.front()[0],
                                   rims[1].uv.front()[0]);
    std::vector<std::vector<uint32_t>> rows(
        static_cast<size_t>(nv - 1),
        std::vector<uint32_t>(static_cast<size_t>(nu) + 1));
    std::vector<std::vector<std::array<double, 2>>> rowUv(
        static_cast<size_t>(nv - 1),
        std::vector<std::array<double, 2>>(static_cast<size_t>(nu) + 1));
    for (int j = 1; j < nv; ++j) {
        constexpr double kTrimTransition = 0.08;
        const double t = nv == 3
                             ? (j == 1 ? kTrimTransition
                                       : 1.0 - kTrimTransition)
                             : kTrimTransition +
                                   (1.0 - 2.0 * kTrimTransition) *
                                       (j - 1) / static_cast<double>(nv - 2);
        for (int i = 0; i < nu; ++i) {
            const double u = uStart + period * i / static_cast<double>(nu);
            const double low = rimVAt(rims[0], u);
            const double high = rimVAt(rims[1], u);
            const std::array<double, 2> uv = {u, low + (high - low) * t};
            try {
                rows[j - 1][i] = builder.facePoint(
                    patch.faceId, uv,
                    pointArray(surface.Value(uv[0], uv[1])));
                rowUv[j - 1][i] = uv;
            } catch (const Standard_Failure&) {
                return false;
            }
        }
        rows[j - 1][nu] = rows[j - 1][0];
        rowUv[j - 1][nu] = {uStart + period, rowUv[j - 1][0][1]};
    }

    const bool lowerCollar = emitRevolutionCollar(
        model, patch, builder, rims[0], rows.front(), rowUv.front(), period);
    if (!lowerCollar &&
        !emitRevolutionZipper(model, patch, builder, rims[0], rows.front(),
                              rowUv.front())) {
        return false;
    }
    for (int j = 0; j + 1 < nv - 1; ++j) {
        for (int i = 0; i < nu; ++i) {
            std::vector<uint32_t> polygon = {
                rows[j][i], rows[j][i + 1], rows[j + 1][i + 1],
                rows[j + 1][i]};
            std::vector<Anchor> corners = {
                {patch.faceId, rowUv[j][i][0], rowUv[j][i][1]},
                {patch.faceId, rowUv[j][i + 1][0], rowUv[j][i + 1][1]},
                {patch.faceId, rowUv[j + 1][i + 1][0],
                 rowUv[j + 1][i + 1][1]},
                {patch.faceId, rowUv[j + 1][i][0],
                 rowUv[j + 1][i][1]}};
            orientToFace(model, patch.faceId, builder.mesh, polygon, corners);
            builder.polygon(std::move(polygon), patch.faceId,
                            std::move(corners));
        }
    }
    if (emitRevolutionCollar(model, patch, builder, rims[1], rows.back(),
                             rowUv.back(), period)) {
        return true;
    }
    return emitRevolutionZipper(model, patch, builder, rims[1], rows.back(),
                                rowUv.back());
}

void appendMesh(PolyMesh& destination, const PolyMesh& source) {
    const uint32_t offset = static_cast<uint32_t>(destination.vertices.size());
    destination.vertices.insert(destination.vertices.end(), source.vertices.begin(),
                                source.vertices.end());
    destination.anchors.insert(destination.anchors.end(), source.anchors.begin(),
                               source.anchors.end());
    if (destination.constraints.size() < offset)
        destination.constraints.resize(offset);
    for (size_t i = 0; i < source.vertices.size(); ++i) {
        destination.constraints.push_back(
            i < source.constraints.size() ? source.constraints[i]
                                          : MeshConstraint{});
    }
    for (size_t p = 0; p < source.polygons.size(); ++p) {
        std::vector<uint32_t> polygon = source.polygons[p];
        for (uint32_t& vertex : polygon) vertex += offset;
        destination.polygons.push_back(std::move(polygon));
        destination.polygonFaceId.push_back(source.polygonFaceId[p]);
        destination.polygonCornerAnchors.push_back(
            p < source.polygonCornerAnchors.size()
                ? source.polygonCornerAnchors[p]
                : std::vector<Anchor>{});
    }
}


int idealEdgeCount(const TopoDS_Edge& edge, const BrepEdgeNode& node,
                   const CompilerSettings& settings) {
    if (node.degenerate) return 1;
    int count = 1;
    try {
        BRepAdaptor_Curve curve(edge);
        const double angle =
            std::max(0.1, settings.angleToleranceDeg) * kPi / 180.0;
        count = stableDeflectionCount(
            curve, angle, std::max(1e-12, settings.chordTolerance));
        if (node.curve == CurveType::Circle) {
            const gp_Circ circle = curve.Circle();
            (void)circle;
            const double span =
                std::abs(curve.LastParameter() - curve.FirstParameter());
            const int radialShare = std::max(
                1, static_cast<int>(std::lround(
                       std::max(3, settings.radialSegments) * span /
                       (2.0 * kPi))));
            count = std::max(count, radialShare);
        }
    } catch (const Standard_Failure&) {
    }
    if (node.closed) {
        count = std::max(count, settings.minimumClosedCurveSegments);
    }
    return std::clamp(count, 1, settings.maximumEdgeSegments);
}

bool evaluateCanonicalEdgePoint(const Model& model, const CompilerPlan& plan,
                                int edgeId, const TopoDS_Edge& edge,
                                double parameter,
                                std::array<double, 3>& position) {
    auto finitePoint = [](const std::array<double, 3>& p) {
        return std::isfinite(p[0]) && std::isfinite(p[1]) &&
               std::isfinite(p[2]);
    };
    try {
        BRepAdaptor_Curve curve(edge);
        position = pointArray(curve.Value(parameter));
        if (finitePoint(position)) return true;
    } catch (const Standard_Failure&) {
    }

    // Some STEP edges are represented only by a pcurve.  They are still
    // perfectly usable canonical boundaries: evaluate one incident coedge on
    // its exact surface instead of silently manufacturing a point at origin.
    const BrepEdgeNode& node = plan.graph.edges[edgeId - 1];
    for (int coedgeId : node.coedges) {
        const BrepCoedgeNode& coedge = plan.graph.coedges[coedgeId - 1];
        try {
            const TopoDS_Face face = TopoDS::Face(model.faces(coedge.faceId));
            double first = 0.0, last = 1.0;
            Handle(Geom2d_Curve) pcurve =
                BRep_Tool::CurveOnSurface(edge, face, first, last);
            if (pcurve.IsNull()) continue;
            const gp_Pnt2d uv = pcurve->Value(std::clamp(
                parameter, std::min(first, last), std::max(first, last)));
            if (!std::isfinite(uv.X()) || !std::isfinite(uv.Y())) continue;
            BRepAdaptor_Surface surface(face, false);
            position = pointArray(surface.Value(uv.X(), uv.Y()));
            if (finitePoint(position)) return true;
        } catch (const Standard_Failure&) {
        }
    }
    return false;
}

void projectCanonicalSamples(const Model& model, CompilerPlan& plan) {
    // Project the shared 3D sample sequence into each incident face.  The
    // canonical ids never change; only the UV coordinates are face-specific.
    for (BrepCoedgeNode& coedge : plan.graph.coedges) {
        coedge.sampleIds.clear();
        coedge.uv.clear();
        const CanonicalEdgePlan& edgePlan = plan.edgePlans[coedge.edgeId - 1];
        TopoDS_Edge edge = TopoDS::Edge(model.edges(coedge.edgeId));
        edge.Orientation(TopAbs_FORWARD);
        const TopoDS_Face face = TopoDS::Face(model.faces(coedge.faceId));
        double pcFirst = 0.0, pcLast = 1.0;
        Handle(Geom2d_Curve) pcurve =
            BRep_Tool::CurveOnSurface(edge, face, pcFirst, pcLast);
        coedge.sampleIds.reserve(edgePlan.samples.size());
        coedge.uv.reserve(edgePlan.samples.size());
        for (size_t oi = 0; oi < edgePlan.samples.size(); ++oi) {
            const size_t i = coedge.reversed
                                 ? edgePlan.samples.size() - 1 - oi
                                 : oi;
            const CanonicalEdgeSample& sample = edgePlan.samples[i];
            coedge.sampleIds.push_back(sample.id);
            std::array<double, 2> uv = {
                std::numeric_limits<double>::quiet_NaN(),
                std::numeric_limits<double>::quiet_NaN()};
            if (!pcurve.IsNull()) {
                try {
                    const double t = std::clamp(
                        sample.curveParameter, std::min(pcFirst, pcLast),
                        std::max(pcFirst, pcLast));
                    const gp_Pnt2d p = pcurve->Value(t);
                    uv = {p.X(), p.Y()};
                } catch (const Standard_Failure&) {
                }
            }
            coedge.uv.push_back(uv);
        }
    }
}

void populateCanonicalSamples(const Model& model, CompilerPlan& plan,
                              const std::vector<int>& solvedCounts) {
    std::uint64_t nextSample =
        static_cast<std::uint64_t>(plan.graph.vertices.size()) + 1;
    plan.edgePlans.resize(model.edgeCount());
    for (int eid = 1; eid <= model.edgeCount(); ++eid) {
        TopoDS_Edge edge = TopoDS::Edge(model.edges(eid));
        edge.Orientation(TopAbs_FORWARD);
        const BrepEdgeNode& edgeNode = plan.graph.edges[eid - 1];
        CanonicalEdgePlan& edgePlan = plan.edgePlans[eid - 1];
        edgePlan.edgeId = eid;
        edgePlan.idealSegmentCount = solvedCounts[eid * 2];
        edgePlan.segmentCount = solvedCounts[eid * 2 + 1];
        edgePlan.closed = edgeNode.closed;

        const int segments = std::max(1, edgePlan.segmentCount);
        double firstParameter = 0.0;
        double lastParameter = 1.0;
        Handle(Geom_Curve) curve3d =
            BRep_Tool::Curve(edge, firstParameter, lastParameter);
        std::vector<double> parameters(static_cast<size_t>(segments) + 1);
        for (int i = 0; i <= segments; ++i) {
            const double fraction = i / static_cast<double>(segments);
            parameters[static_cast<size_t>(i)] =
                firstParameter + (lastParameter - firstParameter) * fraction;
        }

        // STEP curves frequently have non-uniform parameterizations.  A
        // uniform t-step on two geometrically corresponding fillet rails can
        // therefore produce visibly mismatched spans.  Solve physical
        // arc-length stations once on the canonical 3D edge and expose the
        // same immutable sequence to every coedge/face that owns it.
        if (!edgeNode.degenerate) {
            try {
                BRepAdaptor_Curve adaptor(edge);
                GCPnts_UniformAbscissa stations(adaptor, segments + 1);
                if (stations.IsDone() && stations.NbPoints() == segments + 1) {
                    for (int i = 0; i <= segments; ++i) {
                        parameters[static_cast<size_t>(i)] =
                            stations.Parameter(i + 1);
                    }
                    edgePlan.uniformAbscissa = true;
                }
            } catch (const Standard_Failure&) {
                // Uniform parameter spacing remains a deterministic fallback
                // for curves whose arc length solver does not converge.
            }
        }
        parameters.front() = firstParameter;
        parameters.back() = lastParameter;

        edgePlan.samples.reserve(static_cast<size_t>(segments) + 1);
        for (int i = 0; i <= segments; ++i) {
            const double fraction = i / static_cast<double>(segments);
            const double parameter = parameters[static_cast<size_t>(i)];
            CanonicalEdgeSample sample;
            sample.curveParameter = parameter;
            sample.normalizedAbscissa = fraction;
            const bool first = i == 0 && edgeNode.firstVertex > 0;
            const bool last = i == segments && edgeNode.lastVertex > 0;
            if (first) {
                const BrepVertexNode& v =
                    plan.graph.vertices[edgeNode.firstVertex - 1];
                sample.id = v.sampleId;
                sample.position = v.position;
                sample.valid = true;
            } else if (last) {
                const BrepVertexNode& v =
                    plan.graph.vertices[edgeNode.lastVertex - 1];
                sample.id = v.sampleId;
                sample.position = v.position;
                sample.valid = true;
            } else {
                sample.id = nextSample++;
                sample.valid = evaluateCanonicalEdgePoint(
                    model, plan, eid, edge, parameter, sample.position);
            }
            edgePlan.samples.push_back(sample);
        }
        if (edgeNode.closed && !edgePlan.samples.empty()) {
            edgePlan.samples.back().id = edgePlan.samples.front().id;
            edgePlan.samples.back().position = edgePlan.samples.front().position;
            edgePlan.samples.back().valid = edgePlan.samples.front().valid;
        }
    }

    projectCanonicalSamples(model, plan);
}

CompilerSettings compilerSettingsFrom(const GenerationSettings& settings) {
    CompilerSettings out;
    out.chordTolerance = settings.defaults.chordTolerance;
    out.angleToleranceDeg = settings.defaults.angleToleranceDeg;
    out.radialSegments = settings.defaults.radial;
    out.axialSegments = settings.defaults.axial;
    out.filletAcrossSegments = settings.defaults.filletLoops;
    out.perEdge = settings.perEdge;
    return out;
}

}  // namespace

CompilerPlan planPrimitiveAware(const Model& model, const Analysis& analysis,
                                const CompilerSettings& settings) {
    CompilerPlan plan;
    plan.graph = buildGraph(model, analysis);
    plan.regions = buildRegions(plan.graph, analysis);

    DisjointSet edgeGroups(model.edgeCount());
    plan.countConstraints = buildCountConstraints(plan.graph, edgeGroups);

    std::vector<int> ideal(model.edgeCount() + 1, 1);
    std::map<int, int> chosenByRoot;
    for (int eid = 1; eid <= model.edgeCount(); ++eid) {
        ideal[eid] = idealEdgeCount(TopoDS::Edge(model.edges(eid)),
                                    plan.graph.edges[eid - 1], settings);
        if (auto it = settings.perEdge.find(eid); it != settings.perEdge.end()) {
            ideal[eid] = std::clamp(it->second, 1,
                                    settings.maximumEdgeSegments);
        }
    }
    // A regular fillet is one ribbon with a fixed transverse count.  Identify
    // the shorter opposite side pair as the cross-blend direction and apply
    // one strip-wide minimum; it may increase to satisfy geometric error, but
    // it never changes row by row.
    for (const BrepFaceNode& face : plan.graph.faces) {
        if (!face.isFillet || face.wires.empty() ||
            face.wires[0].size() != 4) {
            continue;
        }
        const std::vector<int>& wire = face.wires[0];
        int edges[4];
        double lengths[4];
        for (int i = 0; i < 4; ++i) {
            edges[i] = plan.graph.coedges[wire[i] - 1].edgeId;
            lengths[i] = plan.graph.edges[edges[i] - 1].length;
        }
        const int acrossPair =
            lengths[0] + lengths[2] <= lengths[1] + lengths[3] ? 0 : 1;
        for (int i : {acrossPair, acrossPair + 2}) {
            ideal[edges[i]] = std::max(
                ideal[edges[i]], std::max(1, settings.filletAcrossSegments));
        }
    }
    for (int eid = 1; eid <= model.edgeCount(); ++eid) {
        const int root = edgeGroups.find(eid);
        chosenByRoot[root] = std::max(chosenByRoot[root], ideal[eid]);
    }
    // Explicit edge pins are hard group constraints.  Several pins in one
    // structured group resolve to the larger value; that is deterministic and
    // preserves the strongest requested geometric bound.
    for (const auto& [eid, requested] : settings.perEdge) {
        if (eid < 1 || eid > model.edgeCount()) continue;
        const int root = edgeGroups.find(eid);
        chosenByRoot[root] = std::max(
            chosenByRoot[root],
            std::clamp(requested, 1, settings.maximumEdgeSegments));
    }

    // Packed as [unused, ideal1, chosen1, ideal2, chosen2, ...] to keep the
    // sampling function independent of the solver implementation.
    std::vector<int> solved(static_cast<size_t>(model.edgeCount() + 1) * 2, 1);
    for (int eid = 1; eid <= model.edgeCount(); ++eid) {
        solved[eid * 2] = ideal[eid];
        solved[eid * 2 + 1] = chosenByRoot[edgeGroups.find(eid)];
    }
    populateCanonicalSamples(model, plan, solved);
    plan.patches = buildPatchPlans(plan, settings);
    synchronizeTrimmedRevolutionRims(model, plan);
    return plan;
}

PolyMesh generatePrimitiveAware(const Model& model, const Analysis& analysis,
                                const GenerationSettings& settings,
                                CompilerPlan* compilerPlan,
                                GenerationReport* generationReport,
                                GenerationCache* cache) {
    return generatePrimitiveAware(model, analysis,
                                  compilerSettingsFrom(settings), settings,
                                  compilerPlan, generationReport, cache);
}

PolyMesh generatePrimitiveAware(const Model& model, const Analysis& analysis,
                                const CompilerSettings& compilerSettings,
                                const GenerationSettings& generationSettings,
                                CompilerPlan* compilerPlan,
                                GenerationReport* generationReport,
                                GenerationCache* cache) {
    CompilerPlan plan =
        planPrimitiveAware(model, analysis, compilerSettings);

    CompilerMeshBuilder native(plan);
    std::set<int> nativeFaces;
    for (const PatchPlan& patch : plan.patches) {
        // Face deletion is an edit operation, not a legacy mesher choice.
        // Respect it before the compiler backend runs; the containment pass
        // receives the same exclusion and therefore cannot add it back.
        if (generationSettings.forFace(patch.faceId).exclude) continue;
        bool built = false;
        if (patch.kind == PatchKind::PlanarNGon) {
            built = meshPlanarPatch(model, plan, patch, native);
        } else if (patch.kind == PatchKind::StructuredSurface ||
                   patch.kind == PatchKind::FilletRibbon) {
            built = meshStructuredPatch(model, plan, patch, native);
        } else if (patch.kind == PatchKind::TrimmedRevolution) {
            built = meshTrimmedRevolutionPatch(model, plan, patch, native);
        }
        if (built) nativeFaces.insert(patch.faceId);
    }
    plan.nativeFaceIds.assign(nativeFaces.begin(), nativeFaces.end());

    GenerationReport mergedReport;
    for (int fid : nativeFaces) {
        const PatchPlan& patch = plan.patches[fid - 1];
        MesherKind kind = MesherKind::CoonsGrid;
        if (patch.kind == PatchKind::PlanarNGon) {
            kind = MesherKind::MinimalNGon;
        } else if (plan.graph.faces[fid - 1].surface.type ==
                       SurfaceType::Cylinder ||
                   plan.graph.faces[fid - 1].surface.type ==
                       SurfaceType::Cone ||
                   plan.graph.faces[fid - 1].surface.type ==
                       SurfaceType::Revolution) {
            kind = MesherKind::RevolutionGrid;
        }
        mergedReport.faceMesher[fid] = kind;
        mergedReport.faceBuild[fid] = 0;
        mergedReport.faceCounts[fid] = {patch.uSegments, patch.vSegments};
        mergedReport.remeshedFaces.push_back(fid);
    }
    for (const CanonicalEdgePlan& edge : plan.edgePlans) {
        mergedReport.edgeDivisions[edge.edgeId] = edge.segmentCount;
    }

    PolyMesh mesh = std::move(native.mesh);
    if (nativeFaces.size() != plan.graph.faces.size()) {
        GenerationSettings fallbackSettings = generationSettings;
        fallbackSettings.perEdge.clear();
        for (const CanonicalEdgePlan& edge : plan.edgePlans) {
            if (!plan.graph.edges[edge.edgeId - 1].degenerate) {
                fallbackSettings.perEdge[edge.edgeId] = edge.segmentCount;
            }
        }
        // The fallback is an explicit containment boundary, not a second
        // topology planner.  It receives immutable compiler edge counts,
        // emits only unsupported faces, and skips global conform/stitch passes.
        fallbackSettings.defaults.quadDominant = false;
        fallbackSettings.defaults.pureTriFloor = true;
        fallbackSettings.conformBorders = false;
        fallbackSettings.finalizeMesh = false;
        for (int fid : nativeFaces) {
            FaceMeshSettings face = fallbackSettings.forFace(fid);
            face.exclude = true;
            face.quadDominant = false;
            face.pureTriFloor = true;
            fallbackSettings.perFace[fid] = face;
        }
        for (auto& [fid, face] : fallbackSettings.perFace) {
            if (!nativeFaces.count(fid)) {
                face.quadDominant = false;
                face.pureTriFloor = true;
                if (face.forceMesher ==
                    1 + static_cast<int>(MesherKind::QuadFill)) {
                    face.forceMesher = 0;
                }
            }
        }

        GenerationReport fallbackReport;
        const PolyMesh fallback =
            generate(model, analysis, fallbackSettings, &fallbackReport, cache);
        appendMesh(mesh, fallback);
        mergedReport.cacheHits += fallbackReport.cacheHits;
        mergedReport.cacheMisses += fallbackReport.cacheMisses;
        mergedReport.reusedFaces.insert(mergedReport.reusedFaces.end(),
                                        fallbackReport.reusedFaces.begin(),
                                        fallbackReport.reusedFaces.end());
        mergedReport.remeshedFaces.insert(
            mergedReport.remeshedFaces.end(),
            fallbackReport.remeshedFaces.begin(),
            fallbackReport.remeshedFaces.end());
        mergedReport.faceMesher.insert(fallbackReport.faceMesher.begin(),
                                       fallbackReport.faceMesher.end());
        mergedReport.faceBuild.insert(fallbackReport.faceBuild.begin(),
                                      fallbackReport.faceBuild.end());
        mergedReport.faceRims.insert(fallbackReport.faceRims.begin(),
                                     fallbackReport.faceRims.end());
        mergedReport.faceCounts.insert(fallbackReport.faceCounts.begin(),
                                       fallbackReport.faceCounts.end());
        mergedReport.faceAcross.insert(fallbackReport.faceAcross.begin(),
                                       fallbackReport.faceAcross.end());
    }

    // Native and fallback vertices are both exact compiler edge samples, but
    // they originate in separate builders.  Fuse only within the same CAD
    // body; contacting assembly parts must remain separate skins.
    std::map<int, int> solidOfFace;
    for (size_t solid = 0; solid < analysis.solidFaces.size(); ++solid) {
        for (int fid : analysis.solidFaces[solid]) {
            solidOfFace[fid] = static_cast<int>(solid);
        }
    }
    std::vector<int> vertexGroup(mesh.vertices.size(), -1);
    for (size_t p = 0; p < mesh.polygons.size(); ++p) {
        const int group = solidOfFace.count(mesh.polygonFaceId[p])
                              ? solidOfFace[mesh.polygonFaceId[p]]
                              : 0;
        for (uint32_t vertex : mesh.polygons[p]) {
            if (vertexGroup[vertex] < 0) vertexGroup[vertex] = group;
        }
    }
    weldVertices(mesh, std::max(1e-9, generationSettings.weldTolerance),
                 &vertexGroup);
    refreshCertifiedTriangulations(mesh);
    if (generationReport) *generationReport = std::move(mergedReport);
    if (compilerPlan) *compilerPlan = std::move(plan);
    return mesh;
}

const char* curveTypeName(CurveType type) {
    switch (type) {
        case CurveType::Line: return "line";
        case CurveType::Circle: return "circle";
        case CurveType::Ellipse: return "ellipse";
        case CurveType::Hyperbola: return "hyperbola";
        case CurveType::Parabola: return "parabola";
        case CurveType::Bezier: return "bezier";
        case CurveType::BSpline: return "bspline";
        case CurveType::Offset: return "offset";
        case CurveType::Other: return "other";
    }
    return "other";
}

const char* semanticEdgeTypeName(SemanticEdgeType type) {
    switch (type) {
        case SemanticEdgeType::Boundary: return "boundary";
        case SemanticEdgeType::SharpConvex: return "sharp-convex";
        case SemanticEdgeType::SharpConcave: return "sharp-concave";
        case SemanticEdgeType::Smooth: return "smooth";
        case SemanticEdgeType::SmoothFaceSplit: return "smooth-face-split";
        case SemanticEdgeType::FilletRail: return "fillet-rail";
        case SemanticEdgeType::PeriodicSeam: return "periodic-seam";
        case SemanticEdgeType::Degenerate: return "degenerate";
    }
    return "boundary";
}

const char* regionTypeName(RegionType type) {
    switch (type) {
        case RegionType::Planar: return "planar";
        case RegionType::Cylinder: return "cylinder";
        case RegionType::Cone: return "cone";
        case RegionType::Sphere: return "sphere";
        case RegionType::Torus: return "torus";
        case RegionType::Revolution: return "revolution";
        case RegionType::Extrusion: return "extrusion";
        case RegionType::Fillet: return "fillet";
        case RegionType::Freeform: return "freeform";
    }
    return "freeform";
}

const char* patchKindName(PatchKind kind) {
    switch (kind) {
        case PatchKind::PlanarNGon: return "planar-ngon";
        case PatchKind::StructuredSurface: return "structured-surface";
        case PatchKind::FilletRibbon: return "fillet-ribbon";
        case PatchKind::TrimmedRevolution: return "trimmed-revolution";
        case PatchKind::TriangleFallback: return "triangle-fallback";
    }
    return "triangle-fallback";
}

const char* patchFallbackReasonName(PatchFallbackReason reason) {
    switch (reason) {
        case PatchFallbackReason::None: return "none";
        case PatchFallbackReason::UnsupportedSurface:
            return "unsupported-surface";
        case PatchFallbackReason::MultipleTrimLoops:
            return "multiple-trim-loops";
        case PatchFallbackReason::NonFourSidedTrim:
            return "non-four-sided-trim";
        case PatchFallbackReason::OppositeCountMismatch:
            return "opposite-count-mismatch";
    }
    return "unsupported-surface";
}

}  // namespace weft
