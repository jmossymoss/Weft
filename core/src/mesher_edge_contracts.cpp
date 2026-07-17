#include "mesher_internal.hpp"

namespace weft::mesher_impl {

// Give every face mesher incident to an orthogonal trim the same immutable
// B-rep edge chain before the preview weld. This belongs at assembly scope,
// not inside one grid builder: cylinders, cones, planar slot walls, fillets
// and minimal n-gons may all own the same CAD edge.
int applyOrthogonalEdgeContracts(
    PolyMesh& mesh, const Model& model,
    const std::map<int, FacePlan>& plans,
    const std::vector<int>& solvedEdge, const PinnedEdges& pins) {
    struct BoundaryUse {
        int faceId = 0;
        size_t polygon = 0;
        size_t corner = 0;
        uint32_t a = 0;
        uint32_t b = 0;
    };
    struct Projection {
        bool valid = false;
        bool projected = false;
        double t = 0.0;
        double distance = 1e300;
        double score = 1e300;
    };
    struct Match {
        int edgeId = 0;
        BoundaryUse use;
        double ta = 0.0;
        double tb = 0.0;
        double score = 1e300;
    };
    struct FaceUvEndpoint {
        double u = 0.0;
        double v = 0.0;
        double uTolerance = 0.0;
        double vTolerance = 0.0;
        int vertexId = 0;
    };

    std::map<int, std::vector<size_t>> facePolys;
    for (size_t pi = 0; pi < mesh.polygons.size(); ++pi) {
        if (pi < mesh.polygonFaceId.size() && mesh.polygonFaceId[pi] > 0) {
            facePolys[mesh.polygonFaceId[pi]].push_back(pi);
        }
    }
    std::map<int, std::vector<BoundaryUse>> faceBoundary;
    for (const auto& [fid, polygons] : facePolys) {
        std::map<std::pair<uint32_t,uint32_t>,
                 std::vector<BoundaryUse>> uses;
        for (size_t pi : polygons) {
            const auto& poly = mesh.polygons[pi];
            for (size_t k = 0; k < poly.size(); ++k) {
                const uint32_t a = poly[k];
                const uint32_t b = poly[(k+1)%poly.size()];
                uses[{std::min(a,b),std::max(a,b)}].push_back(
                    {fid,pi,k,a,b});
            }
        }
        auto& boundary = faceBoundary[fid];
        for (auto& [edge, edgeUses] : uses) {
            (void)edge;
            if (edgeUses.size() == 1) boundary.push_back(edgeUses.front());
        }
    }
    struct BoundaryLink {
        uint32_t other = 0;
        BoundaryUse use;
    };
    std::map<int,std::map<uint32_t,std::vector<BoundaryLink>>>
        faceBoundaryGraph;
    for (const auto& [fid,boundary] : faceBoundary) {
        auto& graph = faceBoundaryGraph[fid];
        for (const BoundaryUse& use : boundary) {
            graph[use.a].push_back({use.b,use});
            graph[use.b].push_back({use.a,use});
        }
    }

    // The primitive face is only one member of a boolean cut. Its slot walls,
    // rounded caps and floor faces also share edges with each other, so the
    // contract scope must include the complete first adjacency ring rather
    // than stopping at edges directly owned by the structured face.
    std::set<int> contractFaceScope;
    for (const auto& [fid,plan] : plans) {
        if (!plan.orthogonalTrimGrid ||
            fid < 1 || fid > model.faceCount()) {
            continue;
        }
        contractFaceScope.insert(fid);
        const TopoDS_Face face = TopoDS::Face(model.faces(fid));
        for (TopExp_Explorer edgeExplorer(
                 face,TopAbs_EDGE);
             edgeExplorer.More();edgeExplorer.Next()) {
            const TopoDS_Shape& edge = edgeExplorer.Current();
            if (!model.edgeToFaces.Contains(edge)) continue;
            const auto& owners =
                model.edgeToFaces.FindFromKey(edge);
            for (const TopoDS_Shape& owner : owners) {
                const int neighbor =
                    model.faces.FindIndex(owner);
                if (neighbor > 0) contractFaceScope.insert(neighbor);
            }
        }
    }

    ShapeMap contractVertices;
    TopExp::MapShapes(model.shape, TopAbs_VERTEX, contractVertices);
    std::map<int,std::vector<FaceUvEndpoint>> faceUvEndpoints;
    std::set<int> builtFaceUvEndpoints;
    auto buildFaceUvEndpoints = [&](int fid) {
        if (!builtFaceUvEndpoints.insert(fid).second) return;
        if (fid < 1 || fid > model.faceCount()) return;
        const TopoDS_Face face = TopoDS::Face(model.faces(fid));
        BRepAdaptor_Surface surface(face);
        const double uSpan = std::abs(
            surface.LastUParameter()-surface.FirstUParameter());
        const double vSpan = std::abs(
            surface.LastVParameter()-surface.FirstVParameter());
        const double uTolerance = 8e-9*std::max(1.0,uSpan);
        const double vTolerance = 8e-9*std::max(1.0,vSpan);
        auto& endpoints = faceUvEndpoints[fid];
        auto addEndpoint = [&](const gp_Pnt2d& uv,
                               const TopoDS_Vertex& a,
                               const TopoDS_Vertex& b) {
            if (a.IsNull() && b.IsNull()) return;
            const gp_Pnt sheet = surface.Value(uv.X(),uv.Y());
            TopoDS_Vertex chosen;
            if (a.IsNull()) {
                chosen = b;
            } else if (b.IsNull()) {
                chosen = a;
            } else {
                chosen =
                    sheet.SquareDistance(BRep_Tool::Pnt(a)) <=
                            sheet.SquareDistance(BRep_Tool::Pnt(b))
                        ? a : b;
            }
            const int vertexId = contractVertices.FindIndex(chosen);
            if (vertexId <= 0) return;
            for (const FaceUvEndpoint& endpoint : endpoints) {
                if (endpoint.vertexId == vertexId &&
                    std::abs(endpoint.u-uv.X()) <= uTolerance &&
                    std::abs(endpoint.v-uv.Y()) <= vTolerance) {
                    return;
                }
            }
            endpoints.push_back(
                {uv.X(),uv.Y(),uTolerance,vTolerance,vertexId});
        };
        for (TopExp_Explorer wx(face,TopAbs_WIRE); wx.More(); wx.Next()) {
            for (BRepTools_WireExplorer wire(
                     TopoDS::Wire(wx.Current()),face);
                 wire.More(); wire.Next()) {
                const TopoDS_Edge edge = wire.Current();
                double first = 0.0, last = 0.0;
                Handle(Geom2d_Curve) pcurve =
                    BRep_Tool::CurveOnSurface(
                        edge,face,first,last);
                if (pcurve.IsNull()) continue;
                TopoDS_Vertex a, b;
                TopExp::Vertices(edge,a,b);
                addEndpoint(pcurve->Value(first),a,b);
                addEndpoint(pcurve->Value(last),a,b);
            }
        }
    };

    struct EdgeFaceProjection {
        Handle(Geom2d_Curve) pcurve;
        Handle(Geom_Curve) curve;
        double pFirst = 0.0;
        double pLast = 0.0;
        double cFirst = 0.0;
        double cLast = 0.0;
        double uvTolerance = 0.0;
        double xyzTolerance = 0.0;
        double endpointTolerance = 0.0;
        int firstVertexId = 0;
        int lastVertexId = 0;
    };
    std::map<std::pair<int,int>, EdgeFaceProjection> projectionData;
    std::map<std::tuple<int,int,uint32_t>, Projection> projectionCache;
    const char* traceContractEnv =
        std::getenv("WEFT_EDGE_CONTRACT_EID");
    const int traceContractEdge =
        traceContractEnv ? std::atoi(traceContractEnv) : 0;
    auto project = [&](int fid, int eid, uint32_t vertex) {
        const auto cacheKey = std::make_tuple(fid,eid,vertex);
        auto cached = projectionCache.find(cacheKey);
        if (cached != projectionCache.end()) return cached->second;
        Projection result;
        if (vertex >= mesh.vertices.size()) {
            projectionCache.emplace(cacheKey,result);
            return result;
        }
        const auto dataKey = std::make_pair(fid,eid);
        auto [dataIt, fresh] = projectionData.try_emplace(dataKey);
        EdgeFaceProjection& data = dataIt->second;
        if (fresh) {
            const TopoDS_Face face = TopoDS::Face(model.faces(fid));
            const TopoDS_Edge edge = TopoDS::Edge(model.edges(eid));
            data.pcurve = BRep_Tool::CurveOnSurface(
                edge,face,data.pFirst,data.pLast);
            data.curve = BRep_Tool::Curve(
                edge,data.cFirst,data.cLast);
            BRepAdaptor_Surface surface(face);
            const double span = std::max({
                1.0,
                std::abs(surface.LastUParameter()-
                         surface.FirstUParameter()),
                std::abs(surface.LastVParameter()-
                         surface.FirstVParameter())});
            data.uvTolerance = 2e-7*span;
            data.xyzTolerance = std::max({
                1e-7,4.0*BRep_Tool::Tolerance(edge),
                4.0*BRep_Tool::Tolerance(face)});
            TopoDS_Vertex firstVertex, lastVertex;
            TopExp::Vertices(edge,firstVertex,lastVertex);
            if (!data.curve.IsNull() &&
                std::abs(data.cLast-data.cFirst) > 1e-15) {
                const gp_Pnt curveFirst = data.curve->Value(data.cFirst);
                const gp_Pnt curveLast = data.curve->Value(data.cLast);
                auto nearerVertex = [&](const gp_Pnt& point) {
                    if (firstVertex.IsNull() && lastVertex.IsNull()) {
                        return 0;
                    }
                    TopoDS_Vertex chosen;
                    if (firstVertex.IsNull()) {
                        chosen = lastVertex;
                    } else if (lastVertex.IsNull()) {
                        chosen = firstVertex;
                    } else {
                        chosen =
                            point.SquareDistance(
                                BRep_Tool::Pnt(firstVertex)) <=
                                    point.SquareDistance(
                                        BRep_Tool::Pnt(lastVertex))
                                ? firstVertex : lastVertex;
                    }
                    return contractVertices.FindIndex(chosen);
                };
                data.firstVertexId = nearerVertex(curveFirst);
                data.lastVertexId = nearerVertex(curveLast);
            }
            if (eid == traceContractEdge) {
                if (!data.pcurve.IsNull()) {
                    const gp_Pnt2d uvFirst =
                        data.pcurve->Value(data.pFirst);
                    const gp_Pnt2d uvLast =
                        data.pcurve->Value(data.pLast);
                    dbg("edge contract definition e%d f%d uv "
                        "(%.12g,%.12g)->(%.12g,%.12g) vertex %d->%d",
                        eid,fid,uvFirst.X(),uvFirst.Y(),
                        uvLast.X(),uvLast.Y(),
                        data.firstVertexId,data.lastVertexId);
                }
            }
            // Imported coedge endpoint UVs and edge curves commonly disagree
            // by a few microns even though they reference the same B-rep
            // vertex. This is an endpoint-only incidence allowance, never an
            // interior nearest-edge snap.
            data.endpointTolerance =
                std::max(1e-4,data.xyzTolerance);
            if (!firstVertex.IsNull()) {
                data.endpointTolerance = std::max(
                    data.endpointTolerance,
                    4.0*BRep_Tool::Tolerance(firstVertex));
            }
            if (!lastVertex.IsNull()) {
                data.endpointTolerance = std::max(
                    data.endpointTolerance,
                    4.0*BRep_Tool::Tolerance(lastVertex));
            }
        }
        try {
            const bool hasFaceAnchor =
                vertex < mesh.anchors.size() &&
                mesh.anchors[vertex].faceId == fid;
            if (eid == traceContractEdge) {
                const auto& point = mesh.vertices[vertex];
                if (hasFaceAnchor) {
                    const Anchor& anchor = mesh.anchors[vertex];
                    dbg("edge contract input e%d f%d v%u uv=(%.12g,%.12g) "
                        "xyz=(%.12g,%.12g,%.12g)",eid,fid,vertex,
                        anchor.u,anchor.v,point[0],point[1],point[2]);
                } else {
                    dbg("edge contract input e%d f%d v%u unanchored "
                        "xyz=(%.12g,%.12g,%.12g)",eid,fid,vertex,
                        point[0],point[1],point[2]);
                }
            }
            const bool distinctEndpoints =
                data.firstVertexId > 0 &&
                data.lastVertexId > 0 &&
                data.firstVertexId != data.lastVertexId;
            if (distinctEndpoints && hasFaceAnchor) {
                buildFaceUvEndpoints(fid);
                const Anchor& anchor = mesh.anchors[vertex];
                const auto endpoints = faceUvEndpoints.find(fid);
                if (endpoints != faceUvEndpoints.end()) {
                    bool foundEndpoint = false;
                    double endpointScore = 1e300;
                    double endpointT = 0.0;
                    for (const FaceUvEndpoint& endpoint :
                         endpoints->second) {
                        const double du =
                            std::abs(anchor.u-endpoint.u);
                        const double dv =
                            std::abs(anchor.v-endpoint.v);
                        if (du > endpoint.uTolerance ||
                            dv > endpoint.vTolerance) {
                            continue;
                        }
                        double t = 0.0;
                        if (endpoint.vertexId ==
                            data.firstVertexId) {
                            t = 0.0;
                        } else if (endpoint.vertexId ==
                                   data.lastVertexId) {
                            t = 1.0;
                        } else {
                            continue;
                        }
                        const double score =
                            du/endpoint.uTolerance+
                            dv/endpoint.vTolerance;
                        if (score < endpointScore) {
                            foundEndpoint = true;
                            endpointScore = score;
                            endpointT = t;
                        }
                    }
                    if (foundEndpoint) {
                        result.valid = true;
                        result.projected = true;
                        result.t = endpointT;
                        result.distance = 0.0;
                        result.score = endpointScore;
                        if (eid == traceContractEdge) {
                            dbg("edge contract project e%d f%d v%u "
                                "topological endpoint t=%.1f score=%.4g",
                                eid,fid,vertex,result.t,result.score);
                        }
                        projectionCache.emplace(cacheKey,result);
                        return result;
                    }
                }
            }
            if (distinctEndpoints && !data.curve.IsNull()) {
                const auto& point = mesh.vertices[vertex];
                const gp_Pnt meshPoint(
                    point[0],point[1],point[2]);
                const double firstDistance =
                    meshPoint.Distance(
                        data.curve->Value(data.cFirst));
                const double lastDistance =
                    meshPoint.Distance(
                        data.curve->Value(data.cLast));
                const double endpointDistance =
                    std::min(firstDistance,lastDistance);
                // Minimal n-gons intentionally carry no face anchor. Their
                // corners can still be identified safely at a distinct CAD
                // edge endpoint, but use a tighter allowance than anchored
                // coedges so this cannot become an interior proximity snap.
                const double endpointTolerance =
                    hasFaceAnchor
                        ? data.endpointTolerance
                        : std::min(data.endpointTolerance,
                                   4.0*data.xyzTolerance);
                if (endpointDistance <= endpointTolerance) {
                    result.valid = true;
                    result.projected = true;
                    result.t =
                        firstDistance <= lastDistance ? 0.0 : 1.0;
                    result.distance = endpointDistance;
                    result.score =
                        endpointDistance/endpointTolerance;
                    if (eid == traceContractEdge) {
                        dbg("edge contract project e%d f%d v%u "
                            "tolerant endpoint t=%.1f d=%.9g "
                            "tol=%.9g",eid,fid,vertex,result.t,
                            endpointDistance,endpointTolerance);
                    }
                    projectionCache.emplace(cacheKey,result);
                    return result;
                }
            }
            bool uvIncident = false;
            double uvScore = 0.0;
            if (hasFaceAnchor &&
                !data.pcurve.IsNull() &&
                std::abs(data.pLast-data.pFirst) > 1e-15) {
                try {
                    const Anchor& anchor = mesh.anchors[vertex];
                    Geom2dAPI_ProjectPointOnCurve projection(
                        gp_Pnt2d(anchor.u,anchor.v),data.pcurve,
                        data.pFirst,data.pLast);
                    if (projection.NbPoints() > 0 &&
                        projection.LowerDistance() <= data.uvTolerance) {
                        uvIncident = true;
                        uvScore =
                            projection.LowerDistance()/data.uvTolerance;
                    }
                } catch (const Standard_Failure&) {
                    // A tolerant STEP endpoint can lie just outside the
                    // bounded pcurve interval. That must not suppress the
                    // independent 3D edge/endpoint incidence check below.
                }
            }
            // Minimal planar and a few legacy boundary meshers carry no UV
            // anchor. Their boundary points come directly from the 3D edge,
            // so a strict curve projection is the topological fallback. For
            // anchored vertices the pcurve proves incidence, but canonical t
            // still comes from the 3D curve: tolerant STEP pcurves are not
            // guaranteed to share its parameterization.
            if (!data.curve.IsNull() &&
                std::abs(data.cLast-data.cFirst) > 1e-15) {
                const auto& p = mesh.vertices[vertex];
                GeomAPI_ProjectPointOnCurve projection(
                    gp_Pnt(p[0],p[1],p[2]),data.curve,
                    data.cFirst,data.cLast);
                if (projection.NbPoints() > 0) {
                    const double t = std::clamp(
                        (projection.LowerDistanceParameter()-data.cFirst)/
                        (data.cLast-data.cFirst),0.0,1.0);
                    const double distance = projection.LowerDistance();
                    result.projected = true;
                    result.t = t;
                    result.distance = distance;
                    const bool endpointFallback =
                        hasFaceAnchor && !uvIncident &&
                        (t <= 1e-6 || t >= 1.0-1e-6) &&
                        distance <= data.endpointTolerance;
                    const bool admitted =
                        (uvIncident && distance <=
                                           data.endpointTolerance) ||
                        (!hasFaceAnchor &&
                         distance <= data.xyzTolerance) ||
                        endpointFallback;
                    if (eid == traceContractEdge) {
                        dbg("edge contract project e%d f%d v%u anchor=%d "
                            "uv=%d t=%.9g d=%.9g xyzTol=%.9g endTol=%.9g "
                            "endpoint=%d admitted=%d",eid,fid,vertex,
                            hasFaceAnchor?1:0,uvIncident?1:0,t,distance,
                            data.xyzTolerance,data.endpointTolerance,
                            endpointFallback?1:0,admitted?1:0);
                    }
                    if (admitted) {
                        result.valid = true;
                        result.score =
                            distance/
                                (endpointFallback
                                     ? data.endpointTolerance
                                     : data.xyzTolerance)+
                            uvScore;
                    }
                }
            }
        } catch (const Standard_Failure&) {
        }
        projectionCache.emplace(cacheKey,result);
        return result;
    };

    using SegmentKey = std::tuple<int,uint32_t,uint32_t>;
    std::map<SegmentKey,Match> bestMatch;
    int routedSegments = 0;
    for (int eid = 1; eid <= model.edgeCount(); ++eid) {
        const TopoDS_Edge edge = TopoDS::Edge(model.edges(eid));
        if (BRep_Tool::Degenerated(edge) ||
            !model.edgeToFaces.Contains(edge)) {
            continue;
        }
        const auto& owners = model.edgeToFaces.FindFromKey(edge);
        if (owners.Extent() != 2) continue;
        const int fa = model.faces.FindIndex(owners.First());
        const int fb = model.faces.FindIndex(owners.Last());
        if (fa < 1 || fb < 1 || fa == fb) continue;
        const auto pa = plans.find(fa);
        const auto pb = plans.find(fb);
        const bool touchesStructured =
            (pa != plans.end() && pa->second.orthogonalTrimGrid) ||
            (pb != plans.end() && pb->second.orthogonalTrimGrid);
        const bool connectsStructuredRing =
            contractFaceScope.count(fa) &&
            contractFaceScope.count(fb);
        if (!touchesStructured && !connectsStructuredRing) {
            continue;
        }
        const bool routeDrumBoundary =
            (pa != plans.end() && pa->second.orthogonalDrumComb &&
             pa->second.kind == MesherKind::RevolutionGrid) ||
            (pb != plans.end() && pb->second.orthogonalDrumComb &&
             pb->second.kind == MesherKind::RevolutionGrid);
        double edgeLength = 0.0;
        if (routeDrumBoundary) {
            try {
                BRepAdaptor_Curve curve(edge);
                edgeLength = GCPnts_AbscissaPoint::Length(curve);
            } catch (const Standard_Failure&) {
                edgeLength = 0.0;
            }
        }
        for (int fid : {fa,fb}) {
            auto boundary = faceBoundary.find(fid);
            if (boundary == faceBoundary.end()) continue;
            bool matchedSegment = false;
            for (const BoundaryUse& use : boundary->second) {
                const Projection a = project(fid,eid,use.a);
                const Projection b = project(fid,eid,use.b);
                if (eid == traceContractEdge &&
                    (a.valid || b.valid)) {
                    dbg("edge contract e%d f%d p%zu:%zu v%u %.6f/%d "
                        "-> v%u %.6f/%d score %.4g/%.4g",
                        eid,fid,use.polygon,use.corner,use.a,a.t,
                        a.valid?1:0,use.b,b.t,b.valid?1:0,
                        a.score,b.score);
                }
                if (!a.valid || !b.valid ||
                    std::abs(a.t-b.t) <= 1e-10) {
                    continue;
                }
                const double score = a.score+b.score;
                const SegmentKey key{
                    fid,std::min(use.a,use.b),std::max(use.a,use.b)};
                auto found = bestMatch.find(key);
                if (found == bestMatch.end() ||
                    score < found->second.score) {
                    bestMatch[key] =
                        {eid,use,a.t,b.t,score};
                }
                matchedSegment = true;
            }
            if (matchedSegment || !routeDrumBoundary ||
                edgeLength <= 1e-12) {
                continue;
            }
            const auto graphIt = faceBoundaryGraph.find(fid);
            if (graphIt == faceBoundaryGraph.end()) continue;
            const auto& graph = graphIt->second;
            std::vector<uint32_t> firstCandidates;
            std::vector<uint32_t> lastCandidates;
            for (const auto& [vertex,links] : graph) {
                if (links.empty()) continue;
                const Projection endpoint = project(fid,eid,vertex);
                if (endpoint.valid && endpoint.t <= 1e-7) {
                    firstCandidates.push_back(vertex);
                }
                if (endpoint.valid && endpoint.t >= 1.0-1e-7) {
                    lastCandidates.push_back(vertex);
                }
            }
            std::map<uint32_t,Projection> recoveredFirst;
            std::map<uint32_t,Projection> recoveredLast;
            auto recoverDrumEndpoint =
                [&](bool first,std::vector<uint32_t>& candidates,
                    std::map<uint32_t,Projection>& recovered) {
                    if (!candidates.empty()) return;
                    const auto planIt = plans.find(fid);
                    if (planIt == plans.end() ||
                        !planIt->second.orthogonalDrumComb ||
                        planIt->second.kind !=
                            MesherKind::RevolutionGrid ||
                        planIt->second.orthogonalDriverU <= 0 ||
                        planIt->second.orthogonalDriverU >=
                            int(solvedEdge.size())) {
                        return;
                    }
                    const auto dataIt =
                        projectionData.find({fid,eid});
                    if (dataIt == projectionData.end()) return;
                    const EdgeFaceProjection& data = dataIt->second;
                    if (data.pcurve.IsNull() || data.curve.IsNull() ||
                        std::abs(data.pLast-data.pFirst) <= 1e-15 ||
                        std::abs(data.cLast-data.cFirst) <= 1e-15) {
                        return;
                    }
                    try {
                        const TopoDS_Face drumFace =
                            TopoDS::Face(model.faces(fid));
                        BRepAdaptor_Surface surface(drumFace);
                        const double uSpan = std::abs(
                            surface.LastUParameter()-
                            surface.FirstUParameter());
                        const double vSpan = std::abs(
                            surface.LastVParameter()-
                            surface.FirstVParameter());
                        const int divisions = std::max(
                            1,solvedEdge[
                                planIt->second.orthogonalDriverU]);
                        const double uPitch = uSpan/divisions;
                        if (uPitch <= 1e-15) return;
                        const gp_Pnt curveEndpoint = data.curve->Value(
                            first ? data.cFirst : data.cLast);
                        const gp_Pnt2d pcurveFirst =
                            data.pcurve->Value(data.pFirst);
                        const gp_Pnt2d pcurveLast =
                            data.pcurve->Value(data.pLast);
                        const gp_Pnt firstSheet = surface.Value(
                            pcurveFirst.X(),pcurveFirst.Y());
                        const gp_Pnt lastSheet = surface.Value(
                            pcurveLast.X(),pcurveLast.Y());
                        const gp_Pnt2d targetUv =
                            curveEndpoint.SquareDistance(firstSheet) <=
                                    curveEndpoint.SquareDistance(lastSheet)
                                ? pcurveFirst : pcurveLast;
                        gp_Pnt derivativePoint;
                        gp_Vec derivativeU,derivativeV;
                        surface.D1(targetUv.X(),targetUv.Y(),
                                   derivativePoint,derivativeU,
                                   derivativeV);
                        if (derivativeU.Magnitude() <= 1e-15) return;
                        const double vTolerance = std::max(
                            4.0*data.uvTolerance,2e-6*vSpan);
                        const double uTolerance = 0.51*uPitch;
                        const double xyzTolerance =
                            0.55*derivativeU.Magnitude()*uPitch+
                            data.endpointTolerance;
                        const bool periodic = surface.IsUPeriodic();
                        const double period =
                            periodic ? surface.UPeriod() : 0.0;
                        auto uDistance = [&](double a,double b) {
                            double distance = std::abs(a-b);
                            if (periodic && period > 1e-15) {
                                distance = std::fmod(distance,period);
                                distance =
                                    std::min(distance,period-distance);
                            }
                            return distance;
                        };
                        struct Candidate {
                            uint32_t vertex = 0;
                            Projection projection;
                        };
                        std::vector<Candidate> possible;
                        for (const auto& [vertex,links] : graph) {
                            if (links.size() != 2 ||
                                vertex >= mesh.vertices.size() ||
                                vertex >= mesh.anchors.size() ||
                                mesh.anchors[vertex].faceId != fid) {
                                continue;
                            }
                            const Anchor& anchor = mesh.anchors[vertex];
                            const double du =
                                uDistance(anchor.u,targetUv.X());
                            const double dv =
                                std::abs(anchor.v-targetUv.Y());
                            if (du > uTolerance || dv > vTolerance) {
                                continue;
                            }
                            const auto& point = mesh.vertices[vertex];
                            const double xyzDistance =
                                gp_Pnt(point[0],point[1],point[2]).
                                    Distance(curveEndpoint);
                            if (xyzDistance > xyzTolerance) continue;
                            Projection synthetic;
                            synthetic.valid = true;
                            synthetic.projected = true;
                            synthetic.t = first ? 0.0 : 1.0;
                            synthetic.distance = xyzDistance;
                            synthetic.score =
                                du/uTolerance+
                                dv/vTolerance+
                                xyzDistance/xyzTolerance;
                            possible.push_back({vertex,synthetic});
                        }
                        std::sort(
                            possible.begin(),possible.end(),
                            [](const Candidate& a,const Candidate& b) {
                                return a.projection.score <
                                       b.projection.score;
                            });
                        if (possible.empty()) return;
                        if (possible.size() > 1 &&
                            possible[1].projection.score <=
                                1.2*possible[0].projection.score+
                                1e-9) {
                            if (eid == traceContractEdge) {
                                dbg("edge contract route endpoint e%d f%d "
                                    "ambiguous %.6g / %.6g",eid,fid,
                                    possible[0].projection.score,
                                    possible[1].projection.score);
                            }
                            return;
                        }
                        candidates.push_back(possible[0].vertex);
                        recovered[possible[0].vertex] =
                            possible[0].projection;
                        if (eid == traceContractEdge) {
                            dbg("edge contract route endpoint e%d f%d "
                                "recovered v%u t=%.1f score %.6g "
                                "distance %.6g",eid,fid,
                                possible[0].vertex,
                                possible[0].projection.t,
                                possible[0].projection.score,
                                possible[0].projection.distance);
                        }
                    } catch (const Standard_Failure&) {
                    }
                };
            recoverDrumEndpoint(
                true,firstCandidates,recoveredFirst);
            recoverDrumEndpoint(
                false,lastCandidates,recoveredLast);
            if (eid == traceContractEdge) {
                dbg("edge contract route candidates e%d f%d: %zu first, "
                    "%zu last, %zu graph vertices",eid,fid,
                    firstCandidates.size(),lastCandidates.size(),
                    graph.size());
            }
            struct BoundaryRoute {
                std::vector<uint32_t> vertices;
                std::vector<BoundaryUse> uses;
                std::vector<double> parameters;
                double score = 1e300;
                bool valid = false;
            };
            // A clipped coarse cell can contain a boundary corner for which
            // OCCT's bounded curve projector returns no point at all. That
            // must never make the corner an ordinary edge match, but a local
            // route whose two CAD endpoint identities are already proven
            // still needs a parameter/distance for whole-path validation.
            // Approximate only that diagnostic projection against a dense
            // polyline of the source curve; the ordinary project() admission
            // rules above remain unchanged.
            std::map<uint32_t,Projection> routeProjectionCache;
            auto routeProjection = [&](uint32_t vertex) {
                Projection result = project(fid,eid,vertex);
                if (result.projected ||
                    vertex >= mesh.vertices.size()) {
                    return result;
                }
                const auto dataIt =
                    projectionData.find({fid,eid});
                if (dataIt == projectionData.end() ||
                    dataIt->second.curve.IsNull() ||
                    std::abs(dataIt->second.cLast-
                             dataIt->second.cFirst) <= 1e-15) {
                    return result;
                }
                const auto cached =
                    routeProjectionCache.find(vertex);
                if (cached != routeProjectionCache.end()) {
                    return cached->second;
                }
                const EdgeFaceProjection& data = dataIt->second;
                const gp_Pnt target(
                    mesh.vertices[vertex][0],
                    mesh.vertices[vertex][1],
                    mesh.vertices[vertex][2]);
                const int curveSamples = std::max(
                    64,8*(eid < int(solvedEdge.size())
                              ? std::max(1,solvedEdge[eid]) : 1));
                double bestDistance = 1e300;
                double bestT = 0.0;
                gp_Pnt previous =
                    data.curve->Value(data.cFirst);
                for (int q = 0; q < curveSamples; ++q) {
                    const double t1 =
                        double(q+1)/curveSamples;
                    const gp_Pnt next = data.curve->Value(
                        data.cFirst+
                        t1*(data.cLast-data.cFirst));
                    const gp_Vec segment(previous,next);
                    const double length2 =
                        segment.SquareMagnitude();
                    double alpha = 0.0;
                    if (length2 > 1e-30) {
                        alpha = std::clamp(
                            gp_Vec(previous,target).Dot(segment)/
                                length2,
                            0.0,1.0);
                    }
                    const gp_Pnt closest =
                        previous.Translated(segment*alpha);
                    const double distance =
                        closest.Distance(target);
                    if (distance < bestDistance) {
                        bestDistance = distance;
                        bestT =
                            (q+alpha)/curveSamples;
                    }
                    previous = next;
                }
                if (bestDistance < 1e299) {
                    result.projected = true;
                    result.valid = false;
                    result.t = std::clamp(bestT,0.0,1.0);
                    result.distance = bestDistance;
                    result.score =
                        bestDistance/
                        std::max(1e-12,edgeLength);
                    if (eid == traceContractEdge) {
                        dbg("edge contract route-only projection e%d f%d "
                            "v%u t=%.9g d=%.9g",eid,fid,vertex,
                            result.t,result.distance);
                    }
                }
                routeProjectionCache[vertex] = result;
                return result;
            };
            auto followRoute =
                [&](uint32_t start,uint32_t next,uint32_t goal) {
                    BoundaryRoute route;
                    route.vertices.push_back(start);
                    uint32_t previous = start;
                    uint32_t current = next;
                    for (size_t steps = 0;
                         steps <= graph.size()+1; ++steps) {
                        const auto at = graph.find(previous);
                        if (at == graph.end()) return BoundaryRoute{};
                        const BoundaryLink* traversed = nullptr;
                        for (const BoundaryLink& link : at->second) {
                            if (link.other == current) {
                                traversed = &link;
                                break;
                            }
                        }
                        if (!traversed) return BoundaryRoute{};
                        route.uses.push_back(traversed->use);
                        route.vertices.push_back(current);
                        if (current == goal) return route;
                        const auto onward = graph.find(current);
                        if (onward == graph.end() ||
                            onward->second.size() != 2) {
                            return BoundaryRoute{};
                        }
                        uint32_t following = UINT32_MAX;
                        for (const BoundaryLink& link : onward->second) {
                            if (link.other != previous) {
                                following = link.other;
                                break;
                            }
                        }
                        if (following == UINT32_MAX ||
                            following == start) {
                            return BoundaryRoute{};
                        }
                        previous = current;
                        current = following;
                    }
                    return BoundaryRoute{};
                };
            auto assessRoute = [&](BoundaryRoute route,
                                   bool allowCoarseBacktrack) {
                if (route.uses.empty() ||
                    route.vertices.size() != route.uses.size()+1) {
                    return BoundaryRoute{};
                }
                double pathLength = 0.0;
                double sumDistance = 0.0;
                double maxDistance = 0.0;
                double previousT = -1.0;
                double rawPreviousT = -1.0;
                double totalBacktrack = 0.0;
                double maxBacktrack = 0.0;
                route.parameters.reserve(route.vertices.size());
                std::vector<Projection> routeProjected(
                    route.vertices.size());
                std::vector<char> synthetic(
                    route.vertices.size(),0);
                for (size_t i = 0; i < route.vertices.size(); ++i) {
                    const uint32_t vertex = route.vertices[i];
                    Projection projected =
                        project(fid,eid,vertex);
                    if (i == 0) {
                        const auto recovered =
                            recoveredFirst.find(vertex);
                        if (recovered != recoveredFirst.end()) {
                            projected = recovered->second;
                        }
                    } else if (i+1 == route.vertices.size()) {
                        const auto recovered =
                            recoveredLast.find(vertex);
                        if (recovered != recoveredLast.end()) {
                            projected = recovered->second;
                        }
                    }
                    if (!projected.projected &&
                        allowCoarseBacktrack) {
                        projected = routeProjection(vertex);
                        synthetic[i] = projected.projected ? 1 : 0;
                    }
                    if (!projected.projected) {
                        if (eid == traceContractEdge) {
                            dbg("edge contract route reject e%d f%d: "
                                "unprojected v%u",eid,fid,vertex);
                        }
                        return BoundaryRoute{};
                    }
                    routeProjected[i] = projected;
                }
                if (allowCoarseBacktrack) {
                    for (size_t firstSynthetic = 0;
                         firstSynthetic < synthetic.size();) {
                        if (!synthetic[firstSynthetic]) {
                            ++firstSynthetic;
                            continue;
                        }
                        size_t afterSynthetic = firstSynthetic;
                        while (afterSynthetic < synthetic.size() &&
                               synthetic[afterSynthetic]) {
                            ++afterSynthetic;
                        }
                        if (firstSynthetic == 0 ||
                            afterSynthetic >= synthetic.size()) {
                            return BoundaryRoute{};
                        }
                        const size_t before = firstSynthetic-1;
                        double runLength = 0.0;
                        std::vector<double> cumulative(
                            afterSynthetic-before+1,0.0);
                        for (size_t k = before+1;
                             k <= afterSynthetic; ++k) {
                            const auto& a =
                                mesh.vertices[route.vertices[k-1]];
                            const auto& b =
                                mesh.vertices[route.vertices[k]];
                            runLength +=
                                gp_Pnt(a[0],a[1],a[2]).Distance(
                                    gp_Pnt(b[0],b[1],b[2]));
                            cumulative[k-before] = runLength;
                        }
                        if (runLength <= 1e-15) {
                            return BoundaryRoute{};
                        }
                        const double t0 =
                            routeProjected[before].t;
                        const double t1 =
                            routeProjected[afterSynthetic].t;
                        for (size_t k = firstSynthetic;
                             k < afterSynthetic; ++k) {
                            const double fraction =
                                cumulative[k-before]/runLength;
                            routeProjected[k].t =
                                t0+fraction*(t1-t0);
                            if (eid == traceContractEdge) {
                                dbg("edge contract route interpolate e%d "
                                    "f%d v%u t=%.9g between %.9g/%.9g",
                                    eid,fid,route.vertices[k],
                                    routeProjected[k].t,t0,t1);
                            }
                        }
                        firstSynthetic = afterSynthetic;
                    }
                }
                for (size_t i = 0; i < route.vertices.size(); ++i) {
                    const uint32_t vertex = route.vertices[i];
                    const Projection& projected =
                        routeProjected[i];
                    const double rawT = projected.t;
                    if (i > 0 && rawT < rawPreviousT) {
                        const double backtrack = rawPreviousT-rawT;
                        totalBacktrack += backtrack;
                        maxBacktrack =
                            std::max(maxBacktrack,backtrack);
                        if (!allowCoarseBacktrack &&
                            rawT+0.02 < rawPreviousT) {
                            if (eid == traceContractEdge) {
                                dbg("edge contract route reject e%d f%d: "
                                    "backtrack %.6g -> %.6g",eid,fid,
                                    rawPreviousT,rawT);
                            }
                            return BoundaryRoute{};
                        }
                    }
                    rawPreviousT = rawT;
                    double t = rawT;
                    if (i > 0) t = std::max(t,previousT);
                    route.parameters.push_back(t);
                    previousT = t;
                    sumDistance += projected.distance;
                    maxDistance =
                        std::max(maxDistance,projected.distance);
                    if (i > 0) {
                        const auto& a =
                            mesh.vertices[route.vertices[i-1]];
                        const auto& b = mesh.vertices[vertex];
                        pathLength +=
                            gp_Pnt(a[0],a[1],a[2]).Distance(
                                gp_Pnt(b[0],b[1],b[2]));
                    }
                }
                if (route.parameters.front() > 1e-6 ||
                    route.parameters.back() < 1.0-1e-6) {
                    if (eid == traceContractEdge) {
                        dbg("edge contract route reject e%d f%d: "
                            "endpoint params %.6g -> %.6g",eid,fid,
                            route.parameters.front(),
                            route.parameters.back());
                    }
                    return BoundaryRoute{};
                }
                // A coarse clipped boundary may project one corner slightly
                // behind its predecessor even though the complete local path
                // is the unique short arc between two proven B-rep endpoint
                // identities. Keep that fallback narrow: a route that turns
                // back by a material fraction of the CAD edge is not the
                // source edge and remains rejected.
                if (allowCoarseBacktrack &&
                    (maxBacktrack > 0.25 ||
                     totalBacktrack > 0.50)) {
                    if (eid == traceContractEdge) {
                        dbg("edge contract route reject e%d f%d: coarse "
                            "backtrack max %.6g total %.6g",eid,fid,
                            maxBacktrack,totalBacktrack);
                    }
                    return BoundaryRoute{};
                }
                const double meanDistance =
                    sumDistance/route.vertices.size();
                // This is a verifier for a local route already emitted by
                // the drum mesher, not a license to pull an arbitrary face
                // border onto a nearby edge. The accepted route must be
                // monotone, comparable in length to the CAD edge, and much
                // closer than a trip around the rest of the trim loop.
                if (pathLength > 2.5*edgeLength ||
                    maxDistance > 0.45*edgeLength ||
                    meanDistance > 0.25*edgeLength) {
                    if (eid == traceContractEdge) {
                        dbg("edge contract route reject e%d f%d: "
                            "path %.6g edge %.6g maxD %.6g meanD %.6g",
                            eid,fid,pathLength,edgeLength,
                            maxDistance,meanDistance);
                    }
                    return BoundaryRoute{};
                }
                route.score =
                    pathLength/edgeLength+
                    4.0*meanDistance/edgeLength+
                    maxDistance/edgeLength+
                    2.0*totalBacktrack;
                route.valid = true;
                return route;
            };
            BoundaryRoute bestRoute;
            for (uint32_t firstVertex : firstCandidates) {
                const auto start = graph.find(firstVertex);
                if (start == graph.end() ||
                    start->second.empty()) {
                    continue;
                }
                for (uint32_t lastVertex : lastCandidates) {
                    if (firstVertex == lastVertex) continue;
                    for (const BoundaryLink& firstLink :
                         start->second) {
                        BoundaryRoute route = assessRoute(
                            followRoute(firstVertex,
                                        firstLink.other,
                                        lastVertex),
                            false);
                        if (route.valid &&
                            route.score < bestRoute.score) {
                            bestRoute = std::move(route);
                        }
                    }
                }
            }
            // A clipped low-density cell can make the face boundary graph
            // branch at an exact CAD endpoint. The degree-two walk above then
            // stops before the goal even though a short monotone route exists.
            // Search only when that simple walk found nothing, and retain the
            // same geometric verifier used by ordinary drum routes.
            if (!bestRoute.valid) {
                size_t exploredRoutes = 0;
                constexpr size_t kMaxExploredRoutes = 4096;
                for (uint32_t firstVertex : firstCandidates) {
                    if (exploredRoutes >= kMaxExploredRoutes) break;
                    const auto start = graph.find(firstVertex);
                    if (start == graph.end() ||
                        start->second.empty()) {
                        continue;
                    }
                    for (uint32_t lastVertex : lastCandidates) {
                        if (exploredRoutes >= kMaxExploredRoutes ||
                            firstVertex == lastVertex) {
                            continue;
                        }
                        Projection firstProjection =
                            project(fid,eid,firstVertex);
                        const auto recovered =
                            recoveredFirst.find(firstVertex);
                        if (recovered != recoveredFirst.end()) {
                            firstProjection = recovered->second;
                        }
                        if (!firstProjection.projected) continue;
                        BoundaryRoute route;
                        route.vertices.push_back(firstVertex);
                        std::set<uint32_t> visited{firstVertex};
                        std::function<void(uint32_t,double,double)> search;
                        search = [&](uint32_t current,double previousT,
                                     double pathLength) {
                            if (exploredRoutes >= kMaxExploredRoutes ||
                                route.vertices.size() > graph.size()+1) {
                                return;
                            }
                            if (current == lastVertex) {
                                ++exploredRoutes;
                                BoundaryRoute assessed =
                                    assessRoute(route,false);
                                if (assessed.valid &&
                                    assessed.score < bestRoute.score) {
                                    bestRoute = std::move(assessed);
                                }
                                return;
                            }
                            const auto at = graph.find(current);
                            if (at == graph.end()) return;
                            for (const BoundaryLink& link : at->second) {
                                const uint32_t next = link.other;
                                if (visited.count(next)) continue;
                                Projection projected =
                                    project(fid,eid,next);
                                if (next == lastVertex) {
                                    const auto endRecovered =
                                        recoveredLast.find(next);
                                    if (endRecovered !=
                                        recoveredLast.end()) {
                                        projected =
                                            endRecovered->second;
                                    }
                                }
                                if (!projected.projected ||
                                    projected.t+0.02 < previousT ||
                                    projected.distance >
                                        0.45*edgeLength) {
                                    continue;
                                }
                                const auto& a = mesh.vertices[current];
                                const auto& b = mesh.vertices[next];
                                const double segmentLength =
                                    gp_Pnt(a[0],a[1],a[2]).Distance(
                                        gp_Pnt(b[0],b[1],b[2]));
                                const double nextLength =
                                    pathLength+segmentLength;
                                if (nextLength > 2.5*edgeLength) {
                                    continue;
                                }
                                visited.insert(next);
                                route.uses.push_back(link.use);
                                route.vertices.push_back(next);
                                search(next,std::max(previousT,projected.t),
                                       nextLength);
                                route.vertices.pop_back();
                                route.uses.pop_back();
                                visited.erase(next);
                            }
                        };
                        search(firstVertex,firstProjection.t,0.0);
                    }
                }
                if (eid == traceContractEdge && bestRoute.valid) {
                    dbg("edge contract branched route e%d f%d: %zu "
                        "segments after %zu candidates",eid,fid,
                        bestRoute.uses.size(),exploredRoutes);
                }
            }
            // Last-resort route for a coarse clipped drum boundary. Exact
            // endpoint identity is already proven by first/lastCandidates,
            // but the nearest-point parameter of one coarse corner can step
            // backwards and make the monotone DFS above prune the correct
            // arc. Enumerate only geometrically local paths, assess them with
            // the same length/distance limits, and accept solely when the
            // best path is unambiguous. No station or column is added.
            if (!bestRoute.valid) {
                std::vector<BoundaryRoute> localRoutes;
                size_t exploredRoutes = 0;
                constexpr size_t kMaxLocalRoutes = 4096;
                for (uint32_t firstVertex : firstCandidates) {
                    if (exploredRoutes >= kMaxLocalRoutes) break;
                    const auto start = graph.find(firstVertex);
                    if (start == graph.end() ||
                        start->second.empty()) {
                        continue;
                    }
                    for (uint32_t lastVertex : lastCandidates) {
                        if (exploredRoutes >= kMaxLocalRoutes ||
                            firstVertex == lastVertex) {
                            continue;
                        }
                        BoundaryRoute route;
                        route.vertices.push_back(firstVertex);
                        std::set<uint32_t> visited{firstVertex};
                        std::function<void(uint32_t,double)> searchLocal;
                        searchLocal = [&](uint32_t current,
                                          double pathLength) {
                            if (exploredRoutes >= kMaxLocalRoutes ||
                                route.vertices.size() > graph.size()+1) {
                                return;
                            }
                            if (current == lastVertex) {
                                ++exploredRoutes;
                                BoundaryRoute assessed =
                                    assessRoute(route,true);
                                if (assessed.valid) {
                                    localRoutes.push_back(
                                        std::move(assessed));
                                }
                                return;
                            }
                            const auto at = graph.find(current);
                            if (at == graph.end()) return;
                            for (const BoundaryLink& link : at->second) {
                                const uint32_t next = link.other;
                                if (visited.count(next)) continue;
                                Projection projected =
                                    routeProjection(next);
                                if (next == lastVertex) {
                                    const auto recovered =
                                        recoveredLast.find(next);
                                    if (recovered !=
                                        recoveredLast.end()) {
                                        projected = recovered->second;
                                    }
                                }
                                if (!projected.projected ||
                                    projected.distance >
                                        0.45*edgeLength) {
                                    continue;
                                }
                                const auto& a = mesh.vertices[current];
                                const auto& b = mesh.vertices[next];
                                const double segmentLength =
                                    gp_Pnt(a[0],a[1],a[2]).Distance(
                                        gp_Pnt(b[0],b[1],b[2]));
                                const double nextLength =
                                    pathLength+segmentLength;
                                if (nextLength > 2.5*edgeLength) {
                                    continue;
                                }
                                visited.insert(next);
                                route.uses.push_back(link.use);
                                route.vertices.push_back(next);
                                searchLocal(next,nextLength);
                                route.vertices.pop_back();
                                route.uses.pop_back();
                                visited.erase(next);
                            }
                        };
                        searchLocal(firstVertex,0.0);
                    }
                }
                std::sort(
                    localRoutes.begin(),localRoutes.end(),
                    [](const BoundaryRoute& a,
                       const BoundaryRoute& b) {
                        return a.score < b.score;
                    });
                const bool unique =
                    !localRoutes.empty() &&
                    (localRoutes.size() == 1 ||
                     localRoutes[1].score >
                         1.10*localRoutes[0].score+0.02);
                if (unique) {
                    bestRoute = std::move(localRoutes.front());
                    if (eid == traceContractEdge) {
                        dbg("edge contract coarse local route e%d f%d: "
                            "%zu segments, score %.6g, %zu candidates",
                            eid,fid,bestRoute.uses.size(),
                            bestRoute.score,localRoutes.size());
                    }
                } else if (eid == traceContractEdge &&
                           !localRoutes.empty()) {
                    dbg("edge contract coarse local route e%d f%d "
                        "ambiguous %.6g / %.6g",eid,fid,
                        localRoutes[0].score,
                        localRoutes.size() > 1
                            ? localRoutes[1].score : 0.0);
                    for (size_t ri = 0;
                         ri < std::min<size_t>(4,localRoutes.size());
                         ++ri) {
                        std::ostringstream routeText;
                        for (uint32_t vertex :
                             localRoutes[ri].vertices) {
                            if (routeText.tellp() > 0) {
                                routeText << "->";
                            }
                            routeText << 'v' << vertex;
                        }
                        dbg("edge contract coarse candidate e%d f%d "
                            "#%zu score %.6g %s",eid,fid,ri,
                            localRoutes[ri].score,
                            routeText.str().c_str());
                    }
                }
            }
            if (!bestRoute.valid) continue;
            std::map<uint32_t,double> routeParameter;
            for (size_t i = 0; i < bestRoute.vertices.size(); ++i) {
                routeParameter[bestRoute.vertices[i]] =
                    bestRoute.parameters[i];
            }
            for (const BoundaryUse& use : bestRoute.uses) {
                const auto ta = routeParameter.find(use.a);
                const auto tb = routeParameter.find(use.b);
                if (ta == routeParameter.end() ||
                    tb == routeParameter.end() ||
                    std::abs(ta->second-tb->second) <= 1e-10) {
                    continue;
                }
                const SegmentKey key{
                    fid,std::min(use.a,use.b),std::max(use.a,use.b)};
                bestMatch[key] =
                    {eid,use,ta->second,tb->second,bestRoute.score};
                ++routedSegments;
            }
            dbg("edge contract route e%d f%d: %zu segments, "
                "score %.6g",eid,fid,bestRoute.uses.size(),
                bestRoute.score);
        }
    }
    std::map<int,std::vector<Match>> matchesByEdge;
    for (const auto& [key,match] : bestMatch) {
        (void)key;
        matchesByEdge[match.edgeId].push_back(match);
    }

    struct CanonicalPoint {
        double t = 0.0;
        uint32_t vertex = 0;
    };
    std::map<int,std::vector<CanonicalPoint>> canonical;
    std::map<uint32_t,uint32_t> remap;
    std::map<int,uint32_t> brepVertexRepresentative;
    int created = 0;
    for (auto& [eid,matches] : matchesByEdge) {
        const TopoDS_Edge edge = TopoDS::Edge(model.edges(eid));
        double first = 0.0, last = 0.0;
        Handle(Geom_Curve) curve =
            BRep_Tool::Curve(edge,first,last);
        if (curve.IsNull() || std::abs(last-first) <= 1e-15) continue;
        TopoDS_Vertex edgeA, edgeB;
        TopExp::Vertices(edge,edgeA,edgeB);
        auto endpointVertexId = [&](double parameter) {
            if (edgeA.IsNull() && edgeB.IsNull()) return 0;
            TopoDS_Vertex chosen;
            if (edgeA.IsNull()) {
                chosen = edgeB;
            } else if (edgeB.IsNull()) {
                chosen = edgeA;
            } else {
                const gp_Pnt point = curve->Value(parameter);
                chosen =
                    point.SquareDistance(BRep_Tool::Pnt(edgeA)) <=
                            point.SquareDistance(BRep_Tool::Pnt(edgeB))
                        ? edgeA : edgeB;
            }
            return contractVertices.FindIndex(chosen);
        };
        const int firstEndpointId = endpointVertexId(first);
        const int lastEndpointId = endpointVertexId(last);
        const bool distinctEndpoints =
            firstEndpointId > 0 && lastEndpointId > 0 &&
            firstEndpointId != lastEndpointId;
        std::vector<double> stations{0.0,1.0};
        const int divisions =
            eid < int(solvedEdge.size())
                ? std::max(1,solvedEdge[eid]) : 1;
        const std::vector<double> requested =
            edgeSampleFractions(eid,divisions,0.0,false,true,
                                &pins,&model);
        stations.insert(stations.end(),requested.begin(),requested.end());
        for (const Match& match : matches) {
            stations.push_back(match.ta);
            stations.push_back(match.tb);
        }
        std::sort(stations.begin(),stations.end());
        stations.erase(std::unique(
            stations.begin(),stations.end(),
            [](double a,double b){return std::abs(a-b)<=1e-8;}),
            stations.end());

        auto& chain = canonical[eid];
        chain.reserve(stations.size());
        for (double t : stations) {
            uint32_t representative = UINT32_MAX;
            const int brepVertexId =
                distinctEndpoints && t <= 1e-7
                    ? firstEndpointId
                    : distinctEndpoints && t >= 1.0-1e-7
                          ? lastEndpointId : 0;
            if (brepVertexId > 0) {
                const auto existing =
                    brepVertexRepresentative.find(brepVertexId);
                if (existing != brepVertexRepresentative.end()) {
                    representative = existing->second;
                }
            }
            // Prefer a representative already chosen by a previous edge at
            // this junction. A flat one-step remap can otherwise pick a fresh
            // duplicate first and split a multi-edge B-rep corner again.
            for (const Match& match : matches) {
                for (const auto& endpoint :
                     {std::make_pair(match.ta,match.use.a),
                      std::make_pair(match.tb,match.use.b)}) {
                    if (std::abs(endpoint.first-t) <= 1e-7) {
                        auto existing = remap.find(endpoint.second);
                        if (existing != remap.end()) {
                            representative = existing->second;
                        } else if (representative == UINT32_MAX) {
                            representative = endpoint.second;
                        }
                        break;
                    }
                }
                if (representative != UINT32_MAX &&
                    (brepVertexId == 0 ||
                     brepVertexRepresentative.count(brepVertexId))) {
                    break;
                }
            }
            const gp_Pnt point = brepVertexId > 0
                ? BRep_Tool::Pnt(
                      TopoDS::Vertex(contractVertices(brepVertexId)))
                : curve->Value(first+(last-first)*t);
            if (representative == UINT32_MAX) {
                representative =
                    static_cast<uint32_t>(mesh.vertices.size());
                mesh.vertices.push_back(
                    {point.X(),point.Y(),point.Z()});
                mesh.anchors.push_back({});
                if (!mesh.constraints.empty()) {
                    mesh.constraints.push_back({});
                }
                ++created;
            } else if (representative < mesh.vertices.size()) {
                mesh.vertices[representative] =
                    {point.X(),point.Y(),point.Z()};
            }
            if (brepVertexId > 0) {
                brepVertexRepresentative[brepVertexId] =
                    representative;
            }
            chain.push_back({t,representative});
            for (const Match& match : matches) {
                if (std::abs(match.ta-t) <= 1e-7) {
                    remap[match.use.a] = representative;
                }
                if (std::abs(match.tb-t) <= 1e-7) {
                    remap[match.use.b] = representative;
                }
            }
        }
    }

    std::map<std::pair<size_t,size_t>,std::vector<uint32_t>> replacement;
    for (const auto& [eid,matches] : matchesByEdge) {
        const auto chainIt = canonical.find(eid);
        if (chainIt == canonical.end()) continue;
        const auto& chain = chainIt->second;
        for (const Match& match : matches) {
            std::vector<CanonicalPoint> interval;
            const double lo = std::min(match.ta,match.tb)-1e-8;
            const double hi = std::max(match.ta,match.tb)+1e-8;
            for (const CanonicalPoint& point : chain) {
                if (point.t >= lo && point.t <= hi) {
                    interval.push_back(point);
                }
            }
            if (interval.size() < 2) continue;
            if (match.tb < match.ta) {
                std::reverse(interval.begin(),interval.end());
            }
            std::vector<uint32_t> sequence;
            sequence.reserve(interval.size()-1);
            for (size_t i = 0; i+1 < interval.size(); ++i) {
                if (sequence.empty() ||
                    sequence.back() != interval[i].vertex) {
                    sequence.push_back(interval[i].vertex);
                }
            }
            if (!sequence.empty()) {
                replacement[{match.use.polygon,
                             match.use.corner}] =
                    std::move(sequence);
            }
        }
    }

    int inserted = 0;
    for (size_t pi = 0; pi < mesh.polygons.size(); ++pi) {
        const auto original = mesh.polygons[pi];
        std::vector<uint32_t> rebuilt;
        for (size_t k = 0; k < original.size(); ++k) {
            auto contract = replacement.find({pi,k});
            if (contract != replacement.end()) {
                for (uint32_t v : contract->second) {
                    if (rebuilt.empty() || rebuilt.back() != v) {
                        rebuilt.push_back(v);
                    }
                }
                inserted += std::max(
                    0,int(contract->second.size())-1);
            } else {
                auto mapped = remap.find(original[k]);
                const uint32_t v =
                    mapped != remap.end()
                        ? mapped->second : original[k];
                if (rebuilt.empty() || rebuilt.back() != v) {
                    rebuilt.push_back(v);
                }
            }
        }
        while (rebuilt.size() > 1 &&
               rebuilt.front() == rebuilt.back()) {
            rebuilt.pop_back();
        }
        if (rebuilt.size() >= 3) {
            mesh.polygons[pi] = std::move(rebuilt);
        }
    }
    if (!replacement.empty()) {
        mesh.polygonCornerAnchors.clear();
        mesh.certifiedTriangles.clear();
    }
    dbg("orthogonal edge contracts: %zu edges, %zu boundary segments, "
        "%d routed, %d inserted, %d created", matchesByEdge.size(),
        replacement.size(),routedSegments,inserted,created);
    return inserted;
}


}  // namespace weft::mesher_impl
