#include "mesher_internal.hpp"

namespace weft::mesher_impl {

// Last resort for trimmed/freeform faces: OCCT chord-tolerance
// triangulation, optionally paired into quads. Pairing is greedy over a
// quality score that prefers near-rectangular quads whose edges follow the
// surface's parametric directions — the seed of the plan's guided quad
// flow (§3.5); a real cross-field solver replaces the guidance later.
void meshFallback(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                  int faceId, const FaceMeshSettings& s, MeshBuilder& out) {
    // Faces mesh on worker threads, but OCCT triangulation writes shared
    // per-EDGE data (adjacent faces touch the same TEdge), so the OCCT
    // calls serialize; the heavy per-node work below stays parallel.
    // Clean first so a loosened deviation actually re-coarsens instead of
    // keeping the cached finer triangulation.
    TopLoc_Location loc;
    Handle(Poly_Triangulation) tri;
    {
        static std::mutex occtMeshMutex;
        std::lock_guard<std::mutex> lock(occtMeshMutex);
        // Triangulation is cache state on the face/edge TShapes. Cleaning and
        // meshing the live model mutates the B-rep, so an identical warm-cache
        // generation can solve different adaptive counts than the cold run.
        // Mesh an isolated topology copy while sharing the exact geometry.
        BRepBuilderAPI_Copy copier(face, Standard_False, Standard_False);
        const TopoDS_Face triangulationFace =
            TopoDS::Face(copier.Shape());
        BRepTools::Clean(triangulationFace);
        IMeshTools_Parameters mp;
        // Relative mode scales the tolerance by THIS FACE's extent
        // ourselves (sagitta as a fraction of feature size — the same
        // meaning the border solver uses). OCCT's own Relative flag
        // multiplies per component edge, which saturates at the
        // coarsest mesh for any typical value — the deviation slider
        // read as dead.
        double defl = std::max(1e-9, s.chordTolerance);
        if (s.relativeDeviation) {
            Bnd_Box bb;
            BRepBndLib::Add(face, bb);
            if (!bb.IsVoid()) {
                double x0, y0, z0, x1, y1, z1;
                bb.Get(x0, y0, z0, x1, y1, z1);
                const double diag = gp_Pnt(x0, y0, z0).Distance(
                    gp_Pnt(x1, y1, z1));
                defl = std::max(1e-9, s.chordTolerance * 0.05 * diag);
            }
        }
        mp.Deflection = defl;
        mp.Angle = s.angleToleranceDeg * M_PI / 180.0;
        mp.Relative = Standard_False;
        if (s.minSize > 0) mp.MinSize = s.minSize;
        mp.InParallel = Standard_True;
        BRepMesh_IncrementalMesh mesher(triangulationFace, mp);
        tri = BRep_Tool::Triangulation(triangulationFace, loc);
    }
    if (tri.IsNull()) return;

    const bool flip = face.Orientation() == TopAbs_REVERSED;
    const bool hasUV = tri->HasUVNodes();
    std::vector<uint32_t> verts(tri->NbNodes());
    std::vector<gp_Pnt> pts(tri->NbNodes());
    for (int i = 1; i <= tri->NbNodes(); ++i) {
        Anchor a;
        if (hasUV) {
            gp_Pnt2d uv = tri->UVNode(i);
            a = {faceId, uv.X(), uv.Y()};
        }
        pts[i - 1] = tri->Node(i).Transformed(loc.Transformation());
        verts[i - 1] = out.addVertex(pts[i - 1], a);
    }

    std::vector<std::array<int, 3>> tris(tri->NbTriangles());
    for (int i = 1; i <= tri->NbTriangles(); ++i) {
        int a, b, c;
        tri->Triangle(i).Get(a, b, c);
        tris[i - 1] = {a - 1, b - 1, c - 1};
    }

    if (!s.quadDominant) {
        for (const auto& t : tris) {
            out.addPolygon({verts[t[0]], verts[t[1]], verts[t[2]]}, faceId, flip);
        }
        return;
    }
    std::vector<std::vector<int>> paired;  // local rings, tris and quads

    // Candidate merges: two triangles sharing an edge form the quad
    // (opp1, a, opp2, b) with the shared diagonal (a,b) removed.
    struct Candidate {
        double cost;
        int t1, t2;
        std::array<int, 4> ring;
    };
    std::map<std::pair<int, int>, std::pair<int, int>> edgeUse;  // edge -> tris
    for (size_t t = 0; t < tris.size(); ++t) {
        for (int i = 0; i < 3; ++i) {
            int a = tris[t][i], b = tris[t][(i + 1) % 3];
            auto key = a < b ? std::make_pair(a, b) : std::make_pair(b, a);
            auto it = edgeUse.find(key);
            if (it == edgeUse.end()) edgeUse[key] = {static_cast<int>(t), -1};
            else it->second.second = static_cast<int>(t);
        }
    }

    auto thirdVertex = [&](int t, int a, int b) {
        for (int v : tris[t]) {
            if (v != a && v != b) return v;
        }
        return -1;
    };

    std::vector<Candidate> candidates;
    for (const auto& [key, owners] : edgeUse) {
        if (owners.second < 0) continue;
        int a = key.first, b = key.second;
        int c1 = thirdVertex(owners.first, a, b);
        int c2 = thirdVertex(owners.second, a, b);
        // Orient the ring with t1's winding: when t1 traverses the shared
        // edge a->b, the merged boundary cycle is c1->a->c2->b; reversed
        // when t1 runs b->a.
        std::array<int, 4> ring{c1, a, c2, b};
        for (int i = 0; i < 3; ++i) {
            if (tris[owners.first][i] == b &&
                tris[owners.first][(i + 1) % 3] == a) {
                ring = {c1, b, c2, a};
                break;
            }
        }
        double cost =
            quadAngleCost({pts[ring[0]], pts[ring[1]], pts[ring[2]],
                           pts[ring[3]]});
        if (cost > 1e8) continue;

        // Guidance: reward quads whose edges follow the parametric
        // directions at the quad center (trivial direction field).
        if (hasUV) {
            gp_Pnt2d uv0 = tri->UVNode(ring[0] + 1);
            gp_Pnt2d uv2 = tri->UVNode(ring[2] + 1);
            gp_Pnt p;
            gp_Vec du, dv;
            surf.D1(0.5 * (uv0.X() + uv2.X()), 0.5 * (uv0.Y() + uv2.Y()), p,
                    du, dv);
            if (du.Magnitude() > 1e-9 && dv.Magnitude() > 1e-9) {
                gp_Vec e(pts[ring[0]], pts[ring[1]]);
                if (e.Magnitude() > 1e-12) {
                    double alignU = std::abs(e.Normalized().Dot(du.Normalized()));
                    double alignV = std::abs(e.Normalized().Dot(dv.Normalized()));
                    // 0 when aligned with u or v, up to ~20 when diagonal.
                    cost += 20.0 * std::min(1.0, 1.0 - std::max(alignU, alignV));
                }
            }
        }
        candidates.push_back({cost, owners.first, owners.second, ring});
    }
    std::sort(candidates.begin(), candidates.end(),
              [](const Candidate& x, const Candidate& y) {
                  return x.cost < y.cost;
              });

    std::vector<bool> used(tris.size(), false);
    for (const Candidate& c : candidates) {
        if (used[c.t1] || used[c.t2]) continue;
        used[c.t1] = used[c.t2] = true;
        paired.push_back({c.ring[0], c.ring[1], c.ring[2], c.ring[3]});
    }
    for (size_t t = 0; t < tris.size(); ++t) {
        if (used[t]) continue;
        paired.push_back({tris[t][0], tris[t][1], tris[t][2]});
    }

    // Emit the paired mesh AS IS: quads where two triangles merged,
    // triangles where nothing paired. The old midpoint subdivision
    // ("pure quads") quadrupled density and salted every border with
    // midpoint vertices no neighbour has — un-triangulating is the
    // whole job here, not adding edges.
    for (const auto& ring : paired) {
        std::vector<uint32_t> poly;
        poly.reserve(ring.size());
        for (int v : ring) poly.push_back(verts[v]);
        out.addPolygon(std::move(poly), faceId, flip);
    }
}


}  // namespace weft::mesher_impl
