#include "mesher_internal.hpp"

namespace weft::mesher_impl {

// A genuine full revolution band's boundary consists only of its two
// v-rims and (possibly) a seam. Sampled through the pcurves: an edge that
// is neither a rim-hugging v-iso nor a u-iso seam means the face is
// TRIMMED, and drawing the full band would over-mesh across the trim —
// the classifier probe grid can miss small notches entirely.
bool edgesHugRims(const TopoDS_Face& face, const BRepAdaptor_Surface& surf) {
    const double u0 = surf.FirstUParameter(), u1 = surf.LastUParameter();
    const double v0 = surf.FirstVParameter(), v1 = surf.LastVParameter();
    const double uspan = std::max(1e-12, u1 - u0);
    const double vspan = std::max(1e-12, v1 - v0);
    for (TopExp_Explorer ex(face, TopAbs_EDGE); ex.More(); ex.Next()) {
        const TopoDS_Edge edge = TopoDS::Edge(ex.Current());
        if (BRep_Tool::Degenerated(edge)) continue;
        double f, l;
        Handle(Geom2d_Curve) pc = BRep_Tool::CurveOnSurface(edge, face, f, l);
        // A boundary edge we can't even place on the surface is exactly
        // the kind the probe grid misses — refuse the full band.
        if (pc.IsNull()) return false;
        double umin = 1e300, umax = -1e300, vmin = 1e300, vmax = -1e300;
        for (int k = 0; k <= 4; ++k) {
            gp_Pnt2d uv = pc->Value(f + (l - f) * k / 4.0);
            umin = std::min(umin, uv.X());
            umax = std::max(umax, uv.X());
            vmin = std::min(vmin, uv.Y());
            vmax = std::max(vmax, uv.Y());
        }
        if (vmax - vmin < 0.02 * vspan) {  // v-iso: must hug a rim
            double v = (vmin + vmax) / 2;
            if (std::min(std::abs(v - v0), std::abs(v - v1)) > 0.05 * vspan) {
                return false;  // a ring mid-band: the face is split there
            }
        } else if (umax - umin < 0.02 * uspan) {
            // u-iso: only a true SEAM (full v traversal) belongs to a
            // full band; a partial u-iso edge is a trim boundary.
            if (vmax - vmin < 0.9 * vspan) return false;
        } else {
            return false;  // slanted/trimmed boundary
        }
    }
    return true;
}

// Partition a closed band's border edges (seams, poles and insert wires
// excluded) into the two rim chains by CONNECTIVITY — a deep pipe-saddle
// weld curve wanders past the band's v middle, so nearest-end tests
// misfile its edges; what actually defines a chain is that its edges
// share vertices. Chains are then named low/high by mean v. Returns
// false when the borders don't form 1 or 2 chains.
bool rimChains(const TopoDS_Face& face, const Model& model,
               const std::set<int>& insertIds, std::vector<int>& low,
               std::vector<int>& high) {
    low.clear();
    high.clear();
    std::map<const void*, int> vertGroup;  // vertex TShape -> chain id
    std::map<int, int> edgeGroup;
    std::map<int, double> edgeMeanV;
    std::vector<int> parent;  // tiny union-find over chain ids
    std::function<int(int)> findG = [&](int g) {
        while (parent[g] != g) g = parent[g] = parent[parent[g]];
        return g;
    };
    for (TopExp_Explorer ex(face, TopAbs_EDGE); ex.More(); ex.Next()) {
        const TopoDS_Edge edge = TopoDS::Edge(ex.Current());
        if (BRep_Tool::Degenerated(edge)) continue;
        if (BRep_Tool::IsClosed(edge, face)) continue;  // seam
        const int eid = model.edges.FindIndex(edge);
        if (eid < 1 || insertIds.count(eid)) continue;
        if (edgeGroup.count(eid)) continue;  // second traversal
        double f, l;
        Handle(Geom2d_Curve) pc = BRep_Tool::CurveOnSurface(edge, face, f, l);
        if (pc.IsNull()) return false;
        double sum = 0;
        for (int k = 0; k <= 8; ++k) {
            sum += pc->Value(f + (l - f) * k / 8.0).Y();
        }
        edgeMeanV[eid] = sum / 9.0;
        TopoDS_Vertex va, vb;
        TopExp::Vertices(edge, va, vb);
        int g = -1;
        for (const TopoDS_Vertex& v : {va, vb}) {
            if (v.IsNull()) continue;
            auto it = vertGroup.find(v.TShape().get());
            if (it == vertGroup.end()) continue;
            const int vg = findG(it->second);
            if (g < 0) {
                g = vg;
            } else if (g != vg) {
                parent[vg] = g;  // edge joins two chains
            }
        }
        if (g < 0) {
            g = int(parent.size());
            parent.push_back(g);
        }
        edgeGroup[eid] = g;
        for (const TopoDS_Vertex& v : {va, vb}) {
            if (!v.IsNull()) vertGroup[v.TShape().get()] = g;
        }
    }
    // No rim edges at all (full sphere: poles + seam only; full torus:
    // nothing) is a valid band — both chains stay empty.
    if (edgeGroup.empty()) return true;
    std::map<int, std::pair<double, int>> chains;  // root -> (sumV, n)
    for (const auto& [eid, g] : edgeGroup) {
        auto& c = chains[findG(g)];
        c.first += edgeMeanV[eid];
        c.second += 1;
    }
    if (chains.size() > 2) return false;
    int lowRoot = -1;
    double lowMean = 1e300;
    for (const auto& [root, c] : chains) {
        const double mean = c.first / c.second;
        if (mean < lowMean) {
            lowMean = mean;
            lowRoot = root;
        }
    }
    if (chains.size() == 1) {
        // A single rim (cone to an apex, sphere cap): it keeps the side
        // its v actually sits on so the grid doesn't build upside down.
        BRepAdaptor_Surface sf(face);
        const double sv0 = sf.FirstVParameter();
        const double sv1 = sf.LastVParameter();
        auto& dst = std::abs(lowMean - sv0) <= std::abs(lowMean - sv1)
                        ? low
                        : high;
        for (const auto& [eid, g] : edgeGroup) dst.push_back(eid);
        return true;
    }
    for (const auto& [eid, g] : edgeGroup) {
        (findG(g) == lowRoot ? low : high).push_back(eid);
    }
    return true;
}

// Like edgesHugRims, but a wire living STRICTLY inside the band (a slot
// or hole through the wall) is collected as an insert instead of
// disqualifying the whole face.
// With `bandSides` (a partial-wrap band's two side edges) the sides are
// excluded from the rim chains, BOTH chains must exist (the sides must
// join two rims, not close a slit), and u never wraps.
bool edgesHugRimsOrInserts(const TopoDS_Face& face,
                           const BRepAdaptor_Surface& surf,
                           const Model& model,
                           std::vector<std::vector<int>>& wires,
                           const std::vector<int>* bandSides,
                           bool repeatedNotchedSeam) {
    const double u0 = surf.FirstUParameter(), u1 = surf.LastUParameter();
    const double v0 = surf.FirstVParameter(), v1 = surf.LastVParameter();
    const double uspan = std::max(1e-12, u1 - u0);
    const double vspan = std::max(1e-12, v1 - v0);
    wires.clear();
    std::vector<std::array<double, 4>> insertBox;
    for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
        double wu0 = 1e300, wu1 = -1e300, wv0 = 1e300, wv1 = -1e300;
        std::vector<int> ids;
        bool pcOk = true;
        for (TopExp_Explorer ex(wx.Current(), TopAbs_EDGE); ex.More();
             ex.Next()) {
            const TopoDS_Edge edge = TopoDS::Edge(ex.Current());
            if (BRep_Tool::Degenerated(edge)) continue;
            double f, l;
            Handle(Geom2d_Curve) pc =
                BRep_Tool::CurveOnSurface(edge, face, f, l);
            if (pc.IsNull()) { pcOk = false; break; }
            for (int k = 0; k <= 8; ++k) {
                gp_Pnt2d uv = pc->Value(f + (l - f) * k / 8.0);
                wu0 = std::min(wu0, uv.X());
                wu1 = std::max(wu1, uv.X());
                wv0 = std::min(wv0, uv.Y());
                wv1 = std::max(wv1, uv.Y());
            }
            int eid = model.edges.FindIndex(edge);
            if (eid > 0) ids.push_back(eid);
        }
        if (!pcOk) return false;
        if (ids.empty()) continue;
        const bool interior = wu0 > u0 + 0.03 * uspan &&
                              wu1 < u1 - 0.03 * uspan &&
                              wv0 > v0 + 0.03 * vspan &&
                              wv1 < v1 - 0.03 * vspan;
        if (interior) {
            wires.push_back(std::move(ids));
            insertBox.push_back({wu0, wu1, wv0, wv1});
            continue;
        }
        // Not interior: the wire's edges are rims (flat or WAVY — a
        // pipe-saddle weld curve winds around u while its v oscillates)
        // and seams. Loftability of the rim chains is checked
        // collectively below.
    }
    // Wavy-rim loftability: bucket every border sample by u and demand
    // clear v separation between the two rim CHAINS in every bucket.
    // (Flat rims pass trivially; crossing or interleaved chains reject.)
    std::set<int> insertIds;
    for (const auto& w : wires) insertIds.insert(w.begin(), w.end());
    if (bandSides) insertIds.insert(bandSides->begin(), bandSides->end());
    std::vector<int> lowChain, highChain;
    if (!rimChains(face, model, insertIds, lowChain, highChain)) {
        return false;
    }
    // An open band's grid runs side column to side column between TWO
    // rims; a lone chain means the sides bound a slit, not a band.
    if (bandSides && (lowChain.empty() || highChain.empty())) return false;
    // Pure bands (full sphere/torus) have no rim chains to vet.
    if (lowChain.empty() && highChain.empty()) return true;
    constexpr int kBins = 64;
    double loMax[kBins], hiMin[kBins], loMin[kBins], hiMax[kBins];
    for (int i = 0; i < kBins; ++i) {
        loMax[i] = -1e300;
        hiMin[i] = 1e300;
        loMin[i] = 1e300;
        hiMax[i] = -1e300;
    }
    for (int pass = 0; pass < 2; ++pass) {
        for (int eid : pass == 0 ? lowChain : highChain) {
            const TopoDS_Edge edge = TopoDS::Edge(model.edges(eid));
            double f, l;
            Handle(Geom2d_Curve) pc =
                BRep_Tool::CurveOnSurface(edge, face, f, l);
            if (pc.IsNull()) return false;
            for (int k = 0; k <= 16; ++k) {
                gp_Pnt2d uv = pc->Value(f + (l - f) * k / 16.0);
                double uu = uv.X() - u0;
                // Open bands never wrap: a sample at exactly u1 belongs
                // to the last bin, not bin 0.
                if (!bandSides) uu -= uspan * std::floor(uu / uspan);
                int bin =
                    std::clamp(int(uu / uspan * kBins), 0, kBins - 1);
                if (pass == 0) {
                    loMax[bin] = std::max(loMax[bin], uv.Y());
                    loMin[bin] = std::min(loMin[bin], uv.Y());
                } else {
                    hiMin[bin] = std::min(hiMin[bin], uv.Y());
                    hiMax[bin] = std::max(hiMax[bin], uv.Y());
                }
            }
        }
    }
    int plunges = 0;
    for (int i = 0; i < kBins; ++i) {
        if (loMax[i] > -1e300 && hiMin[i] < 1e300 && loMax[i] >= hiMin[i]) {
            return false;
        }
        // Loftable rims are FUNCTIONS of u: a chain that doubles back
        // stacks several v's over one u and cannot drive a lofted row.
        // A FEW deep bins are a NOTCH — a channel cut through the rim
        // (flaregun face 81): its walls drop the whole way at one u
        // each, and the loft handles them (rows never rise above the
        // notch floor inside the mouth). MANY deep bins are gear teeth
        // and still reject.
        if (loMax[i] > -1e300 && loMax[i] - loMin[i] > 0.3 * vspan) {
            ++plunges;
        }
        if (hiMax[i] > -1e300 && hiMax[i] - hiMin[i] > 0.3 * vspan) {
            ++plunges;
        }
    }
    // A geometry-validated repeated chamfer seam legitimately doubles back
    // in many bins (one plunge per groove).  The caller only enables this
    // after proving a single plain rim, alternating far-rim lands, and two
    // side edges; retain the ordinary strict limit everywhere else.
    const int plungeLimit = repeatedNotchedSeam ? kBins / 2
                                                 : std::max(2, kBins / 8);
    if (plunges > plungeLimit) return false;
    // Between-chain coverage: the loft region must actually belong to
    // the face — a band with a large un-modeled cutout (not an insert
    // wire) cannot loft. Insert-wire boxes are skipped: their cells are
    // removed and webbed after the grid.
    const double tolF = BRep_Tool::Tolerance(face);
    for (int i = 0; i < kBins; i += 4) {
        if (loMax[i] <= -1e300 || hiMin[i] >= 1e300) continue;
        const double uu = u0 + (i + 0.5) * uspan / kBins;
        const double vv = 0.5 * (loMax[i] + hiMin[i]);
        bool inInsert = false;
        for (const auto& b : insertBox) {
            if (uu >= b[0] && uu <= b[1] && vv >= b[2] && vv <= b[3]) {
                inInsert = true;
                break;
            }
        }
        if (inInsert) continue;
        BRepClass_FaceClassifier cls(const_cast<TopoDS_Face&>(face),
                                     gp_Pnt2d(uu, vv), tolF);
        if (cls.State() == TopAbs_OUT) return false;
    }
    return true;
}

// Partial-wrap revolution band: an analytic periodic surface trimmed
// short of the full period, bounded by exactly two full-height u-iso
// side edges (a barrel wall that stops at 92% of the circle). The sides
// are the band's u extremes; everything else on that wire — however
// castellated — is a rim chain. Additional wires are interior cutouts
// (slots and holes through the wall) that edgesHugRimsOrInserts
// classifies; they mesh as boolean-cut inserts, so they no longer
// disqualify the band (the old coons-cutout route fanned the slot ends
// across the primitive). A side-like edge on a cutout wire (a slot as
// tall as the wall) still rejects — that geometry is not a band.
bool openBandSides(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                   const Model& model, std::vector<int>& sides,
                   bool* repeatedNotched,
                   int* repeatedFeatureCount) {
    sides.clear();
    if (repeatedNotched) *repeatedNotched = false;
    if (repeatedFeatureCount) *repeatedFeatureCount = 0;
    switch (surf.GetType()) {
        case GeomAbs_Cylinder:
        case GeomAbs_Cone:
        case GeomAbs_SurfaceOfRevolution: break;
        default: return false;  // spheres/tori: poles and v-wrap instead
    }
    if (surf.IsUClosed() || surf.IsVClosed()) return false;
    const double uspan = std::max(
        1e-12, surf.LastUParameter() - surf.FirstUParameter());
    const double vspan = std::max(
        1e-12, surf.LastVParameter() - surf.FirstVParameter());
    struct EdgeBox {
        int eid = 0;
        double u0 = 1e300, u1 = -1e300, v0 = 1e300, v1 = -1e300;
    };
    std::vector<std::vector<EdgeBox>> wireBoxes;
    for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) {
        std::vector<EdgeBox> boxes;
        for (TopExp_Explorer ex(wx.Current(), TopAbs_EDGE); ex.More();
             ex.Next()) {
            const TopoDS_Edge edge = TopoDS::Edge(ex.Current());
            if (BRep_Tool::Degenerated(edge)) continue;
            if (BRep_Tool::IsClosed(edge, face)) return false;
            EdgeBox b;
            b.eid = model.edges.FindIndex(edge);
            if (b.eid < 1) return false;
            double f,l;
            Handle(Geom2d_Curve) pc =
                BRep_Tool::CurveOnSurface(edge,face,f,l);
            if (pc.IsNull()) return false;
            for (int k=0;k<=16;++k) {
                const gp_Pnt2d uv=pc->Value(f+(l-f)*k/16.0);
                b.u0=std::min(b.u0,uv.X()); b.u1=std::max(b.u1,uv.X());
                b.v0=std::min(b.v0,uv.Y()); b.v1=std::max(b.v1,uv.Y());
            }
            boxes.push_back(b);
        }
        if (!boxes.empty()) wireBoxes.push_back(std::move(boxes));
    }

    // Historic strict case: exactly two full-height u-iso sides on one wire.
    int outerWires=0;
    for (const auto& boxes:wireBoxes) {
        std::vector<int> wireSides;
        for (const EdgeBox& b:boxes) {
            if (b.u1-b.u0 < 0.02*uspan && b.v1-b.v0 >= 0.9*vspan)
                wireSides.push_back(b.eid);
        }
        if (wireSides.size()==2) { ++outerWires; sides=wireSides; }
        else if (!wireSides.empty()) { outerWires=-100; break; }
    }
    if (outerWires==1 && sides.size()==2) return true;
    sides.clear();

    // Conservative repeated-notch seam relaxation.  A conical chamfer can
    // have one clean full-span rim while the opposite rim is a repeated
    // groove chain whose first/last notch shortens both angular side edges.
    // It is still an open band, but only when the complete structural pattern
    // is present: one wire, one plain rim, one side touching each u extreme,
    // and at least three alternating deep feature pieces / far-rim lands.
    if (surf.GetType()!=GeomAbs_Cone || wireBoxes.size()!=1) return false;
    const auto& boxes=wireBoxes.front();
    const double su0=surf.FirstUParameter(), su1=surf.LastUParameter();
    const double sv0=surf.FirstVParameter(), sv1=surf.LastVParameter();
    int plain=0, plainAtLow=0;
    for (const EdgeBox& b:boxes) {
        const double du=b.u1-b.u0, dv=b.v1-b.v0;
        if (du < 0.95*uspan || dv > 0.03*vspan) continue;
        const bool low=std::abs(0.5*(b.v0+b.v1)-sv0) <= 0.04*vspan;
        const bool high=std::abs(0.5*(b.v0+b.v1)-sv1) <= 0.04*vspan;
        if (!low && !high) continue;
        if (plain) return false; // two plain rims are the ordinary case
        plain=b.eid; plainAtLow=low?1:0;
    }
    if (!plain) return false;
    const double plainV=plainAtLow?sv0:sv1;
    const double farV=plainAtLow?sv1:sv0;
    int sideLo=0,sideHi=0; double spanLo=0,spanHi=0;
    int farLands=0,deepPieces=0;
    for (const EdgeBox& b:boxes) {
        if (b.eid==plain) continue;
        const double du=b.u1-b.u0, dv=b.v1-b.v0;
        const bool touchesPlain=std::min(std::abs(b.v0-plainV),
                                         std::abs(b.v1-plainV))<=0.04*vspan;
        if (du<=0.02*uspan && touchesPlain && dv>=0.25*vspan) {
            if (std::abs(0.5*(b.u0+b.u1)-su0)<=0.03*uspan && dv>spanLo)
                {sideLo=b.eid;spanLo=dv;}
            if (std::abs(0.5*(b.u0+b.u1)-su1)<=0.03*uspan && dv>spanHi)
                {sideHi=b.eid;spanHi=dv;}
        }
        const bool hugsFar=dv<=0.03*vspan &&
            std::abs(0.5*(b.v0+b.v1)-farV)<=0.04*vspan;
        if (hugsFar && du>=0.04*uspan) ++farLands;
        if (du>=0.04*uspan && dv>=0.12*vspan) ++deepPieces;
        // Nothing in the notched chain may cross through the plain rim.
        if (plainAtLow ? b.v0 < sv0-0.02*vspan
                       : b.v1 > sv1+0.02*vspan) return false;
    }
    if (!sideLo || !sideHi || sideLo==sideHi || farLands<3 ||
        deepPieces<3) return false;
    sides={sideLo,sideHi};
    if (repeatedNotched) *repeatedNotched=true;
    if (repeatedFeatureCount) {
        *repeatedFeatureCount = std::max(farLands, deepPieces);
    }
    dbg("open band: repeated notched cone sides %d/%d (%d lands, %d deep)",
        sideLo,sideHi,farLands,deepPieces);
    return true;
}

// Recognize one connected UV-orthogonal trim as a structured lattice, not
// as a generic polygon to triangulate. STEP exporters commonly split a
// perfectly regular side at every adjacent feature; counting B-rep edges
// therefore says nothing about the patch topology. What matters is that all
// non-degenerate pcurves run along one surface parameter and that horizontal
// scanlines meet one connected interval.
bool planOrthogonalTrimGrid(const TopoDS_Face& face,
                            const BRepAdaptor_Surface& surf,
                            const Model& model, FacePlan& plan) {
    const int dbgFid = model.faces.FindIndex(face);
    if (surf.IsUClosed() || surf.IsVClosed()) return false;
    int wires = 0;
    for (TopExp_Explorer wx(face, TopAbs_WIRE); wx.More(); wx.Next()) ++wires;
    if (wires != 1) return false;

    const double u0 = surf.FirstUParameter(), u1 = surf.LastUParameter();
    const double v0 = surf.FirstVParameter(), v1 = surf.LastVParameter();
    const double us = std::max(1e-12, u1 - u0);
    const double vs = std::max(1e-12, v1 - v0);
    const double ut = 2e-5 * us, vt = 2e-5 * vs;
    const double nearU = 0.03 * us, nearV = 0.03 * vs;
    std::vector<int> horizontal, vertical;
    double bestU = -1.0, bestV = -1.0;
    bool hasInteriorStep = false;
    int fullHeightVertical = 0, insetVertical = 0;
    int fullWidthRims = 0, longRails = 0, curvedCaps = 0;
    int realEdges = 0;
    for (TopExp_Explorer ex(face, TopAbs_EDGE); ex.More(); ex.Next()) {
        const TopoDS_Edge edge = TopoDS::Edge(ex.Current());
        if (BRep_Tool::Degenerated(edge)) continue;
        ++realEdges;
        double f, l;
        Handle(Geom2d_Curve) pc = BRep_Tool::CurveOnSurface(edge, face, f, l);
        if (pc.IsNull()) return false;
        double eu0 = 1e300, eu1 = -1e300, ev0 = 1e300, ev1 = -1e300;
        for (int k = 0; k <= 12; ++k) {
            const gp_Pnt2d uv = pc->Value(f + (l - f) * k / 12.0);
            eu0 = std::min(eu0, uv.X()); eu1 = std::max(eu1, uv.X());
            ev0 = std::min(ev0, uv.Y()); ev1 = std::max(ev1, uv.Y());
        }
        const double du = eu1 - eu0, dv = ev1 - ev0;
        const int eid = model.edges.FindIndex(edge);
        if (eid < 1) return false;
        plan.orthogonalEdges.push_back(eid);
        if (du >= 0.9 * us && dv <= 0.04 * vs) ++fullWidthRims;
        if (du <= nearU && dv >= 0.35 * vs) ++longRails;
        if (du >= 0.03 * us && du <= 0.25 * us &&
            dv >= 0.05 * vs && dv <= 0.35 * vs) {
            ++curvedCaps;
        }
        if (du > ut && (dv <= nearV || du / us >= dv / vs)) {
            horizontal.push_back(eid);
            if (du > bestU) { bestU = du; plan.orthogonalDriverU = eid; }
            const double vm = 0.5 * (ev0 + ev1);
            hasInteriorStep |= vm > v0 + 0.05*vs && vm < v1 - 0.05*vs;
        } else if (dv > vt && (du <= nearU || dv / vs > du / us)) {
            vertical.push_back(eid);
            if (dv > bestV) { bestV = dv; plan.orthogonalDriverV = eid; }
            const double um = 0.5 * (eu0 + eu1);
            if (dv >= 0.9*vs) ++fullHeightVertical;
            if (um > u0 + 0.05*us && um < u1 - 0.05*us) ++insetVertical;
            hasInteriorStep |= um > u0 + 0.05*us && um < u1 - 0.05*us;
        } else if (du > ut || dv > vt) {
            dbg("orthogonal face %d rejected: diagonal edge %d du %.6g "
                "dv %.6g", dbgFid, eid, du/us, dv/vs);
            return false; // a real diagonal/curved trim needs a local web
        }
    }
    if (realEdges < 6 || horizontal.size() < 2 || vertical.size() < 2 ||
        plan.orthogonalDriverU == 0 || plan.orthogonalDriverV == 0) {
        return false;
    }
    const GeomAbs_SurfaceType st = surf.GetType();
    const bool drum = st == GeomAbs_Cylinder || st == GeomAbs_Cone ||
                      st == GeomAbs_SurfaceOfRevolution;
    const bool freeformComb = !drum && realEdges >= 20 &&
                              fullHeightVertical == 1 &&
                              insetVertical >= 2 && horizontal.size() >= 8;
    // Analytic repeated-groove half drums are a single comb-shaped wire:
    // one clean full-width rim, paired long groove rails, and paired curved
    // cap pieces.  Keep this gate deliberately structural; a merely split
    // rectangle or a one-off notch continues through the historic lattice.
    const bool drumComb = drum && realEdges >= 16 && fullWidthRims == 1 &&
                          fullHeightVertical == 1 && longRails >= 4 &&
                          curvedCaps >= 4;
    if (drum) {
        dbg("orthogonal drum face %d: edges=%d rims=%d sides=%d rails=%d "
            "caps=%d comb=%d", dbgFid, realEdges, fullWidthRims,
            fullHeightVertical, longRails, curvedCaps, drumComb ? 1 : 0);
    }
    if (!drum && realEdges >= 20) {
        dbg("orthogonal freeform face %d: edges=%d h=%zu v=%zu full=%d "
            "inset=%d interior=%d comb=%d", dbgFid, realEdges,
            horizontal.size(), vertical.size(), fullHeightVertical,
            insetVertical, hasInteriorStep ? 1 : 0,
            freeformComb ? 1 : 0);
    }
    // On a freeform surface only accept the split-sided rectangle case.
    // Orthogonal interior steps on arbitrary UV charts need a more general
    // trim solver; drums are safe because U/V are angular/axial directions.
    if (drum && (fullHeightVertical != 1 ||
                 (insetVertical < 1 && horizontal.size() < 4))) {
        return false;
    }
    if (!drum && !freeformComb &&
        (hasInteriorStep || horizontal.size() != 2)) {
        dbg("orthogonal face %d rejected: freeform interior step", dbgFid);
        return false;
    }

    // Scan every V slab. A valid band has exactly one connected inside run;
    // this excludes disjoint combs even when all of their edges are axis
    // aligned.
    std::vector<double> levels{v0, v1};
    for (int eid : vertical) {
        double f, l;
        Handle(Geom2d_Curve) pc = BRep_Tool::CurveOnSurface(
            TopoDS::Edge(model.edges(eid)), face, f, l);
        levels.push_back(pc->Value(f).Y());
        levels.push_back(pc->Value(l).Y());
    }
    std::sort(levels.begin(), levels.end());
    levels.erase(std::unique(levels.begin(), levels.end(),
                             [&](double a, double b) {
                                 return std::abs(a - b) <= vt;
                             }), levels.end());
    const double tolF = BRep_Tool::Tolerance(face);
    for (size_t j = 0; j + 1 < levels.size(); ++j) {
        if (levels[j + 1] - levels[j] <= vt) continue;
        const double v = 0.5 * (levels[j] + levels[j + 1]);
        int runs = 0; bool wasIn = false;
        for (int i = 0; i < 96; ++i) {
            const double u = u0 + (i + 0.5) * us / 96.0;
            BRepClass_FaceClassifier cls(const_cast<TopoDS_Face&>(face),
                                         gp_Pnt2d(u, v), tolF);
            const bool in = cls.State() != TopAbs_OUT;
            if (in && !wasIn) ++runs;
            wasIn = in;
        }
        if (runs != 1 && !drum && !freeformComb) {
            dbg("orthogonal face %d rejected: v slab %.6g has %d runs",
                dbgFid, v, runs);
            return false;
        }
    }

    plan.orthogonalTrimGrid = true;
    plan.orthogonalLocalComb = freeformComb;
    plan.orthogonalDrumComb = drumComb;
    if (drumComb) {
        plan.orthogonalFeatureCount = std::max(longRails, curvedCaps);
    }
    plan.kind = drum ? MesherKind::RevolutionGrid : MesherKind::CoonsGrid;
    plan.uEdges = std::move(horizontal);
    plan.vEdges = std::move(vertical);
    auto driverFirst = [](std::vector<int>& es, int driver) {
        auto it = std::find(es.begin(), es.end(), driver);
        if (it != es.end()) std::rotate(es.begin(), it, it + 1);
    };
    driverFirst(plan.uEdges, plan.orthogonalDriverU);
    driverFirst(plan.vEdges, plan.orthogonalDriverV);
    plan.constrains = true;
    plan.bandWrapFrac = drum ? us / (2.0 * M_PI) : 1.0;
    return true;
}

// Full-wrap castellated rim: a u-closed straight-ruling band (cylinder or
// cone) whose two rim chains are one PLAIN full circle and one NOTCHED rim
// — a channel cut clean through the rim, its two walls dropping from the
// rim to an interior floor. Detected so the face meshes as a straight
// uniform lattice driven by the plain rim (the notch boolean-cut, exactly
// like a partial-wrap open band) instead of a sheared chained loft. Returns
// the plain rim's edge id (which drives the column count), or 0 if the face
// is not this shape. Gated tightly: plain cylinders/cones with two matching
// plain rims fall through to the clean chained loft and never take this
// path (both rims flat -> neither notched).
int castellatedRimBand(const TopoDS_Face& face,
                       const BRepAdaptor_Surface& surf, const Model& model,
                       const std::vector<int>& rimLow,
                       const std::vector<int>& rimHigh) {
    switch (surf.GetType()) {
        case GeomAbs_Cylinder:
        case GeomAbs_Cone: break;  // u-isolines are straight rulings
        default: return 0;
    }
    if (!surf.IsUClosed() || surf.IsVClosed()) return 0;
    if (rimLow.empty() || rimHigh.empty()) return 0;
    const double u0 = surf.FirstUParameter(), u1 = surf.LastUParameter();
    const double v0 = surf.FirstVParameter(), v1 = surf.LastVParameter();
    const double uspan = std::max(1e-12, u1 - u0);
    const double vspan = std::max(1e-12, v1 - v0);
    // A lone flat full circle (the plain rim).
    auto plainCircleOf = [&](const std::vector<int>& chain) {
        if (chain.size() != 1) return false;
        double f, l;
        Handle(Geom2d_Curve) pc = BRep_Tool::CurveOnSurface(
            TopoDS::Edge(model.edges(chain[0])), face, f, l);
        if (pc.IsNull()) return false;
        double vLo = 1e300, vHi = -1e300, uLo = 1e300, uHi = -1e300;
        for (int k = 0; k <= 16; ++k) {
            gp_Pnt2d uv = pc->Value(f + (l - f) * k / 16.0);
            vLo = std::min(vLo, uv.Y());
            vHi = std::max(vHi, uv.Y());
            uLo = std::min(uLo, uv.X());
            uHi = std::max(uHi, uv.X());
        }
        return (vHi - vLo) < 0.02 * vspan && (uHi - uLo) > 0.98 * uspan;
    };
    // A rim carrying a single rim-open NOTCH: mostly flat at one rim level
    // (the base arcs), with ONE localized dip whose walls reach a real way
    // toward the interior. A wavy pipe-saddle weld rim (mohne) oscillates
    // around the whole wrap — its off-rim samples span most of u and few sit
    // at the rim level — and is REJECTED so it keeps the chained loft.
    auto notchedOk = [&](const std::vector<int>& chain) {
        int nearV0 = 0, nearV1 = 0, total = 0;
        bool hasWall = false;
        double notchU0 = 1e300, notchU1 = -1e300;
        for (int eid : chain) {
            double f, l;
            Handle(Geom2d_Curve) pc = BRep_Tool::CurveOnSurface(
                TopoDS::Edge(model.edges(eid)), face, f, l);
            if (pc.IsNull()) return false;
            double evLo = 1e300, evHi = -1e300;
            gp_Pnt2d uv[17];
            for (int k = 0; k <= 16; ++k) {
                uv[k] = pc->Value(f + (l - f) * k / 16.0);
                evLo = std::min(evLo, uv[k].Y());
                evHi = std::max(evHi, uv[k].Y());
            }
            if (evHi - evLo > 0.15 * vspan) hasWall = true;
            for (int k = 0; k <= 16; ++k) {
                ++total;
                const bool at0 = std::abs(uv[k].Y() - v0) < 0.05 * vspan;
                const bool at1 = std::abs(uv[k].Y() - v1) < 0.05 * vspan;
                if (at0) ++nearV0;
                if (at1) ++nearV1;
                if (!at0 && !at1) {
                    notchU0 = std::min(notchU0, uv[k].X());
                    notchU1 = std::max(notchU1, uv[k].X());
                }
            }
        }
        if (!hasWall || total == 0) return false;
        const double arcFrac = double(std::max(nearV0, nearV1)) / total;
        const double notchUspan = notchU1 > notchU0 ? notchU1 - notchU0 : 0.0;
        // Most of the rim flat, one dip narrower than half the wrap.
        return arcFrac > 0.4 && notchUspan < 0.5 * uspan;
    };
    const bool loPlain = plainCircleOf(rimLow);
    const bool hiPlain = plainCircleOf(rimHigh);
    if (loPlain && !hiPlain && notchedOk(rimHigh)) return rimLow[0];
    if (hiPlain && !loPlain && notchedOk(rimLow)) return rimHigh[0];
    return 0;
}

bool isClosedRevolution(const BRepAdaptor_Surface& surf) {
    switch (surf.GetType()) {
        case GeomAbs_Cylinder:
        case GeomAbs_Cone:
        case GeomAbs_Sphere:
        case GeomAbs_Torus:
        case GeomAbs_SurfaceOfRevolution: return surf.IsUClosed();
        case GeomAbs_BSplineSurface:
        case GeomAbs_BezierSurface:
        case GeomAbs_OffsetSurface:
        case GeomAbs_OtherSurface: return surf.IsUClosed();
        default: return false;
    }
}

// Adaptor-independent closed-revolution probe (MVP demand #2): OFFSET
// surfaces report IsUClosed()=false even across a full 2-pi period
// (foam's can body — 411 coons patches instead of columns), and geometry
// kernels export revolves as plain bsplines whose closure flag can lie
// too. Geometry doesn't: sample a probe grid and accept iff every
// u-isoline ring closes on itself and its points share one radius and
// one height about a common fitted axis. Typed revolution surfaces are
// excluded here — they are the fast path's business.
bool isGeometricClosedRevolution(const BRepAdaptor_Surface& surf) {
    switch (surf.GetType()) {
        case GeomAbs_Plane:
        case GeomAbs_Cylinder:
        case GeomAbs_Cone:
        case GeomAbs_Sphere:
        case GeomAbs_Torus:
        case GeomAbs_SurfaceOfRevolution: return false;
        default: break;
    }
    const double u0 = surf.FirstUParameter(), u1 = surf.LastUParameter();
    const double v0 = surf.FirstVParameter(), v1 = surf.LastVParameter();
    if (Precision::IsInfinite(u0) || Precision::IsInfinite(u1) ||
        Precision::IsInfinite(v0) || Precision::IsInfinite(v1) ||
        !(u1 > u0) || !(v1 > v0)) {
        return false;
    }
    constexpr int NV = 5, NU = 12;
    gp_XYZ P[NV][NU + 1];
    gp_XYZ lo(1e300, 1e300, 1e300), hi(-1e300, -1e300, -1e300);
    for (int j = 0; j < NV; ++j) {
        const double v = v0 + (v1 - v0) * j / double(NV - 1);
        for (int i = 0; i <= NU; ++i) {
            const double u = u0 + (u1 - u0) * i / double(NU);
            P[j][i] = surf.Value(u, v).XYZ();
            lo.SetX(std::min(lo.X(), P[j][i].X()));
            lo.SetY(std::min(lo.Y(), P[j][i].Y()));
            lo.SetZ(std::min(lo.Z(), P[j][i].Z()));
            hi.SetX(std::max(hi.X(), P[j][i].X()));
            hi.SetY(std::max(hi.Y(), P[j][i].Y()));
            hi.SetZ(std::max(hi.Z(), P[j][i].Z()));
        }
    }
    const double scale = (hi - lo).Modulus();
    if (scale < 1e-12) return false;
    // Every ring must close over the full u period.
    const double tolClose = 1e-5 * scale;
    for (int j = 0; j < NV; ++j) {
        if ((P[j][NU] - P[j][0]).Modulus() > tolClose) return false;
    }
    // Fitted axis: through the ring centroids. A flat washer's centroids
    // coincide, so fall back to ring 0's Newell normal.
    gp_XYZ C[NV];
    for (int j = 0; j < NV; ++j) {
        C[j] = gp_XYZ(0, 0, 0);
        for (int i = 0; i < NU; ++i) C[j] += P[j][i];
        C[j] /= double(NU);
    }
    gp_XYZ dir = C[NV - 1] - C[0];
    if (dir.Modulus() < 1e-6 * scale) {
        gp_XYZ n(0, 0, 0);
        for (int i = 0; i < NU; ++i) {
            n += (P[0][i] - C[0]).Crossed(P[0][(i + 1) % NU] - C[0]);
        }
        dir = n;
    }
    if (dir.Modulus() < 1e-12) return false;
    dir.Normalize();
    // Each ring: one radius, one height about the axis.
    const double tolGeom = 5e-4 * scale;
    for (int j = 0; j < NV; ++j) {
        double rMin = 1e300, rMax = -1e300, hMin = 1e300, hMax = -1e300;
        for (int i = 0; i < NU; ++i) {
            const gp_XYZ d = P[j][i] - C[0];
            const double h = d.Dot(dir);
            const gp_XYZ radial = d - dir * h;
            const double r = radial.Modulus();
            rMin = std::min(rMin, r);
            rMax = std::max(rMax, r);
            hMin = std::min(hMin, h);
            hMax = std::max(hMax, h);
        }
        if (rMax - rMin > tolGeom || hMax - hMin > tolGeom) return false;
    }
    return true;
}

// --- Closed-ring phase anchor ----------------------------------------------
// A full-circle edge has no natural sample start: its curve origin sits
// wherever the CAD kernel left it, so two coaxial rings sampled at
// "t = i/n" land at slightly different angles and the flat faces joining
// grooved cylinder segments TWIST. Anchor every closed circle's first
// sample to a fixed world direction projected into its plane — coaxial
// rings then share column angles by construction. Open arcs are
// endpoint-pinned and need no anchor.
double ringAnchorAngle(const gp_Circ& circ) {
    const gp_Dir d = circ.Axis().Direction();
    // Irrational-ish mix dodges symmetric ties with model axes.
    gp_XYZ g(0.7548776662466927, 0.5698402909980532, 0.3247179572447461);
    gp_XYZ pr = g - d.XYZ() * g.Dot(d.XYZ());
    if (pr.SquareModulus() < 1e-18) {
        pr = gp_XYZ(1, 0, 0) - d.XYZ() * d.X();
    }
    const double x = pr.Dot(circ.Position().XDirection().XYZ());
    const double y = pr.Dot(circ.Position().YDirection().XYZ());
    double a = std::atan2(y, x);
    if (a < 0) a += 2.0 * M_PI;
    return a;
}

// Lazy vertex->edges adjacency per model (read-only after build; the
// mutex covers concurrent meshing threads).
const EdgeFaceMap& modelVertexEdges(
    const Model& model) {
    static std::mutex mx;
    static std::map<const void*, std::unique_ptr<EdgeFaceMap>> cache;
    std::lock_guard<std::mutex> lock(mx);
    const void* key = model.shape.TShape().get();
    if (!cache.count(key) && cache.size() > 8) {
        // Bounded: drop other models' maps (session apps reload often).
        for (auto it = cache.begin(); it != cache.end();) {
            it = it->first != key ? cache.erase(it) : std::next(it);
        }
    }
    auto& slot = cache[key];
    if (!slot) {
        slot = std::make_unique<EdgeFaceMap>();
        TopExp::MapShapesAndAncestors(model.shape, TopAbs_VERTEX,
                                      TopAbs_EDGE, *slot);
    }
    return *slot;
}

// Sample-phase fraction for an edge: >0 only for closed CIRCULAR edges
// whose seam vertex is FREE — shared with nothing but revolve seam
// edges. A vertex shared with a real border edge (a bore tangent to the
// plate outline: valence-10 junctions exist) is a hard constraint the
// phased sampling would skip, and the conform pass then kidnaps the
// outline corner onto the shifted ring. Part of the border contract —
// every border sampler applies it identically.
double closedEdgePhase(const TopoDS_Edge& edge, const Model& model) {
    double f, l;
    Handle(Geom_Curve) c = BRep_Tool::Curve(edge, f, l);
    if (c.IsNull()) return 0.0;
    if (l - f < 2.0 * M_PI - 1e-9) return 0.0;
    if (c->Value(f).Distance(c->Value(l)) > 1e-9) return 0.0;
    GeomAdaptor_Curve gc(c, f, l);
    if (gc.GetType() != GeomAbs_Circle) return 0.0;
    const auto& v2e = modelVertexEdges(model);
    TopoDS_Vertex va, vb;
    TopExp::Vertices(edge, va, vb);
    for (const TopoDS_Vertex& v : {va, vb}) {
        if (v.IsNull()) continue;
        const int idx = v2e.FindIndex(v);
        if (!idx) continue;
        for (const TopoDS_Shape& s : v2e.FindFromIndex(idx)) {
            const TopoDS_Edge e2 = TopoDS::Edge(s);
            if (e2.IsSame(edge)) continue;
            if (BRep_Tool::Degenerated(e2)) continue;
            // Revolve seams are mesh-internal (the ring wraps through
            // them); anything else pins the vertex.
            const int eid2 = model.edges.FindIndex(e2);
            bool seam = false;
            if (eid2 >= 1 && model.edgeToFaces.Contains(e2)) {
                const ShapeList& fl =
                    model.edgeToFaces.FindFromKey(e2);
                if (fl.Extent() == 1) {
                    seam = BRep_Tool::IsClosed(
                        e2, TopoDS::Face(fl.First()));
                }
            }
            if (!seam) return 0.0;
        }
    }
    double ph = (ringAnchorAngle(gc.Circle()) - f) / (l - f);
    ph -= std::floor(ph);
    return ph;
}

// Shared phased, pinned, and even-arc edge sampling lives in
// mesher_sampling.cpp so every legacy strategy consumes the same contract.
// A planar face bounded by exactly one full-circle edge (a cylinder cap).
bool boundingCircle(const TopoDS_Face& face, gp_Circ& circOut, int& edgeIdOut,
                    const Model& model) {
    int edgeCount = 0;
    TopoDS_Edge only;
    for (TopExp_Explorer ex(face, TopAbs_EDGE); ex.More(); ex.Next()) {
        ++edgeCount;
        only = TopoDS::Edge(ex.Current());
    }
    if (edgeCount != 1) return false;

    double f = 0, l = 0;
    Handle(Geom_Curve) curve = BRep_Tool::Curve(only, f, l);
    Handle(Geom_Circle) circle = Handle(Geom_Circle)::DownCast(curve);
    if (circle.IsNull()) return false;
    if (std::abs((l - f) - 2.0 * M_PI) > 1e-7) return false;
    circOut = circle->Circ();
    edgeIdOut = model.edges.FindIndex(only);
    return true;
}

enum class EdgeIso { UAligned, VAligned, Neither };

// Classify a boundary edge by its pcurve: does it run along u (constant v)
// or along v (constant u)? Density matching only binds iso-aligned edges,
// where "edge subdivisions" and "grid divisions" are the same thing.
EdgeIso edgeIsoDirection(const TopoDS_Edge& edge, const TopoDS_Face& face,
                         double uRange, double vRange) {
    double f = 0, l = 0;
    Handle(Geom2d_Curve) pcurve = BRep_Tool::CurveOnSurface(edge, face, f, l);
    if (pcurve.IsNull()) return EdgeIso::Neither;
    gp_Pnt2d a = pcurve->Value(f);
    gp_Pnt2d b = pcurve->Value(l);
    gp_Pnt2d m = pcurve->Value(0.5 * (f + l));
    double du = std::max(std::abs(b.X() - a.X()), std::abs(m.X() - a.X())) /
                std::max(uRange, 1e-12);
    double dv = std::max(std::abs(b.Y() - a.Y()), std::abs(m.Y() - a.Y())) /
                std::max(vRange, 1e-12);
    const double kIsoTol = 1e-6;
    if (dv < kIsoTol && du > kIsoTol) return EdgeIso::UAligned;
    if (du < kIsoTol && dv > kIsoTol) return EdgeIso::VAligned;
    return EdgeIso::Neither;
}

void collectIsoEdges(const TopoDS_Face& face, const Model& model,
                     const std::vector<int>& edgeIds, FacePlan& plan,
                     bool skipNonIso) {
    BRepAdaptor_Surface surf(face);
    double uRange = surf.LastUParameter() - surf.FirstUParameter();
    double vRange = surf.LastVParameter() - surf.FirstVParameter();
    for (int eid : edgeIds) {
        const TopoDS_Edge edge = TopoDS::Edge(model.edges(eid));
        if (BRep_Tool::Degenerated(edge)) continue;  // apex/pole edges
        switch (edgeIsoDirection(edge, face, uRange, vRange)) {
            case EdgeIso::UAligned: plan.uEdges.push_back(eid); break;
            case EdgeIso::VAligned: plan.vEdges.push_back(eid); break;
            case EdgeIso::Neither:
                // Revolution bands tolerate non-iso edges (pocket cuts,
                // forced full bands): the rims still constrain. Grids
                // need the full 2u+2v structure and bail instead.
                if (skipNonIso) {
                    // A bore exiting through a slanted or curved wall has
                    // a WAVY rim: near-constant v but not iso. It still
                    // owns the border row — the grid must sample its
                    // curve, not a uniform ring, or the contract breaks.
                    double f, l;
                    Handle(Geom2d_Curve) pc =
                        BRep_Tool::CurveOnSurface(edge, face, f, l);
                    if (!pc.IsNull()) {
                        double lo = 1e300, hi = -1e300;
                        for (int k = 0; k <= 8; ++k) {
                            double y = pc->Value(f + (l - f) * k / 8.0).Y();
                            lo = std::min(lo, y);
                            hi = std::max(hi, y);
                        }
                        // Wavy rims (pipe-saddle weld curves) wander in
                        // v; the loftability gate in edgesHugRims-
                        // OrInserts already vetted separation, so any
                        // border edge short of a full-band crossing is
                        // a rim chain member here.
                        if (hi - lo < 0.8 * vRange) {
                            plan.uEdges.push_back(eid);
                            break;
                        }
                    }
                    continue;
                }
                plan.constrains = false;
                return;
        }
    }
    plan.constrains = true;
}

// A planar face with a rectangular (2u+2v iso-edge) outer wire and exactly
// one full-circle inner wire: the cylinder-to-plane junction case. Fills
// plan.circ / circleEdgeId / uEdges / vEdges on success.
bool planRingJunction(const TopoDS_Face& face, const Model& model,
                      FacePlan& plan) {
    BRepAdaptor_Surface surf(face);
    if (surf.GetType() != GeomAbs_Plane) return false;

    TopoDS_Wire outer = BRepTools::OuterWire(face);
    TopoDS_Wire inner;
    int wireCount = 0;
    for (TopExp_Explorer ex(face, TopAbs_WIRE); ex.More(); ex.Next()) {
        ++wireCount;
        if (!ex.Current().IsSame(outer)) inner = TopoDS::Wire(ex.Current());
    }
    if (wireCount != 2 || inner.IsNull()) return false;

    int innerEdgeCount = 0;
    TopoDS_Edge circleEdge;
    for (TopExp_Explorer ex(inner, TopAbs_EDGE); ex.More(); ex.Next()) {
        ++innerEdgeCount;
        circleEdge = TopoDS::Edge(ex.Current());
    }
    if (innerEdgeCount != 1) return false;
    double f = 0, l = 0;
    Handle(Geom_Curve) curve = BRep_Tool::Curve(circleEdge, f, l);
    Handle(Geom_Circle) circle = Handle(Geom_Circle)::DownCast(curve);
    if (circle.IsNull() || std::abs((l - f) - 2.0 * M_PI) > 1e-7) return false;

    std::vector<int> outerIds;
    for (TopExp_Explorer ex(outer, TopAbs_EDGE); ex.More(); ex.Next()) {
        int eid = model.edges.FindIndex(ex.Current());
        if (eid > 0 && std::find(outerIds.begin(), outerIds.end(), eid) ==
                           outerIds.end()) {
            outerIds.push_back(eid);
        }
    }
    if (outerIds.size() != 4) return false;
    collectIsoEdges(face, model, outerIds, plan);
    if (!plan.constrains || plan.uEdges.size() != 2 || plan.vEdges.size() != 2) {
        plan.uEdges.clear();
        plan.vEdges.clear();
        plan.constrains = false;
        return false;
    }

    plan.kind = MesherKind::RingJunction;
    plan.circ = circle->Circ();
    plan.circleEdgeId = model.edges.FindIndex(circleEdge);
    return true;
}

// UV grid feasibility: the grid must lie inside the trim boundary, verified
// by classifying every node and cell center. Conforming a grid to arbitrary
// trim curves is the hard problem (plan §7.1) and stays out of scope.
bool parametricGridFits(const TopoDS_Face& face, const BRepAdaptor_Surface& surf,
                        int nu, int nv) {
    double umin, umax, vmin, vmax;
    BRepTools::UVBounds(face, umin, umax, vmin, vmax);
    // A face spanning a full period needs wrap handling this mesher doesn't
    // do; closed revolutions are handled by RevolutionGrid instead.
    if (surf.IsUPeriodic() && umax - umin > surf.UPeriod() - 1e-9) return false;
    if (surf.IsVPeriodic() && vmax - vmin > surf.VPeriod() - 1e-9) return false;

    // The grid meshes the UV bounding box, so the face must actually FILL
    // its box. Node/center classification alone is far too sparse at low
    // counts (a 1x1 grid probes 5 points) and lets a near-rectangle with
    // small notches through — the grid then overlaps the notch faces. For
    // a plane u/v are arc length, so bbox area is exact: compare it to the
    // true face area.
    if (surf.GetType() == GeomAbs_Plane) {
        GProp_GProps props;
        BRepGProp::SurfaceProperties(face, props);
        const double rect = (umax - umin) * (vmax - vmin);
        if (rect <= 0 || props.Mass() < 0.999 * rect) return false;
    }

    const double du = (umax - umin) / nu;
    const double dv = (vmax - vmin) / nv;
    const double tol = BRep_Tool::Tolerance(face);

    for (int j = 0; j <= nv; ++j) {
        for (int i = 0; i <= nu; ++i) {
            gp_Pnt2d node(umin + i * du, vmin + j * dv);
            BRepClass_FaceClassifier cls(const_cast<TopoDS_Face&>(face), node, tol);
            if (cls.State() == TopAbs_OUT) return false;
        }
    }
    for (int j = 0; j < nv; ++j) {
        for (int i = 0; i < nu; ++i) {
            gp_Pnt2d center(umin + (i + 0.5) * du, vmin + (j + 0.5) * dv);
            BRepClass_FaceClassifier cls(const_cast<TopoDS_Face&>(face), center,
                                         tol);
            if (cls.State() != TopAbs_IN) return false;
        }
    }
    return true;
}

// A closed revolution face is only a clean band if it actually covers its
// surface's full parametric rectangle — a cylinder with pockets trimmed
// into it must NOT mesh as an untrimmed band overlapping its neighbours.
bool revolutionCovers(const TopoDS_Face& face) {
    double umin, umax, vmin, vmax;
    BRepTools::UVBounds(face, umin, umax, vmin, vmax);
    const double tol = BRep_Tool::Tolerance(face);
    for (int j = 1; j < 4; ++j) {
        for (int i = 0; i < 8; ++i) {
            gp_Pnt2d p(umin + (i + 0.5) / 8.0 * (umax - umin),
                       vmin + j / 4.0 * (vmax - vmin));
            BRepClass_FaceClassifier cls(const_cast<TopoDS_Face&>(face), p,
                                         tol);
            if (cls.State() == TopAbs_OUT) return false;
        }
    }
    return true;
}


}  // namespace weft::mesher_impl
