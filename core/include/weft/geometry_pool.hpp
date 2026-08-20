// Type-specific geometric primitive pools (data-oriented).
//
// Extracted once at analyze()/load from BRepAdaptor_Surface. Runtime
// generate() solvers (RevolutionGrid, etc.) evaluate points via plain
// ElSLib-equivalent math on these structs — no BRepAdaptor_Surface on the
// hot path for Plane/Cylinder/Cone/Sphere/Torus. NURBS poles/knots are
// stored for future lock-free sampling; until then BSpline still falls
// back to OCCT. This accelerates the existing weft::generate() path (AD-1);
// it is not a second mesher.
//
#pragma once

#include <cstdint>
#include <vector>

#include <gp_Ax1.hxx>
#include <gp_Ax3.hxx>
#include <gp_Dir.hxx>
#include <gp_Pnt.hxx>

namespace weft {

struct Model;
struct Analysis;

enum class GeomPrimKind : uint8_t {
    None = 0,
    Plane,
    Cylinder,
    Cone,
    Sphere,
    Torus,
    Nurb,
    Other,
};

struct PlanePrimitive {
    uint32_t faceId = 0;
    gp_Ax3 frame;  // location + XDir/YDir/ZDir (normal = ZDir)
    double u0 = 0, u1 = 0, v0 = 0, v1 = 0;
};

struct CylinderPrimitive {
    uint32_t faceId = 0;
    gp_Ax3 frame;  // cylinder position (axis = ZDir through Location)
    double radius = 0;
    double u0 = 0, u1 = 0, v0 = 0, v1 = 0;  // u = angle, v = height
};

struct ConePrimitive {
    uint32_t faceId = 0;
    gp_Ax3 frame;
    double radius = 0;     // at v=0
    double semiAngle = 0;  // radians
    double u0 = 0, u1 = 0, v0 = 0, v1 = 0;
};

struct SpherePrimitive {
    uint32_t faceId = 0;
    gp_Ax3 frame;
    double radius = 0;
    double u0 = 0, u1 = 0, v0 = 0, v1 = 0;
};

struct TorusPrimitive {
    uint32_t faceId = 0;
    gp_Ax3 frame;
    double majorRadius = 0;
    double minorRadius = 0;
    double u0 = 0, u1 = 0, v0 = 0, v1 = 0;
};

struct NurbPrimitive {
    uint32_t faceId = 0;
    int uDegree = 0, vDegree = 0;
    int nU = 0, nV = 0;  // poles grid
    bool rational = false;
    std::vector<gp_Pnt> poles;    // row-major nV rows of nU
    std::vector<double> weights;  // empty if !rational
    std::vector<double> uKnots, vKnots;
    double u0 = 0, u1 = 0, v0 = 0, v1 = 0;
};

// O(1) faceId → (kind, index into the matching pool).
struct FaceGeomSlot {
    GeomPrimKind kind = GeomPrimKind::None;
    uint32_t index = 0;  // into planes/cylinders/...
};

struct GeometryPool {
    std::vector<PlanePrimitive> planes;
    std::vector<CylinderPrimitive> cylinders;
    std::vector<ConePrimitive> cones;
    std::vector<SpherePrimitive> spheres;
    std::vector<TorusPrimitive> tori;
    std::vector<NurbPrimitive> nurbs;
    // Index 0 unused; faceId 1..N → slot. Empty if not built.
    std::vector<FaceGeomSlot> byFace;

    bool empty() const { return byFace.size() <= 1; }
    void clear();

    // OCCT extract — call once from analyze(model).
    void extractFromModel(const Model& model, const Analysis& analysis);

    // Lock-free point evaluation for analytic kinds. Returns false if the
    // face has no analytic slot (Nurb/Other/missing) — caller keeps OCCT.
    bool value(uint32_t faceId, double u, double v, gp_Pnt& out) const;

    const FaceGeomSlot& slot(uint32_t faceId) const {
        static const FaceGeomSlot kNone{};
        if (faceId == 0 || faceId >= byFace.size()) return kNone;
        return byFace[faceId];
    }
};

}  // namespace weft
