#include "weft/secure_reconnaissance.hpp"
#include "weft/model.hpp"
#include "weft/occt_failure.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepGProp.hxx>
#include <BRepTools.hxx>
#include <BRepTopAdaptor_FClass2d.hxx>
#include <BRep_Tool.hxx>
#include <GProp_GProps.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom_OffsetCurve.hxx>
#include <Geom_OffsetSurface.hxx>
#include <Geom_RectangularTrimmedSurface.hxx>
#include <Geom_Surface.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <GeomAbs_CurveType.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <Precision.hxx>
#include <ShapeAnalysis_Wire.hxx>
#include <Standard_Failure.hxx>
#include <TopAbs.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Dir.hxx>
#include <gp_Pln.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <map>
#include <set>
#include <string>
#include <tuple>
#include <utility>
#include <vector>

namespace weft {
namespace {

struct FamilyInfo {
    std::string code;
    bool analytic = false;
    bool known = false;
    bool firstTemplate = false;
};

FamilyInfo curveFamily(GeomAbs_CurveType type) {
    switch (type) {
        case GeomAbs_Line: return {"line", true, true, true};
        case GeomAbs_Circle: return {"circle", true, true, true};
        case GeomAbs_Ellipse: return {"ellipse", true, true, true};
        case GeomAbs_Hyperbola: return {"hyperbola", true, true, false};
        case GeomAbs_Parabola: return {"parabola", true, true, false};
        case GeomAbs_BezierCurve: return {"bezier", false, true, false};
        case GeomAbs_BSplineCurve: return {"bspline", false, true, false};
        case GeomAbs_OffsetCurve: return {"offset", false, true, false};
        case GeomAbs_OtherCurve:
        default: return {"kernel_specific", false, false, false};
    }
}

FamilyInfo surfaceFamily(GeomAbs_SurfaceType type) {
    switch (type) {
        case GeomAbs_Plane: return {"plane", true, true, true};
        case GeomAbs_Cylinder: return {"cylinder", true, true, true};
        case GeomAbs_Cone: return {"cone", true, true, true};
        case GeomAbs_Sphere: return {"sphere", true, true, true};
        case GeomAbs_Torus: return {"torus", true, true, true};
        case GeomAbs_BezierSurface: return {"bezier", false, true, false};
        case GeomAbs_BSplineSurface: return {"bspline", false, true, false};
        case GeomAbs_SurfaceOfRevolution:
            return {"revolution", false, true, false};
        case GeomAbs_SurfaceOfExtrusion:
            return {"extrusion", false, true, false};
        case GeomAbs_OffsetSurface: return {"offset", false, true, false};
        case GeomAbs_OtherSurface:
        default: return {"kernel_specific", false, false, false};
    }
}

ExactFamilyProbe toProbe(const FamilyInfo& family) {
    ExactFamilyProbe probe;
    probe.familyCode = family.code;
    if (!family.known) {
        probe.confidence = RecognitionConfidence::NotRecognised;
        probe.support = GeometrySupportState::UnrecognisedExactGeometry;
        probe.strategyOrReasonCode = "reason.unrecognised_exact_geometry";
        return probe;
    }
    if (family.analytic && family.firstTemplate) {
        probe.confidence = RecognitionConfidence::ProvenAnalytic;
        probe.support = GeometrySupportState::SupportedAnalyticTemplate;
        probe.strategyOrReasonCode = "strategy." + family.code;
        return probe;
    }
    probe.confidence = family.analytic ? RecognitionConfidence::ProvenAnalytic
                                       : RecognitionConfidence::ExactlyTyped;
    probe.support = GeometrySupportState::DeferredResidualSurface;
    probe.strategyOrReasonCode = "reason.deferred_residual_surface";
    return probe;
}

std::pair<std::string, std::vector<std::string>> concreteCurve(
    Handle(Geom_Curve) curve) {
    std::vector<std::string> wrappers;
    while (!curve.IsNull()) {
        Handle(Geom_TrimmedCurve) trimmed =
            Handle(Geom_TrimmedCurve)::DownCast(curve);
        if (!trimmed.IsNull()) {
            wrappers.push_back("trimmed");
            curve = trimmed->BasisCurve();
            continue;
        }
        Handle(Geom_OffsetCurve) offset =
            Handle(Geom_OffsetCurve)::DownCast(curve);
        if (!offset.IsNull()) {
            wrappers.push_back("offset");
            curve = offset->BasisCurve();
            continue;
        }
        break;
    }
    return {curve.IsNull() ? std::string("Geom_Curve")
                           : std::string(curve->DynamicType()->Name()),
            std::move(wrappers)};
}

std::pair<std::string, std::vector<std::string>> concreteSurface(
    Handle(Geom_Surface) surface) {
    std::vector<std::string> wrappers;
    while (!surface.IsNull()) {
        Handle(Geom_RectangularTrimmedSurface) trimmed =
            Handle(Geom_RectangularTrimmedSurface)::DownCast(surface);
        if (!trimmed.IsNull()) {
            wrappers.push_back("rectangular_trimmed");
            surface = trimmed->BasisSurface();
            continue;
        }
        Handle(Geom_OffsetSurface) offset =
            Handle(Geom_OffsetSurface)::DownCast(surface);
        if (!offset.IsNull()) {
            wrappers.push_back("offset");
            surface = offset->BasisSurface();
            continue;
        }
        break;
    }
    return {surface.IsNull() ? std::string("Geom_Surface")
                             : std::string(surface->DynamicType()->Name()),
            std::move(wrappers)};
}

std::vector<StableId> sourceSubjects(const ImportedModel& imported,
                                     StableId workingId) {
    std::vector<StableId> result;
    for (const CorrespondenceRecord& record : imported.correspondence.records) {
        if (!record.sourceId.valid()) continue;
        if (std::find(record.workingIds.begin(), record.workingIds.end(),
                      workingId) != record.workingIds.end()) {
            result.push_back(record.sourceId);
        }
    }
    std::sort(result.begin(), result.end());
    result.erase(std::unique(result.begin(), result.end()), result.end());
    return result;
}

bool wireHasOpenGap(const TopoDS_Wire& wire, const TopoDS_Face& face) {
    if (wire.IsNull()) return true;
    if (!wire.Closed()) return true;
    try {
        TopoDS_Vertex first;
        TopoDS_Vertex last;
        TopExp::Vertices(wire, first, last);
        if (first.IsNull() || last.IsNull()) return true;
        // Endpoint coincidence is the hard open-loop witness. Broader
        // ShapeAnalysis connectivity/gap probes false-positive on valid
        // periodic seam wires (cylinder walls) under OCCT 7.9/8.0.
        if (!first.IsSame(last) ||
            BRep_Tool::Pnt(first).Distance(BRep_Tool::Pnt(last)) >
                Precision::Confusion()) {
            return true;
        }
        ShapeAnalysis_Wire analysis(wire, face, Precision::Confusion());
        return analysis.CheckClosed(Precision::Confusion());
    } catch (const Standard_Failure&) {
        return false;
    }
}

bool wireSelfIntersects(const TopoDS_Wire& wire, const TopoDS_Face& face) {
    if (wire.IsNull()) return false;
    try {
        ShapeAnalysis_Wire analysis(wire, face, Precision::Confusion());
        return analysis.CheckSelfIntersection();
    } catch (const Standard_Failure&) {
        return false;
    }
}

bool planarOuterWireIsConcave(const TopoDS_Face& face,
                              const TopoDS_Wire& outer) {
    if (outer.IsNull()) return false;
    BRepAdaptor_Surface surface(face, true);
    if (surface.GetType() != GeomAbs_Plane) return false;
    const gp_Dir normal = surface.Plane().Axis().Direction();
    std::vector<gp_Pnt> points;
    for (TopoDS_Iterator edgeIt(outer); edgeIt.More(); edgeIt.Next()) {
        if (edgeIt.Value().ShapeType() != TopAbs_EDGE) continue;
        const TopoDS_Edge edge = TopoDS::Edge(edgeIt.Value());
        TopoDS_Vertex first;
        TopoDS_Vertex last;
        TopExp::Vertices(edge, first, last);
        if (first.IsNull()) continue;
        const gp_Pnt point = BRep_Tool::Pnt(first);
        if (points.empty() || points.back().Distance(point) > 1.0e-9) {
            points.push_back(point);
        }
    }
    if (points.size() < 3) return false;
    if (points.front().Distance(points.back()) <= 1.0e-9) {
        points.pop_back();
    }
    if (points.size() < 3) return false;

    int sign = 0;
    for (std::size_t index = 0; index < points.size(); ++index) {
        const gp_Pnt& a = points[index];
        const gp_Pnt& b = points[(index + 1) % points.size()];
        const gp_Pnt& c = points[(index + 2) % points.size()];
        const gp_Vec ab(a, b);
        const gp_Vec bc(b, c);
        const double orient = ab.Crossed(bc).Dot(gp_Vec(normal.XYZ()));
        if (std::abs(orient) <= 1.0e-12) continue;
        const int current = orient > 0.0 ? 1 : -1;
        if (sign == 0) {
            sign = current;
        } else if (sign != current) {
            return true;
        }
    }
    return false;
}

std::size_t countWires(const TopoDS_Face& face) {
    std::size_t wires = 0;
    for (TopExp_Explorer explorer(face, TopAbs_WIRE); explorer.More();
         explorer.Next()) {
        ++wires;
    }
    return wires;
}

std::size_t countDegenerateEdges(const TopoDS_Face& face) {
    std::size_t degenerateEdges = 0;
    for (TopExp_Explorer explorer(face, TopAbs_EDGE); explorer.More();
         explorer.Next()) {
        if (BRep_Tool::Degenerated(TopoDS::Edge(explorer.Current()))) {
            ++degenerateEdges;
        }
    }
    return degenerateEdges;
}

bool wireSampleInsideOuter(const TopoDS_Wire& wire, const TopoDS_Face& face,
                           const TopoDS_Wire& outer) {
    if (wire.IsNull() || outer.IsNull() || wire.IsSame(outer)) return true;
    Handle(Geom_Surface) exact = BRep_Tool::Surface(face);
    if (exact.IsNull()) return false;
    BRepBuilderAPI_MakeFace outerFace(exact, outer);
    if (!outerFace.IsDone()) return false;
    BRepTopAdaptor_FClass2d classifier(outerFace.Face(),
                                       Precision::Confusion());
    for (TopoDS_Iterator edgeIt(wire); edgeIt.More(); edgeIt.Next()) {
        if (edgeIt.Value().ShapeType() != TopAbs_EDGE) continue;
        const TopoDS_Edge edge = TopoDS::Edge(edgeIt.Value());
        double first = 0.0;
        double last = 0.0;
        Handle(Geom2d_Curve) pcurve =
            BRep_Tool::CurveOnSurface(edge, face, first, last);
        if (pcurve.IsNull() || !(last > first)) continue;
        const gp_Pnt2d sample = pcurve->Value(0.5 * (first + last));
        // Require a strict interior sample so overlapping/ambiguous outer-like
        // wires do not count as proven holes.
        return classifier.Perform(sample) == TopAbs_IN;
    }
    return false;
}

bool allNonOuterWiresInsideOuter(const TopoDS_Face& face,
                                 const TopoDS_Wire& outer) {
    if (outer.IsNull()) return false;
    for (TopExp_Explorer explorer(face, TopAbs_WIRE); explorer.More();
         explorer.Next()) {
        const TopoDS_Wire wire = TopoDS::Wire(explorer.Current());
        if (wire.IsSame(outer)) continue;
        if (!wireSampleInsideOuter(wire, face, outer)) return false;
    }
    return true;
}

bool nestingIsProven(const TopoDS_Face& face, const TopoDS_Wire& outer,
                     std::size_t wires) {
    if (outer.IsNull() || wires < 2) return false;
    if (!allNonOuterWiresInsideOuter(face, outer)) return false;
    Handle(Geom_Surface) exact = BRep_Tool::Surface(face);
    if (exact.IsNull()) return false;
    BRepBuilderAPI_MakeFace outerFace(exact, outer);
    if (!outerFace.IsDone()) return false;
    GProp_GProps faceProps;
    GProp_GProps outerProps;
    BRepGProp::SurfaceProperties(face, faceProps);
    BRepGProp::SurfaceProperties(outerFace.Face(), outerProps);
    const double faceArea = std::abs(faceProps.Mass());
    const double outerArea = std::abs(outerProps.Mass());
    if (!(faceArea > 0.0) || !(outerArea > 0.0)) return false;
    // Proven holes must remove measurable area from the outer claim.
    return faceArea + 1.0e-6 < outerArea;
}

TrimDomainClass classifyTrim(const TopoDS_Face& face,
                             const BRepAdaptor_Surface& surface) {
    const TopoDS_Wire outer = BRepTools::OuterWire(face);
    const std::size_t wires = countWires(face);
    if (wires == 0) return TrimDomainClass::InvalidOrUnresolved;

    bool openOrGapped = false;
    bool selfIntersecting = false;
    for (TopExp_Explorer explorer(face, TopAbs_WIRE); explorer.More();
         explorer.Next()) {
        const TopoDS_Wire wire = TopoDS::Wire(explorer.Current());
        if (wireHasOpenGap(wire, face)) openOrGapped = true;
        if (wireSelfIntersects(wire, face)) selfIntersecting = true;
    }
    if (selfIntersecting) return TrimDomainClass::SelfIntersectingOrInvalid;
    if (openOrGapped) return TrimDomainClass::OpenOrGappedLoop;

    const std::size_t degenerateEdges = countDegenerateEdges(face);
    if (degenerateEdges >= 2) return TrimDomainClass::TouchesTwoSingularities;
    if (degenerateEdges == 1) return TrimDomainClass::TouchesOneSingularity;

    if (surface.IsUPeriodic() || surface.IsVPeriodic()) {
        double u0 = 0.0;
        double u1 = 0.0;
        double v0 = 0.0;
        double v1 = 0.0;
        BRepTools::UVBounds(face, u0, u1, v0, v1);
        const double uSpan = u1 - u0;
        const double vSpan = v1 - v0;
        const double uPeriod =
            surface.IsUPeriodic() ? surface.UPeriod() : 0.0;
        const double vPeriod =
            surface.IsVPeriodic() ? surface.VPeriod() : 0.0;
        // Treat near-full periods as full. Tiny UV-bound shortfalls (common
        // across OCCT 7.9/8.0 cylinder imports) must not demote a closed wall
        // to a seam-crossing band and refuse the cylinder template.
        const auto isMeaningfulBand = [](double span, double period) {
            if (!(period > 0.0) || !(span >= 0.0)) return false;
            const double gap = period - span;
            const double threshold =
                std::max(Precision::Confusion() * 10.0, period * 1.0e-4);
            return gap > threshold;
        };
        const bool uBand =
            surface.IsUPeriodic() && isMeaningfulBand(uSpan, uPeriod);
        const bool vBand =
            surface.IsVPeriodic() && isMeaningfulBand(vSpan, vPeriod);
        if (uBand || vBand) {
            return TrimDomainClass::PeriodicBandCrossingSeam;
        }
        return TrimDomainClass::FullPeriodicWithCapBoundaries;
    }

    if (wires == 2) {
        if (outer.IsNull()) {
            return TrimDomainClass::AmbiguousNestingOrOrientation;
        }
        return TrimDomainClass::Annulus;
    }
    if (wires > 2) {
        // Multiply-perforated only when every non-outer wire is strictly inside
        // the outer claim and the face area is strictly smaller than the outer
        // disk. Otherwise the nesting/orientation is unproven.
        if (!nestingIsProven(face, outer, wires)) {
            return TrimDomainClass::AmbiguousNestingOrOrientation;
        }
        return TrimDomainClass::MultiplyPerforatedDisk;
    }

    if (!outer.IsNull()) {
        std::size_t edges = 0;
        bool circular = false;
        for (TopoDS_Iterator iterator(outer); iterator.More();
             iterator.Next()) {
            if (iterator.Value().ShapeType() != TopAbs_EDGE) continue;
            ++edges;
            BRepAdaptor_Curve curve(TopoDS::Edge(iterator.Value()));
            circular = curve.GetType() == GeomAbs_Circle;
        }
        if (edges == 1 && circular) return TrimDomainClass::SimpleDisk;
        if (planarOuterWireIsConcave(face, outer)) {
            return TrimDomainClass::ConcaveSimpleRegion;
        }
    }
    return TrimDomainClass::ConvexSimpleRegion;
}

ParameterDomain surfaceDomain(double first, double last, bool closed,
                              bool periodic, double period) {
    ParameterDomain domain;
    if (std::isfinite(first)) domain.lower = first;
    if (std::isfinite(last)) domain.upper = last;
    domain.closed = closed;
    domain.periodic = periodic;
    if (periodic && std::isfinite(period) && period > 0.0) {
        domain.period = period;
    }
    return domain;
}

bool exactSurfaceEvaluates(const GeometryEvaluator& evaluator, StableId face,
                           double u0, double u1, double v0, double v1) {
    if (!std::isfinite(u0) || !std::isfinite(u1) || !std::isfinite(v0) ||
        !std::isfinite(v1)) {
        return false;
    }
    return static_cast<bool>(
        evaluator.evaluateSurface(face, {(u0 + u1) * 0.5, (v0 + v1) * 0.5}));
}

bool edgeIsDegenerate(const BRepSnapshot& snapshot, StableId edgeId) {
    for (const EdgeTopologyRecord& topology : snapshot.edgeTopology) {
        if (topology.id == edgeId) return topology.degenerate;
    }
    return false;
}

bool curvedFaceHasExactMappings(const ImportedModel& imported, StableId faceId) {
    if (!imported.working || !imported.workingEvaluator) return false;
    bool sawCoedge = false;
    for (const CoedgeRecord& coedge : imported.working->snapshot.coedges) {
        if (coedge.faceId != faceId) continue;
        sawCoedge = true;
        if (coedge.pcurveRepresentations.empty()) return false;
        // Apex/pole singular edges may lack a finite curve domain; CONE-B
        // builds their UV stations separately. Do not block template readiness.
        if (edgeIsDegenerate(imported.working->snapshot, coedge.edgeId)) {
            continue;
        }
        const auto domain = imported.workingEvaluator->curveDomain(coedge.edgeId);
        if (!domain || !domain.value->lower || !domain.value->upper) return false;
        const double parameter =
            (*domain.value->lower + *domain.value->upper) * 0.5;
        for (const PcurveRef& representation : coedge.pcurveRepresentations) {
            if (!imported.workingEvaluator->evaluateCurveOnSurface(
                    representation, parameter)) {
                return false;
            }
        }
    }
    return sawCoedge;
}

void decideSupport(ExactGeometryClassification& record, const FamilyInfo& family,
                   bool evaluates, bool templateReady,
                   ReconnaissanceReport& report) {
    if (!evaluates) {
        record.confidence = family.analytic
                                ? RecognitionConfidence::ProvenAnalytic
                                : RecognitionConfidence::ExactlyTyped;
        record.support = GeometrySupportState::InvalidImportedGeometry;
        record.strategyOrReasonCode = "reason.invalid_imported_geometry";
        report.diagnostics.push_back(
            {"reconnaissance.geometry.invalid", record.subjectId,
             "exact geometry or its required representation did not evaluate"});
        ++report.unsupportedSubjects;
        return;
    }
    if (!family.known) {
        record.confidence = RecognitionConfidence::NotRecognised;
        record.support = GeometrySupportState::UnrecognisedExactGeometry;
        record.strategyOrReasonCode = "reason.unrecognised_exact_geometry";
        report.diagnostics.push_back(
            {"reconnaissance.geometry.unrecognised", record.subjectId,
             "exact OCCT type is not a recognised family"});
        ++report.unsupportedSubjects;
        return;
    }
    if (family.analytic && family.firstTemplate && templateReady) {
        record.confidence = RecognitionConfidence::ProvenAnalytic;
        record.support = GeometrySupportState::SupportedAnalyticTemplate;
        record.strategyOrReasonCode =
            record.taxonomy == GeometryTaxonomy::Curve
                ? "strategy.curve.exact_" + family.code
                : "strategy.surface." + family.code;
        return;
    }
    record.confidence = family.analytic
                            ? RecognitionConfidence::ProvenAnalytic
                            : RecognitionConfidence::ExactlyTyped;
    record.support = GeometrySupportState::DeferredResidualSurface;
    record.strategyOrReasonCode = "reason.deferred_residual_surface";
    ++report.unsupportedSubjects;
}

bool facesAreCoplanarPartners(const TopoDS_Face& left, const TopoDS_Face& right) {
    BRepAdaptor_Surface leftSurface(left, true);
    BRepAdaptor_Surface rightSurface(right, true);
    if (leftSurface.GetType() != GeomAbs_Plane ||
        rightSurface.GetType() != GeomAbs_Plane) {
        return false;
    }
    const gp_Pln leftPlane = leftSurface.Plane();
    const gp_Pln rightPlane = rightSurface.Plane();
    if (!leftPlane.Axis().Direction().IsParallel(rightPlane.Axis().Direction(),
                                                 1.0e-9) &&
        !leftPlane.Axis().Direction().IsOpposite(rightPlane.Axis().Direction(),
                                                 1.0e-9)) {
        return false;
    }
    return std::abs(leftPlane.Distance(rightPlane.Location())) <= 1.0e-7;
}

using PlaneBucketKey = std::tuple<long, long, long, long>;

PlaneBucketKey planeBucketKey(const TopoDS_Face& face) {
    BRepAdaptor_Surface surface(face, true);
    if (surface.GetType() != GeomAbs_Plane) {
        return {0, 0, 0, std::numeric_limits<long>::min()};
    }
    gp_Pln plane = surface.Plane();
    gp_Dir direction = plane.Axis().Direction();
    // Canonicalize opposite normals into one bucket.
    if (direction.Z() < -1.0e-12 ||
        (std::abs(direction.Z()) <= 1.0e-12 && direction.Y() < -1.0e-12) ||
        (std::abs(direction.Z()) <= 1.0e-12 &&
         std::abs(direction.Y()) <= 1.0e-12 && direction.X() < 0.0)) {
        direction.Reverse();
    }
    const gp_Pnt origin = plane.Location();
    const double offset =
        direction.X() * origin.X() + direction.Y() * origin.Y() +
        direction.Z() * origin.Z();
    return {std::lround(direction.X() * 1.0e6),
            std::lround(direction.Y() * 1.0e6),
            std::lround(direction.Z() * 1.0e6), std::lround(offset * 1.0e6)};
}

void reconProgress(const char* stage) {
    static const bool enabled = [] {
        const char* value = std::getenv("WEFT_IMPORT_PROGRESS");
        return value != nullptr && value[0] != '\0' && value[0] != '0';
    }();
    if (!enabled || stage == nullptr) return;
    std::fprintf(stderr, "WEFT_PROGRESS recon.%s\n", stage);
    std::fflush(stderr);
}

}  // namespace

const ExactGeometryClassification* ReconnaissanceReport::find(
    StableId subject) const noexcept {
    const auto found = std::find_if(
        records.begin(), records.end(),
        [subject](const ExactGeometryClassification& record) {
            return record.subjectId == subject;
        });
    return found == records.end() ? nullptr : &*found;
}

const char* geometrySupportStateName(GeometrySupportState state) noexcept {
    switch (state) {
        case GeometrySupportState::SupportedAnalyticTemplate:
            return "supported_analytic_template";
        case GeometrySupportState::DeferredResidualSurface:
            return "deferred_residual_surface";
        case GeometrySupportState::InvalidImportedGeometry:
            return "invalid_imported_geometry";
        case GeometrySupportState::UnrecognisedExactGeometry:
            return "unrecognised_exact_geometry";
    }
    return "unrecognised_exact_geometry";
}

const char* trimDomainClassName(TrimDomainClass value) noexcept {
    switch (value) {
        case TrimDomainClass::SimpleDisk: return "simple_disk";
        case TrimDomainClass::Annulus: return "annulus";
        case TrimDomainClass::MultiplyPerforatedDisk:
            return "multiply_perforated_disk";
        case TrimDomainClass::ConcaveSimpleRegion:
            return "concave_simple_region";
        case TrimDomainClass::ConvexSimpleRegion:
            return "convex_simple_region";
        case TrimDomainClass::PeriodicBandCrossingSeam:
            return "periodic_band_crossing_seam";
        case TrimDomainClass::FullPeriodicWithCapBoundaries:
            return "full_periodic_with_cap_boundaries";
        case TrimDomainClass::TouchesOneSingularity:
            return "touches_one_singularity";
        case TrimDomainClass::TouchesTwoSingularities:
            return "touches_two_singularities";
        case TrimDomainClass::MultipleDisconnectedDomains:
            return "multiple_disconnected_domains";
        case TrimDomainClass::SelfIntersectingOrInvalid:
            return "self_intersecting_or_invalid";
        case TrimDomainClass::OpenOrGappedLoop: return "open_or_gapped_loop";
        case TrimDomainClass::AmbiguousNestingOrOrientation:
            return "ambiguous_nesting_or_orientation";
        case TrimDomainClass::InvalidOrUnresolved:
            return "invalid_or_unresolved";
    }
    return "invalid_or_unresolved";
}

ExactFamilyProbe probeCurveFamily(int geomAbsCurveType) noexcept {
    return toProbe(curveFamily(static_cast<GeomAbs_CurveType>(geomAbsCurveType)));
}

ExactFamilyProbe probeSurfaceFamily(int geomAbsSurfaceType) noexcept {
    return toProbe(
        surfaceFamily(static_cast<GeomAbs_SurfaceType>(geomAbsSurfaceType)));
}

ReconnaissanceReport reconnoitre(const ImportedModel& imported) {
    ReconnaissanceReport report;
    if (!imported.working || !imported.workingEvaluator) {
        report.diagnostics.push_back(
            {"reconnaissance.working_model_missing", {},
             "reconnaissance requires a working B-rep and exact evaluator"});
        return report;
    }

    const BRepSnapshot& snapshot = imported.working->snapshot;
    report.expectedSubjects = static_cast<std::size_t>(
        snapshot.model.faces.Extent() + snapshot.model.edges.Extent());
    const int edgeCount = snapshot.model.edges.Extent();
    const int faceCount = snapshot.model.faces.Extent();
    const int edgeStride = std::max(1, edgeCount / 20);
    const int faceStride = std::max(1, faceCount / 20);
    {
        const std::string msg = "classify.begin edges=" +
            std::to_string(edgeCount) + " faces=" + std::to_string(faceCount);
        reconProgress(msg.c_str());
    }

    for (int edgeIndex = 1; edgeIndex <= edgeCount; ++edgeIndex) {
        if (edgeIndex % edgeStride == 0) {
            const std::string msg = "classify.edge " +
                std::to_string(edgeIndex) + "/" + std::to_string(edgeCount);
            reconProgress(msg.c_str());
        }
        const StableId edgeId{StableIdKind::Edge,
                              static_cast<std::uint64_t>(edgeIndex)};
        const TopoDS_Edge edge = TopoDS::Edge(snapshot.model.edges(edgeIndex));
        ExactGeometryClassification record;
        record.subjectId = edgeId;
        record.taxonomy = GeometryTaxonomy::Curve;
        record.sourceSubjects = sourceSubjects(imported, edgeId);
        try {
            double first = 0.0;
            double last = 0.0;
            Handle(Geom_Curve) exact = BRep_Tool::Curve(edge, first, last);
            const auto concrete = concreteCurve(exact);
            record.concreteType = concrete.first;
            record.wrapperStack = concrete.second;
            BRepAdaptor_Curve adaptor(edge);
            const FamilyInfo family = curveFamily(adaptor.GetType());
            record.familyCode = family.code;
            const auto domain = imported.workingEvaluator->curveDomain(edgeId);
            const bool evaluates = domain && domain.value->lower &&
                domain.value->upper &&
                imported.workingEvaluator->evaluateCurve(
                    edgeId, (*domain.value->lower + *domain.value->upper) * 0.5);
            if (domain) record.parameterDomains.push_back(*domain.value);
            if (BRep_Tool::Degenerated(edge)) {
                record.conditionCodes.push_back("degenerate");
            }
            decideSupport(record, family, evaluates, evaluates, report);
        } catch (const Standard_Failure& error) {
            record.familyCode = "kernel_specific";
            record.concreteType = "Geom_Curve";
            record.support = GeometrySupportState::InvalidImportedGeometry;
            record.strategyOrReasonCode = "reason.invalid_imported_geometry";
            report.diagnostics.push_back(
                {"reconnaissance.curve.failure", edgeId,
                 occtFailureMessage(error)});
            ++report.unsupportedSubjects;
        }
        report.records.push_back(std::move(record));
    }
    reconProgress("classify.edges.done");

    for (int faceIndex = 1; faceIndex <= faceCount; ++faceIndex) {
        if (faceIndex % faceStride == 0) {
            const std::string msg = "classify.face " +
                std::to_string(faceIndex) + "/" + std::to_string(faceCount);
            reconProgress(msg.c_str());
        }
        const StableId faceId{StableIdKind::Face,
                              static_cast<std::uint64_t>(faceIndex)};
        const TopoDS_Face face = TopoDS::Face(snapshot.model.faces(faceIndex));
        ExactGeometryClassification record;
        record.subjectId = faceId;
        record.taxonomy = GeometryTaxonomy::Surface;
        record.sourceSubjects = sourceSubjects(imported, faceId);
        try {
            Handle(Geom_Surface) exact = BRep_Tool::Surface(face);
            const auto concrete = concreteSurface(exact);
            record.concreteType = concrete.first;
            record.wrapperStack = concrete.second;
            BRepAdaptor_Surface adaptor(face, true);
            const FamilyInfo family = surfaceFamily(adaptor.GetType());
            record.familyCode = family.code;
            double u0 = 0.0;
            double u1 = 0.0;
            double v0 = 0.0;
            double v1 = 0.0;
            BRepTools::UVBounds(face, u0, u1, v0, v1);
            record.parameterDomains = {
                surfaceDomain(u0, u1, adaptor.IsUClosed(), adaptor.IsUPeriodic(),
                              adaptor.IsUPeriodic() ? adaptor.UPeriod() : 0.0),
                surfaceDomain(v0, v1, adaptor.IsVClosed(), adaptor.IsVPeriodic(),
                              adaptor.IsVPeriodic() ? adaptor.VPeriod() : 0.0),
            };
            record.trimDomain = classifyTrim(face, adaptor);
            if (adaptor.IsUPeriodic()) {
                record.conditionCodes.push_back("u_periodic");
            }
            if (adaptor.IsVPeriodic()) {
                record.conditionCodes.push_back("v_periodic");
            }
            if (record.trimDomain ==
                TrimDomainClass::MultipleDisconnectedDomains) {
                record.conditionCodes.push_back(
                    "reason.multidomain_decomposition_unproven");
            }
            // MAP-A: tag four-sided non-plane analytic/typed faces as Coons
            // candidates. Support stays deferred until MAP-C has a consumer.
            {
                std::set<StableId> faceEdges;
                for (const CoedgeRecord& coedge : snapshot.coedges) {
                    if (coedge.faceId == faceId) {
                        faceEdges.insert(coedge.edgeId);
                    }
                }
                const bool mappedFamily =
                    family.code == "bspline" || family.code == "bezier" ||
                    family.code == "extrusion" || family.code == "revolution" ||
                    family.code == "offset";
                if (mappedFamily && faceEdges.size() == 4) {
                    // WP-174: periodic extrusion/offset bands are not Coons
                    // patches — UV rectangles self-intersect in 3D.
                    const bool periodicBand =
                        record.trimDomain &&
                        (*record.trimDomain ==
                             TrimDomainClass::FullPeriodicWithCapBoundaries ||
                         *record.trimDomain ==
                             TrimDomainClass::PeriodicBandCrossingSeam);
                    if (periodicBand && (family.code == "extrusion" ||
                                         family.code == "offset")) {
                        record.conditionCodes.push_back(
                            family.code == "offset"
                                ? "offset.periodic_band_deferred"
                                : "extrusion.periodic_band_deferred");
                    } else {
                        record.conditionCodes.push_back(
                            "mapped.four_sided_candidate");
                    }
                } else if (mappedFamily && faceEdges.size() != 4) {
                    record.conditionCodes.push_back(
                        "mapped.non_four_sided_deferred");
                    if (family.code == "offset" ||
                        family.code == "extrusion") {
                        record.conditionCodes.push_back(
                            family.code == "offset"
                                ? "offset.non_four_sided_deferred"
                                : "extrusion.non_four_sided_deferred");
                    }
                }
                // FREE-A / WP-173: rectangular UV-grid floor is proven for
                // ≤5 outer edges (MP9 f28 pent + four-sided patches). Six or
                // more edges land off the structured border
                // (seam_sample_unmatched / UV-degenerate). Name that subclass.
                if (family.code == "bspline" || family.code == "bezier") {
                    const bool uvGridTrim =
                        record.trimDomain &&
                        (*record.trimDomain == TrimDomainClass::Annulus ||
                         *record.trimDomain ==
                             TrimDomainClass::MultiplyPerforatedDisk ||
                         *record.trimDomain ==
                             TrimDomainClass::ConvexSimpleRegion ||
                         *record.trimDomain ==
                             TrimDomainClass::ConcaveSimpleRegion ||
                         *record.trimDomain == TrimDomainClass::SimpleDisk);
                    if (uvGridTrim && faceEdges.size() <= 5) {
                        record.conditionCodes.push_back(
                            "freeform.uv_grid_candidate");
                    } else if (uvGridTrim && faceEdges.size() > 5) {
                        record.conditionCodes.push_back(
                            "freeform.high_edge_count_deferred");
                    } else {
                        record.conditionCodes.push_back(
                            "freeform.general_deferred");
                    }
                }
            }
            // CUT-A: planar faces with hole trim + cylindrical bore walls.
            if (family.code == "plane" && record.trimDomain &&
                (*record.trimDomain == TrimDomainClass::Annulus ||
                 *record.trimDomain ==
                     TrimDomainClass::MultiplyPerforatedDisk)) {
                record.conditionCodes.push_back("cutout.planar_perforated");
                int circleEdges = 0;
                int lineEdges = 0;
                std::set<StableId> faceEdgeIds;
                for (const CoedgeRecord& coedge : snapshot.coedges) {
                    if (coedge.faceId == faceId) {
                        faceEdgeIds.insert(coedge.edgeId);
                    }
                }
                for (const StableId& edgeId : faceEdgeIds) {
                    const ExactGeometryClassification* edgeRec = nullptr;
                    for (const ExactGeometryClassification& prior :
                         report.records) {
                        if (prior.subjectId == edgeId) {
                            edgeRec = &prior;
                            break;
                        }
                    }
                    if (!edgeRec) continue;
                    if (edgeRec->familyCode == "circle") {
                        ++circleEdges;
                    } else if (edgeRec->familyCode == "line") {
                        ++lineEdges;
                    }
                }
                // Rectangular/slot-like inner wires are all lines (no circles).
                if (circleEdges == 0 && lineEdges >= 6) {
                    record.conditionCodes.push_back("cutout.planar_slotted");
                }
            }
            if (family.code == "cylinder" && record.trimDomain &&
                (*record.trimDomain ==
                     TrimDomainClass::FullPeriodicWithCapBoundaries ||
                 *record.trimDomain ==
                     TrimDomainClass::PeriodicBandCrossingSeam)) {
                // A cylinder adjacent only to perforated planes is a bore.
                bool touchesPerforatedPlane = false;
                for (const CoedgeRecord& coedge : snapshot.coedges) {
                    if (coedge.faceId != faceId) continue;
                    for (const CoedgeRecord& other : snapshot.coedges) {
                        if (other.edgeId != coedge.edgeId ||
                            other.faceId == faceId) {
                            continue;
                        }
                        const ExactGeometryClassification* neighbor = nullptr;
                        for (const ExactGeometryClassification& prior :
                             report.records) {
                            if (prior.subjectId == other.faceId) {
                                neighbor = &prior;
                                break;
                            }
                        }
                        // Neighbor may not be classified yet (faces are in
                        // ascending index order). Fall back to edge count:
                        // through-hole bores typically share circular edges
                        // with planar faces that have >4 boundary edges.
                        std::set<StableId> neighborEdges;
                        for (const CoedgeRecord& nc : snapshot.coedges) {
                            if (nc.faceId == other.faceId) {
                                neighborEdges.insert(nc.edgeId);
                            }
                        }
                        if (neighborEdges.size() >= 5) {
                            touchesPerforatedPlane = true;
                        }
                        if (neighbor &&
                            std::find(neighbor->conditionCodes.begin(),
                                      neighbor->conditionCodes.end(),
                                      "cutout.planar_perforated") !=
                                neighbor->conditionCodes.end()) {
                            touchesPerforatedPlane = true;
                        }
                    }
                }
                if (touchesPerforatedPlane) {
                    record.conditionCodes.push_back(
                        "cutout.cylindrical_bore");
                }
            }
            const bool evaluates = exactSurfaceEvaluates(
                *imported.workingEvaluator, faceId, u0, u1, v0, v1);
            // Analytic families may derive UV via ElSLib; mapped/freeform
            // candidates may use GeomAPI projection when STEP omitted
            // p-curves (MP9 extrusion/offset/bspline patches).
            const bool analyticUvDerivable =
                family.code == "plane" || family.code == "cylinder" ||
                family.code == "cone" || family.code == "sphere" ||
                family.code == "torus";
            const bool mappedFourSided =
                std::find(record.conditionCodes.begin(),
                          record.conditionCodes.end(),
                          "mapped.four_sided_candidate") !=
                record.conditionCodes.end();
            const bool freeformUvGrid =
                std::find(record.conditionCodes.begin(),
                          record.conditionCodes.end(),
                          "freeform.uv_grid_candidate") !=
                record.conditionCodes.end();
            const bool representationReady =
                analyticUvDerivable || mappedFourSided || freeformUvGrid ||
                curvedFaceHasExactMappings(imported, faceId);
            // MAP-C / FREE-C: UV-grid candidates are first-template ready when
            // representations evaluate, even though FamilyInfo::firstTemplate
            // stays false for general bspline/extrusion.
            if ((mappedFourSided || freeformUvGrid) && evaluates &&
                representationReady) {
                record.confidence = RecognitionConfidence::ProvenAnalytic;
                record.support =
                    GeometrySupportState::SupportedAnalyticTemplate;
                record.strategyOrReasonCode =
                    mappedFourSided ? "strategy.mapped_four_sided"
                                    : "strategy.freeform_uv_grid";
            } else {
                decideSupport(record, family, evaluates,
                              evaluates && representationReady, report);
            }
            // WP-175: demote analytic subclasses the certified templates do
            // not consume, so ADR-0014 fails with a named residual instead of
            // a late template refusal after expensive boundary work.
            if (record.support ==
                GeometrySupportState::SupportedAnalyticTemplate) {
                const auto demote = [&](const char* condition) {
                    record.support =
                        GeometrySupportState::DeferredResidualSurface;
                    record.strategyOrReasonCode =
                        "reason.deferred_residual_surface";
                    record.conditionCodes.push_back(condition);
                    ++report.unsupportedSubjects;
                };
                if (family.code == "cone") {
                    const bool apex =
                        record.trimDomain ==
                        TrimDomainClass::TouchesOneSingularity;
                    const bool frustumBand =
                        record.trimDomain ==
                            TrimDomainClass::FullPeriodicWithCapBoundaries ||
                        record.trimDomain ==
                            TrimDomainClass::PeriodicBandCrossingSeam;
                    if (!apex && !frustumBand) {
                        demote("cone.non_apex_deferred");
                    } else if (frustumBand &&
                               record.trimDomain ==
                                   TrimDomainClass::PeriodicBandCrossingSeam) {
                        std::set<StableId> uniqueEdges;
                        for (const CoedgeRecord& coedge : snapshot.coedges) {
                            if (coedge.faceId == faceId) {
                                uniqueEdges.insert(coedge.edgeId);
                            }
                        }
                        // Same rail budget as cylinder bands (2 rims + 2 rails).
                        if (uniqueEdges.size() > 4) {
                            demote("cone.complex_boundary_deferred");
                        }
                    }
                } else if (family.code == "sphere") {
                    // Full sphere (two poles) is consumed. Single-pole caps
                    // with only pole+rim edges use buildSphericalCapWall;
                    // Plasticity caps with meridians/extra edges stay named.
                    const bool fullSphere =
                        record.trimDomain ==
                        TrimDomainClass::TouchesTwoSingularities;
                    const bool sphericalCap =
                        record.trimDomain ==
                        TrimDomainClass::TouchesOneSingularity;
                    if (sphericalCap) {
                        std::set<StableId> uniqueEdges;
                        for (const CoedgeRecord& coedge : snapshot.coedges) {
                            if (coedge.faceId == faceId) {
                                uniqueEdges.insert(coedge.edgeId);
                            }
                        }
                        if (uniqueEdges.size() > 2) {
                            demote("sphere.complex_cap_deferred");
                        }
                    } else if (!fullSphere) {
                        demote("sphere.partial_deferred");
                    }
                } else if (family.code == "cylinder") {
                    const bool partialBand =
                        record.trimDomain ==
                        TrimDomainClass::PeriodicBandCrossingSeam;
                    if (partialBand) {
                        std::set<StableId> uniqueEdges;
                        for (const CoedgeRecord& coedge : snapshot.coedges) {
                            if (coedge.faceId == faceId) {
                                uniqueEdges.insert(coedge.edgeId);
                            }
                        }
                        if (uniqueEdges.size() > 4) {
                            demote("cylinder.complex_boundary_deferred");
                        }
                    }
                }
            }
        } catch (const Standard_Failure& error) {
            record.familyCode = "kernel_specific";
            record.concreteType = "Geom_Surface";
            record.support = GeometrySupportState::InvalidImportedGeometry;
            record.strategyOrReasonCode = "reason.invalid_imported_geometry";
            report.diagnostics.push_back(
                {"reconnaissance.surface.failure", faceId,
                 occtFailureMessage(error)});
            ++report.unsupportedSubjects;
        }
        report.records.push_back(std::move(record));
    }
    reconProgress("classify.faces.done");

    EdgeFaceMap edgeToFaces;
    TopExp::MapShapesAndAncestors(snapshot.model.shape, TopAbs_EDGE,
                                  TopAbs_FACE, edgeToFaces);

    std::map<StableId, std::size_t> regionIndexByFace;
    std::uint64_t regionOrdinal = 0;
    for (const ExactGeometryClassification& record : report.records) {
        if (record.taxonomy != GeometryTaxonomy::Surface) continue;
        LogicalRegion region;
        region.id = {StableIdKind::Region, ++regionOrdinal};
        region.code = "region." + record.familyCode;
        region.workingFaces = {record.subjectId};
        region.sourceFaces = record.sourceSubjects;
        std::set<StableId> edges;
        for (const CoedgeRecord& coedge : snapshot.coedges) {
            if (coedge.faceId == record.subjectId) edges.insert(coedge.edgeId);
        }
        region.boundaryEdges.assign(edges.begin(), edges.end());
        regionIndexByFace.emplace(record.subjectId, report.regions.size());
        report.regions.push_back(std::move(region));
    }

    // Coplanar artificial splits: O(edges). Merge region accounts when two
    // plane faces share a manifold edge. The previous all-pairs face scan was
    // O(faces^2 * edges) via shareManifoldEdge and hung recon on MP9 (~4k
    // faces) after import completed.
    reconProgress("coplanar_merge.begin");
    std::set<std::pair<std::uint64_t, std::uint64_t>> manifoldPlanePairs;
    for (int edgeIndex = 1; edgeIndex <= edgeToFaces.Extent(); ++edgeIndex) {
        const ShapeList& faces = edgeToFaces(edgeIndex);
        if (faces.Extent() != 2) continue;
        ShapeList::Iterator it(faces);
        const TopoDS_Face leftFace = TopoDS::Face(it.Value());
        it.Next();
        const TopoDS_Face rightFace = TopoDS::Face(it.Value());
        if (!facesAreCoplanarPartners(leftFace, rightFace)) continue;
        int leftIndex = snapshot.model.faces.FindIndex(leftFace);
        int rightIndex = snapshot.model.faces.FindIndex(rightFace);
        if (leftIndex <= 0 || rightIndex <= 0) {
            for (int index = 1; index <= snapshot.model.faces.Extent();
                 ++index) {
                if (leftIndex <= 0 &&
                    snapshot.model.faces(index).IsPartner(leftFace)) {
                    leftIndex = index;
                }
                if (rightIndex <= 0 &&
                    snapshot.model.faces(index).IsPartner(rightFace)) {
                    rightIndex = index;
                }
            }
        }
        if (leftIndex <= 0 || rightIndex <= 0 || leftIndex == rightIndex) {
            continue;
        }
        if (leftIndex > rightIndex) std::swap(leftIndex, rightIndex);
        manifoldPlanePairs.emplace(static_cast<std::uint64_t>(leftIndex),
                                   static_cast<std::uint64_t>(rightIndex));
        const StableId leftId{StableIdKind::Face,
                              static_cast<std::uint64_t>(leftIndex)};
        const StableId rightId{StableIdKind::Face,
                               static_cast<std::uint64_t>(rightIndex)};
        const auto leftRegion = regionIndexByFace.find(leftId);
        const auto rightRegion = regionIndexByFace.find(rightId);
        if (leftRegion == regionIndexByFace.end() ||
            rightRegion == regionIndexByFace.end()) {
            continue;
        }
        if (leftRegion->second == rightRegion->second) continue;
        LogicalRegion& keep = report.regions[leftRegion->second];
        LogicalRegion& drop = report.regions[rightRegion->second];
        keep.code = "region.plane.artificial_split_merged";
        keep.conditionCodes.push_back(
            "reason.artificial_split_merge_accounted");
        keep.workingFaces.insert(keep.workingFaces.end(),
                                 drop.workingFaces.begin(),
                                 drop.workingFaces.end());
        keep.sourceFaces.insert(keep.sourceFaces.end(),
                                drop.sourceFaces.begin(),
                                drop.sourceFaces.end());
        keep.boundaryEdges.insert(keep.boundaryEdges.end(),
                                  drop.boundaryEdges.begin(),
                                  drop.boundaryEdges.end());
        std::sort(keep.workingFaces.begin(), keep.workingFaces.end());
        keep.workingFaces.erase(
            std::unique(keep.workingFaces.begin(), keep.workingFaces.end()),
            keep.workingFaces.end());
        std::sort(keep.sourceFaces.begin(), keep.sourceFaces.end());
        keep.sourceFaces.erase(
            std::unique(keep.sourceFaces.begin(), keep.sourceFaces.end()),
            keep.sourceFaces.end());
        std::sort(keep.boundaryEdges.begin(), keep.boundaryEdges.end());
        keep.boundaryEdges.erase(std::unique(keep.boundaryEdges.begin(),
                                             keep.boundaryEdges.end()),
                                 keep.boundaryEdges.end());
        for (const StableId& faceId : drop.workingFaces) {
            regionIndexByFace[faceId] = leftRegion->second;
        }
        drop.workingFaces.clear();
        drop.sourceFaces.clear();
        drop.boundaryEdges.clear();
        drop.code = "region.merged_away";
        report.diagnostics.push_back(
            {"reconnaissance.region.artificial_split_merged", leftId,
             "coplanar faces sharing a manifold edge accounted as one "
             "logical region"});
    }
    report.regions.erase(
        std::remove_if(report.regions.begin(), report.regions.end(),
                       [](const LogicalRegion& region) {
                           return region.workingFaces.empty();
                       }),
        report.regions.end());
    reconProgress("coplanar_merge.done");

    // Multidomain same-support planes that do not share an edge: bucket by
    // discretized plane equation so we only compare coplanar candidates.
    reconProgress("multidomain.begin");
    std::map<PlaneBucketKey, std::vector<int>> planeBuckets;
    for (int faceIndex = 1; faceIndex <= snapshot.model.faces.Extent();
         ++faceIndex) {
        const TopoDS_Face face =
            TopoDS::Face(snapshot.model.faces(faceIndex));
        const PlaneBucketKey key = planeBucketKey(face);
        if (std::get<3>(key) == std::numeric_limits<long>::min()) continue;
        planeBuckets[key].push_back(faceIndex);
    }
    for (const auto& [key, indices] : planeBuckets) {
        (void)key;
        if (indices.size() < 2) continue;
        for (std::size_t i = 0; i < indices.size(); ++i) {
            for (std::size_t j = i + 1; j < indices.size(); ++j) {
                int leftIndex = indices[i];
                int rightIndex = indices[j];
                if (leftIndex > rightIndex) std::swap(leftIndex, rightIndex);
                if (manifoldPlanePairs.count(
                        {static_cast<std::uint64_t>(leftIndex),
                         static_cast<std::uint64_t>(rightIndex)})) {
                    continue;
                }
                const TopoDS_Face leftFace =
                    TopoDS::Face(snapshot.model.faces(leftIndex));
                const TopoDS_Face rightFace =
                    TopoDS::Face(snapshot.model.faces(rightIndex));
                if (!facesAreCoplanarPartners(leftFace, rightFace)) continue;
                const StableId leftId{StableIdKind::Face,
                                      static_cast<std::uint64_t>(leftIndex)};
                const StableId rightId{StableIdKind::Face,
                                       static_cast<std::uint64_t>(rightIndex)};
                for (ExactGeometryClassification& record : report.records) {
                    if (record.subjectId != leftId &&
                        record.subjectId != rightId) {
                        continue;
                    }
                    record.trimDomain =
                        TrimDomainClass::MultipleDisconnectedDomains;
                    if (std::find(
                            record.conditionCodes.begin(),
                            record.conditionCodes.end(),
                            "reason.multidomain_decomposition_unproven") ==
                        record.conditionCodes.end()) {
                        record.conditionCodes.push_back(
                            "reason.multidomain_decomposition_unproven");
                    }
                }
                report.diagnostics.push_back(
                    {"reconnaissance.region.multidomain_deferred", leftId,
                     "coplanar faces without a shared edge remain disconnected "
                     "same-support domains"});
            }
        }
    }
    reconProgress("multidomain.done");

    report.checkedSubjects = report.records.size();
    report.complete = report.checkedSubjects == report.expectedSubjects &&
        std::all_of(report.records.begin(), report.records.end(),
                    [](const ExactGeometryClassification& record) {
                        return record.subjectId.valid() &&
                               !record.familyCode.empty() &&
                               !record.concreteType.empty() &&
                               !record.strategyOrReasonCode.empty() &&
                               !record.sourceSubjects.empty();
                    });

    // CUT-A post-pass: name unsupported cut graphs on cylinders that are not
    // simple plate bores, and on fillet/blend faces that participate in slots.
    {
        int cylinderFaces = 0;
        int complexPerforatedPlanes = 0;
        for (ExactGeometryClassification& record : report.records) {
            if (record.taxonomy != GeometryTaxonomy::Surface) continue;
            if (record.familyCode == "cylinder") {
                ++cylinderFaces;
            }
            if (record.familyCode == "plane" &&
                std::find(record.conditionCodes.begin(),
                          record.conditionCodes.end(),
                          "cutout.planar_perforated") !=
                    record.conditionCodes.end()) {
                std::set<StableId> faceEdges;
                for (const CoedgeRecord& coedge : snapshot.coedges) {
                    if (coedge.faceId == record.subjectId) {
                        faceEdges.insert(coedge.edgeId);
                    }
                }
                // Filleted slots produce densely edged perforated planes.
                if (faceEdges.size() >= 10) {
                    ++complexPerforatedPlanes;
                }
            }
        }
        // Any model with 2+ cylinders is an unsupported multi-bore/slot
        // cut-graph for the current narrow cutout floor (single plate bore /
        // rectangular plate slot).
        const bool multiBoreGraph = cylinderFaces >= 2;
        const bool filletedSlotGraph =
            multiBoreGraph && complexPerforatedPlanes >= 1;
        for (ExactGeometryClassification& record : report.records) {
            if (record.taxonomy != GeometryTaxonomy::Surface) continue;
            if (multiBoreGraph && record.familyCode == "cylinder") {
                if (std::find(record.conditionCodes.begin(),
                              record.conditionCodes.end(),
                              "cutout.multi_bore_cylinder_deferred") ==
                    record.conditionCodes.end()) {
                    record.conditionCodes.push_back(
                        "cutout.multi_bore_cylinder_deferred");
                }
            }
            if (filletedSlotGraph &&
                (record.familyCode == "cylinder" ||
                 record.familyCode == "plane")) {
                if (std::find(record.conditionCodes.begin(),
                              record.conditionCodes.end(),
                              "cutout.filleted_slot_deferred") ==
                    record.conditionCodes.end()) {
                    record.conditionCodes.push_back(
                        "cutout.filleted_slot_deferred");
                }
            }
        }
    }

    if (!report.complete) {
        report.diagnostics.push_back(
            {"reconnaissance.coverage.incomplete", {},
             "not every working face and edge has complete source-backed classification"});
    }
    return report;
}

}  // namespace weft
