#include "weft/secure_reconnaissance.hpp"
#include "weft/occt_failure.hpp"

#include <BRepAdaptor_Curve.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepTools.hxx>
#include <BRep_Tool.hxx>
#include <Geom_OffsetCurve.hxx>
#include <Geom_OffsetSurface.hxx>
#include <Geom_RectangularTrimmedSurface.hxx>
#include <Geom_TrimmedCurve.hxx>
#include <Standard_Failure.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopoDS_Wire.hxx>

#include <algorithm>
#include <cmath>
#include <set>
#include <utility>

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
        case GeomAbs_Ellipse: return {"ellipse", true, true, false};
        case GeomAbs_Hyperbola: return {"hyperbola", true, true, false};
        case GeomAbs_Parabola: return {"parabola", true, true, false};
        case GeomAbs_BezierCurve: return {"bezier", false, true, false};
        case GeomAbs_BSplineCurve: return {"bspline", false, true, false};
        case GeomAbs_OffsetCurve: return {"offset", false, true, false};
        default: return {"kernel_specific", false, false, false};
    }
}

FamilyInfo surfaceFamily(GeomAbs_SurfaceType type) {
    switch (type) {
        case GeomAbs_Plane: return {"plane", true, true, true};
        case GeomAbs_Cylinder: return {"cylinder", true, true, true};
        case GeomAbs_Cone: return {"cone", true, true, false};
        case GeomAbs_Sphere: return {"sphere", true, true, false};
        case GeomAbs_Torus: return {"torus", true, true, false};
        case GeomAbs_BezierSurface: return {"bezier", false, true, false};
        case GeomAbs_BSplineSurface: return {"bspline", false, true, false};
        case GeomAbs_SurfaceOfRevolution:
            return {"revolution", false, true, false};
        case GeomAbs_SurfaceOfExtrusion:
            return {"extrusion", false, true, false};
        case GeomAbs_OffsetSurface: return {"offset", false, true, false};
        default: return {"kernel_specific", false, false, false};
    }
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

TrimDomainClass classifyTrim(const TopoDS_Face& face,
                             const BRepAdaptor_Surface& surface) {
    if (surface.IsUPeriodic() || surface.IsVPeriodic()) {
        return TrimDomainClass::FullPeriodicWithCapBoundaries;
    }
    std::size_t wires = 0;
    for (TopExp_Explorer explorer(face, TopAbs_WIRE); explorer.More();
         explorer.Next()) {
        ++wires;
    }
    std::size_t degenerateEdges = 0;
    for (TopExp_Explorer explorer(face, TopAbs_EDGE); explorer.More();
         explorer.Next()) {
        if (BRep_Tool::Degenerated(TopoDS::Edge(explorer.Current()))) {
            ++degenerateEdges;
        }
    }
    if (degenerateEdges >= 2) return TrimDomainClass::TouchesTwoSingularities;
    if (degenerateEdges == 1) return TrimDomainClass::TouchesOneSingularity;
    if (wires == 0) return TrimDomainClass::InvalidOrUnresolved;
    if (wires > 2) return TrimDomainClass::MultiplyPerforated;
    if (wires == 2) return TrimDomainClass::Annulus;

    const TopoDS_Wire outer = BRepTools::OuterWire(face);
    if (!outer.IsNull()) {
        std::size_t edges = 0;
        bool circular = false;
        for (TopoDS_Iterator iterator(outer); iterator.More(); iterator.Next()) {
            if (iterator.Value().ShapeType() != TopAbs_EDGE) continue;
            ++edges;
            BRepAdaptor_Curve curve(TopoDS::Edge(iterator.Value()));
            circular = curve.GetType() == GeomAbs_Circle;
        }
        if (edges == 1 && circular) return TrimDomainClass::SimpleDisk;
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

bool curvedFaceHasExactMappings(const ImportedModel& imported, StableId faceId) {
    if (!imported.working || !imported.workingEvaluator) return false;
    bool sawCoedge = false;
    for (const CoedgeRecord& coedge : imported.working->snapshot.coedges) {
        if (coedge.faceId != faceId) continue;
        sawCoedge = true;
        if (coedge.pcurveRepresentations.empty()) return false;
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

    for (int edgeIndex = 1; edgeIndex <= snapshot.model.edges.Extent();
         ++edgeIndex) {
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

    for (int faceIndex = 1; faceIndex <= snapshot.model.faces.Extent();
         ++faceIndex) {
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
            if (adaptor.IsUPeriodic()) record.conditionCodes.push_back("u_periodic");
            if (adaptor.IsVPeriodic()) record.conditionCodes.push_back("v_periodic");
            const bool evaluates = exactSurfaceEvaluates(
                *imported.workingEvaluator, faceId, u0, u1, v0, v1);
            // Planes are exactly projection-meshable without stored p-curves.
            // Curved supported faces require every stored face-specific mapping.
            const bool representationReady =
                family.code == "plane" || curvedFaceHasExactMappings(imported, faceId);
            decideSupport(record, family, evaluates,
                          evaluates && representationReady, report);
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
        report.regions.push_back(std::move(region));
    }

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
    if (!report.complete) {
        report.diagnostics.push_back(
            {"reconnaissance.coverage.incomplete", {},
             "not every working face and edge has complete source-backed classification"});
    }
    return report;
}

}  // namespace weft
