#include "secure_core_internal.hpp"

#include "weft/io/reader.hpp"
#include "weft/io/system.hpp"

#include <BRepCheck_Analyzer.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepTools.hxx>
#include <BRepTools_WireExplorer.hxx>
#include <BRep_Tool.hxx>
#include <ElSLib.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <Geom2d_Curve.hxx>
#include <Geom_Curve.hxx>
#include <Geom_Surface.hxx>
#include <Standard_Failure.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Vertex.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Vec.hxx>
#include <gp_Vec2d.hxx>

#include <algorithm>
#include <array>
#include <bit>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <locale>
#include <map>
#include <sstream>
#include <streambuf>
#include <stdexcept>
#include <tuple>
#include <utility>

namespace weft {
namespace {

TopologyOrientation orientationOf(TopAbs_Orientation orientation) {
    switch (orientation) {
        case TopAbs_FORWARD: return TopologyOrientation::Forward;
        case TopAbs_REVERSED: return TopologyOrientation::Reversed;
        case TopAbs_INTERNAL: return TopologyOrientation::Internal;
        case TopAbs_EXTERNAL: return TopologyOrientation::External;
    }
    return TopologyOrientation::Forward;
}

StableId stableId(StableIdKind kind, int ordinal) {
    return {kind, ordinal > 0 ? static_cast<std::uint64_t>(ordinal) : 0};
}

double shapeTolerance(const TopoDS_Shape& shape) {
    try {
        switch (shape.ShapeType()) {
            case TopAbs_FACE:
                return BRep_Tool::Tolerance(TopoDS::Face(shape));
            case TopAbs_EDGE:
                return BRep_Tool::Tolerance(TopoDS::Edge(shape));
            case TopAbs_VERTEX:
                return BRep_Tool::Tolerance(TopoDS::Vertex(shape));
            default:
                return 0.0;
        }
    } catch (const Standard_Failure&) {
        return 0.0;
    }
}

BRepSnapshot buildSnapshot(Model model) {
    BRepSnapshot snapshot;
    snapshot.model = std::move(model);

    ShapeMap wires;
    ShapeMap vertices;
    TopExp::MapShapes(snapshot.model.shape, TopAbs_WIRE, wires);
    TopExp::MapShapes(snapshot.model.shape, TopAbs_VERTEX, vertices);

    auto addOccurrences = [&](const ShapeMap& shapes, StableIdKind kind) {
        for (int i = 1; i <= shapes.Extent(); ++i) {
            const StableId id = stableId(kind, i);
            snapshot.occurrences.push_back(
                {id, id, std::nullopt, {}, orientationOf(shapes(i).Orientation()),
                 shapeTolerance(shapes(i)), !shapes(i).IsNull(), {}});
        }
    };
    addOccurrences(snapshot.model.solids, StableIdKind::Solid);
    addOccurrences(snapshot.model.faces, StableIdKind::Face);
    addOccurrences(wires, StableIdKind::Wire);
    addOccurrences(snapshot.model.edges, StableIdKind::Edge);
    addOccurrences(vertices, StableIdKind::Vertex);

    snapshot.edgeTopology.reserve(
        static_cast<std::size_t>(snapshot.model.edges.Extent()));
    for (int edgeIndex = 1; edgeIndex <= snapshot.model.edges.Extent();
         ++edgeIndex) {
        const TopoDS_Edge edge =
            TopoDS::Edge(snapshot.model.edges(edgeIndex));
        EdgeTopologyRecord record;
        record.id = stableId(StableIdKind::Edge, edgeIndex);
        record.degenerate = BRep_Tool::Degenerated(edge);
        try {
            TopoDS_Vertex firstVertex;
            TopoDS_Vertex lastVertex;
            TopExp::Vertices(edge, firstVertex, lastVertex, true);
            const auto vertexId = [&](const TopoDS_Vertex& vertex)
                -> std::optional<StableId> {
                if (vertex.IsNull()) return std::nullopt;
                const int index = vertices.FindIndex(vertex);
                if (index <= 0) return std::nullopt;
                return stableId(StableIdKind::Vertex, index);
            };
            record.lowerVertex = vertexId(firstVertex);
            record.upperVertex = vertexId(lastVertex);
            if (!firstVertex.IsNull() && !lastVertex.IsNull() &&
                !firstVertex.IsSame(lastVertex)) {
                const double firstParameter =
                    BRep_Tool::Parameter(firstVertex, edge);
                const double lastParameter =
                    BRep_Tool::Parameter(lastVertex, edge);
                if (lastParameter < firstParameter) {
                    std::swap(record.lowerVertex, record.upperVertex);
                }
            }
        } catch (const Standard_Failure&) {
            record.lowerVertex.reset();
            record.upperVertex.reset();
        }
        snapshot.edgeTopology.push_back(std::move(record));
    }

    std::uint64_t coedgeOrdinal = 0;
    for (int faceIndex = 1; faceIndex <= snapshot.model.faces.Extent(); ++faceIndex) {
        const TopoDS_Face face = TopoDS::Face(snapshot.model.faces(faceIndex));
        for (TopExp_Explorer wireExplorer(face, TopAbs_WIRE);
             wireExplorer.More(); wireExplorer.Next()) {
            const TopoDS_Wire wire = TopoDS::Wire(wireExplorer.Current());
            const int wireIndex = wires.FindIndex(wire);
            std::uint32_t ordinalInWire = 0;
            for (BRepTools_WireExplorer edgeExplorer(wire, face);
                 edgeExplorer.More(); edgeExplorer.Next(), ++ordinalInWire) {
                const TopoDS_Edge edge = edgeExplorer.Current();
                const int edgeIndex = snapshot.model.edges.FindIndex(edge);
                CoedgeRecord coedge;
                coedge.id = {StableIdKind::Coedge, ++coedgeOrdinal};
                coedge.edgeId = stableId(StableIdKind::Edge, edgeIndex);
                coedge.wireId = stableId(StableIdKind::Wire, wireIndex);
                coedge.faceId = stableId(StableIdKind::Face, faceIndex);
                coedge.ordinalInWire = ordinalInWire;
                coedge.orientation = orientationOf(edge.Orientation());
                const bool seam = BRep_Tool::IsClosed(edge, face);
                const auto appendStoredPcurve =
                    [&](TopoDS_Edge orientedEdge,
                        std::uint32_t representationIndex) {
                        double first = 0.0;
                        double last = 0.0;
                        bool stored = false;
                        Handle(Geom2d_Curve) pcurve = BRep_Tool::CurveOnSurface(
                            orientedEdge, face, first, last, &stored);
                        if (!pcurve.IsNull() && stored) {
                            coedge.pcurveRepresentations.push_back(
                                {coedge.id, coedge.faceId,
                                 representationIndex});
                            return true;
                        }
                        return false;
                    };
                const bool firstStored = appendStoredPcurve(edge, 0);
                bool secondStored = true;
                if (seam) {
                    TopoDS_Edge reversed = edge;
                    reversed.Reverse();
                    secondStored = appendStoredPcurve(reversed, 1);
                }
                if (!firstStored || !secondStored) {
                    coedge.conditionCodes.push_back(
                        seam ? "pcurve.incomplete_seam" : "pcurve.missing");
                }
                if (!BRep_Tool::SameParameter(edge)) {
                    coedge.conditionCodes.push_back("pcurve.not_same_parameter");
                }
                if (!BRep_Tool::SameRange(edge)) {
                    coedge.conditionCodes.push_back("pcurve.not_same_range");
                }
                snapshot.coedges.push_back(std::move(coedge));
            }
        }
    }
    return snapshot;
}

template <typename T>
EvaluationResult<T> evaluationFailure(std::string code, std::string message,
                                      std::optional<StableId> subject) {
    EvaluationResult<T> result;
    result.failure = EvaluationFailure{std::move(code), std::move(message), subject};
    return result;
}

class OcctGeometryEvaluator final : public GeometryEvaluator {
public:
    explicit OcctGeometryEvaluator(BRepSnapshot snapshot)
        : snapshot_(std::move(snapshot)) {}

    EvaluationResult<ParameterDomain> curveDomain(StableId edge) const override {
        const TopoDS_Edge shape = edgeShape(edge);
        if (shape.IsNull()) {
            return evaluationFailure<ParameterDomain>(
                "geometry.edge_not_found", "edge id does not resolve", edge);
        }
        try {
            double first = 0.0;
            double last = 0.0;
            Handle(Geom_Curve) curve = BRep_Tool::Curve(shape, first, last);
            if (curve.IsNull() || !std::isfinite(first) || !std::isfinite(last) ||
                !(last > first)) {
                return evaluationFailure<ParameterDomain>(
                    "geometry.curve_domain_invalid",
                    "edge has no finite increasing 3D curve domain", edge);
            }
            ParameterDomain domain;
            domain.lower = first;
            domain.upper = last;
            domain.closed = curve->IsClosed();
            domain.periodic = curve->IsPeriodic();
            if (domain.periodic) domain.period = curve->Period();
            return EvaluationResult<ParameterDomain>{domain, std::nullopt};
        } catch (const Standard_Failure& error) {
            return evaluationFailure<ParameterDomain>(
                "geometry.curve_domain_failure", error.what(), edge);
        }
    }

    EvaluationResult<CurveEvaluation> evaluateCurve(
        StableId edge, double parameter) const override {
        const TopoDS_Edge shape = edgeShape(edge);
        if (shape.IsNull()) {
            return evaluationFailure<CurveEvaluation>(
                "geometry.edge_not_found", "edge id does not resolve", edge);
        }
        try {
            double first = 0.0;
            double last = 0.0;
            Handle(Geom_Curve) curve = BRep_Tool::Curve(shape, first, last);
            if (curve.IsNull() || parameter < first || parameter > last ||
                !std::isfinite(parameter)) {
                return evaluationFailure<CurveEvaluation>(
                    "geometry.curve_parameter_out_of_range",
                    "curve parameter is outside the exact edge domain", edge);
            }
            gp_Pnt point;
            gp_Vec derivative;
            curve->D1(parameter, point, derivative);
            CurveEvaluation evaluation;
            evaluation.edgeId = edge;
            evaluation.parameter = parameter;
            evaluation.position = {point.X(), point.Y(), point.Z()};
            evaluation.firstDerivative =
                {derivative.X(), derivative.Y(), derivative.Z()};
            return EvaluationResult<CurveEvaluation>{evaluation, std::nullopt};
        } catch (const Standard_Failure& error) {
            return evaluationFailure<CurveEvaluation>(
                "geometry.curve_evaluation_failure", error.what(), edge);
        }
    }

    EvaluationResult<PcurveEvaluation> evaluatePcurve(
        PcurveRef representation, double parameter) const override {
        const CoedgeRecord* coedge = findCoedge(representation.coedgeId);
        if (!coedge || coedge->faceId != representation.faceId ||
            std::find(coedge->pcurveRepresentations.begin(),
                      coedge->pcurveRepresentations.end(), representation) ==
                coedge->pcurveRepresentations.end()) {
            return evaluationFailure<PcurveEvaluation>(
                "geometry.pcurve_not_found", "p-curve reference does not resolve",
                representation.coedgeId);
        }
        TopoDS_Edge edge = edgeShape(coedge->edgeId);
        if (representation.representationIndex == 1) edge.Reverse();
        const TopoDS_Face face = faceShape(coedge->faceId);
        try {
            double first = 0.0;
            double last = 0.0;
            bool stored = false;
            Handle(Geom2d_Curve) pcurve = BRep_Tool::CurveOnSurface(
                edge, face, first, last, &stored);
            if (pcurve.IsNull() || !stored || parameter < first || parameter > last ||
                !std::isfinite(parameter)) {
                return evaluationFailure<PcurveEvaluation>(
                    "geometry.pcurve_parameter_out_of_range",
                    "p-curve parameter is outside the representation domain",
                    representation.coedgeId);
            }
            gp_Pnt2d point;
            gp_Vec2d derivative;
            pcurve->D1(parameter, point, derivative);
            PcurveEvaluation evaluation;
            evaluation.reference = representation;
            evaluation.parameter = parameter;
            evaluation.uv = {point.X(), point.Y()};
            evaluation.firstDerivative = {derivative.X(), derivative.Y()};
            return EvaluationResult<PcurveEvaluation>{evaluation, std::nullopt};
        } catch (const Standard_Failure& error) {
            return evaluationFailure<PcurveEvaluation>(
                "geometry.pcurve_evaluation_failure", error.what(),
                representation.coedgeId);
        }
    }

    EvaluationResult<SurfaceEvaluation> evaluateSurface(
        StableId faceId, std::array<double, 2> uv) const override {
        const TopoDS_Face face = faceShape(faceId);
        if (face.IsNull()) {
            return evaluationFailure<SurfaceEvaluation>(
                "geometry.face_not_found", "face id does not resolve", faceId);
        }
        try {
            Handle(Geom_Surface) surface = BRep_Tool::Surface(face);
            if (surface.IsNull() || !std::isfinite(uv[0]) || !std::isfinite(uv[1])) {
                return evaluationFailure<SurfaceEvaluation>(
                    "geometry.surface_domain_invalid",
                    "face has no evaluable surface or UV is non-finite", faceId);
            }
            gp_Pnt point;
            gp_Vec du;
            gp_Vec dv;
            surface->D1(uv[0], uv[1], point, du, dv);
            SurfaceEvaluation evaluation;
            evaluation.faceId = faceId;
            evaluation.uv = uv;
            evaluation.position = {point.X(), point.Y(), point.Z()};
            evaluation.derivativeU = {du.X(), du.Y(), du.Z()};
            evaluation.derivativeV = {dv.X(), dv.Y(), dv.Z()};
            gp_Vec normal = du.Crossed(dv);
            if (normal.SquareMagnitude() > 0.0) {
                normal.Normalize();
                if (face.Orientation() == TopAbs_REVERSED) normal.Reverse();
                evaluation.unitNormal =
                    std::array<double, 3>{normal.X(), normal.Y(), normal.Z()};
            }
            return EvaluationResult<SurfaceEvaluation>{evaluation, std::nullopt};
        } catch (const Standard_Failure& error) {
            return evaluationFailure<SurfaceEvaluation>(
                "geometry.surface_evaluation_failure", error.what(), faceId);
        }
    }

    EvaluationResult<PlanarProjectionEvaluation> projectPointToPlane(
        StableId faceId, std::array<double, 3> position) const override {
        const TopoDS_Face face = faceShape(faceId);
        if (face.IsNull()) {
            return evaluationFailure<PlanarProjectionEvaluation>(
                "geometry.face_not_found", "face id does not resolve", faceId);
        }
        if (!std::all_of(position.begin(), position.end(),
                         [](double coordinate) {
                             return std::isfinite(coordinate);
                         })) {
            return evaluationFailure<PlanarProjectionEvaluation>(
                "geometry.planar_projection_input_invalid",
                "planar projection position is non-finite", faceId);
        }
        try {
            BRepAdaptor_Surface adaptor(face, true);
            if (adaptor.GetType() != GeomAbs_Plane) {
                return evaluationFailure<PlanarProjectionEvaluation>(
                    "geometry.surface_not_plane",
                    "planar projection is restricted to exactly classified planes",
                    faceId);
            }
            double u = 0.0;
            double v = 0.0;
            const gp_Pnt point(position[0], position[1], position[2]);
            ElSLib::Parameters(adaptor.Plane(), point, u, v);
            const auto surface = evaluateSurface(faceId, {u, v});
            if (!surface) {
                return evaluationFailure<PlanarProjectionEvaluation>(
                    "geometry.planar_projection_surface_failure",
                    "projected planar UV did not evaluate", faceId);
            }
            PlanarProjectionEvaluation evaluation;
            evaluation.faceId = faceId;
            evaluation.inputPosition = position;
            evaluation.uv = {u, v};
            evaluation.surfacePosition = surface.value->position;
            evaluation.discrepancy = std::hypot(
                position[0] - evaluation.surfacePosition[0],
                position[1] - evaluation.surfacePosition[1],
                position[2] - evaluation.surfacePosition[2]);
            return EvaluationResult<PlanarProjectionEvaluation>{evaluation,
                                                                 std::nullopt};
        } catch (const Standard_Failure& error) {
            return evaluationFailure<PlanarProjectionEvaluation>(
                "geometry.planar_projection_failure", error.what(), faceId);
        }
    }

    EvaluationResult<CurveOnSurfaceEvaluation> evaluateCurveOnSurface(
        PcurveRef representation, double edgeParameter) const override {
        const CoedgeRecord* coedge = findCoedge(representation.coedgeId);
        if (!coedge || coedge->faceId != representation.faceId ||
            std::find(coedge->pcurveRepresentations.begin(),
                      coedge->pcurveRepresentations.end(), representation) ==
                coedge->pcurveRepresentations.end()) {
            return evaluationFailure<CurveOnSurfaceEvaluation>(
                "geometry.pcurve_not_found", "p-curve reference does not resolve",
                representation.coedgeId);
        }
        const TopoDS_Edge edge = edgeShape(coedge->edgeId);
        try {
            if (!BRep_Tool::SameParameter(edge) || !BRep_Tool::SameRange(edge)) {
                return evaluationFailure<CurveOnSurfaceEvaluation>(
                    "geometry.curve_on_surface_parameter_unproven",
                    "edge does not retain identical 3D and p-curve parameters and ranges",
                    representation.coedgeId);
            }
            const auto curve = evaluateCurve(coedge->edgeId, edgeParameter);
            const auto pcurve = evaluatePcurve(representation, edgeParameter);
            if (!curve || !pcurve) {
                return evaluationFailure<CurveOnSurfaceEvaluation>(
                    "geometry.curve_on_surface_sub_evaluation_failure",
                    "exact curve or p-curve evaluation failed",
                    representation.coedgeId);
            }
            const auto surface = evaluateSurface(coedge->faceId, pcurve.value->uv);
            if (!surface) {
                return evaluationFailure<CurveOnSurfaceEvaluation>(
                    "geometry.curve_on_surface_sub_evaluation_failure",
                    "exact surface evaluation failed", coedge->faceId);
            }
            CurveOnSurfaceEvaluation evaluation;
            evaluation.reference = representation;
            evaluation.edgeParameter = edgeParameter;
            evaluation.pcurveParameter = edgeParameter;
            evaluation.uv = pcurve.value->uv;
            evaluation.curvePosition = curve.value->position;
            evaluation.surfacePosition = surface.value->position;
            const double dx = evaluation.curvePosition[0] -
                              evaluation.surfacePosition[0];
            const double dy = evaluation.curvePosition[1] -
                              evaluation.surfacePosition[1];
            const double dz = evaluation.curvePosition[2] -
                              evaluation.surfacePosition[2];
            evaluation.discrepancy = std::hypot(dx, dy, dz);
            return EvaluationResult<CurveOnSurfaceEvaluation>{evaluation,
                                                               std::nullopt};
        } catch (const Standard_Failure& error) {
            return evaluationFailure<CurveOnSurfaceEvaluation>(
                "geometry.curve_on_surface_evaluation_failure",
                error.what(), representation.coedgeId);
        }
    }

private:
    TopoDS_Edge edgeShape(StableId id) const {
        if (id.kind != StableIdKind::Edge || id.ordinal == 0 ||
            id.ordinal > static_cast<std::uint64_t>(snapshot_.model.edges.Extent())) {
            return {};
        }
        return TopoDS::Edge(snapshot_.model.edges(static_cast<int>(id.ordinal)));
    }

    TopoDS_Face faceShape(StableId id) const {
        if (id.kind != StableIdKind::Face || id.ordinal == 0 ||
            id.ordinal > static_cast<std::uint64_t>(snapshot_.model.faces.Extent())) {
            return {};
        }
        return TopoDS::Face(snapshot_.model.faces(static_cast<int>(id.ordinal)));
    }

    const CoedgeRecord* findCoedge(StableId id) const {
        if (id.kind != StableIdKind::Coedge || id.ordinal == 0 ||
            id.ordinal > snapshot_.coedges.size()) {
            return nullptr;
        }
        return &snapshot_.coedges[static_cast<std::size_t>(id.ordinal - 1)];
    }

    BRepSnapshot snapshot_;
};

bool shapeIsValid(const TopoDS_Shape& shape) {
    if (shape.IsNull()) return false;
    try {
        return BRepCheck_Analyzer(shape, true).IsValid();
    } catch (const Standard_Failure&) {
        return false;
    }
}

class Sha256 final {
public:
    void update(const unsigned char* data, std::size_t size) noexcept {
        for (std::size_t index = 0; index < size; ++index) {
            block_[blockSize_++] = data[index];
            if (blockSize_ == block_.size()) {
                transform();
                bitCount_ += 512U;
                blockSize_ = 0;
            }
        }
    }

    std::string finish() noexcept {
        const std::uint64_t totalBits =
            bitCount_ + static_cast<std::uint64_t>(blockSize_) * 8U;
        block_[blockSize_++] = 0x80U;
        if (blockSize_ > 56U) {
            while (blockSize_ < block_.size()) block_[blockSize_++] = 0U;
            transform();
            blockSize_ = 0;
        }
        while (blockSize_ < 56U) block_[blockSize_++] = 0U;
        for (std::size_t index = 0; index < 8U; ++index) {
            block_[63U - index] = static_cast<unsigned char>(
                (totalBits >> (index * 8U)) & 0xffU);
        }
        transform();
        std::ostringstream stream;
        stream.imbue(std::locale::classic());
        stream << std::hex << std::setfill('0');
        for (std::uint32_t value : state_) stream << std::setw(8) << value;
        return stream.str();
    }

private:
    static constexpr std::array<std::uint32_t, 64> kRoundConstants{
        0x428a2f98U,0x71374491U,0xb5c0fbcfU,0xe9b5dba5U,0x3956c25bU,0x59f111f1U,0x923f82a4U,0xab1c5ed5U,
        0xd807aa98U,0x12835b01U,0x243185beU,0x550c7dc3U,0x72be5d74U,0x80deb1feU,0x9bdc06a7U,0xc19bf174U,
        0xe49b69c1U,0xefbe4786U,0x0fc19dc6U,0x240ca1ccU,0x2de92c6fU,0x4a7484aaU,0x5cb0a9dcU,0x76f988daU,
        0x983e5152U,0xa831c66dU,0xb00327c8U,0xbf597fc7U,0xc6e00bf3U,0xd5a79147U,0x06ca6351U,0x14292967U,
        0x27b70a85U,0x2e1b2138U,0x4d2c6dfcU,0x53380d13U,0x650a7354U,0x766a0abbU,0x81c2c92eU,0x92722c85U,
        0xa2bfe8a1U,0xa81a664bU,0xc24b8b70U,0xc76c51a3U,0xd192e819U,0xd6990624U,0xf40e3585U,0x106aa070U,
        0x19a4c116U,0x1e376c08U,0x2748774cU,0x34b0bcb5U,0x391c0cb3U,0x4ed8aa4aU,0x5b9cca4fU,0x682e6ff3U,
        0x748f82eeU,0x78a5636fU,0x84c87814U,0x8cc70208U,0x90befffaU,0xa4506cebU,0xbef9a3f7U,0xc67178f2U};

    void transform() noexcept {
        std::array<std::uint32_t, 64> words{};
        for (std::size_t index = 0; index < 16U; ++index) {
            const std::size_t offset = index * 4U;
            words[index] = (static_cast<std::uint32_t>(block_[offset]) << 24U) |
                           (static_cast<std::uint32_t>(block_[offset + 1U]) << 16U) |
                           (static_cast<std::uint32_t>(block_[offset + 2U]) << 8U) |
                           static_cast<std::uint32_t>(block_[offset + 3U]);
        }
        for (std::size_t index = 16U; index < words.size(); ++index) {
            const std::uint32_t s0 = std::rotr(words[index - 15U], 7) ^
                                     std::rotr(words[index - 15U], 18) ^
                                     (words[index - 15U] >> 3U);
            const std::uint32_t s1 = std::rotr(words[index - 2U], 17) ^
                                     std::rotr(words[index - 2U], 19) ^
                                     (words[index - 2U] >> 10U);
            words[index] = words[index - 16U] + s0 +
                           words[index - 7U] + s1;
        }
        std::uint32_t a=state_[0],b=state_[1],c=state_[2],d=state_[3];
        std::uint32_t e=state_[4],f=state_[5],g=state_[6],h=state_[7];
        for (std::size_t index = 0; index < words.size(); ++index) {
            const std::uint32_t sigma1 =
                std::rotr(e,6) ^ std::rotr(e,11) ^ std::rotr(e,25);
            const std::uint32_t choose = (e & f) ^ ((~e) & g);
            const std::uint32_t t1 = h + sigma1 + choose +
                                     kRoundConstants[index] + words[index];
            const std::uint32_t sigma0 =
                std::rotr(a,2) ^ std::rotr(a,13) ^ std::rotr(a,22);
            const std::uint32_t majority = (a & b) ^ (a & c) ^ (b & c);
            const std::uint32_t t2 = sigma0 + majority;
            h=g; g=f; f=e; e=d+t1; d=c; c=b; b=a; a=t1+t2;
        }
        state_[0]+=a; state_[1]+=b; state_[2]+=c; state_[3]+=d;
        state_[4]+=e; state_[5]+=f; state_[6]+=g; state_[7]+=h;
    }

    std::array<std::uint32_t, 8> state_{0x6a09e667U,0xbb67ae85U,0x3c6ef372U,0xa54ff53aU,
                                        0x510e527fU,0x9b05688cU,0x1f83d9abU,0x5be0cd19U};
    std::array<unsigned char, 64> block_{};
    std::size_t blockSize_ = 0;
    std::uint64_t bitCount_ = 0;
};

class Sha256StreamBuffer final : public std::streambuf {
public:
    explicit Sha256StreamBuffer(Sha256& digest) : digest_(digest) {}

protected:
    std::streamsize xsputn(const char* text,
                           std::streamsize count) override {
        if (count > 0) {
            digest_.update(reinterpret_cast<const unsigned char*>(text),
                           static_cast<std::size_t>(count));
        }
        return count;
    }

    int_type overflow(int_type character) override {
        if (!traits_type::eq_int_type(character, traits_type::eof())) {
            const unsigned char byte = static_cast<unsigned char>(
                traits_type::to_char_type(character));
            digest_.update(&byte, 1);
        }
        return character;
    }

private:
    Sha256& digest_;
};

std::string exactShapeDigest(const TopoDS_Shape& shape) {
    if (shape.IsNull()) return {};
    Sha256 digest;
    Sha256StreamBuffer buffer(digest);
    std::ostream stream(&buffer);
    stream.imbue(std::locale::classic());
    try {
        BRepTools::Write(shape, stream, false, false,
                         TopTools_FormatVersion_VERSION_3);
    } catch (const Standard_Failure&) {
        return {};
    }
    if (!stream) return {};
    return digest.finish();
}

std::size_t occurrenceCount(const BRepSnapshot& snapshot,
                            StableIdKind kind) {
    return static_cast<std::size_t>(std::count_if(
        snapshot.occurrences.begin(), snapshot.occurrences.end(),
        [kind](const TopologyOccurrence& occurrence) {
            return occurrence.id.kind == kind;
        }));
}

const TopoDS_Shape* shapeForId(const BRepSnapshot& snapshot, StableId id) {
    if (id.ordinal == 0) return nullptr;
    const int ordinal = static_cast<int>(id.ordinal);
    if (id.kind == StableIdKind::Face &&
        ordinal <= snapshot.model.faces.Extent()) {
        return &snapshot.model.faces(ordinal);
    }
    if (id.kind == StableIdKind::Edge &&
        ordinal <= snapshot.model.edges.Extent()) {
        return &snapshot.model.edges(ordinal);
    }
    return nullptr;
}

std::size_t storedPcurveUseCount(const BRepSnapshot& snapshot,
                                 StableId edgeId) {
    std::size_t count = 0;
    for (const CoedgeRecord& coedge : snapshot.coedges) {
        if (coedge.edgeId == edgeId) {
            count += coedge.pcurveRepresentations.size();
        }
    }
    return count;
}

template <typename Map>
void addCorrespondenceForMap(const Map& source, const Map& working,
                             StableIdKind kind,
                             const Handle(BRepTools_History)& history,
                             SourceWorkingMap& correspondence,
                             std::map<StableId, int>& workingUse) {
    for (int sourceIndex = 1; sourceIndex <= source.Extent(); ++sourceIndex) {
        const TopoDS_Shape& sourceShape = source(sourceIndex);
        CorrespondenceRecord record;
        record.sourceId = stableId(kind, sourceIndex);
        const int identityIndex = working.FindIndex(sourceShape);
        if (identityIndex > 0) {
            record.workingIds.push_back(stableId(kind, identityIndex));
            record.relation = CorrespondenceRelation::Identity;
        } else if (!history.IsNull() && history->IsRemoved(sourceShape)) {
            record.relation = CorrespondenceRelation::Removed;
        } else if (!history.IsNull()) {
            const auto& modified = history->Modified(sourceShape);
            for (const TopoDS_Shape& candidate : modified) {
                const int index = working.FindIndex(candidate);
                if (index > 0) record.workingIds.push_back(stableId(kind, index));
            }
            std::sort(record.workingIds.begin(), record.workingIds.end());
            record.workingIds.erase(
                std::unique(record.workingIds.begin(), record.workingIds.end()),
                record.workingIds.end());
            record.relation = record.workingIds.size() > 1
                                  ? CorrespondenceRelation::Split
                                  : CorrespondenceRelation::Modified;
        } else {
            record.relation = CorrespondenceRelation::Modified;
        }
        for (StableId workingId : record.workingIds) ++workingUse[workingId];
        correspondence.records.push_back(std::move(record));
    }

    for (int workingIndex = 1; workingIndex <= working.Extent(); ++workingIndex) {
        const StableId workingId = stableId(kind, workingIndex);
        if (!workingUse.contains(workingId)) {
            correspondence.records.push_back(
                {{}, {workingId}, CorrespondenceRelation::Introduced});
        }
    }
}

SourceWorkingMap buildCorrespondence(const Model& source, const Model& working,
                                     const Handle(BRepTools_History)& history) {
    SourceWorkingMap correspondence;
    std::map<StableId, int> workingUse;
    addCorrespondenceForMap(source.faces, working.faces, StableIdKind::Face,
                            history, correspondence, workingUse);
    addCorrespondenceForMap(source.edges, working.edges, StableIdKind::Edge,
                            history, correspondence, workingUse);

    for (CorrespondenceRecord& record : correspondence.records) {
        if (record.sourceId.valid() && record.workingIds.size() == 1 &&
            workingUse[record.workingIds.front()] > 1) {
            record.relation = CorrespondenceRelation::Merged;
        }
    }
    correspondence.complete = std::all_of(
        correspondence.records.begin(), correspondence.records.end(),
        [](const CorrespondenceRecord& record) {
            if (!record.sourceId.valid()) return false;
            return record.relation != CorrespondenceRelation::Removed &&
                   !record.workingIds.empty();
        });
    // Identity inputs have no introduced records and every source resolves.
    if (source.faces.Extent() == working.faces.Extent() &&
        source.edges.Extent() == working.edges.Extent()) {
        correspondence.complete = std::all_of(
            correspondence.records.begin(), correspondence.records.end(),
            [](const CorrespondenceRecord& record) {
                return record.sourceId.valid() &&
                       record.relation == CorrespondenceRelation::Identity &&
                       record.workingIds.size() == 1;
            });
    }
    return correspondence;
}

}  // namespace

const char* repairProfileName(RepairProfile profile) noexcept {
    switch (profile) {
        case RepairProfile::Conservative: return "conservative";
        case RepairProfile::Compatibility: return "compatibility";
    }
    return "conservative";
}

bool ImportDiagnostics::hasErrors() const noexcept {
    return std::any_of(events.begin(), events.end(), [](const ImportDiagnostic& event) {
        return event.severity == DiagnosticSeverity::Error ||
               event.severity == DiagnosticSeverity::Fatal;
    });
}

const CorrespondenceRecord* SourceWorkingMap::find(StableId source) const noexcept {
    const auto found = std::find_if(records.begin(), records.end(),
                                    [source](const CorrespondenceRecord& record) {
                                        return record.sourceId == source;
                                    });
    return found == records.end() ? nullptr : &*found;
}

const Model& ImportedModel::workingModel() const {
    if (!working) throw std::logic_error("secure import has no working B-rep");
    return working->snapshot.model;
}

namespace secure_detail {

SourceMetadata readSourceMetadata(const std::string& path,
                                  std::string_view sourceBytes) {
    SourceMetadata metadata;
    metadata.sourceName = std::filesystem::path(path).filename().string();
    metadata.importerVersion = "weft-secure-core-0.1";
    Sha256 digest;
    if (!sourceBytes.empty()) {
        digest.update(
            reinterpret_cast<const unsigned char*>(sourceBytes.data()),
            sourceBytes.size());
    }
    metadata.sourceByteLength = static_cast<std::uint64_t>(sourceBytes.size());
    metadata.sourceSha256 = digest.finish();
    return metadata;
}

ImportedModel buildImportedModel(
    Model sourceModel, Model workingModel, SourceMetadata metadata,
    RepairProfile profile, const Handle(BRepTools_History)& history,
    std::vector<RepairOperation> operations) {
    ImportedModel imported;
    const bool sourceValid = shapeIsValid(sourceModel.shape);
    const bool workingValid = shapeIsValid(workingModel.shape);
    const std::string sourceShapeDigest = exactShapeDigest(sourceModel.shape);
    const std::string workingShapeDigest = exactShapeDigest(workingModel.shape);
    imported.correspondence =
        buildCorrespondence(sourceModel, workingModel, history);

    BRepSnapshot sourceSnapshot = buildSnapshot(std::move(sourceModel));
    BRepSnapshot workingSnapshot = buildSnapshot(std::move(workingModel));
    imported.sourceEvaluator =
        std::make_shared<OcctGeometryEvaluator>(sourceSnapshot);
    imported.workingEvaluator =
        std::make_shared<OcctGeometryEvaluator>(workingSnapshot);

    auto source = std::make_shared<SourceBRep>();
    source->metadata = std::move(metadata);
    source->snapshot = std::move(sourceSnapshot);
    auto working = std::make_shared<WorkingBRep>();
    working->profile = profile;
    working->snapshot = std::move(workingSnapshot);
    imported.source = std::move(source);
    imported.working = std::move(working);

    imported.repair.profile = profile;
    imported.repair.sourceShapeSha256 = sourceShapeDigest;
    imported.repair.workingShapeSha256 = workingShapeDigest;
    imported.repair.sourceValid = sourceValid;
    imported.repair.workingValid = workingValid;
    imported.repair.correspondenceComplete = imported.correspondence.complete;
    imported.repair.sourceFaces = imported.source->snapshot.model.faceCount();
    imported.repair.workingFaces = imported.working->snapshot.model.faceCount();
    imported.repair.sourceEdges = imported.source->snapshot.model.edgeCount();
    imported.repair.workingEdges = imported.working->snapshot.model.edgeCount();
    imported.repair.identity = !sourceShapeDigest.empty() &&
        sourceShapeDigest == workingShapeDigest && imported.correspondence.complete &&
        std::all_of(imported.correspondence.records.begin(),
                    imported.correspondence.records.end(),
                    [](const CorrespondenceRecord& record) {
                        return record.relation == CorrespondenceRelation::Identity;
                    });
    imported.repair.meshable = workingValid && imported.correspondence.complete;
    imported.repair.operations = std::move(operations);

    for (StableIdKind kind : {StableIdKind::Solid, StableIdKind::Face,
                              StableIdKind::Wire, StableIdKind::Coedge,
                              StableIdKind::Edge, StableIdKind::Vertex}) {
        const std::size_t before = occurrenceCount(imported.source->snapshot, kind) +
            (kind == StableIdKind::Coedge
                 ? imported.source->snapshot.coedges.size()
                 : 0);
        const std::size_t after = occurrenceCount(imported.working->snapshot, kind) +
            (kind == StableIdKind::Coedge
                 ? imported.working->snapshot.coedges.size()
                 : 0);
        if (before != after) {
            imported.repair.topologyCardinalityChanges.push_back(
                {kind, before, after});
        }
    }

    std::size_t mappedExpected = 0;
    std::size_t mappedChecked = 0;
    std::size_t mappedFailed = 0;
    std::size_t toleranceExpected = 0;
    std::size_t toleranceChecked = 0;
    std::size_t toleranceSkipped = 0;
    std::size_t representationExpected = 0;
    std::size_t representationChecked = 0;
    std::size_t representationSkipped = 0;
    for (const CorrespondenceRecord& record : imported.correspondence.records) {
        if (!record.sourceId.valid()) continue;
        ++mappedExpected;
        if (record.workingIds.empty()) {
            ++mappedFailed;
            continue;
        }
        ++mappedChecked;
        if (record.workingIds.size() != 1) {
            ++toleranceSkipped;
            if (record.sourceId.kind == StableIdKind::Edge) {
                ++representationSkipped;
            }
            continue;
        }
        const StableId workingId = record.workingIds.front();
        const TopoDS_Shape* sourceShape =
            shapeForId(imported.source->snapshot, record.sourceId);
        const TopoDS_Shape* workingShape =
            shapeForId(imported.working->snapshot, workingId);
        if (!sourceShape || !workingShape) {
            ++toleranceSkipped;
            if (record.sourceId.kind == StableIdKind::Edge) {
                ++representationSkipped;
            }
            continue;
        }
        ++toleranceExpected;
        ++toleranceChecked;
        const double before = shapeTolerance(*sourceShape);
        const double after = shapeTolerance(*workingShape);
        if (before != after) {
            imported.repair.toleranceChanges.push_back(
                {record.sourceId, workingId, before, after});
        }
        if (record.sourceId.kind == StableIdKind::Edge) {
            ++representationExpected;
            ++representationChecked;
            const std::size_t sourceUses = storedPcurveUseCount(
                imported.source->snapshot, record.sourceId);
            const std::size_t workingUses = storedPcurveUseCount(
                imported.working->snapshot, workingId);
            if (sourceUses != workingUses) {
                imported.repair.representationChanges.push_back(
                    {record.sourceId, workingId, sourceUses, workingUses});
            }
        }
    }

    imported.repair.validationEvidence = {
        {"repair.exact_shape_hash", 2,
         static_cast<std::size_t>(!sourceShapeDigest.empty()) +
             static_cast<std::size_t>(!workingShapeDigest.empty()),
         0,
         static_cast<std::size_t>(sourceShapeDigest.empty()) +
             static_cast<std::size_t>(workingShapeDigest.empty())},
        {"repair.source_working_correspondence", mappedExpected,
         mappedChecked, 0, mappedFailed},
        {"repair.tolerance_audit", toleranceExpected, toleranceChecked,
         toleranceSkipped, 0},
        {"repair.representation_audit", representationExpected,
         representationChecked, representationSkipped, 0},
    };
    if (imported.repair.operations.empty() && imported.repair.identity) {
        imported.repair.operations.push_back(
            {"repair.identity", {}, {}, "working B-rep is the source B-rep"});
    }

    std::uint64_t diagnosticOrdinal = 0;
    auto diagnostic = [&](std::string code, DiagnosticSeverity severity,
                          std::string message) {
        imported.diagnostics.events.push_back(
            {{StableIdKind::Diagnostic, ++diagnosticOrdinal}, std::move(code),
             severity, {{StableIdKind::Model, 1}}, std::move(message)});
    };
    if (!sourceValid) {
        diagnostic("import.source.invalid", DiagnosticSeverity::Warning,
                   "the immutable source B-rep is invalid");
    }
    if (!workingValid) {
        diagnostic("import.working.invalid", DiagnosticSeverity::Error,
                   "the derived working B-rep is invalid");
    }
    if (!imported.correspondence.complete) {
        diagnostic("import.correspondence.incomplete", DiagnosticSeverity::Error,
                   "source-to-working correspondence is incomplete");
    }
    if (!imported.repair.meshable) {
        diagnostic("import.working.non_meshable", DiagnosticSeverity::Error,
                   "working B-rep cannot enter authoritative meshing");
    }
    return imported;
}

}  // namespace secure_detail

namespace io {

ImportedModel Reader::transferSecure(RepairProfile profile) {
    Model model = transfer();
    SourceMetadata metadata;
    metadata.importerVersion = "weft-reader-identity-0.1";
    Handle(BRepTools_History) history = new BRepTools_History();
    return secure_detail::buildImportedModel(
        model, model, std::move(metadata), profile, history);
}

}  // namespace io

ImportedModel importStepSecure(const std::string& path, RepairProfile profile) {
    io::System system;
    io::bootstrapIo(system);
    std::unique_ptr<io::Reader> reader = system.createReader(io::Format::Step);
    if (!reader || !reader->readFile(path)) {
        throw SecureImportError("import.step.read_failed",
                                "failed to read STEP file securely: " + path);
    }
    return reader->transferSecure(profile);
}

}  // namespace weft
