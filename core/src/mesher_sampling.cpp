#include "mesher_sampling.hpp"

#include "weft/model.hpp"

#include <Adaptor3d_Curve.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRep_Tool.hxx>
#include <GCPnts_UniformAbscissa.hxx>
#include <GeomAdaptor_Curve.hxx>
#include <Geom_Curve.hxx>
#include <Standard_Failure.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Vertex.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <limits>

namespace weft::mesher_detail {

namespace {

std::vector<double> evenArcFractions(const Model& model, int edgeId,
                                     int divisions) {
    std::vector<double> out;
    if (divisions < 1 || edgeId < 1 || edgeId > model.edgeCount()) return out;
    try {
        const TopoDS_Edge edge = TopoDS::Edge(model.edges(edgeId));
        double first = 0.0, last = 0.0;
        Handle(Geom_Curve) curve3d = BRep_Tool::Curve(edge, first, last);
        if (curve3d.IsNull() || last - first <= 1e-12) return out;
        GeomAdaptor_Curve curve(curve3d, first, last);
        const GeomAbs_CurveType type = curve.GetType();
        if (type == GeomAbs_Line || type == GeomAbs_Circle) return out;
        GCPnts_UniformAbscissa stations(curve, divisions + 1);
        if (!stations.IsDone() || stations.NbPoints() != divisions + 1) {
            return out;
        }
        out.reserve(static_cast<size_t>(divisions) + 1);
        double previous = -1.0;
        for (int i = 1; i <= divisions + 1; ++i) {
            const double fraction = std::clamp(
                (stations.Parameter(i) - first) / (last - first), 0.0, 1.0);
            if (fraction < previous) return {};
            previous = fraction;
            out.push_back(fraction);
        }
        out.front() = 0.0;
        out.back() = 1.0;
        return out;
    } catch (const Standard_Failure&) {
        // Enrollment is fail-open: malformed freeform geometry keeps the
        // deterministic legacy uniform sequence for this edge only.
        return {};
    }
}

std::vector<double> rawEdgeSampleFractions(int edgeId, int divisions,
                                           double phase, bool reversed,
                                           bool includeLast,
                                           const PinnedEdges* pins,
                                           const Model* model) {
    std::vector<double> fractions;
    if (edgeIsPinned(edgeId, pins)) {
        fractions = (*pins)[edgeId];
        if (reversed) std::reverse(fractions.begin(), fractions.end());
        if (!includeLast && fractions.size() > 1) fractions.pop_back();
        return fractions;
    }
    divisions = std::max(1, divisions);
    if (model && phase == 0.0) {
        fractions = evenArcFractions(*model, edgeId, divisions);
        if (!fractions.empty()) {
            if (reversed) std::reverse(fractions.begin(), fractions.end());
            if (!includeLast && fractions.size() > 1) fractions.pop_back();
            return fractions;
        }
    }
    const int last = includeLast ? divisions : divisions - 1;
    fractions.reserve(static_cast<size_t>(last) + 1);
    for (int i = 0; i <= last; ++i) {
        fractions.push_back(phasedT(i, divisions, phase, reversed));
    }
    return fractions;
}

std::uint64_t interiorSampleId(int edgeId, int station) {
    return (std::uint64_t{1} << 63) |
           (static_cast<std::uint64_t>(static_cast<std::uint32_t>(edgeId))
            << 32) |
           static_cast<std::uint32_t>(station);
}

} // namespace

bool edgeIsPinned(int edgeId, const PinnedEdges* pins) {
    return pins && edgeId >= 1 && edgeId < static_cast<int>(pins->size()) &&
           !(*pins)[edgeId].empty();
}

double phasedT(int index, int divisions, double phase, bool reversed) {
    if (phase <= 0.0) {
        return reversed ? 1.0 - double(index) / divisions
                        : double(index) / divisions;
    }
    const int sample = reversed ? (divisions - index % divisions) % divisions
                                : index % divisions;
    double parameter = phase + double(sample) / divisions;
    parameter -= std::floor(parameter);
    return parameter;
}

std::vector<double> edgeSampleFractions(int edgeId, int divisions, double phase,
                                        bool reversed, bool includeLast,
                                        const PinnedEdges* pins,
                                        const Model* model) {
    const CanonicalEdgePlan* contract =
        pins && pins->canonical ? pins->canonical->find(edgeId) : nullptr;
    const int requested = edgeIsPinned(edgeId, pins)
                              ? static_cast<int>((*pins)[edgeId].size()) - 1
                              : std::max(1, divisions);
    if (contract && contract->parameterLinear && model &&
        contract->segmentCount == requested &&
        (edgeIsPinned(edgeId, pins) ||
         std::abs(contract->phase - phase) <= 1e-14)) {
        if (contract->samples.size() ==
            static_cast<size_t>(contract->segmentCount) + 1) {
            std::vector<double> fractions;
            fractions.reserve(contract->samples.size());
            for (const CanonicalEdgeSample& sample : contract->samples) {
                fractions.push_back(sample.curveFraction);
            }
            if (reversed) std::reverse(fractions.begin(), fractions.end());
            if (!includeLast && fractions.size() > 1) fractions.pop_back();
            return fractions;
        }
    }
    return rawEdgeSampleFractions(edgeId, divisions, phase, reversed,
                                  includeLast, pins, model);
}

std::shared_ptr<const CanonicalEdgeTable> buildCanonicalEdgeTable(
    const Model& model, const std::vector<int>& finalSegmentCounts,
    const PinnedEdges& pins, const std::vector<double>& edgePhases) {
    auto table = std::make_shared<CanonicalEdgeTable>();
    table->edgePlans.resize(model.edgeCount());

    ShapeMap vertices;
    TopExp::MapShapes(model.shape, TopAbs_VERTEX, vertices);
    for (int edgeId = 1; edgeId <= model.edgeCount(); ++edgeId) {
        try {
            TopoDS_Edge edge = TopoDS::Edge(model.edges(edgeId));
            if (BRep_Tool::Degenerated(edge) ||
                !model.edgeToFaces.Contains(edge) ||
                model.edgeToFaces.FindFromKey(edge).Extent() != 2) {
                continue;
            }
            edge.Orientation(TopAbs_FORWARD);
            const int ideal =
                edgeId < static_cast<int>(finalSegmentCounts.size())
                    ? std::max(1, finalSegmentCounts[edgeId])
                    : 1;
            const double phase = edgeId < static_cast<int>(edgePhases.size())
                                     ? edgePhases[edgeId]
                                     : 0.0;
            double first = 0.0, last = 0.0;
            Handle(Geom_Curve) curve3d = BRep_Tool::Curve(edge, first, last);
            if (curve3d.IsNull() || std::abs(last - first) <= 1e-15) continue;
            GeomAdaptor_Curve adaptor(curve3d, first, last);
            const GeomAbs_CurveType type = adaptor.GetType();
            const bool parameterLinear =
                type == GeomAbs_Line || type == GeomAbs_Circle;
            // R1 migrates one curve family at a time.  Freeform edges keep
            // their existing even-arc legacy path until their face-UV
            // projection can consume sample ids directly; do not pay to build
            // an unused table.
            if (!parameterLinear) continue;

            const std::vector<double> fractions = rawEdgeSampleFractions(
                edgeId, ideal, phase, false, true, &pins, &model);
            if (fractions.size() < 2) continue;

            CanonicalEdgePlan& plan = table->edgePlans[edgeId - 1];
            plan.edgeId = edgeId;
            plan.idealSegmentCount = ideal;
            plan.segmentCount = static_cast<int>(fractions.size()) - 1;
            plan.phase = phase;
            plan.uniformAbscissa = !edgeIsPinned(edgeId, &pins);
            plan.parameterLinear = parameterLinear;

            TopoDS_Vertex startVertex, endVertex;
            TopExp::Vertices(edge, startVertex, endVertex);
            const int startVertexId =
                startVertex.IsNull() ? 0 : vertices.FindIndex(startVertex);
            const int endVertexId =
                endVertex.IsNull() ? 0 : vertices.FindIndex(endVertex);
            plan.closed = startVertexId > 0 && startVertexId == endVertexId;
            try {
                plan.closed = plan.closed || BRepAdaptor_Curve(edge).IsClosed();
            } catch (const Standard_Failure&) {
            }
            plan.samples.reserve(fractions.size());
            for (size_t i = 0; i < fractions.size(); ++i) {
                CanonicalEdgeSample sample;
                sample.curveFraction = fractions[i];
                sample.curveParameter = first + (last - first) * fractions[i];
                sample.normalizedAbscissa =
                    plan.closed && phase != 0.0
                        ? i / static_cast<double>(fractions.size() - 1)
                        : sample.curveFraction;
                const bool atStart = phase == 0.0 && i == 0 &&
                                     fractions.front() == 0.0 &&
                                     startVertexId > 0;
                const bool atEnd = phase == 0.0 && i + 1 == fractions.size() &&
                                   fractions.back() == 1.0 && endVertexId > 0;
                sample.id = atStart ? static_cast<std::uint64_t>(startVertexId)
                            : atEnd
                                ? static_cast<std::uint64_t>(endVertexId)
                                : interiorSampleId(edgeId, static_cast<int>(i));
                try {
                    // A topological vertex owns one position across every
                    // incident edge.  Curve evaluation is reserved for interior
                    // stations so equal endpoint ids can never carry subtly
                    // different coordinates on tolerant CAD.
                    const gp_Pnt point =
                        atStart ? BRep_Tool::Pnt(startVertex)
                        : atEnd ? BRep_Tool::Pnt(endVertex)
                                : curve3d->Value(sample.curveParameter);
                    sample.position = {point.X(), point.Y(), point.Z()};
                    sample.valid = std::isfinite(point.X()) &&
                                   std::isfinite(point.Y()) &&
                                   std::isfinite(point.Z());
                } catch (const Standard_Failure&) {
                    sample.valid = false;
                }
                plan.samples.push_back(sample);
            }
            if (std::any_of(plan.samples.begin(), plan.samples.end(),
                            [](const CanonicalEdgeSample& sample) {
                                return !sample.valid;
                            })) {
                plan = CanonicalEdgePlan{};
                continue;
            }
            if (plan.closed && !plan.samples.empty()) {
                plan.samples.back().id = plan.samples.front().id;
                plan.samples.back().position = plan.samples.front().position;
                plan.samples.back().valid = plan.samples.front().valid;
            }
        } catch (const Standard_Failure&) {
            // A bad edge is excluded from the guarded table; all legacy face
            // builders retain their established fail-open sampling path.
            table->edgePlans[edgeId - 1] = CanonicalEdgePlan{};
        }
    }
    return table;
}

// Platform-stable tangential-deflection count. OCCT's iterative subdivider
// can take different data-dependent branches on last-ulp libm differences.
// This fixed-order scan keeps topology deterministic across platforms.
int stableDeflectionCount(const Adaptor3d_Curve& curve,
                          double angleTolerance, double chordTolerance) {
    const double first = curve.FirstParameter();
    const double last = curve.LastParameter();
    if (!(last > first)) return 1;

    constexpr int kSamples = 33;
    gp_Pnt points[kSamples];
    double turn[kSamples] = {0};
    gp_Vec previousDirection;
    bool previousDirectionValid = false;
    bool previousPointValid = false;
    for (int i = 0; i < kSamples; ++i) {
        const double parameter =
            first + (last - first) * i / double(kSamples - 1);
        gp_Vec direction;
        bool directionValid = false;
        bool pointValid = true;
        try {
            gp_Vec derivative;
            curve.D1(parameter, points[i], derivative);
            if (derivative.Magnitude() > 1e-12) {
                direction = derivative.Normalized();
                directionValid = true;
            }
        } catch (const Standard_Failure&) {
            try {
                points[i] = curve.Value(parameter);
            } catch (const Standard_Failure&) {
                pointValid = false;
            }
        }
        if (i > 0) {
            turn[i] = turn[i - 1];
            if (!pointValid) points[i] = points[i - 1];
        }
        if (!pointValid) continue;
        if (previousPointValid) {
            if (directionValid && previousDirectionValid) {
                turn[i] =
                    turn[i - 1] + previousDirection.Angle(direction);
            }
        }
        if (directionValid) {
            previousDirection = direction;
            previousDirectionValid = true;
        }
        previousPointValid = true;
    }

    const double angleLimit =
        std::max(angleTolerance, 1e-3) * (1.0 + 1e-9);
    const double chordLimit =
        std::max(chordTolerance, 1e-12) * (1.0 + 1e-9);
    auto interpolate = [&](const double* values, double fraction) {
        const double sample = fraction * (kSamples - 1);
        const int index =
            std::min(kSamples - 2, std::max(0, int(sample)));
        return values[index] +
               (values[index + 1] - values[index]) * (sample - index);
    };
    auto pointAt = [&](double fraction) {
        const double sample = fraction * (kSamples - 1);
        const int index =
            std::min(kSamples - 2, std::max(0, int(sample)));
        const double weight = sample - index;
        return gp_Pnt(
            points[index].X() +
                (points[index + 1].X() - points[index].X()) * weight,
            points[index].Y() +
                (points[index + 1].Y() - points[index].Y()) * weight,
            points[index].Z() +
                (points[index + 1].Z() - points[index].Z()) * weight);
    };

    for (int divisions = 1; divisions < 256; ++divisions) {
        const bool belowSamplingResolution = divisions >= kSamples - 1;
        bool valid = true;
        for (int i = 0; i < divisions && valid; ++i) {
            const double start = i / double(divisions);
            const double end = (i + 1) / double(divisions);
            const double intervalTurn =
                interpolate(turn, end) - interpolate(turn, start);
            if (!belowSamplingResolution && intervalTurn > angleLimit) {
                valid = false;
                break;
            }

            const gp_Pnt a = pointAt(start);
            const gp_Pnt b = pointAt(end);
            const gp_Pnt midpoint = pointAt(0.5 * (start + end));
            const gp_Vec chord(a, b);
            const double chordSquared = chord.SquareMagnitude();
            const gp_Vec toMidpoint(a, midpoint);
            double weight = chordSquared > 1e-24
                                ? toMidpoint.Dot(chord) / chordSquared
                                : 0.0;
            weight = std::clamp(weight, 0.0, 1.0);
            const gp_Pnt projected(a.X() + chord.X() * weight,
                                   a.Y() + chord.Y() * weight,
                                   a.Z() + chord.Z() * weight);
            if (midpoint.Distance(projected) > chordLimit) valid = false;
        }
        if (valid) return divisions;
    }
    return 256;
}

}  // namespace weft::mesher_detail
