#include "mesher_sampling.hpp"

#include <Adaptor3d_Curve.hxx>
#include <Standard_Failure.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>

#include <algorithm>
#include <cmath>

namespace weft::mesher_detail {

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
