#pragma once

#include <array>
#include <memory>
#include <optional>
#include <string>

namespace weft {

using PredicatePoint2 = std::array<double, 2>;

enum class ExactSign {
    Negative = -1,
    Zero = 0,
    Positive = 1,
};

enum class SegmentIntersectionKind {
    None,
    Proper,
    EndpointTouch,
    CollinearOverlap,
};

struct PredicateFailure {
    std::string code;
    std::string message;
};

template <typename T>
struct PredicateResult {
    std::optional<T> value;
    std::optional<PredicateFailure> failure;

    explicit operator bool() const noexcept { return value.has_value(); }
};

class GeometricPredicates {
public:
    virtual ~GeometricPredicates() = default;

    virtual const char* backendCode() const noexcept = 0;
    virtual bool exactForFiniteDoubleInputs() const noexcept = 0;

    // Positive means a,b,c are counter-clockwise.
    virtual PredicateResult<ExactSign> orient2d(
        PredicatePoint2 a, PredicatePoint2 b,
        PredicatePoint2 c) const = 0;

    // For counter-clockwise a,b,c, positive means d is inside their circle.
    virtual PredicateResult<ExactSign> incircle(
        PredicatePoint2 a, PredicatePoint2 b, PredicatePoint2 c,
        PredicatePoint2 d) const = 0;

    // Compares squared distances from origin without rounding the products.
    // Negative means first is nearer, positive means second is nearer.
    virtual PredicateResult<ExactSign> compareSquaredDistance(
        PredicatePoint2 origin, PredicatePoint2 first,
        PredicatePoint2 second) const = 0;

    virtual PredicateResult<SegmentIntersectionKind> segmentIntersection(
        PredicatePoint2 a, PredicatePoint2 b, PredicatePoint2 c,
        PredicatePoint2 d) const = 0;
};

// Distribution-safe reference backend. Each finite IEEE-754 double is decoded
// into its exact dyadic rational and determinant signs are evaluated with an
// unbounded signed integer. It favours auditability over throughput; an
// adaptive Shewchuk backend may replace it behind the same interface.
std::shared_ptr<const GeometricPredicates> makeExactDyadicPredicates();

}  // namespace weft
