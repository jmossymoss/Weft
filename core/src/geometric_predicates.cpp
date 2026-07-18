#include "weft/geometric_predicates.hpp"

#include <algorithm>
#include <bit>
#include <cmath>
#include <cstdint>
#include <limits>
#include <utility>
#include <vector>

namespace weft {
namespace {

class SignedBigInteger {
public:
    SignedBigInteger() = default;

    static SignedBigInteger fromMagnitude(std::uint64_t magnitude,
                                          bool negative) {
        SignedBigInteger result;
        if (magnitude == 0) return result;
        result.limbs_.push_back(static_cast<std::uint32_t>(magnitude));
        const std::uint32_t high =
            static_cast<std::uint32_t>(magnitude >> 32U);
        if (high != 0) result.limbs_.push_back(high);
        result.negative_ = negative;
        return result;
    }

    bool zero() const noexcept { return limbs_.empty(); }

    int sign() const noexcept {
        if (zero()) return 0;
        return negative_ ? -1 : 1;
    }

    SignedBigInteger negated() const {
        SignedBigInteger result = *this;
        if (!result.zero()) result.negative_ = !result.negative_;
        return result;
    }

    SignedBigInteger shiftedLeft(std::size_t bits) const {
        if (zero() || bits == 0) return *this;
        const std::size_t words = bits / 32U;
        const unsigned remainder = static_cast<unsigned>(bits % 32U);
        SignedBigInteger result;
        result.negative_ = negative_;
        result.limbs_.assign(words, 0U);
        std::uint64_t carry = 0;
        for (std::uint32_t limb : limbs_) {
            const std::uint64_t shifted =
                (static_cast<std::uint64_t>(limb) << remainder) | carry;
            result.limbs_.push_back(static_cast<std::uint32_t>(shifted));
            carry = shifted >> 32U;
        }
        if (carry != 0) {
            result.limbs_.push_back(static_cast<std::uint32_t>(carry));
        }
        return result;
    }

    friend SignedBigInteger add(const SignedBigInteger& left,
                                const SignedBigInteger& right) {
        if (left.zero()) return right;
        if (right.zero()) return left;
        SignedBigInteger result;
        if (left.negative_ == right.negative_) {
            result = addMagnitude(left, right);
            result.negative_ = left.negative_;
            return result;
        }
        const int comparison = compareMagnitude(left, right);
        if (comparison == 0) return result;
        if (comparison > 0) {
            result = subtractMagnitude(left, right);
            result.negative_ = left.negative_;
        } else {
            result = subtractMagnitude(right, left);
            result.negative_ = right.negative_;
        }
        return result;
    }

    friend SignedBigInteger subtract(const SignedBigInteger& left,
                                     const SignedBigInteger& right) {
        return add(left, right.negated());
    }

    friend SignedBigInteger multiply(const SignedBigInteger& left,
                                     const SignedBigInteger& right) {
        SignedBigInteger result;
        if (left.zero() || right.zero()) return result;
        result.limbs_.assign(left.limbs_.size() + right.limbs_.size(), 0U);
        for (std::size_t i = 0; i < left.limbs_.size(); ++i) {
            std::uint64_t carry = 0;
            for (std::size_t j = 0; j < right.limbs_.size(); ++j) {
                const std::size_t target = i + j;
                const std::uint64_t product =
                    static_cast<std::uint64_t>(left.limbs_[i]) *
                    static_cast<std::uint64_t>(right.limbs_[j]);
                const std::uint64_t accumulated =
                    product + result.limbs_[target] + carry;
                result.limbs_[target] =
                    static_cast<std::uint32_t>(accumulated);
                carry = accumulated >> 32U;
            }
            std::size_t target = i + right.limbs_.size();
            while (carry != 0) {
                if (target == result.limbs_.size()) {
                    result.limbs_.push_back(0U);
                }
                const std::uint64_t accumulated =
                    static_cast<std::uint64_t>(result.limbs_[target]) + carry;
                result.limbs_[target] =
                    static_cast<std::uint32_t>(accumulated);
                carry = accumulated >> 32U;
                ++target;
            }
        }
        result.negative_ = left.negative_ != right.negative_;
        result.normalise();
        return result;
    }

private:
    static int compareMagnitude(const SignedBigInteger& left,
                                const SignedBigInteger& right) {
        if (left.limbs_.size() != right.limbs_.size()) {
            return left.limbs_.size() < right.limbs_.size() ? -1 : 1;
        }
        for (std::size_t index = left.limbs_.size(); index-- > 0;) {
            if (left.limbs_[index] != right.limbs_[index]) {
                return left.limbs_[index] < right.limbs_[index] ? -1 : 1;
            }
        }
        return 0;
    }

    static SignedBigInteger addMagnitude(const SignedBigInteger& left,
                                         const SignedBigInteger& right) {
        SignedBigInteger result;
        const std::size_t size =
            std::max(left.limbs_.size(), right.limbs_.size());
        result.limbs_.resize(size);
        std::uint64_t carry = 0;
        for (std::size_t index = 0; index < size; ++index) {
            const std::uint64_t a = index < left.limbs_.size()
                ? left.limbs_[index]
                : 0U;
            const std::uint64_t b = index < right.limbs_.size()
                ? right.limbs_[index]
                : 0U;
            const std::uint64_t sum = a + b + carry;
            result.limbs_[index] = static_cast<std::uint32_t>(sum);
            carry = sum >> 32U;
        }
        if (carry != 0) {
            result.limbs_.push_back(static_cast<std::uint32_t>(carry));
        }
        return result;
    }

    // Requires |larger| > |smaller|.
    static SignedBigInteger subtractMagnitude(
        const SignedBigInteger& larger, const SignedBigInteger& smaller) {
        SignedBigInteger result;
        result.limbs_.resize(larger.limbs_.size());
        std::uint64_t borrow = 0;
        for (std::size_t index = 0; index < larger.limbs_.size(); ++index) {
            const std::uint64_t a = larger.limbs_[index];
            const std::uint64_t b = index < smaller.limbs_.size()
                ? smaller.limbs_[index]
                : 0U;
            const std::uint64_t subtrahend = b + borrow;
            result.limbs_[index] = static_cast<std::uint32_t>(a - subtrahend);
            borrow = a < subtrahend ? 1U : 0U;
        }
        result.normalise();
        return result;
    }

    void normalise() {
        while (!limbs_.empty() && limbs_.back() == 0) limbs_.pop_back();
        if (limbs_.empty()) negative_ = false;
    }

    bool negative_ = false;
    std::vector<std::uint32_t> limbs_;
};

struct ExactDyadic {
    SignedBigInteger significand;
    int exponent = 0;
};

ExactDyadic exactDouble(double value) {
    const std::uint64_t bits = std::bit_cast<std::uint64_t>(value);
    const bool negative = (bits >> 63U) != 0;
    const std::uint64_t encodedExponent = (bits >> 52U) & 0x7ffU;
    const std::uint64_t fraction = bits & ((std::uint64_t{1} << 52U) - 1U);
    if (encodedExponent == 0) {
        return {SignedBigInteger::fromMagnitude(fraction, negative), -1074};
    }
    const std::uint64_t mantissa =
        (std::uint64_t{1} << 52U) | fraction;
    return {SignedBigInteger::fromMagnitude(mantissa, negative),
            static_cast<int>(encodedExponent) - 1023 - 52};
}

ExactDyadic add(ExactDyadic left, ExactDyadic right) {
    if (left.significand.zero()) return right;
    if (right.significand.zero()) return left;
    if (left.exponent < right.exponent) {
        right.significand = right.significand.shiftedLeft(
            static_cast<std::size_t>(right.exponent - left.exponent));
        right.exponent = left.exponent;
    } else if (right.exponent < left.exponent) {
        left.significand = left.significand.shiftedLeft(
            static_cast<std::size_t>(left.exponent - right.exponent));
        left.exponent = right.exponent;
    }
    return {add(left.significand, right.significand), left.exponent};
}

ExactDyadic subtract(ExactDyadic left, ExactDyadic right) {
    right.significand = right.significand.negated();
    return add(std::move(left), std::move(right));
}

ExactDyadic multiply(const ExactDyadic& left, const ExactDyadic& right) {
    return {multiply(left.significand, right.significand),
            left.exponent + right.exponent};
}

ExactDyadic difference(double left, double right) {
    return subtract(exactDouble(left), exactDouble(right));
}

ExactDyadic cross(const ExactDyadic& ax, const ExactDyadic& ay,
                  const ExactDyadic& bx, const ExactDyadic& by) {
    return subtract(multiply(ax, by), multiply(ay, bx));
}

ExactSign signOf(const ExactDyadic& value) {
    const int sign = value.significand.sign();
    if (sign < 0) return ExactSign::Negative;
    if (sign > 0) return ExactSign::Positive;
    return ExactSign::Zero;
}

bool finite(PredicatePoint2 point) {
    return std::isfinite(point[0]) && std::isfinite(point[1]);
}

bool finite(PredicatePoint3 point) {
    return std::isfinite(point[0]) && std::isfinite(point[1]) &&
        std::isfinite(point[2]);
}

bool finite(const PredicateTriangle3& triangle) {
    return finite(triangle[0]) && finite(triangle[1]) && finite(triangle[2]);
}

bool pointsEqual(PredicatePoint3 left, PredicatePoint3 right) {
    return left[0] == right[0] && left[1] == right[1] && left[2] == right[2];
}

ExactDyadic scalarTriple(const ExactDyadic& ax, const ExactDyadic& ay,
                         const ExactDyadic& az, const ExactDyadic& bx,
                         const ExactDyadic& by, const ExactDyadic& bz,
                         const ExactDyadic& cx, const ExactDyadic& cy,
                         const ExactDyadic& cz) {
    return add(
        add(multiply(ax, cross(by, bz, cy, cz)),
            multiply(ay, cross(bz, bx, cz, cx))),
        multiply(az, cross(bx, by, cx, cy)));
}

int dominantAxis(const ExactDyadic& nx, const ExactDyadic& ny,
                 const ExactDyadic& nz) {
    const ExactDyadic nx2 = multiply(nx, nx);
    const ExactDyadic ny2 = multiply(ny, ny);
    const ExactDyadic nz2 = multiply(nz, nz);
    if (signOf(subtract(nx2, ny2)) != ExactSign::Negative &&
        signOf(subtract(nx2, nz2)) != ExactSign::Negative) {
        return 0;
    }
    if (signOf(subtract(ny2, nz2)) != ExactSign::Negative) return 1;
    return 2;
}

PredicatePoint2 projectPoint(PredicatePoint3 point, int dropAxis) {
    if (dropAxis == 0) return {point[1], point[2]};
    if (dropAxis == 1) return {point[0], point[2]};
    return {point[0], point[1]};
}

template <typename T>
PredicateResult<T> predicateFailure(std::string code, std::string message) {
    PredicateResult<T> result;
    result.failure = PredicateFailure{std::move(code), std::move(message)};
    return result;
}

bool pointsEqual(PredicatePoint2 left, PredicatePoint2 right) {
    return left[0] == right[0] && left[1] == right[1];
}

bool between(double value, double first, double second) {
    return value >= std::min(first, second) &&
        value <= std::max(first, second);
}

bool onBoundingBox(PredicatePoint2 point, PredicatePoint2 first,
                   PredicatePoint2 second) {
    return between(point[0], first[0], second[0]) &&
        between(point[1], first[1], second[1]);
}

bool opposite(ExactSign left, ExactSign right) {
    return (left == ExactSign::Negative && right == ExactSign::Positive) ||
        (left == ExactSign::Positive && right == ExactSign::Negative);
}

class ExactDyadicPredicates final : public GeometricPredicates {
public:
    const char* backendCode() const noexcept override {
        return "exact_ieee754_dyadic";
    }

    bool exactForFiniteDoubleInputs() const noexcept override { return true; }

    PredicateResult<ExactSign> orient2d(
        PredicatePoint2 a, PredicatePoint2 b,
        PredicatePoint2 c) const override {
        if (!finite(a) || !finite(b) || !finite(c)) {
            return predicateFailure<ExactSign>(
                "predicate.non_finite_input",
                "orientation requires finite IEEE-754 coordinates");
        }
        const ExactDyadic acx = difference(a[0], c[0]);
        const ExactDyadic acy = difference(a[1], c[1]);
        const ExactDyadic bcx = difference(b[0], c[0]);
        const ExactDyadic bcy = difference(b[1], c[1]);
        PredicateResult<ExactSign> result;
        result.value = signOf(cross(acx, acy, bcx, bcy));
        return result;
    }

    PredicateResult<ExactSign> orient3d(
        PredicatePoint3 a, PredicatePoint3 b, PredicatePoint3 c,
        PredicatePoint3 d) const override {
        if (!finite(a) || !finite(b) || !finite(c) || !finite(d)) {
            return predicateFailure<ExactSign>(
                "predicate.non_finite_input",
                "3D orientation requires finite IEEE-754 coordinates");
        }
        // Positive when d lies above the oriented plane through a,b,c
        // (a,b,c counter-clockwise when viewed from the positive side).
        const ExactDyadic bax = difference(b[0], a[0]);
        const ExactDyadic bay = difference(b[1], a[1]);
        const ExactDyadic baz = difference(b[2], a[2]);
        const ExactDyadic cax = difference(c[0], a[0]);
        const ExactDyadic cay = difference(c[1], a[1]);
        const ExactDyadic caz = difference(c[2], a[2]);
        const ExactDyadic dax = difference(d[0], a[0]);
        const ExactDyadic day = difference(d[1], a[1]);
        const ExactDyadic daz = difference(d[2], a[2]);
        PredicateResult<ExactSign> result;
        result.value = signOf(scalarTriple(
            bax, bay, baz, cax, cay, caz, dax, day, daz));
        return result;
    }

    PredicateResult<ExactSign> incircle(
        PredicatePoint2 a, PredicatePoint2 b, PredicatePoint2 c,
        PredicatePoint2 d) const override {
        if (!finite(a) || !finite(b) || !finite(c) || !finite(d)) {
            return predicateFailure<ExactSign>(
                "predicate.non_finite_input",
                "incircle requires finite IEEE-754 coordinates");
        }
        const ExactDyadic adx = difference(a[0], d[0]);
        const ExactDyadic ady = difference(a[1], d[1]);
        const ExactDyadic bdx = difference(b[0], d[0]);
        const ExactDyadic bdy = difference(b[1], d[1]);
        const ExactDyadic cdx = difference(c[0], d[0]);
        const ExactDyadic cdy = difference(c[1], d[1]);

        const ExactDyadic abdet = cross(adx, ady, bdx, bdy);
        const ExactDyadic bcdet = cross(bdx, bdy, cdx, cdy);
        const ExactDyadic cadet = cross(cdx, cdy, adx, ady);
        const ExactDyadic alift =
            add(multiply(adx, adx), multiply(ady, ady));
        const ExactDyadic blift =
            add(multiply(bdx, bdx), multiply(bdy, bdy));
        const ExactDyadic clift =
            add(multiply(cdx, cdx), multiply(cdy, cdy));
        const ExactDyadic determinant = add(
            add(multiply(alift, bcdet), multiply(blift, cadet)),
            multiply(clift, abdet));
        PredicateResult<ExactSign> result;
        result.value = signOf(determinant);
        return result;
    }

    PredicateResult<ExactSign> compareSquaredDistance(
        PredicatePoint2 origin, PredicatePoint2 first,
        PredicatePoint2 second) const override {
        if (!finite(origin) || !finite(first) || !finite(second)) {
            return predicateFailure<ExactSign>(
                "predicate.non_finite_input",
                "squared-distance comparison requires finite IEEE-754 coordinates");
        }
        const ExactDyadic firstX = difference(first[0], origin[0]);
        const ExactDyadic firstY = difference(first[1], origin[1]);
        const ExactDyadic secondX = difference(second[0], origin[0]);
        const ExactDyadic secondY = difference(second[1], origin[1]);
        const ExactDyadic firstSquared = add(
            multiply(firstX, firstX), multiply(firstY, firstY));
        const ExactDyadic secondSquared = add(
            multiply(secondX, secondX), multiply(secondY, secondY));
        PredicateResult<ExactSign> result;
        result.value = signOf(subtract(firstSquared, secondSquared));
        return result;
    }

    PredicateResult<SegmentIntersectionKind> segmentIntersection(
        PredicatePoint2 a, PredicatePoint2 b, PredicatePoint2 c,
        PredicatePoint2 d) const override {
        if (!finite(a) || !finite(b) || !finite(c) || !finite(d)) {
            return predicateFailure<SegmentIntersectionKind>(
                "predicate.non_finite_input",
                "segment intersection requires finite IEEE-754 coordinates");
        }
        const auto o1 = orient2d(a, b, c);
        const auto o2 = orient2d(a, b, d);
        const auto o3 = orient2d(c, d, a);
        const auto o4 = orient2d(c, d, b);
        if (!o1 || !o2 || !o3 || !o4) {
            return predicateFailure<SegmentIntersectionKind>(
                "predicate.internal_failure",
                "finite orientation unexpectedly failed");
        }

        PredicateResult<SegmentIntersectionKind> result;
        if (opposite(*o1.value, *o2.value) &&
            opposite(*o3.value, *o4.value)) {
            result.value = SegmentIntersectionKind::Proper;
            return result;
        }

        const bool allCollinear = *o1.value == ExactSign::Zero &&
            *o2.value == ExactSign::Zero &&
            *o3.value == ExactSign::Zero && *o4.value == ExactSign::Zero;
        if (allCollinear) {
            const double xSpread = std::max(
                {std::abs(a[0] - b[0]), std::abs(c[0] - d[0]),
                 std::abs(a[0] - c[0])});
            const double ySpread = std::max(
                {std::abs(a[1] - b[1]), std::abs(c[1] - d[1]),
                 std::abs(a[1] - c[1])});
            const std::size_t axis = xSpread >= ySpread ? 0U : 1U;
            const double lower = std::max(std::min(a[axis], b[axis]),
                                          std::min(c[axis], d[axis]));
            const double upper = std::min(std::max(a[axis], b[axis]),
                                          std::max(c[axis], d[axis]));
            if (upper < lower) {
                result.value = SegmentIntersectionKind::None;
            } else if (upper == lower ||
                       (pointsEqual(a, b) && pointsEqual(c, d))) {
                result.value = SegmentIntersectionKind::EndpointTouch;
            } else {
                result.value = SegmentIntersectionKind::CollinearOverlap;
            }
            return result;
        }

        const bool touches =
            (*o1.value == ExactSign::Zero && onBoundingBox(c, a, b)) ||
            (*o2.value == ExactSign::Zero && onBoundingBox(d, a, b)) ||
            (*o3.value == ExactSign::Zero && onBoundingBox(a, c, d)) ||
            (*o4.value == ExactSign::Zero && onBoundingBox(b, c, d));
        result.value = touches ? SegmentIntersectionKind::EndpointTouch
                               : SegmentIntersectionKind::None;
        return result;
    }

    PredicateResult<bool> pointInTriangle2d(
        PredicatePoint2 point, PredicatePoint2 a, PredicatePoint2 b,
        PredicatePoint2 c) const {
        const auto oab = orient2d(a, b, point);
        const auto obc = orient2d(b, c, point);
        const auto oca = orient2d(c, a, point);
        if (!oab || !obc || !oca) {
            return predicateFailure<bool>(
                "predicate.internal_failure",
                "finite 2D orientation unexpectedly failed");
        }
        const bool hasPositive = *oab.value == ExactSign::Positive ||
            *obc.value == ExactSign::Positive ||
            *oca.value == ExactSign::Positive;
        const bool hasNegative = *oab.value == ExactSign::Negative ||
            *obc.value == ExactSign::Negative ||
            *oca.value == ExactSign::Negative;
        PredicateResult<bool> result;
        result.value = !(hasPositive && hasNegative);
        return result;
    }

    PredicateResult<TriangleIntersection3dKind> classifyCoplanar(
        PredicateTriangle3 first, PredicateTriangle3 second) const {
        const ExactDyadic abx = difference(first[1][0], first[0][0]);
        const ExactDyadic aby = difference(first[1][1], first[0][1]);
        const ExactDyadic abz = difference(first[1][2], first[0][2]);
        const ExactDyadic acx = difference(first[2][0], first[0][0]);
        const ExactDyadic acy = difference(first[2][1], first[0][1]);
        const ExactDyadic acz = difference(first[2][2], first[0][2]);
        const ExactDyadic nx = cross(aby, abz, acy, acz);
        const ExactDyadic ny = cross(abz, abx, acz, acx);
        const ExactDyadic nz = cross(abx, aby, acx, acy);
        const int axis = dominantAxis(nx, ny, nz);
        const PredicatePoint2 a = projectPoint(first[0], axis);
        const PredicatePoint2 b = projectPoint(first[1], axis);
        const PredicatePoint2 c = projectPoint(first[2], axis);
        const PredicatePoint2 d = projectPoint(second[0], axis);
        const PredicatePoint2 e = projectPoint(second[1], axis);
        const PredicatePoint2 f = projectPoint(second[2], axis);

        bool areaOverlap = false;
        bool edgeContact = false;
        bool pointContact = false;
        const PredicatePoint2 firstVerts[3] = {a, b, c};
        const PredicatePoint2 secondVerts[3] = {d, e, f};
        for (const PredicatePoint2& point : secondVerts) {
            const auto inside = pointInTriangle2d(point, a, b, c);
            if (!inside) {
                return predicateFailure<TriangleIntersection3dKind>(
                    inside.failure->code, inside.failure->message);
            }
            if (*inside.value) {
                bool onVertex = false;
                for (const PredicatePoint2& vertex : firstVerts) {
                    if (pointsEqual(point, vertex)) {
                        onVertex = true;
                        pointContact = true;
                        break;
                    }
                }
                if (!onVertex) {
                    bool onEdge = false;
                    for (std::size_t edge = 0; edge < 3; ++edge) {
                        const auto relation = segmentIntersection(
                            firstVerts[edge], firstVerts[(edge + 1) % 3],
                            point, point);
                        if (!relation) {
                            return predicateFailure<TriangleIntersection3dKind>(
                                relation.failure->code,
                                relation.failure->message);
                        }
                        if (*relation.value != SegmentIntersectionKind::None) {
                            onEdge = true;
                            if (*relation.value ==
                                SegmentIntersectionKind::CollinearOverlap) {
                                edgeContact = true;
                            } else {
                                pointContact = true;
                            }
                            break;
                        }
                    }
                    if (!onEdge) areaOverlap = true;
                }
            }
        }
        for (const PredicatePoint2& point : firstVerts) {
            const auto inside = pointInTriangle2d(point, d, e, f);
            if (!inside) {
                return predicateFailure<TriangleIntersection3dKind>(
                    inside.failure->code, inside.failure->message);
            }
            if (*inside.value) {
                bool onVertex = false;
                for (const PredicatePoint2& vertex : secondVerts) {
                    if (pointsEqual(point, vertex)) {
                        onVertex = true;
                        pointContact = true;
                        break;
                    }
                }
                if (!onVertex) {
                    bool onEdge = false;
                    for (std::size_t edge = 0; edge < 3; ++edge) {
                        const auto relation = segmentIntersection(
                            secondVerts[edge], secondVerts[(edge + 1) % 3],
                            point, point);
                        if (!relation) {
                            return predicateFailure<TriangleIntersection3dKind>(
                                relation.failure->code,
                                relation.failure->message);
                        }
                        if (*relation.value != SegmentIntersectionKind::None) {
                            onEdge = true;
                            if (*relation.value ==
                                SegmentIntersectionKind::CollinearOverlap) {
                                edgeContact = true;
                            } else {
                                pointContact = true;
                            }
                            break;
                        }
                    }
                    if (!onEdge) areaOverlap = true;
                }
            }
        }
        for (std::size_t firstEdge = 0; firstEdge < 3; ++firstEdge) {
            for (std::size_t secondEdge = 0; secondEdge < 3; ++secondEdge) {
                const auto relation = segmentIntersection(
                    firstVerts[firstEdge],
                    firstVerts[(firstEdge + 1) % 3],
                    secondVerts[secondEdge],
                    secondVerts[(secondEdge + 1) % 3]);
                if (!relation) {
                    return predicateFailure<TriangleIntersection3dKind>(
                        relation.failure->code, relation.failure->message);
                }
                if (*relation.value == SegmentIntersectionKind::Proper) {
                    areaOverlap = true;
                } else if (*relation.value ==
                           SegmentIntersectionKind::CollinearOverlap) {
                    edgeContact = true;
                } else if (*relation.value ==
                           SegmentIntersectionKind::EndpointTouch) {
                    pointContact = true;
                }
            }
        }

        PredicateResult<TriangleIntersection3dKind> result;
        if (areaOverlap) {
            result.value = TriangleIntersection3dKind::CoplanarOverlap;
        } else if (edgeContact) {
            result.value = TriangleIntersection3dKind::SharedEdgeOnly;
        } else if (pointContact) {
            result.value = TriangleIntersection3dKind::SharedVertexOnly;
        } else {
            result.value = TriangleIntersection3dKind::None;
        }
        return result;
    }

    PredicateResult<int> planeProjectionAxis(PredicatePoint3 a,
                                             PredicatePoint3 b,
                                             PredicatePoint3 c) const {
        const ExactDyadic abx = difference(b[0], a[0]);
        const ExactDyadic aby = difference(b[1], a[1]);
        const ExactDyadic abz = difference(b[2], a[2]);
        const ExactDyadic acx = difference(c[0], a[0]);
        const ExactDyadic acy = difference(c[1], a[1]);
        const ExactDyadic acz = difference(c[2], a[2]);
        PredicateResult<int> result;
        result.value = dominantAxis(cross(aby, abz, acy, acz),
                                    cross(abz, abx, acz, acx),
                                    cross(abx, aby, acx, acy));
        return result;
    }

    PredicateResult<bool> pointInTriangle3dOnPlane(
        PredicatePoint3 point, PredicatePoint3 a, PredicatePoint3 b,
        PredicatePoint3 c) const {
        const auto axis = planeProjectionAxis(a, b, c);
        if (!axis) {
            return predicateFailure<bool>(
                "predicate.internal_failure",
                "failed to choose a projection axis");
        }
        return pointInTriangle2d(projectPoint(point, *axis.value),
                                 projectPoint(a, *axis.value),
                                 projectPoint(b, *axis.value),
                                 projectPoint(c, *axis.value));
    }

    static void strengthenKind(TriangleIntersection3dKind& kind,
                               TriangleIntersection3dKind candidate) {
        const auto rank = [](TriangleIntersection3dKind value) {
            switch (value) {
                case TriangleIntersection3dKind::None:
                    return 0;
                case TriangleIntersection3dKind::SharedVertexOnly:
                    return 1;
                case TriangleIntersection3dKind::SharedEdgeOnly:
                    return 2;
                case TriangleIntersection3dKind::ProperIntersection:
                    return 3;
                case TriangleIntersection3dKind::CoplanarOverlap:
                    return 4;
            }
            return 0;
        };
        if (rank(candidate) > rank(kind)) kind = candidate;
    }

    // Returns whether closed segment PQ intersects closed triangle ABC. The
    // optional kind accumulates the strongest contact discovered so far.
    PredicateResult<bool> segmentHitsTriangle(
        PredicatePoint3 p, PredicatePoint3 q, PredicatePoint3 a,
        PredicatePoint3 b, PredicatePoint3 c,
        TriangleIntersection3dKind& kind) const {
        const auto oP = orient3d(a, b, c, p);
        const auto oQ = orient3d(a, b, c, q);
        if (!oP || !oQ) {
            return predicateFailure<bool>(
                oP ? oQ.failure->code : oP.failure->code,
                oP ? oQ.failure->message : oP.failure->message);
        }
        if (*oP.value != ExactSign::Zero && *oQ.value != ExactSign::Zero &&
            !opposite(*oP.value, *oQ.value)) {
            PredicateResult<bool> miss;
            miss.value = false;
            return miss;
        }
        if (*oP.value == ExactSign::Zero && *oQ.value == ExactSign::Zero) {
            const auto axis = planeProjectionAxis(a, b, c);
            if (!axis) {
                return predicateFailure<bool>(
                    "predicate.internal_failure",
                    "failed to choose a projection axis");
            }
            const auto relation = segmentIntersection(
                projectPoint(p, *axis.value), projectPoint(q, *axis.value),
                projectPoint(a, *axis.value), projectPoint(b, *axis.value));
            const auto relation2 = segmentIntersection(
                projectPoint(p, *axis.value), projectPoint(q, *axis.value),
                projectPoint(b, *axis.value), projectPoint(c, *axis.value));
            const auto relation3 = segmentIntersection(
                projectPoint(p, *axis.value), projectPoint(q, *axis.value),
                projectPoint(c, *axis.value), projectPoint(a, *axis.value));
            if (!relation || !relation2 || !relation3) {
                const auto& failed = !relation
                    ? relation
                    : (!relation2 ? relation2 : relation3);
                return predicateFailure<bool>(
                    failed.failure->code, failed.failure->message);
            }
            const auto inP = pointInTriangle2d(
                projectPoint(p, *axis.value), projectPoint(a, *axis.value),
                projectPoint(b, *axis.value), projectPoint(c, *axis.value));
            const auto inQ = pointInTriangle2d(
                projectPoint(q, *axis.value), projectPoint(a, *axis.value),
                projectPoint(b, *axis.value), projectPoint(c, *axis.value));
            if (!inP || !inQ) {
                return predicateFailure<bool>(
                    inP ? inQ.failure->code : inP.failure->code,
                    inP ? inQ.failure->message : inP.failure->message);
            }
            const bool hits = *inP.value || *inQ.value ||
                *relation.value != SegmentIntersectionKind::None ||
                *relation2.value != SegmentIntersectionKind::None ||
                *relation3.value != SegmentIntersectionKind::None;
            if (hits) {
                if (*relation.value == SegmentIntersectionKind::CollinearOverlap ||
                    *relation2.value ==
                        SegmentIntersectionKind::CollinearOverlap ||
                    *relation3.value ==
                        SegmentIntersectionKind::CollinearOverlap ||
                    (*inP.value && *inQ.value &&
                     !pointsEqual(p, q))) {
                    strengthenKind(
                        kind, TriangleIntersection3dKind::CoplanarOverlap);
                } else if (*relation.value ==
                               SegmentIntersectionKind::EndpointTouch ||
                           *relation2.value ==
                               SegmentIntersectionKind::EndpointTouch ||
                           *relation3.value ==
                               SegmentIntersectionKind::EndpointTouch ||
                           (*inP.value &&
                            (pointsEqual(p, a) || pointsEqual(p, b) ||
                             pointsEqual(p, c))) ||
                           (*inQ.value &&
                            (pointsEqual(q, a) || pointsEqual(q, b) ||
                             pointsEqual(q, c)))) {
                    strengthenKind(
                        kind, TriangleIntersection3dKind::SharedVertexOnly);
                } else {
                    strengthenKind(
                        kind, TriangleIntersection3dKind::SharedEdgeOnly);
                }
            }
            PredicateResult<bool> result;
            result.value = hits;
            return result;
        }

        // An endpoint on the plane must be tested by containment, not by the
        // infinite PQ prism (which falsely accepts exterior on-plane points).
        if (*oP.value == ExactSign::Zero || *oQ.value == ExactSign::Zero) {
            bool hits = false;
            if (*oP.value == ExactSign::Zero) {
                const auto inP = pointInTriangle3dOnPlane(p, a, b, c);
                if (!inP) {
                    return predicateFailure<bool>(
                        inP.failure->code, inP.failure->message);
                }
                if (*inP.value) {
                    hits = true;
                    strengthenKind(
                        kind, TriangleIntersection3dKind::SharedVertexOnly);
                }
            }
            if (*oQ.value == ExactSign::Zero) {
                const auto inQ = pointInTriangle3dOnPlane(q, a, b, c);
                if (!inQ) {
                    return predicateFailure<bool>(
                        inQ.failure->code, inQ.failure->message);
                }
                if (*inQ.value) {
                    hits = true;
                    strengthenKind(
                        kind, TriangleIntersection3dKind::SharedVertexOnly);
                }
            }
            PredicateResult<bool> result;
            result.value = hits;
            return result;
        }

        const auto o1 = orient3d(p, q, a, b);
        const auto o2 = orient3d(p, q, b, c);
        const auto o3 = orient3d(p, q, c, a);
        if (!o1 || !o2 || !o3) {
            return predicateFailure<bool>(
                "predicate.internal_failure",
                "finite 3D orientation unexpectedly failed");
        }
        const bool hasPositive = *o1.value == ExactSign::Positive ||
            *o2.value == ExactSign::Positive ||
            *o3.value == ExactSign::Positive;
        const bool hasNegative = *o1.value == ExactSign::Negative ||
            *o2.value == ExactSign::Negative ||
            *o3.value == ExactSign::Negative;
        PredicateResult<bool> result;
        result.value = !(hasPositive && hasNegative);
        if (*result.value) {
            const bool boundary =
                *o1.value == ExactSign::Zero ||
                *o2.value == ExactSign::Zero ||
                *o3.value == ExactSign::Zero;
            strengthenKind(kind,
                           boundary
                               ? TriangleIntersection3dKind::SharedEdgeOnly
                               : TriangleIntersection3dKind::ProperIntersection);
        }
        return result;
    }

    PredicateResult<TriangleIntersection3dKind> triangleIntersection3d(
        PredicateTriangle3 first, PredicateTriangle3 second) const override {
        if (!finite(first) || !finite(second)) {
            return predicateFailure<TriangleIntersection3dKind>(
                "predicate.non_finite_input",
                "triangle intersection requires finite IEEE-754 coordinates");
        }
        if (pointsEqual(first[0], first[1]) || pointsEqual(first[1], first[2]) ||
            pointsEqual(first[2], first[0]) || pointsEqual(second[0], second[1]) ||
            pointsEqual(second[1], second[2]) ||
            pointsEqual(second[2], second[0])) {
            return predicateFailure<TriangleIntersection3dKind>(
                "predicate.degenerate_triangle",
                "triangle intersection requires non-degenerate corner inputs");
        }

        ExactSign s2[3];
        ExactSign s1[3];
        bool allZeroSecond = true;
        bool allZeroFirst = true;
        for (std::size_t index = 0; index < 3; ++index) {
            const auto side = orient3d(
                first[0], first[1], first[2], second[index]);
            if (!side) {
                return predicateFailure<TriangleIntersection3dKind>(
                    side.failure->code, side.failure->message);
            }
            s2[index] = *side.value;
            if (s2[index] != ExactSign::Zero) allZeroSecond = false;
        }
        for (std::size_t index = 0; index < 3; ++index) {
            const auto side = orient3d(
                second[0], second[1], second[2], first[index]);
            if (!side) {
                return predicateFailure<TriangleIntersection3dKind>(
                    side.failure->code, side.failure->message);
            }
            s1[index] = *side.value;
            if (s1[index] != ExactSign::Zero) allZeroFirst = false;
        }
        if (allZeroSecond || allZeroFirst) {
            return classifyCoplanar(first, second);
        }

        TriangleIntersection3dKind kind = TriangleIntersection3dKind::None;
        for (std::size_t edge = 0; edge < 3; ++edge) {
            const auto hit = segmentHitsTriangle(
                first[edge], first[(edge + 1) % 3], second[0], second[1],
                second[2], kind);
            if (!hit) {
                return predicateFailure<TriangleIntersection3dKind>(
                    hit.failure->code, hit.failure->message);
            }
        }
        for (std::size_t edge = 0; edge < 3; ++edge) {
            const auto hit = segmentHitsTriangle(
                second[edge], second[(edge + 1) % 3], first[0], first[1],
                first[2], kind);
            if (!hit) {
                return predicateFailure<TriangleIntersection3dKind>(
                    hit.failure->code, hit.failure->message);
            }
        }
        PredicateResult<TriangleIntersection3dKind> result;
        result.value = kind;
        return result;
    }
};

}  // namespace

std::shared_ptr<const GeometricPredicates> makeExactDyadicPredicates() {
    static const std::shared_ptr<const GeometricPredicates> instance =
        std::make_shared<ExactDyadicPredicates>();
    return instance;
}

}  // namespace weft
