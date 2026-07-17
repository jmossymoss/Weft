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
        const auto finite3 = [](PredicatePoint3 point) {
            return std::isfinite(point[0]) && std::isfinite(point[1]) &&
                std::isfinite(point[2]);
        };
        if (!finite3(a) || !finite3(b) || !finite3(c) || !finite3(d)) {
            return predicateFailure<ExactSign>(
                "predicate.non_finite_input",
                "orientation requires finite IEEE-754 coordinates");
        }
        const ExactDyadic adx = difference(a[0], d[0]);
        const ExactDyadic ady = difference(a[1], d[1]);
        const ExactDyadic adz = difference(a[2], d[2]);
        const ExactDyadic bdx = difference(b[0], d[0]);
        const ExactDyadic bdy = difference(b[1], d[1]);
        const ExactDyadic bdz = difference(b[2], d[2]);
        const ExactDyadic cdx = difference(c[0], d[0]);
        const ExactDyadic cdy = difference(c[1], d[1]);
        const ExactDyadic cdz = difference(c[2], d[2]);
        const ExactDyadic determinant = add(
            add(multiply(adx, cross(bdy, bdz, cdy, cdz)),
                multiply(ady, cross(bdz, bdx, cdz, cdx))),
            multiply(adz, cross(bdx, bdy, cdx, cdy)));
        return PredicateResult<ExactSign>{signOf(determinant),
                                          std::nullopt};
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
};

}  // namespace

std::shared_ptr<const GeometricPredicates> makeExactDyadicPredicates() {
    static const std::shared_ptr<const GeometricPredicates> instance =
        std::make_shared<ExactDyadicPredicates>();
    return instance;
}

}  // namespace weft
