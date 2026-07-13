#pragma once

// Analytic longitudinal profiles for the V1 coaster generator.
//
// The independent variable is plan-view distance along the route (the same
// distance advanced by SEG_LEN in coaster_track.cpp), not three-dimensional
// rail arc length.  Every authored join carries height, grade and vertical
// curvature.  The convenience profiles below are C3 at their internal joins,
// so they are intended to be sampled directly into finalized control points;
// a second, downstream smoothing pass is neither necessary nor desirable.

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <cstddef>
#include <limits>

namespace v1profile {

constexpr double kPi = 3.14159265358979323846264338327950288;
constexpr double kDegreesToRadians = kPi / 180.0;
constexpr double kRadiansToDegrees = 180.0 / kPi;
constexpr double kTopHatMinFaceDegrees = 60.0;
constexpr double kTopHatMaxFaceDegrees = 65.0;
constexpr std::size_t kMaxSegments = 24;
constexpr std::size_t kMaxChainHills = 4;

inline double clamp01(double value) {
    return std::max(0.0, std::min(1.0, value));
}

inline bool finite(double value) {
    return std::isfinite(value);
}

// Integral of quintic smootherstep.  Its value at one is 1/2, which makes a
// slope blend's height delta exactly length * (startGrade + endGrade) / 2.
inline double smootherstep5(double t) {
    t = clamp01(t);
    return t * t * t * (10.0 + t * (-15.0 + 6.0 * t));
}

inline double smootherstep5Integral(double t) {
    t = clamp01(t);
    const double t2 = t * t;
    const double t4 = t2 * t2;
    return t4 * (2.5 + t * (-3.0 + t));
}

struct Boundary {
    double height = 0.0;       // y
    double grade = 0.0;        // dy / ds
    double curvature = 0.0;    // d2y / ds2
};

struct Sample : Boundary {
    double jerk = 0.0;         // d3y / ds3

    double pitchRadians() const { return std::atan(grade); }
    double pitchDegrees() const { return pitchRadians() * kRadiansToDegrees; }

    // Signed curvature of the graph y(s), useful for force estimates.  The
    // Boundary::curvature member intentionally remains the cheaper y'' value.
    double geometricCurvature() const {
        const double metric = 1.0 + grade * grade;
        return curvature / (metric * std::sqrt(metric));
    }

    double railDistanceScale() const { return std::sqrt(1.0 + grade * grade); }
};

inline bool near(double a, double b, double tolerance) {
    return std::abs(a - b) <= tolerance * (1.0 + std::max(std::abs(a), std::abs(b)));
}

inline bool nearBoundary(const Boundary& a, const Boundary& b,
                         double tolerance = 1.0e-9) {
    return near(a.height, b.height, tolerance) &&
           near(a.grade, b.grade, tolerance) &&
           near(a.curvature, b.curvature, tolerance);
}

inline bool finiteBoundary(const Boundary& boundary) {
    return finite(boundary.height) && finite(boundary.grade) &&
           finite(boundary.curvature);
}

// Polynomial in normalized segment coordinate u = localDistance / length.
// Coefficients are stored in height units; physical derivatives are obtained
// by the appropriate power of 1/length.  Degree six is enough for an
// integrated quintic grade blend, while generic explicit-boundary segments
// use the quintic subset.
struct Segment {
    static constexpr int kDegree = 6;

    double length = 0.0;
    std::array<double, kDegree + 1> coefficient{};
    bool cosineGrade = false;
    double cosineStartHeight = 0.0;
    double cosineStartGrade = 0.0;
    double cosineEndGrade = 0.0;

    static Segment line(double startHeight, double grade, double segmentLength) {
        assert(segmentLength > 0.0);
        Segment result;
        result.length = segmentLength;
        result.coefficient[0] = startHeight;
        result.coefficient[1] = grade * segmentLength;
        return result;
    }

    // Blend grade monotonically using quintic smootherstep, then integrate it
    // analytically.  Height, grade, curvature and jerk match a neighboring
    // line at both ends (C3).  This is the primitive used for crowns, troughs,
    // pull-ups and pull-outs.
    static Segment slopeBlend(double startHeight, double startGrade,
                              double endGrade, double segmentLength) {
        assert(segmentLength > 0.0);
        Segment result;
        result.length = segmentLength;
        const double delta = endGrade - startGrade;
        result.coefficient[0] = startHeight;
        result.coefficient[1] = startGrade * segmentLength;
        result.coefficient[4] = 2.5 * delta * segmentLength;
        result.coefficient[5] = -3.0 * delta * segmentLength;
        result.coefficient[6] = delta * segmentLength;
        return result;
    }

    // Integrate a raised-cosine grade transition.  Unlike smootherstep, its
    // curvature is distributed across the whole section instead of dwelling
    // near a constant endpoint grade; position, grade and curvature still
    // match neighboring sections exactly (C2).
    static Segment slopeCosine(double startHeight, double startGrade,
                               double endGrade, double segmentLength) {
        assert(segmentLength > 0.0);
        Segment result;
        result.length = segmentLength;
        result.cosineGrade = true;
        result.cosineStartHeight = startHeight;
        result.cosineStartGrade = startGrade;
        result.cosineEndGrade = endGrade;
        return result;
    }

    // General quintic Hermite segment.  The supplied states are in physical
    // distance units, so position, grade and curvature are explicit at both
    // boundaries.  Adjacent segments sharing a Boundary are exactly C2.
    static Segment quintic(const Boundary& begin, const Boundary& end,
                           double segmentLength) {
        assert(segmentLength > 0.0);
        Segment result;
        result.length = segmentLength;

        const double l2 = segmentLength * segmentLength;
        const double v0 = begin.grade * segmentLength;
        const double v1 = end.grade * segmentLength;
        const double a0 = begin.curvature * l2;
        const double a1 = end.curvature * l2;

        result.coefficient[0] = begin.height;
        result.coefficient[1] = v0;
        result.coefficient[2] = 0.5 * a0;

        const double positionRemainder =
            end.height - result.coefficient[0] - result.coefficient[1] -
            result.coefficient[2];
        const double gradeRemainder =
            v1 - result.coefficient[1] - 2.0 * result.coefficient[2];
        const double curvatureRemainder = a1 - 2.0 * result.coefficient[2];

        result.coefficient[3] = 10.0 * positionRemainder -
                                4.0 * gradeRemainder +
                                0.5 * curvatureRemainder;
        result.coefficient[4] = -15.0 * positionRemainder +
                                7.0 * gradeRemainder -
                                curvatureRemainder;
        result.coefficient[5] = 6.0 * positionRemainder -
                                3.0 * gradeRemainder +
                                0.5 * curvatureRemainder;
        return result;
    }

    double derivativeNormalized(double u, int order) const {
        assert(order >= 0 && order <= 3);
        u = clamp01(u);
        if (cosineGrade) {
            const double delta = cosineEndGrade - cosineStartGrade;
            if (order == 0)
                return cosineStartHeight + length *
                    (cosineStartGrade * u + delta *
                     (0.5 * u - std::sin(kPi * u) / (2.0 * kPi)));
            if (order == 1)
                return length * (cosineStartGrade +
                    0.5 * delta * (1.0 - std::cos(kPi * u)));
            if (order == 2)
                return length * 0.5 * delta * kPi * std::sin(kPi * u);
            return length * 0.5 * delta * kPi * kPi * std::cos(kPi * u);
        }
        double result = 0.0;
        for (int index = kDegree; index >= order; --index) {
            double factor = 1.0;
            for (int derivative = 0; derivative < order; ++derivative)
                factor *= static_cast<double>(index - derivative);
            result = result * u + coefficient[static_cast<std::size_t>(index)] * factor;
        }
        return result;
    }

    Sample sampleNormalized(double u) const {
        assert(length > 0.0);
        const double inverseLength = 1.0 / length;
        const double inverseLength2 = inverseLength * inverseLength;
        return {{derivativeNormalized(u, 0),
                 derivativeNormalized(u, 1) * inverseLength,
                 derivativeNormalized(u, 2) * inverseLength2},
                derivativeNormalized(u, 3) * inverseLength2 * inverseLength};
    }

    Sample sampleDistance(double localDistance) const {
        return sampleNormalized(localDistance / length);
    }

    Boundary begin() const {
        const Sample sample = sampleNormalized(0.0);
        return {sample.height, sample.grade, sample.curvature};
    }

    Boundary end() const {
        const Sample sample = sampleNormalized(1.0);
        return {sample.height, sample.grade, sample.curvature};
    }
};

// Fixed-capacity storage keeps construction allocation-free and sampling
// predictable.  The largest built-in shape (four hills) uses 17 segments.
class Profile {
public:
    bool append(const Segment& segment) {
        if (segmentCount_ >= segment_.size() || !(segment.length > 0.0) ||
            !finite(segment.length))
            return false;
        if (segment.cosineGrade &&
            (!finite(segment.cosineStartHeight) ||
             !finite(segment.cosineStartGrade) ||
             !finite(segment.cosineEndGrade)))
            return false;
        for (double coefficient : segment.coefficient)
            if (!finite(coefficient)) return false;
        segment_[segmentCount_++] = segment;
        totalLength_ += segment.length;
        return true;
    }

    bool empty() const { return segmentCount_ == 0; }
    std::size_t segmentCount() const { return segmentCount_; }
    double length() const { return totalLength_; }

    const Segment& segment(std::size_t index) const {
        assert(index < segmentCount_);
        return segment_[index];
    }

    double segmentStartDistance(std::size_t index) const {
        assert(index <= segmentCount_);
        double distance = 0.0;
        for (std::size_t i = 0; i < index; ++i) distance += segment_[i].length;
        return distance;
    }

    Sample sampleDistance(double distance) const {
        if (empty()) return {};
        distance = std::max(0.0, std::min(totalLength_, distance));
        for (std::size_t i = 0; i < segmentCount_; ++i) {
            const bool last = i + 1 == segmentCount_;
            if (distance <= segment_[i].length || last)
                return segment_[i].sampleDistance(distance);
            distance -= segment_[i].length;
        }
        return segment_[segmentCount_ - 1].sampleNormalized(1.0);
    }

    Sample sampleNormalized(double t) const {
        return sampleDistance(clamp01(t) * totalLength_);
    }

    double heightDistance(double distance) const {
        return sampleDistance(distance).height;
    }

    double heightNormalized(double t) const {
        return sampleNormalized(t).height;
    }

    double heightDelta(double fromDistance, double toDistance) const {
        return heightDistance(toDistance) - heightDistance(fromDistance);
    }

    Boundary begin() const {
        if (empty()) return {};
        return segment_[0].begin();
    }

    Boundary end() const {
        if (empty()) return {};
        return segment_[segmentCount_ - 1].end();
    }

private:
    std::array<Segment, kMaxSegments> segment_{};
    std::size_t segmentCount_ = 0;
    double totalLength_ = 0.0;
};

// Small stateful composer for custom V1 profiles.  It refuses discontinuous
// appends rather than relying on a later spline pass to hide them.
class ProfileBuilder {
public:
    explicit ProfileBuilder(Boundary initial = {}) : cursor_(initial) {}

    bool good() const { return good_; }
    const Boundary& cursor() const { return cursor_; }
    const Profile& profile() const { return profile_; }

    bool appendLine(double segmentLength) {
        if (!good_ || !(segmentLength > 0.0) || !finite(segmentLength) ||
            !finiteBoundary(cursor_) || std::abs(cursor_.curvature) > 1.0e-10)
            return fail();
        return appendChecked(Segment::line(cursor_.height, cursor_.grade, segmentLength));
    }

    bool appendSlopeBlend(double endGrade, double segmentLength) {
        if (!good_ || !(segmentLength > 0.0) || !finite(segmentLength) ||
            !finite(endGrade) || !finiteBoundary(cursor_) ||
            std::abs(cursor_.curvature) > 1.0e-10)
            return fail();
        return appendChecked(
            Segment::slopeBlend(cursor_.height, cursor_.grade, endGrade, segmentLength));
    }

    bool appendSlopeCosine(double endGrade, double segmentLength) {
        if (!good_ || !(segmentLength > 0.0) || !finite(segmentLength) ||
            !finite(endGrade) || !finiteBoundary(cursor_) ||
            std::abs(cursor_.curvature) > 1.0e-10)
            return fail();
        return appendChecked(
            Segment::slopeCosine(cursor_.height, cursor_.grade, endGrade, segmentLength));
    }

    bool appendQuintic(const Boundary& end, double segmentLength) {
        if (!good_ || !(segmentLength > 0.0) || !finite(segmentLength) ||
            !finiteBoundary(cursor_) || !finiteBoundary(end))
            return fail();
        return appendChecked(Segment::quintic(cursor_, end, segmentLength));
    }

private:
    bool fail() {
        good_ = false;
        return false;
    }

    bool appendChecked(const Segment& segment) {
        if (!(segment.length > 0.0) || !finite(segment.length) ||
            !nearBoundary(cursor_, segment.begin(), 1.0e-8) ||
            !profile_.append(segment))
            return fail();
        cursor_ = segment.end();
        return true;
    }

    Profile profile_{};
    Boundary cursor_{};
    bool good_ = true;
};

struct ContinuityMetrics {
    double maximumHeightJump = 0.0;
    double maximumGradeJump = 0.0;
    double maximumCurvatureJump = 0.0;
    std::size_t worstJoin = 0;

    bool isC2(double tolerance = 1.0e-8) const {
        return maximumHeightJump <= tolerance &&
               maximumGradeJump <= tolerance &&
               maximumCurvatureJump <= tolerance;
    }
};

inline ContinuityMetrics continuityMetrics(const Profile& profile) {
    ContinuityMetrics result;
    double worst = 0.0;
    for (std::size_t i = 1; i < profile.segmentCount(); ++i) {
        const Boundary left = profile.segment(i - 1).end();
        const Boundary right = profile.segment(i).begin();
        const double heightJump = std::abs(left.height - right.height);
        const double gradeJump = std::abs(left.grade - right.grade);
        const double curvatureJump = std::abs(left.curvature - right.curvature);
        result.maximumHeightJump = std::max(result.maximumHeightJump, heightJump);
        result.maximumGradeJump = std::max(result.maximumGradeJump, gradeJump);
        result.maximumCurvatureJump =
            std::max(result.maximumCurvatureJump, curvatureJump);
        const double joinWorst = std::max(heightJump, std::max(gradeJump, curvatureJump));
        if (joinWorst > worst) {
            worst = joinWorst;
            result.worstJoin = i;
        }
    }
    return result;
}

struct ShapeMetrics {
    double minimumHeight = std::numeric_limits<double>::infinity();
    double maximumHeight = -std::numeric_limits<double>::infinity();
    double maximumAbsoluteGrade = 0.0;
    double maximumAbsoluteCurvature = 0.0;
    std::size_t localMaxima = 0;
    std::size_t localMinima = 0;

    double maximumPitchDegrees() const {
        return std::atan(maximumAbsoluteGrade) * kRadiansToDegrees;
    }
};

inline ShapeMetrics shapeMetrics(const Profile& profile,
                                 std::size_t samplesPerSegment = 32) {
    ShapeMetrics result;
    if (profile.empty()) {
        result.minimumHeight = 0.0;
        result.maximumHeight = 0.0;
        return result;
    }
    samplesPerSegment = std::max<std::size_t>(samplesPerSegment, 4);
    int previousGradeSign = 0;
    for (std::size_t segmentIndex = 0;
         segmentIndex < profile.segmentCount(); ++segmentIndex) {
        for (std::size_t sampleIndex = 0; sampleIndex <= samplesPerSegment;
             ++sampleIndex) {
            if (segmentIndex > 0 && sampleIndex == 0) continue;
            const Sample sample = profile.segment(segmentIndex).sampleNormalized(
                static_cast<double>(sampleIndex) /
                static_cast<double>(samplesPerSegment));
            result.minimumHeight = std::min(result.minimumHeight, sample.height);
            result.maximumHeight = std::max(result.maximumHeight, sample.height);
            result.maximumAbsoluteGrade =
                std::max(result.maximumAbsoluteGrade, std::abs(sample.grade));
            result.maximumAbsoluteCurvature =
                std::max(result.maximumAbsoluteCurvature, std::abs(sample.curvature));

            const int gradeSign = sample.grade > 1.0e-9 ? 1 :
                                  sample.grade < -1.0e-9 ? -1 : 0;
            if (gradeSign != 0) {
                if (previousGradeSign > 0 && gradeSign < 0) ++result.localMaxima;
                if (previousGradeSign < 0 && gradeSign > 0) ++result.localMinima;
                previousGradeSign = gradeSign;
            }
        }
    }
    return result;
}

inline void assertC2(const Profile& profile) {
#ifndef NDEBUG
    assert(!profile.empty());
    assert(continuityMetrics(profile).isC2());
#else
    (void)profile;
#endif
}

// The integral of a +grade to -grade crown at u=1/2 is 11/32 of
// grade*length.  This exact constant is useful both for sizing and validation.
constexpr double kSymmetricBlendExtremumRise = 11.0 / 32.0;
constexpr double kSymmetricCosineExtremumRise = 1.0 / kPi;

struct TopHatSpec {
    double startHeight = 0.0;
    double crestHeight = 250.0;
    double endHeight = 0.0;
    double faceDegrees = 67.5;
    double entryTransitionLength = 28.0;
    double crownLength = 34.0;
    double exitTransitionLength = 30.0;
    double horizontalDilation = 1.30;
};

struct TopHatProfile {
    Profile profile{};
    TopHatSpec spec{};
    double apexDistance = 0.0;
    double ascentFaceStartDistance = 0.0;
    double descentFaceEndDistance = 0.0;
    bool valid = false;

    explicit operator bool() const { return valid; }
};

inline bool validateTopHat(const TopHatProfile& topHat,
                           double tolerance = 1.0e-6) {
    if (topHat.profile.empty() ||
        !continuityMetrics(topHat.profile).isC2(tolerance))
        return false;
    const Boundary begin = topHat.profile.begin();
    const Boundary end = topHat.profile.end();
    const Sample apex = topHat.profile.sampleDistance(topHat.apexDistance);
    const ShapeMetrics shape = shapeMetrics(topHat.profile);
    const double dilatedPitch = std::atan(
        std::tan(topHat.spec.faceDegrees * kDegreesToRadians) /
        topHat.spec.horizontalDilation) * kRadiansToDegrees;
    return topHat.spec.faceDegrees >= kTopHatMinFaceDegrees - tolerance &&
           topHat.spec.faceDegrees <= kTopHatMaxFaceDegrees + tolerance &&
           near(begin.height, topHat.spec.startHeight, tolerance) &&
           near(begin.grade, 0.0, tolerance) &&
           near(begin.curvature, 0.0, tolerance) &&
           near(end.height, topHat.spec.endHeight, tolerance) &&
           near(end.grade, 0.0, tolerance) &&
           near(end.curvature, 0.0, tolerance) &&
           near(apex.height, topHat.spec.crestHeight, tolerance) &&
           near(apex.grade, 0.0, tolerance) && apex.curvature < 0.0 &&
           shape.localMaxima == 1 &&
           near(shape.maximumPitchDegrees(), dilatedPitch, 2.0e-5);
}

inline TopHatProfile makeTopHat(const TopHatSpec& spec) {
    TopHatProfile result;
    result.spec = spec;

    const bool finiteInput = finite(spec.startHeight) && finite(spec.crestHeight) &&
                             finite(spec.endHeight) && finite(spec.faceDegrees) &&
                             finite(spec.entryTransitionLength) &&
                             finite(spec.crownLength) &&
                             finite(spec.exitTransitionLength) &&
                             finite(spec.horizontalDilation);
    if (!finiteInput ||
        spec.faceDegrees < kTopHatMinFaceDegrees ||
        spec.faceDegrees > kTopHatMaxFaceDegrees ||
        !(spec.entryTransitionLength > 0.0) || !(spec.crownLength > 0.0) ||
        !(spec.exitTransitionLength > 0.0) || !(spec.horizontalDilation >= 1.0))
        return result;

    const double faceGrade = std::tan(spec.faceDegrees * kDegreesToRadians);
    const double ascentRise = spec.crestHeight - spec.startHeight;
    const double descentDrop = spec.crestHeight - spec.endHeight;
    if (!(ascentRise > 0.0) || !(descentDrop > 0.0)) return result;

    // One continuous camelback: the crown curve begins one third of the way
    // up and remains active until two thirds of the way down.  Raised-cosine
    // grade keeps pitch changing across each whole section instead of
    // dwelling at a constant face angle.  There is still exactly one apex.
    constexpr double crownRiseFraction = 2.0 / 3.0;
    const double entryLength = 2.0 * (1.0 - crownRiseFraction) * ascentRise / faceGrade;
    const double crownLength = crownRiseFraction * ascentRise /
                               (kSymmetricCosineExtremumRise * faceGrade);
    const double exitLength = entryLength -
                              2.0 * (spec.endHeight - spec.startHeight) / faceGrade;
    if (!(entryLength > 0.0) || !(crownLength > 0.0) || !(exitLength > 0.0))
        return result;

    ProfileBuilder builder({spec.startHeight, 0.0, 0.0});
    if (!builder.appendSlopeCosine(faceGrade, entryLength)) return result;
    result.ascentFaceStartDistance = builder.profile().length();
    const double crownStartDistance = builder.profile().length();
    if (!builder.appendSlopeCosine(-faceGrade, crownLength)) return result;
    result.apexDistance = crownStartDistance + 0.5 * crownLength;
    result.descentFaceEndDistance = builder.profile().length();
    if (!builder.appendSlopeCosine(0.0, exitLength)) return result;

    Profile dilated;
    for (std::size_t i = 0; i < builder.profile().segmentCount(); ++i) {
        Segment segment = builder.profile().segment(i);
        segment.length *= spec.horizontalDilation;
        if (segment.cosineGrade) {
            segment.cosineStartGrade /= spec.horizontalDilation;
            segment.cosineEndGrade /= spec.horizontalDilation;
        }
        if (!dilated.append(segment)) return result;
    }
    result.ascentFaceStartDistance *= spec.horizontalDilation;
    result.apexDistance *= spec.horizontalDilation;
    result.descentFaceEndDistance *= spec.horizontalDilation;
    result.profile = dilated;
    result.valid = builder.good() && validateTopHat(result);
    if (result.valid) assertC2(result.profile);
    return result;
}

struct HillChainSpec {
    std::size_t hillCount = 2;             // compact V1 chain: two or three
    double startHeight = 0.0;
    double terrainRise = 0.0;              // smooth baseline lift across a rising corridor
    double firstCrestRise = 28.0;
    double crestHeightDecay = 0.82;        // absolute crests descend each hop
    double troughDropPerHill = 3.0;
    double faceDegrees = 42.0;
    double entryTransitionLength = 10.0;
    double crownLength = 18.0;
    double crownLengthDecay = 0.90;
    double troughLength = 12.0;            // deliberately shorter than a crown
    double exitTransitionLength = 10.0;
    double designSpeed = 60.0;              // m/s; sizes curvature, not propulsion
};

struct HillChainProfile {
    Profile profile{};
    HillChainSpec spec{};
    std::array<double, kMaxChainHills> crestDistance{};
    std::array<double, kMaxChainHills> crestHeight{};
    std::array<double, kMaxChainHills> troughDistance{};
    std::array<double, kMaxChainHills> troughHeight{};
    bool valid = false;

    explicit operator bool() const { return valid; }
};

inline bool validateHillChain(const HillChainProfile& chain,
                              double tolerance = 1.0e-6) {
    if (chain.spec.hillCount < 2 || chain.spec.hillCount > kMaxChainHills ||
        chain.profile.empty() ||
        !continuityMetrics(chain.profile).isC2(tolerance))
        return false;

    const Boundary begin = chain.profile.begin();
    const Boundary end = chain.profile.end();
    const double expectedEnd = chain.spec.startHeight -
                               chain.spec.troughDropPerHill *
                               static_cast<double>(chain.spec.hillCount) +
                               chain.spec.terrainRise;
    if (!near(begin.height, chain.spec.startHeight, tolerance) ||
        !near(begin.grade, 0.0, tolerance) ||
        !near(begin.curvature, 0.0, tolerance) ||
        !near(end.height, expectedEnd, tolerance) ||
        !near(end.grade, 0.0, tolerance) ||
        !near(end.curvature, 0.0, tolerance))
        return false;

    for (std::size_t i = 0; i < chain.spec.hillCount; ++i) {
        const Sample crest = chain.profile.sampleDistance(chain.crestDistance[i]);
        if (!near(crest.height, chain.crestHeight[i], tolerance) ||
            !near(crest.grade, 0.0, tolerance) || !(crest.curvature < 0.0))
            return false;

        const Sample trough = chain.profile.sampleDistance(chain.troughDistance[i]);
        if (!near(trough.height, chain.troughHeight[i], tolerance) ||
            !near(trough.grade, 0.0, tolerance))
            return false;
        if (i + 1 < chain.spec.hillCount && !(trough.curvature > 0.0))
            return false;
    }

    const ShapeMetrics shape = shapeMetrics(chain.profile);
    return shape.localMaxima == chain.spec.hillCount &&
           shape.localMinima == chain.spec.hillCount - 1;
}

inline HillChainProfile makeDescendingHillChain(const HillChainSpec& spec) {
    HillChainProfile result;
    result.spec = spec;

    const bool finiteInput = finite(spec.startHeight) && finite(spec.terrainRise) &&
                             finite(spec.firstCrestRise) &&
                             finite(spec.crestHeightDecay) &&
                             finite(spec.troughDropPerHill) &&
                             finite(spec.faceDegrees) &&
                             finite(spec.entryTransitionLength) &&
                             finite(spec.crownLength) &&
                             finite(spec.crownLengthDecay) &&
                             finite(spec.troughLength) &&
                             finite(spec.exitTransitionLength) &&
                             finite(spec.designSpeed);
    if (!finiteInput || spec.hillCount < 2 || spec.hillCount > kMaxChainHills ||
        !(spec.terrainRise >= 0.0) ||
        !(spec.firstCrestRise > 0.0) ||
        !(spec.crestHeightDecay > 0.0 && spec.crestHeightDecay < 1.0) ||
        !(spec.troughDropPerHill >= 0.0) ||
        !(spec.faceDegrees > 0.0 && spec.faceDegrees < 80.0) ||
        !(spec.entryTransitionLength > 0.0) || !(spec.crownLength > 0.0) ||
        !(spec.crownLengthDecay > 0.0 && spec.crownLengthDecay <= 1.0) ||
        !(spec.troughLength > 0.0 && spec.troughLength <= spec.crownLength) ||
        !(spec.exitTransitionLength > 0.0) || !(spec.designSpeed > 0.0))
        return result;

    const double faceGrade = std::tan(spec.faceDegrees * kDegreesToRadians);
    const std::size_t extremaCount = 2 * spec.hillCount + 1;
    std::array<double, 2 * kMaxChainHills + 1> height{};
    std::array<double, 2 * kMaxChainHills> span{};
    std::array<double, 2 * kMaxChainHills + 1> curvature{};
    height[0] = spec.startHeight;
    for (std::size_t i = 0; i < spec.hillCount; ++i) {
        double crestU = static_cast<double>(2 * i + 1) /
                        static_cast<double>(extremaCount - 1);
        double troughU = static_cast<double>(2 * i + 2) /
                         static_cast<double>(extremaCount - 1);
        double crestBase = spec.terrainRise * crestU * crestU * (3.0 - 2.0 * crestU);
        double troughBase = spec.terrainRise * troughU * troughU * (3.0 - 2.0 * troughU);
        height[2 * i + 1] = spec.startHeight + crestBase + spec.firstCrestRise *
                            std::pow(spec.crestHeightDecay, static_cast<double>(i));
        height[2 * i + 2] = spec.startHeight + troughBase - spec.troughDropPerHill *
                            static_cast<double>(i + 1);
    }
    // A sinusoidal camelback is best understood as a sequence of smooth
    // extrema, not straight faces plus crown fillets.  Quintic Hermite spans
    // approximate those half-waves while allowing curvature to match across
    // unequal, descending hills (something independent cosine pieces cannot).
    for (std::size_t i = 0; i + 1 < extremaCount; ++i) {
        const double delta = std::abs(height[i + 1] - height[i]);
        // A half-cosine of height delta reaches max grade pi*delta/(2L).
        // Use that exact sizing, then reproduce its endpoint curvature below.
        const double gradeLength = 0.5 * kPi * delta / faceGrade;
        // Endpoint curvature of this Hermite half-wave is approximately
        // 6*delta/L^2.  Size it for a -5 g crest at the actual planned ride
        // speed; pitch alone made 30 m hills dangerously short at 240 km/h.
        const double forceLength = spec.designSpeed *
            std::sqrt(delta / (0.92 * 9.81));
        span[i] = std::max(18.0, std::max(gradeLength, forceLength));
    }
    curvature[0] = curvature[extremaCount - 1] = 0.0;
    for (std::size_t i = 1; i + 1 < extremaCount; ++i) {
        const double left = std::abs(height[i] - height[i - 1]) / (span[i - 1] * span[i - 1]);
        const double right = std::abs(height[i + 1] - height[i]) / (span[i] * span[i]);
        const double sign = (i & 1u) ? -1.0 : 1.0;
        // Half-cosine endpoint curvature is pi^2*delta/(2L^2).
        curvature[i] = sign * 6.0 * std::min(left, right);
    }

    ProfileBuilder builder({height[0], 0.0, curvature[0]});
    for (std::size_t i = 0; i + 1 < extremaCount; ++i) {
        if (!builder.appendQuintic({height[i + 1], 0.0, curvature[i + 1]}, span[i]))
            return result;
        if ((i & 1u) == 0u) {
            std::size_t hill = i / 2;
            result.crestDistance[hill] = builder.profile().length();
            result.crestHeight[hill] = height[i + 1];
        } else {
            std::size_t hill = i / 2;
            result.troughDistance[hill] = builder.profile().length();
            result.troughHeight[hill] = height[i + 1];
        }
    }
    result.troughDistance[spec.hillCount - 1] = builder.profile().length();
    result.troughHeight[spec.hillCount - 1] = height[extremaCount - 1];

    result.profile = builder.profile();
    result.valid = builder.good() && validateHillChain(result);
    if (result.valid) assertC2(result.profile);
    return result;
}

}  // namespace v1profile
