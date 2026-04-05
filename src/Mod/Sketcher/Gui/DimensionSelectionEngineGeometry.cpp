// SPDX-License-Identifier: LGPL-2.1-or-later
#include "DimensionSelectionEngineDetails.h"

#include "DimensionGeometry.h"

#include <Mod/Part/App/Geometry.h>
#include <Mod/Sketcher/App/SketchObject.h>

#include <utility>

#include <Base/Precision.h>

#include <algorithm>
#include <cmath>

namespace SketcherGui::DimensionSelectionEngineDetail {

using namespace DimensionGeometry;

constexpr double kMinLinearOffset = 6.0;
constexpr double kMaxLinearOffset = 14.0;
constexpr double kMinAngularRadius = 8.0;
constexpr double kMaxAngularRadius = 18.0;
constexpr double kPi = 3.14159265358979323846;

double boundedOffset(double sourceLength, double minOffset, double maxOffset)
{
    return std::clamp(sourceLength * 0.25, minOffset, maxOffset);
}

bool isAxisAlignedLengthDuplicate(const Base::Vector2d& a, const Base::Vector2d& b)
{
    const double dx = std::abs(b.x - a.x);
    const double dy = std::abs(b.y - a.y);
    return dx < kEpsilon || dy < kEpsilon;
}

bool intersectRays(const Base::Vector2d& p1,
                   const Base::Vector2d& dir1,
                   const Base::Vector2d& p2,
                   const Base::Vector2d& dir2,
                   Base::Vector2d& intersection)
{
    const double det = dir1.x * dir2.y - dir1.y * dir2.x;
    if (std::abs(det) < kEpsilon) {
        intersection = midpoint(p1, p2);
        return false;
    }

    const double c1 = dir1.y * p1.x - dir1.x * p1.y;
    const double c2 = dir2.y * p2.x - dir2.x * p2.y;
    intersection = Base::Vector2d((dir1.x * c2 - dir2.x * c1) / det,
                                  (dir1.y * c2 - dir2.y * c1) / det);
    return true;
}

bool angleRayData(const Sketcher::SketchObject& sketch,
                  int geoId,
                  Sketcher::PointPos pos,
                  Base::Vector2d& vertex,
                  Base::Vector2d& dir,
                  double& extent)
{
    const DimensionReference startRef {geoId, Sketcher::PointPos::start};
    const DimensionReference endRef {geoId, Sketcher::PointPos::end};
    const Base::Vector2d a = sketchPoint(sketch, startRef);
    const Base::Vector2d b = sketchPoint(sketch, endRef);
    const Base::Vector2d tangent = tangentFromLine(a, b);
    vertex = pos == Sketcher::PointPos::end ? b : a;
    dir = pos == Sketcher::PointPos::end ? Base::Vector2d(-tangent.x, -tangent.y) : tangent;
    extent = std::max(length(Base::Vector2d(b.x - a.x, b.y - a.y)), 1.0);
    return true;
}

void appendAngleCandidate(std::vector<DimensionCandidate>& result,
                          const AngleRayChoice& choice)
{
    if (!choice.valid || choice.angleValue < 1e-3) {
        return;
    }

    DimensionCandidate candidate;
    candidate.semantic = DimensionSemantic::Angle;
    candidate.refs = {{choice.firstGeoId, choice.firstPos}, {choice.secondGeoId, choice.secondPos}};
    candidate.labelPos = makeStableAngularLabelPos(choice.vertex,
                                                   choice.dir1,
                                                   choice.dir2,
                                                   choice.firstLength,
                                                   choice.secondLength);
    candidate.previewValue = choice.angleValue;
    result.push_back(std::move(candidate));
}


Base::Vector2d makeStableLinearLabelPos(const Base::Vector2d& a,
                                        const Base::Vector2d& b,
                                        DimensionSemantic semantic)
{
    const auto frame = makeLinearPreviewFrame(a, b, semantic);
    if (!frame.valid) {
        return midpoint(a, b);
    }

    const double segmentLength = length(Base::Vector2d(frame.b.x - frame.a.x, frame.b.y - frame.a.y));
    const double offset = boundedOffset(segmentLength, kMinLinearOffset, kMaxLinearOffset);

    return preferredExteriorLinearLabelPosition(frame, semantic, offset, 0.0);
}

bool shouldCreateProjectedX(const Base::Vector2d& a, const Base::Vector2d& b)
{
    return std::abs(b.x - a.x) > kEpsilon && !isAxisAlignedLengthDuplicate(a, b);
}

bool shouldCreateProjectedY(const Base::Vector2d& a, const Base::Vector2d& b)
{
    return std::abs(b.y - a.y) > kEpsilon && !isAxisAlignedLengthDuplicate(a, b);
}

double computeLinearPreviewValue(const Base::Vector2d& a,
                                 const Base::Vector2d& b,
                                 DimensionSemantic semantic)
{
    const Base::Vector2d delta(b.x - a.x, b.y - a.y);
    switch (semantic) {
        case DimensionSemantic::ProjectedX:
            return std::abs(delta.x);
        case DimensionSemantic::ProjectedY:
            return std::abs(delta.y);
        case DimensionSemantic::DirectLength:
        case DimensionSemantic::DirectDistance:
        default:
            return length(delta);
    }
}

double computeRoundPreviewValue(double radius, DimensionSemantic semantic)
{
    return semantic == DimensionSemantic::Diameter ? radius * 2.0 : radius;
}

bool isDegenerateAngle(double angleValue)
{
    constexpr double kMinAngle = 1e-3;
    return angleValue < kMinAngle || std::abs(kPi - angleValue) < kMinAngle;
}

Base::Vector2d makeStableAngularLabelPos(const Base::Vector2d& vertex,
                                         const Base::Vector2d& dir1,
                                         const Base::Vector2d& dir2,
                                         double firstLength,
                                         double secondLength)
{
    Base::Vector2d bisector = normalized(Base::Vector2d(dir1.x + dir2.x, dir1.y + dir2.y), dir1);
    if (bisector.y < -kEpsilon || (std::abs(bisector.y) < kEpsilon && bisector.x < 0.0)) {
        bisector.x = -bisector.x;
        bisector.y = -bisector.y;
    }

    const double radius = std::clamp(std::min(firstLength, secondLength) * 0.35,
                                     kMinAngularRadius,
                                     kMaxAngularRadius);
    return Base::Vector2d(vertex.x + bisector.x * radius, vertex.y + bisector.y * radius);
}

bool buildAngleRayChoice(const Sketcher::SketchObject& sketch,
                         int firstGeoId,
                         Sketcher::PointPos firstPos,
                         int secondGeoId,
                         Sketcher::PointPos secondPos,
                         AngleRayChoice& choice)
{
    Base::Vector2d firstVertex;
    Base::Vector2d firstDir;
    Base::Vector2d secondVertex;
    Base::Vector2d secondDir;
    double firstLength = 0.0;
    double secondLength = 0.0;
    if (!angleRayData(sketch, firstGeoId, firstPos, firstVertex, firstDir, firstLength)
        || !angleRayData(sketch, secondGeoId, secondPos, secondVertex, secondDir, secondLength)) {
        return false;
    }

    Base::Vector2d intersection;
    intersectRays(firstVertex, firstDir, secondVertex, secondDir, intersection);

    const double cross = firstDir.x * secondDir.y - firstDir.y * secondDir.x;
    const double dot = firstDir.x * secondDir.x + firstDir.y * secondDir.y;
    double angle = std::atan2(cross, dot);
    if (angle < 0.0) {
        angle += 2.0 * kPi;
    }

    if (isDegenerateAngle(angle) || angle >= (kPi - Precision::Angular())) {
        return false;
    }

    choice.valid = true;
    choice.firstGeoId = firstGeoId;
    choice.secondGeoId = secondGeoId;
    choice.firstPos = firstPos;
    choice.secondPos = secondPos;
    choice.vertex = intersection;
    choice.dir1 = firstDir;
    choice.dir2 = secondDir;
    choice.firstLength = firstLength;
    choice.secondLength = secondLength;
    choice.angleValue = angle;
    return true;
}

bool isLineLikeSelection(const Sketcher::SketchObject& sketch, const DimensionReference& ref)
{
    if (isPointReference(ref)) {
        return false;
    }
    if (isAxisGeoId(ref.geoId)) {
        return true;
    }
    return isLineGeometry(sketch.getGeometry(ref.geoId));
}

void appendResolvedAngleCandidates(std::vector<DimensionCandidate>& result,
                                   const Sketcher::SketchObject& sketch,
                                   const TwoLineDimensionResolution& resolved)
{
    if (resolved.semantic != DimensionSemantic::Angle) {
        return;
    }

    AngleRayChoice primary;
    if (!buildAngleRayChoice(sketch,
                             resolved.firstGeoId,
                             resolved.firstPos,
                             resolved.secondGeoId,
                             resolved.secondPos,
                             primary)) {
        return;
    }

    appendAngleCandidate(result, primary);
}

DimensionCandidate makeLinearCandidate(DimensionSemantic semantic,
                                       std::vector<DimensionReference> refs,
                                       const Base::Vector2d& a,
                                       const Base::Vector2d& b,
                                       double previewValue)
{
    DimensionCandidate candidate;
    candidate.semantic = semantic;
    candidate.refs = std::move(refs);
    candidate.labelPos = makeStableLinearLabelPos(a, b, semantic);
    candidate.previewValue = previewValue;
    return candidate;
}

std::optional<DimensionCandidate> buildDirectDistanceCandidate(
    const Sketcher::SketchObject& sketch,
    std::vector<DimensionReference> refs)
{
    if (refs.size() < 2) {
        return std::nullopt;
    }

    const auto segment = directDistanceSegment(sketch, refs[0], refs[1]);
    if (!segment.valid) {
        return std::nullopt;
    }

    DimensionCandidate candidate;
    candidate.semantic = DimensionSemantic::DirectDistance;
    candidate.refs = std::move(refs);
    candidate.labelPos = makeStableLinearLabelPos(segment.a, segment.b, DimensionSemantic::DirectDistance);
    candidate.previewValue = segment.value;
    return candidate;
}

std::optional<DimensionCandidate> buildPrimaryDistanceCandidate(
    Sketcher::SketchObject* sketch,
    BasicDimensionSelectionPattern pattern,
    const DimensionReference& first,
    const DimensionReference* second)
{
    const auto resolved = resolvePrimaryDistanceRefs(sketch, pattern, first, second);
    if (!resolved) {
        return std::nullopt;
    }

    return buildDirectDistanceCandidate(*sketch, {(*resolved)[0], (*resolved)[1]});
}

void appendLinearCandidates(std::vector<DimensionCandidate>& result,
                            const std::vector<DimensionSemantic>& order,
                            const Base::Vector2d& a,
                            const Base::Vector2d& b,
                            const std::vector<DimensionReference>& refs,
                            std::optional<DimensionCandidate> directCandidate)
{
    for (const auto semantic : order) {
        switch (semantic) {
            case DimensionSemantic::DirectLength: {
                if (directCandidate) {
                    directCandidate->semantic = DimensionSemantic::DirectLength;
                    result.push_back(std::move(*directCandidate));
                }
                break;
            }
            case DimensionSemantic::ProjectedX:
                if (shouldCreateProjectedX(a, b)) {
                    result.push_back(makeLinearCandidate(DimensionSemantic::ProjectedX,
                                                         refs,
                                                         a,
                                                         b,
                                                         computeLinearPreviewValue(a, b, DimensionSemantic::ProjectedX)));
                }
                break;
            case DimensionSemantic::ProjectedY:
                if (shouldCreateProjectedY(a, b)) {
                    result.push_back(makeLinearCandidate(DimensionSemantic::ProjectedY,
                                                         refs,
                                                         a,
                                                         b,
                                                         computeLinearPreviewValue(a, b, DimensionSemantic::ProjectedY)));
                }
                break;
            case DimensionSemantic::Unknown:
            case DimensionSemantic::Radius:
            case DimensionSemantic::Diameter:
            case DimensionSemantic::Angle:
            case DimensionSemantic::ArcLength:
                break;
        }
    }
}

Base::Vector2d makeStableArcLengthLabelPos(const Part::GeomArcOfCircle& arc)
{
    const auto placement = describeArcLengthPlacement(arc);
    if (!placement.valid) {
        const auto center = arc.getCenter();
        return Base::Vector2d(center.x, center.y);
    }

    const double offset = std::clamp(placement.radius * 0.15, 2.5, 7.0);
    return Base::Vector2d(placement.center.x + placement.visualDir.x * (placement.radius + offset),
                          placement.center.y + placement.visualDir.y * (placement.radius + offset));
}

} // namespace SketcherGui::DimensionSelectionEngineDetail
