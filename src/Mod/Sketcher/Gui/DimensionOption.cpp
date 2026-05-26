// SPDX-License-Identifier: LGPL-2.1-or-later
#include "DimensionOption.h"

#include "CommandConstraints.h"
#include "Utils.h"
#include <Mod/Part/App/Geometry.h>
#include <Mod/Sketcher/App/Constraint.h>
#include <Mod/Sketcher/App/GeometryFacade.h>
#include <Mod/Sketcher/App/SketchObject.h>

#include <Base/Precision.h>
#include <Base/Type.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <initializer_list>
#include <numbers>
#include <optional>
#include <utility>
#include <vector>

namespace SketcherGui {
namespace {

using Sketcher::getRadiusCenterCircleArc;
using Sketcher::isCircleOrArc;
using Sketcher::isLineSegment;

constexpr double kEpsilon = 1e-9;
constexpr double kDimensionOptionAngleOffset = std::numbers::pi / 4.0;

bool isPointReference(const DimensionReference& ref)
{
    return ref.posId != Sketcher::PointPos::none;
}

bool isValidDimensionOption(const DimensionOption& option)
{
    const auto refIsValid = [](const DimensionReference& ref) {
        return ref.geoId != Sketcher::GeoEnum::GeoUndef;
    };

    if (option.semantic == DimensionSemantic::Unknown
        || !std::isfinite(option.previewValue)
        || !std::all_of(option.refs.begin(), option.refs.end(), refIsValid)) {
        return false;
    }

    switch (option.semantic) {
        case DimensionSemantic::ProjectedX:
        case DimensionSemantic::ProjectedY:
        case DimensionSemantic::DirectLength:
        case DimensionSemantic::DirectDistance:
            return !option.refs.empty() && option.refs.size() <= 2;
        case DimensionSemantic::Radius:
        case DimensionSemantic::Diameter:
            return option.refs.size() == 1;
        case DimensionSemantic::Angle:
            return !option.refs.empty() && option.refs.size() <= 3;
        case DimensionSemantic::Unknown:
        default:
            return false;
    }
}

std::optional<Sketcher::ConstraintType> constraintTypeForDimensionSemantic(
    DimensionSemantic semantic)
{
    switch (semantic) {
        case DimensionSemantic::DirectLength:
        case DimensionSemantic::DirectDistance:
            return Sketcher::ConstraintType::Distance;
        case DimensionSemantic::ProjectedX:
            return Sketcher::ConstraintType::DistanceX;
        case DimensionSemantic::ProjectedY:
            return Sketcher::ConstraintType::DistanceY;
        case DimensionSemantic::Radius:
            return Sketcher::ConstraintType::Radius;
        case DimensionSemantic::Diameter:
            return Sketcher::ConstraintType::Diameter;
        case DimensionSemantic::Angle:
            return Sketcher::ConstraintType::Angle;
        case DimensionSemantic::Unknown:
        default:
            return std::nullopt;
    }
}

struct DistanceSegment
{
    bool valid {false};
    Base::Vector2d a;
    Base::Vector2d b;
    double value {0.0};
};

Base::Vector2d toVector2d(const Base::Vector3d& p)
{
    return Base::Vector2d(p.x, p.y);
}

bool shouldSwapDirectLinearEndpoints(const Base::Vector2d& a,
                                    const Base::Vector2d& b)
{
    if (b.x < a.x - kEpsilon) {
        return true;
    }
    if (std::abs(b.x - a.x) <= kEpsilon && b.y < a.y - kEpsilon) {
        return true;
    }
    return false;
}

bool shouldSwapProjectedEndpoints(const Base::Vector2d& a,
                                  const Base::Vector2d& b,
                                  DimensionSemantic semantic)
{
    switch (semantic) {
        case DimensionSemantic::ProjectedX:
            return b.x < a.x - kEpsilon;
        case DimensionSemantic::ProjectedY:
            return b.y < a.y - kEpsilon;
        default:
            return false;
    }
}

Base::Vector2d sketchPoint(const Sketcher::SketchObject& sketch,
                          const DimensionReference& ref)
{
    return toVector2d(sketch.getPoint(ref.geoId, ref.posId));
}

bool isAxisGeoId(int geoId)
{
    return geoId == Sketcher::GeoEnum::HAxis || geoId == Sketcher::GeoEnum::VAxis;
}

bool isLineGeometry(const Part::Geometry* geometry)
{
    return geometry && isLineSegment(*geometry);
}

bool isRoundGeometry(const Part::Geometry* geometry)
{
    return geometry && isCircleOrArc(*geometry);
}

const Part::GeomLineSegment* asLineSegment(const Sketcher::SketchObject& sketch, int geoId)
{
    const Part::Geometry* geometry = sketch.getGeometry(geoId);
    return freecad_cast<const Part::GeomLineSegment*>(geometry);
}

DistanceSegment makeDistanceSegment(const Base::Vector3d& a,
                                    const Base::Vector3d& b,
                                    double value)
{
    return DistanceSegment {true, toVector2d(a), toVector2d(b), value};
}

DistanceSegment pointLineDistance(const Sketcher::SketchObject& sketch,
                                  const DimensionReference& pointRef,
                                  int lineGeoId)
{
    const auto* line = asLineSegment(sketch, lineGeoId);
    if (!line) {
        return {};
    }

    const Base::Vector3d point = sketch.getPoint(pointRef.geoId, pointRef.posId);
    const Base::Vector3d start = line->getStartPoint();
    const Base::Vector3d end = line->getEndPoint();
    Base::Vector3d projected;
    projected.ProjectToLine(point - start, end - start);
    projected += start;
    return makeDistanceSegment(point, projected, (projected - point).Length());
}

DistanceSegment pointRoundDistance(const Sketcher::SketchObject& sketch,
                                   const DimensionReference& pointRef,
                                   int roundGeoId)
{
    const Part::Geometry* geometry = sketch.getGeometry(roundGeoId);
    if (!geometry || !isCircleOrArc(*geometry)) {
        return {};
    }

    const auto [radius, center] = getRadiusCenterCircleArc(geometry);
    const Base::Vector3d point = sketch.getPoint(pointRef.geoId, pointRef.posId);
    Base::Vector3d direction = point - center;
    if (direction.Length() <= Precision::Confusion()) {
        direction = Base::Vector3d(1.0, 0.0, 0.0);
    }
    direction.Normalize();
    const Base::Vector3d curvePoint = center + radius * direction;
    return makeDistanceSegment(point, curvePoint, (curvePoint - point).Length());
}

DistanceSegment roundLineDistance(const Sketcher::SketchObject& sketch,
                                  int roundGeoId,
                                  int lineGeoId)
{
    const Part::Geometry* round = sketch.getGeometry(roundGeoId);
    const auto* line = asLineSegment(sketch, lineGeoId);
    if (!round || !isCircleOrArc(*round) || !line) {
        return {};
    }

    const auto [radius, center] = getRadiusCenterCircleArc(round);
    const Base::Vector3d start = line->getStartPoint();
    const Base::Vector3d end = line->getEndPoint();
    Base::Vector3d projected;
    projected.ProjectToLine(center - start, end - start);
    projected += start;
    Base::Vector3d direction = projected - center;
    if (direction.Length() <= Precision::Confusion()) {
        direction = (end - start).Normalize();
        direction.RotateZ(std::numbers::pi / 2.0);
    }
    direction.Normalize();
    const Base::Vector3d curvePoint = center + radius * direction;
    return makeDistanceSegment(projected, curvePoint, (curvePoint - projected).Length());
}

DistanceSegment roundRoundDistance(const Sketcher::SketchObject& sketch,
                                   int firstGeoId,
                                   int secondGeoId)
{
    const Part::Geometry* first = sketch.getGeometry(firstGeoId);
    const Part::Geometry* second = sketch.getGeometry(secondGeoId);
    if (!first || !second || !isCircleOrArc(*first) || !isCircleOrArc(*second)) {
        return {};
    }

    Base::Vector3d firstPoint;
    Base::Vector3d secondPoint;
    GetCirclesMinimalDistance(first, second, firstPoint, secondPoint);
    return makeDistanceSegment(firstPoint, secondPoint, (secondPoint - firstPoint).Length());
}

DistanceSegment directDistanceSegment(const Sketcher::SketchObject& sketch,
                                      const DimensionReference& first,
                                      const DimensionReference& second)
{
    const bool firstIsPoint = first.posId != Sketcher::PointPos::none;
    const bool secondIsPoint = second.posId != Sketcher::PointPos::none;

    if (firstIsPoint && secondIsPoint) {
        const Base::Vector3d a = sketch.getPoint(first.geoId, first.posId);
        const Base::Vector3d b = sketch.getPoint(second.geoId, second.posId);
        return makeDistanceSegment(a, b, (b - a).Length());
    }

    if (firstIsPoint) {
        const Part::Geometry* geometry = second.geoId == Sketcher::GeoEnum::GeoUndef
            ? nullptr
            : sketch.getGeometry(second.geoId);
        if (isLineGeometry(geometry)) {
            return pointLineDistance(sketch, first, second.geoId);
        }
        if (isRoundGeometry(geometry)) {
            return pointRoundDistance(sketch, first, second.geoId);
        }
        return {};
    }

    if (secondIsPoint) {
        return directDistanceSegment(sketch, second, first);
    }

    const Part::Geometry* firstGeometry = first.geoId == Sketcher::GeoEnum::GeoUndef
        ? nullptr
        : sketch.getGeometry(first.geoId);
    const Part::Geometry* secondGeometry = second.geoId == Sketcher::GeoEnum::GeoUndef
        ? nullptr
        : sketch.getGeometry(second.geoId);
    if (!firstGeometry || !secondGeometry) {
        return {};
    }

    if (isRoundGeometry(firstGeometry) && isLineGeometry(secondGeometry)) {
        return roundLineDistance(sketch, first.geoId, second.geoId);
    }
    if (isLineGeometry(firstGeometry) && isRoundGeometry(secondGeometry)) {
        const auto segment = roundLineDistance(sketch, second.geoId, first.geoId);
        if (!segment.valid) {
            return {};
        }
        return DistanceSegment {true, segment.b, segment.a, segment.value};
    }
    if (isRoundGeometry(firstGeometry) && isRoundGeometry(secondGeometry)) {
        return roundRoundDistance(sketch, first.geoId, second.geoId);
    }

    return {};
}

bool angleRayData(const Sketcher::SketchObject& sketch,
                  int geoId,
                  Sketcher::PointPos pos,
                  Base::Vector2d& vertex,
                  Base::Vector2d& dir)
{
    const DimensionReference startRef {geoId, Sketcher::PointPos::start};
    const DimensionReference endRef {geoId, Sketcher::PointPos::end};
    const Base::Vector2d a = sketchPoint(sketch, startRef);
    const Base::Vector2d b = sketchPoint(sketch, endRef);
    Base::Vector2d tangent = b - a;
    if (tangent.Length() <= kEpsilon) {
        return false;
    }
    tangent.Normalize();
    vertex = pos == Sketcher::PointPos::end ? b : a;
    dir = pos == Sketcher::PointPos::end ? Base::Vector2d(-tangent.x, -tangent.y) : tangent;
    return true;
}

bool isDegenerateAngle(double angleValue)
{
    constexpr double kMinAngle = 1e-3;
    return angleValue < kMinAngle || std::abs(std::numbers::pi - angleValue) < kMinAngle;
}

std::optional<DimensionOption> buildAngleOption(const Sketcher::SketchObject& sketch,
                                                int firstGeoId,
                                                Sketcher::PointPos firstPos,
                                                int secondGeoId,
                                                Sketcher::PointPos secondPos)
{
    Base::Vector2d firstVertex;
    Base::Vector2d firstDir;
    Base::Vector2d secondVertex;
    Base::Vector2d secondDir;
    if (!angleRayData(sketch, firstGeoId, firstPos, firstVertex, firstDir)
        || !angleRayData(sketch, secondGeoId, secondPos, secondVertex, secondDir)) {
        return std::nullopt;
    }

    const double cross = firstDir.x * secondDir.y - firstDir.y * secondDir.x;
    const double dot = firstDir.x * secondDir.x + firstDir.y * secondDir.y;
    double angle = std::atan2(cross, dot);
    if (angle < 0.0) {
        angle += 2.0 * std::numbers::pi;
    }

    if (isDegenerateAngle(angle) || angle >= (std::numbers::pi - Precision::Angular())) {
        return std::nullopt;
    }

    return DimensionOption {DimensionSemantic::Angle,
                            angle,
                            {{firstGeoId, firstPos}, {secondGeoId, secondPos}}};
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

std::optional<DimensionOption> buildDirectDistanceOption(
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

    return DimensionOption {DimensionSemantic::DirectDistance, segment.value, std::move(refs)};
}

std::vector<DimensionOption> buildLinearOptions(
    std::initializer_list<DimensionSemantic> order,
    const Base::Vector2d& a,
    const Base::Vector2d& b,
    const std::vector<DimensionReference>& refs,
    std::optional<DimensionOption> directOption)
{
    std::vector<DimensionOption> result;
    const Base::Vector2d delta = b - a;
    const bool hasProjectedDimension = std::abs(delta.x) > kEpsilon
        && std::abs(delta.y) > kEpsilon;
    for (const auto semantic : order) {
        if (semantic == DimensionSemantic::DirectLength && directOption) {
            directOption->semantic = DimensionSemantic::DirectLength;
            result.push_back(std::move(*directOption));
        }
        else if (hasProjectedDimension && semantic == DimensionSemantic::ProjectedX) {
            result.push_back({semantic, std::abs(delta.x), refs});
        }
        else if (hasProjectedDimension && semantic == DimensionSemantic::ProjectedY) {
            result.push_back({semantic, std::abs(delta.y), refs});
        }
    }
    return result;
}

std::vector<DimensionOption> buildLineOptions(Sketcher::SketchObject* sketch, int geoId)
{
    const auto* line = asLineSegment(*sketch, geoId);
    if (!line) {
        return {};
    }

    const Base::Vector2d a = toVector2d(line->getStartPoint());
    const Base::Vector2d b = toVector2d(line->getEndPoint());
    const std::vector<DimensionReference> refs {{geoId, Sketcher::PointPos::start},
                                                {geoId, Sketcher::PointPos::end}};
    return buildLinearOptions({DimensionSemantic::DirectLength,
                               DimensionSemantic::ProjectedX,
                               DimensionSemantic::ProjectedY},
                              a,
                              b,
                              refs,
                              buildDirectDistanceOption(*sketch, refs));
}

std::vector<DimensionOption> buildRoundOptions(Sketcher::SketchObject* sketch, int geoId)
{
    std::vector<DimensionOption> result;

    const Part::Geometry* geometry = sketch->getGeometry(geoId);
    if (!geometry || !isCircleOrArc(*geometry)) {
        return result;
    }

    const auto [radius, center] = getRadiusCenterCircleArc(geometry);
    if (radius <= kEpsilon) {
        return result;
    }

    const DimensionReference ref {geoId, Sketcher::PointPos::none};
    result.push_back({DimensionSemantic::Radius, radius, {ref}});
    result.push_back({DimensionSemantic::Diameter, radius * 2.0, {ref}});

    if (const auto* arc = freecad_cast<const Part::GeomArcOfCircle*>(geometry)) {
        result.push_back({DimensionSemantic::Angle, arc->getAngle(true), {ref}});
    }

    return result;
}

std::vector<DimensionOption> buildTwoLineOptions(Sketcher::SketchObject* sketch,
                                                       int firstGeoId,
                                                       int secondGeoId)
{
    Sketcher::PointPos firstPos = Sketcher::PointPos::none;
    Sketcher::PointPos secondPos = Sketcher::PointPos::none;
    double angleValue = 0.0;
    if (!calculateAngle(sketch, firstGeoId, secondGeoId, firstPos, secondPos, angleValue)) {
        return {};
    }

    if (angleValue == 0.0) {
        const std::vector<DimensionReference> refs {{secondGeoId, Sketcher::PointPos::start},
                                                    {firstGeoId, Sketcher::PointPos::none}};
        if (auto option = buildDirectDistanceOption(*sketch, refs)) {
            return {std::move(*option)};
        }
    }
    else if (auto option = buildAngleOption(*sketch, firstGeoId, firstPos, secondGeoId, secondPos)) {
        return {std::move(*option)};
    }

    return {};
}

std::vector<DimensionOption> buildTwoPointOptions(Sketcher::SketchObject* sketch,
                                                        const DimensionReference& firstRef,
                                                        const DimensionReference& secondRef)
{
    const Base::Vector2d a = sketchPoint(*sketch, firstRef);
    const Base::Vector2d b = sketchPoint(*sketch, secondRef);
    if ((b - a).Length() < kEpsilon) {
        return {};
    }

    const std::vector<DimensionReference> refs {firstRef, secondRef};
    return buildLinearOptions({DimensionSemantic::DirectLength,
                               DimensionSemantic::ProjectedX,
                               DimensionSemantic::ProjectedY},
                              a,
                              b,
                              refs,
                              buildDirectDistanceOption(*sketch, refs));
}

std::vector<DimensionOption> buildSinglePointOptions(
    Sketcher::SketchObject* sketch,
    const DimensionReference& pointRef)
{
    const DimensionReference origin {Sketcher::GeoEnum::RtPnt, Sketcher::PointPos::start};
    return buildLinearOptions({DimensionSemantic::ProjectedX, DimensionSemantic::ProjectedY},
                              sketchPoint(*sketch, origin),
                              sketchPoint(*sketch, pointRef),
                              {origin, pointRef},
                              std::nullopt);
}

std::vector<DimensionOption> buildPointAxisOptions(
    Sketcher::SketchObject* sketch,
    const DimensionReference& pointRef,
    int axisGeoId)
{
    const DimensionReference axisRef {axisGeoId, Sketcher::PointPos::start};
    const Base::Vector2d axisPoint = sketchPoint(*sketch, axisRef);
    const Base::Vector2d point = sketchPoint(*sketch, pointRef);

    if (axisGeoId == Sketcher::GeoEnum::HAxis && std::abs(point.y - axisPoint.y) > kEpsilon) {
        return {{DimensionSemantic::ProjectedY,
                     std::abs(point.y - axisPoint.y),
                     {axisRef, pointRef}}};
    }
    if (axisGeoId == Sketcher::GeoEnum::VAxis && std::abs(point.x - axisPoint.x) > kEpsilon) {
        return {{DimensionSemantic::ProjectedX,
                     std::abs(point.x - axisPoint.x),
                     {axisRef, pointRef}}};
    }
    return {};
}

bool isSameRef(const DimensionReference& lhs, const DimensionReference& rhs)
{
    return lhs.geoId == rhs.geoId && lhs.posId == rhs.posId;
}

bool isSameRefPairUnordered(const DimensionReference& lhsFirst,
                            const DimensionReference& lhsSecond,
                            const DimensionReference& rhsFirst,
                            const DimensionReference& rhsSecond)
{
    return (isSameRef(lhsFirst, rhsFirst) && isSameRef(lhsSecond, rhsSecond))
        || (isSameRef(lhsFirst, rhsSecond) && isSameRef(lhsSecond, rhsFirst));
}

bool areEquivalentPreviewOptions(const DimensionOption& lhs, const DimensionOption& rhs)
{
    if (lhs.semantic != rhs.semantic || lhs.refs.size() != rhs.refs.size()) {
        return false;
    }

    return lhs.refs.size() == 2
        ? isSameRefPairUnordered(lhs.refs[0], lhs.refs[1], rhs.refs[0], rhs.refs[1])
        : std::equal(lhs.refs.begin(), lhs.refs.end(), rhs.refs.begin(), isSameRef);
}

bool existingConstraintMatchesPreview(const Sketcher::Constraint& existing,
                                      const Sketcher::Constraint& preview)
{
    const bool roundSizeExisting = existing.Type == Sketcher::ConstraintType::Radius
        || existing.Type == Sketcher::ConstraintType::Diameter;
    const bool roundSizePreview = preview.Type == Sketcher::ConstraintType::Radius
        || preview.Type == Sketcher::ConstraintType::Diameter;
    if (roundSizeExisting && roundSizePreview) {
        return existing.First == preview.First
            && existing.FirstPos == Sketcher::PointPos::none
            && preview.FirstPos == Sketcher::PointPos::none;
    }

    if (existing.Type != preview.Type) {
        return false;
    }

    if (preview.Second == Sketcher::GeoEnum::GeoUndef) {
        return existing.Second == Sketcher::GeoEnum::GeoUndef
            && existing.First == preview.First
            && existing.FirstPos == preview.FirstPos;
    }

    return existing.Second != Sketcher::GeoEnum::GeoUndef
        && isSameRefPairUnordered({existing.First, existing.FirstPos},
                                  {existing.Second, existing.SecondPos},
                                  {preview.First, preview.FirstPos},
                                  {preview.Second, preview.SecondPos});
}

void filterPreviewOptions(const Sketcher::SketchObject& sketch,
                          std::vector<DimensionOption>& options)
{
    std::erase_if(options, [](const DimensionOption& option) {
        return !isValidDimensionOption(option);
    });

    std::vector<DimensionOption> unique;
    unique.reserve(options.size());
    for (auto& option : options) {
        const bool duplicate = std::any_of(unique.begin(), unique.end(), [&](const auto& existing) {
            return areEquivalentPreviewOptions(existing, option);
        });
        if (!duplicate) {
            unique.push_back(std::move(option));
        }
    }

    const auto& constraints = sketch.Constraints.getValues();
    std::erase_if(unique, [&](const DimensionOption& option) {
        const auto preview = buildDimensionConstraint(sketch, option);
        return preview && std::any_of(constraints.begin(), constraints.end(), [&](const auto* constraint) {
            return constraint && existingConstraintMatchesPreview(*constraint, *preview);
        });
    });

    options = std::move(unique);
}

} // namespace

std::vector<DimensionOption> buildDimensionOptions(
    Sketcher::SketchObject* sketch,
    const std::vector<DimensionReference>& selectionRefs)
{
    if (!sketch || selectionRefs.empty() || selectionRefs.size() > 2) {
        return {};
    }

    std::vector<DimensionOption> options;
    const auto setDistanceOption = [&](std::vector<DimensionReference> refs) {
        if (auto option = buildDirectDistanceOption(*sketch, std::move(refs))) {
            options = {std::move(*option)};
        }
    };

    if (selectionRefs.size() == 1) {
        const auto& item = selectionRefs.front();
        if (isPointReference(item)) {
            options = buildSinglePointOptions(sketch, item);
        }
        else if (!isAxisGeoId(item.geoId)) {
            const Part::Geometry* geometry = sketch->getGeometry(item.geoId);
            if (isLineGeometry(geometry)) {
                options = buildLineOptions(sketch, item.geoId);
            }
            else if (isRoundGeometry(geometry)) {
                options = buildRoundOptions(sketch, item.geoId);
            }
        }
    }
    else {
        auto first = selectionRefs[0];
        auto second = selectionRefs[1];
        if (isPointReference(first) && isPointReference(second)) {
            options = buildTwoPointOptions(sketch, first, second);
        }
        else {
            if (isPointReference(second)) {
                std::swap(first, second);
            }
            if (isPointReference(first)) {
                if (isAxisGeoId(second.geoId)) {
                    options = buildPointAxisOptions(sketch, first, second.geoId);
                }
                else {
                    const Part::Geometry* geometry = sketch->getGeometry(second.geoId);
                    if (isLineGeometry(geometry) || isRoundGeometry(geometry)) {
                        setDistanceOption({first, second});
                    }
                }
            }
            else if (isLineLikeSelection(*sketch, first) && isLineLikeSelection(*sketch, second)) {
                options = buildTwoLineOptions(sketch, first.geoId, second.geoId);
            }
            else {
                const Part::Geometry* firstGeometry = isAxisGeoId(first.geoId)
                    ? nullptr
                    : sketch->getGeometry(first.geoId);
                const Part::Geometry* secondGeometry = isAxisGeoId(second.geoId)
                    ? nullptr
                    : sketch->getGeometry(second.geoId);
                const bool firstRound = isRoundGeometry(firstGeometry);
                const bool secondRound = isRoundGeometry(secondGeometry);
                const bool firstLine = isLineGeometry(firstGeometry);
                const bool secondLine = isLineGeometry(secondGeometry);
                if ((firstRound && secondRound) || (firstRound && secondLine)) {
                    setDistanceOption({first, second});
                }
                else if (firstLine && secondRound) {
                    setDistanceOption({second, first});
                }
            }
        }
    }

    filterPreviewOptions(*sketch, options);
    return options;
}

namespace {

void canonicalizeLinearConstraint(const Sketcher::SketchObject& sketch,
                                  const DimensionOption& option,
                                  Sketcher::Constraint& constraint)
{
    if (option.refs.size() < 2
        || !isPointReference(option.refs[0])
        || !isPointReference(option.refs[1])) {
        return;
    }

    const auto segment = directDistanceSegment(sketch, option.refs[0], option.refs[1]);
    if (!segment.valid) {
        return;
    }

    bool swap = false;
    if (option.semantic == DimensionSemantic::ProjectedX
        || option.semantic == DimensionSemantic::ProjectedY) {
        swap = shouldSwapProjectedEndpoints(segment.a, segment.b, option.semantic);
    }
    else if (option.semantic == DimensionSemantic::DirectLength
             || option.semantic == DimensionSemantic::DirectDistance) {
        swap = shouldSwapDirectLinearEndpoints(segment.a, segment.b);
    }

    if (!swap) {
        return;
    }

    std::swap(constraint.First, constraint.Second);
    std::swap(constraint.FirstPos, constraint.SecondPos);
}

} // namespace

std::unique_ptr<Sketcher::Constraint> buildDimensionConstraint(
    const Sketcher::SketchObject& sketch,
    const DimensionOption& option)
{
    const auto type = constraintTypeForDimensionSemantic(option.semantic);
    if (!type || !isValidDimensionOption(option)) {
        assert(false && "invalid dimension option");
        return {};
    }

    auto constraint = std::make_unique<Sketcher::Constraint>();
    constraint->Type = *type;
    constraint->setValue(option.previewValue);
    constraint->isDriving = true;
    if (!option.refs.empty()) {
        constraint->First = option.refs[0].geoId;
        constraint->FirstPos = option.refs[0].posId;
    }
    if (option.refs.size() > 1) {
        constraint->Second = option.refs[1].geoId;
        constraint->SecondPos = option.refs[1].posId;
    }
    if (option.refs.size() > 2) {
        constraint->Third = option.refs[2].geoId;
        constraint->ThirdPos = option.refs[2].posId;
    }

    if (option.semantic == DimensionSemantic::Radius
        || option.semantic == DimensionSemantic::Diameter) {
        double angle = 0.0;
        bool isArc = false;
        if (!option.refs.empty()) {
            const Part::Geometry* geometry = sketch.getGeometry(option.refs.front().geoId);
            if (const auto* arc = freecad_cast<const Part::GeomArcOfCircle*>(geometry)) {
                double startAngle = 0.0;
                double endAngle = 0.0;
                arc->getRange(startAngle, endAngle, /*emulateCCW=*/true);
                angle = 0.5 * (startAngle + endAngle);
                isArc = true;
            }
        }
        if (isArc) {
            constraint->LabelPosition = option.semantic == DimensionSemantic::Diameter
                ? angle + 2.0 * kDimensionOptionAngleOffset
                : angle + kDimensionOptionAngleOffset;
        }
        else {
            constraint->LabelPosition = option.semantic == DimensionSemantic::Diameter
                ? angle + kDimensionOptionAngleOffset
                : angle;
        }
    }
    else if (option.semantic == DimensionSemantic::Angle && option.refs.size() == 1) {
        const Part::Geometry* geometry = sketch.getGeometry(option.refs.front().geoId);
        if (const auto* arc = freecad_cast<const Part::GeomArcOfCircle*>(geometry)) {
            double startAngle = 0.0;
            double endAngle = 0.0;
            arc->getRange(startAngle, endAngle, /*emulateCCW=*/true);
            constraint->LabelPosition = 0.5 * (startAngle + endAngle) - kDimensionOptionAngleOffset;
        }
    }
    canonicalizeLinearConstraint(sketch, option, *constraint);
    return constraint;
}

} // namespace SketcherGui
