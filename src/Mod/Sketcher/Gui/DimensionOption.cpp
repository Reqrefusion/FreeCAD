// SPDX-License-Identifier: LGPL-2.1-or-later

/***************************************************************************
 *   Copyright (c) 2026 Reqrefusion                                        *
 *                                                                         *
 *   This file is part of the FreeCAD CAx development system.              *
 *                                                                         *
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Library General Public           *
 *   License as published by the Free Software Foundation; either          *
 *   version 2 of the License, or (at your option) any later version.      *
 *                                                                         *
 *   This library is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          *
 *   GNU Library General Public License for more details.                  *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this library; see the file COPYING.LIB. If not,    *
 *   write to the Free Software Foundation, Inc., 59 Temple Place,         *
 *   Suite 330, Boston, MA 02111-1307, USA                                 *
 *                                                                         *
 ***************************************************************************/

#include "DimensionOption.h"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <numbers>
#include <optional>
#include <utility>
#include <vector>

#include <Base/Precision.h>
#include <Base/Type.h>
#include <Mod/Part/App/Geometry.h>
#include <Mod/Sketcher/App/Constraint.h>
#include <Mod/Sketcher/App/GeometryFacade.h>
#include <Mod/Sketcher/App/SketchObject.h>

#include "CommandConstraints.h"
#include "DimensionDatumPlacement.h"
#include "Utils.h"

namespace SketcherGui::DimensionOptionDetail
{

using Sketcher::getRadiusCenterCircleArc;
using Sketcher::isCircleOrArc;
using Sketcher::isLineSegment;

constexpr double kDimensionOptionAngleOffset = std::numbers::pi / 4.0;
constexpr double kMinimumUsableAngle = 1e-3;

bool isPointReference(const DimensionReference& ref)
{
    return ref.Pos != Sketcher::PointPos::none;
}

bool isValidDimensionOption(const DimensionOption& option)
{
    const auto refIsValid = [](const DimensionReference& ref) {
        return ref.GeoId != Sketcher::GeoEnum::GeoUndef;
    };

    if (option.constraintType == Sketcher::None || !std::isfinite(option.previewValue)
        || option.previewValue <= Base::Precision::Confusion()
        || !std::all_of(option.refs.begin(), option.refs.end(), refIsValid)) {
        return false;
    }

    switch (option.constraintType) {
        case Sketcher::Distance:
        case Sketcher::DistanceX:
        case Sketcher::DistanceY:
            return !option.refs.empty() && option.refs.size() <= 2;
        case Sketcher::Radius:
        case Sketcher::Diameter:
            return option.refs.size() == 1;
        case Sketcher::Angle:
            return !option.refs.empty() && option.refs.size() <= 3;
        default:
            return false;
    }
}

struct DistanceSegment
{
    Base::Vector2d a;
    Base::Vector2d b;
    double value {0.0};
};

Base::Vector2d toVector2d(const Base::Vector3d& p)
{
    return Base::Vector2d(p.x, p.y);
}

bool shouldSwapDirectLinearEndpoints(const Base::Vector2d& a, const Base::Vector2d& b)
{
    if (b.x < a.x - Base::Precision::Confusion()) {
        return true;
    }
    if (std::abs(b.x - a.x) <= Base::Precision::Confusion()
        && b.y < a.y - Base::Precision::Confusion()) {
        return true;
    }
    return false;
}

bool shouldSwapProjectedEndpoints(
    const Base::Vector2d& a,
    const Base::Vector2d& b,
    Sketcher::ConstraintType constraintType
)
{
    switch (constraintType) {
        case Sketcher::DistanceX:
            return b.x < a.x - Base::Precision::Confusion();
        case Sketcher::DistanceY:
            return b.y < a.y - Base::Precision::Confusion();
        default:
            return false;
    }
}

Base::Vector2d sketchPoint(const Sketcher::SketchObject& sketch, const DimensionReference& ref)
{
    return toVector2d(sketch.getPoint(ref.GeoId, ref.Pos));
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

DistanceSegment makeDistanceSegment(const Base::Vector3d& a, const Base::Vector3d& b, double value)
{
    return DistanceSegment {toVector2d(a), toVector2d(b), value};
}

std::optional<DistanceSegment> directDistanceSegment(
    const Sketcher::SketchObject& sketch,
    const DimensionReference& first,
    const DimensionReference& second
)
{
    const bool firstIsPoint = first.Pos != Sketcher::PointPos::none;
    const bool secondIsPoint = second.Pos != Sketcher::PointPos::none;

    if (!firstIsPoint && secondIsPoint) {
        return directDistanceSegment(sketch, second, first);
    }

    Sketcher::Constraint constraint;
    constraint.Type = Sketcher::Distance;
    constraint.First = first.GeoId;
    constraint.FirstPos = first.Pos;
    constraint.Second = second.GeoId;
    constraint.SecondPos = second.Pos;
    const auto endpoints = resolveDimensionDatumEndpoints(sketch, constraint, Base::Vector2d());
    if (!endpoints) {
        return std::nullopt;
    }

    return makeDistanceSegment(
        endpoints->first,
        endpoints->second,
        (endpoints->second - endpoints->first).Length()
    );
}

bool isDegenerateAngle(double angleValue)
{
    return angleValue < kMinimumUsableAngle
        || std::abs(std::numbers::pi - angleValue) < kMinimumUsableAngle;
}

bool isLineLikeSelection(const Sketcher::SketchObject& sketch, const DimensionReference& ref)
{
    if (isPointReference(ref)) {
        return false;
    }
    if (isAxisGeoId(ref.GeoId)) {
        return true;
    }
    return isLineGeometry(sketch.getGeometry(ref.GeoId));
}

std::optional<DimensionOption> buildDirectDistanceOption(
    const Sketcher::SketchObject& sketch,
    std::vector<DimensionReference> refs
)
{
    if (refs.size() < 2) {
        return std::nullopt;
    }

    const auto segment = directDistanceSegment(sketch, refs[0], refs[1]);
    if (!segment) {
        return std::nullopt;
    }

    return DimensionOption {Sketcher::Distance, segment->value, std::move(refs)};
}

std::vector<DimensionOption> buildLinearOptions(
    std::initializer_list<Sketcher::ConstraintType> order,
    const Base::Vector2d& a,
    const Base::Vector2d& b,
    const std::vector<DimensionReference>& refs,
    std::optional<DimensionOption> directOption
)
{
    std::vector<DimensionOption> result;
    const Base::Vector2d delta = b - a;
    const bool hasProjectedDimension = std::abs(delta.x) > Base::Precision::Confusion()
        && std::abs(delta.y) > Base::Precision::Confusion();
    for (const auto constraintType : order) {
        if (constraintType == Sketcher::Distance && directOption) {
            result.push_back(std::move(*directOption));
        }
        else if (hasProjectedDimension && constraintType == Sketcher::DistanceX) {
            result.push_back({constraintType, std::abs(delta.x), refs});
        }
        else if (hasProjectedDimension && constraintType == Sketcher::DistanceY) {
            result.push_back({constraintType, std::abs(delta.y), refs});
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
    const std::vector<DimensionReference> refs {
        {geoId, Sketcher::PointPos::start},
        {geoId, Sketcher::PointPos::end}
    };
    return buildLinearOptions(
        {Sketcher::Distance, Sketcher::DistanceX, Sketcher::DistanceY},
        a,
        b,
        refs,
        buildDirectDistanceOption(*sketch, refs)
    );
}

std::vector<DimensionOption> buildRoundOptions(Sketcher::SketchObject* sketch, int geoId)
{
    std::vector<DimensionOption> result;

    const Part::Geometry* geometry = sketch->getGeometry(geoId);
    if (!geometry || !isCircleOrArc(*geometry)) {
        return result;
    }

    const auto [radius, center] = getRadiusCenterCircleArc(geometry);
    if (radius <= Base::Precision::Confusion()) {
        return result;
    }

    const DimensionReference ref {geoId, Sketcher::PointPos::none};
    result.push_back({Sketcher::Radius, radius, {ref}});
    result.push_back({Sketcher::Diameter, radius * 2.0, {ref}});

    if (const auto* arc = freecad_cast<const Part::GeomArcOfCircle*>(geometry)) {
        result.push_back({Sketcher::Angle, arc->getAngle(true), {ref}});
    }

    return result;
}

std::vector<DimensionOption> buildTwoLineOptions(
    Sketcher::SketchObject* sketch,
    int firstGeoId,
    int secondGeoId
)
{
    Sketcher::PointPos firstPos = Sketcher::PointPos::none;
    Sketcher::PointPos secondPos = Sketcher::PointPos::none;
    double angleValue = 0.0;
    if (!calculateAngle(sketch, firstGeoId, secondGeoId, firstPos, secondPos, angleValue)) {
        return {};
    }

    if (angleValue == 0.0) {
        const std::vector<DimensionReference> refs {
            {secondGeoId, Sketcher::PointPos::start},
            {firstGeoId, Sketcher::PointPos::none}
        };
        if (auto option = buildDirectDistanceOption(*sketch, refs)) {
            return {std::move(*option)};
        }
    }
    else if (!isDegenerateAngle(angleValue)) {
        return {{Sketcher::Angle, angleValue, {{firstGeoId, firstPos}, {secondGeoId, secondPos}}}};
    }

    return {};
}

std::vector<DimensionOption> buildTwoPointOptions(
    Sketcher::SketchObject* sketch,
    const DimensionReference& firstRef,
    const DimensionReference& secondRef
)
{
    const Base::Vector2d a = sketchPoint(*sketch, firstRef);
    const Base::Vector2d b = sketchPoint(*sketch, secondRef);
    if ((b - a).Length() < Base::Precision::Confusion()) {
        return {};
    }

    const std::vector<DimensionReference> refs {firstRef, secondRef};
    return buildLinearOptions(
        {Sketcher::Distance, Sketcher::DistanceX, Sketcher::DistanceY},
        a,
        b,
        refs,
        buildDirectDistanceOption(*sketch, refs)
    );
}

std::vector<DimensionOption> buildSinglePointOptions(
    Sketcher::SketchObject* sketch,
    const DimensionReference& pointRef
)
{
    const DimensionReference origin {Sketcher::GeoEnum::RtPnt, Sketcher::PointPos::start};
    const Base::Vector2d delta = sketchPoint(*sketch, pointRef) - sketchPoint(*sketch, origin);
    const std::vector<DimensionReference> refs {origin, pointRef};

    std::vector<DimensionOption> options;
    if (std::abs(delta.x) > Base::Precision::Confusion()) {
        options.push_back({Sketcher::DistanceX, std::abs(delta.x), refs});
    }
    if (std::abs(delta.y) > Base::Precision::Confusion()) {
        options.push_back({Sketcher::DistanceY, std::abs(delta.y), refs});
    }
    return options;
}

std::vector<DimensionOption> buildPointAxisOptions(
    Sketcher::SketchObject* sketch,
    const DimensionReference& pointRef,
    int axisGeoId
)
{
    const DimensionReference axisRef {axisGeoId, Sketcher::PointPos::start};
    const Base::Vector2d axisPoint = sketchPoint(*sketch, axisRef);
    const Base::Vector2d point = sketchPoint(*sketch, pointRef);

    if (axisGeoId == Sketcher::GeoEnum::HAxis
        && std::abs(point.y - axisPoint.y) > Base::Precision::Confusion()) {
        return {{Sketcher::DistanceY, std::abs(point.y - axisPoint.y), {axisRef, pointRef}}};
    }
    if (axisGeoId == Sketcher::GeoEnum::VAxis
        && std::abs(point.x - axisPoint.x) > Base::Precision::Confusion()) {
        return {{Sketcher::DistanceX, std::abs(point.x - axisPoint.x), {axisRef, pointRef}}};
    }
    return {};
}

bool isSameRef(const DimensionReference& lhs, const DimensionReference& rhs)
{
    return lhs == rhs;
}

bool isSameRefPairUnordered(
    const DimensionReference& lhsFirst,
    const DimensionReference& lhsSecond,
    const DimensionReference& rhsFirst,
    const DimensionReference& rhsSecond
)
{
    return (isSameRef(lhsFirst, rhsFirst) && isSameRef(lhsSecond, rhsSecond))
        || (isSameRef(lhsFirst, rhsSecond) && isSameRef(lhsSecond, rhsFirst));
}

bool areEquivalentPreviewOptions(const DimensionOption& lhs, const DimensionOption& rhs)
{
    if (lhs.constraintType != rhs.constraintType || lhs.refs.size() != rhs.refs.size()) {
        return false;
    }

    return lhs.refs.size() == 2
        ? isSameRefPairUnordered(lhs.refs[0], lhs.refs[1], rhs.refs[0], rhs.refs[1])
        : std::equal(lhs.refs.begin(), lhs.refs.end(), rhs.refs.begin(), isSameRef);
}

bool singleElementConstraintMatchesPreviewPair(
    const Sketcher::Constraint& existing,
    const Sketcher::Constraint& preview
)
{
    if (existing.Type != preview.Type || existing.Second != Sketcher::GeoEnum::GeoUndef
        || preview.Second == Sketcher::GeoEnum::GeoUndef) {
        return false;
    }

    if (existing.Type == Sketcher::Distance && existing.FirstPos == Sketcher::PointPos::none) {
        const bool sameGeometry = preview.First == existing.First && preview.Second == existing.First;
        const bool usesBothEndpoints = (preview.FirstPos == Sketcher::PointPos::start
                                        && preview.SecondPos == Sketcher::PointPos::end)
            || (preview.FirstPos == Sketcher::PointPos::end
                && preview.SecondPos == Sketcher::PointPos::start);
        return sameGeometry && usesBothEndpoints;
    }

    if (existing.FirstPos == Sketcher::PointPos::none
        || (existing.Type != Sketcher::Distance && existing.Type != Sketcher::DistanceX
            && existing.Type != Sketcher::DistanceY)) {
        return false;
    }

    return isSameRefPairUnordered(
        {existing.First, existing.FirstPos},
        {Sketcher::GeoEnum::RtPnt, Sketcher::PointPos::start},
        {preview.First, preview.FirstPos},
        {preview.Second, preview.SecondPos}
    );
}

bool existingConstraintMatchesPreview(
    const Sketcher::Constraint& existing,
    const Sketcher::Constraint& preview
)
{
    const bool roundSizeExisting = existing.Type == Sketcher::ConstraintType::Radius
        || existing.Type == Sketcher::ConstraintType::Diameter;
    const bool roundSizePreview = preview.Type == Sketcher::ConstraintType::Radius
        || preview.Type == Sketcher::ConstraintType::Diameter;
    if (roundSizeExisting && roundSizePreview) {
        return existing.First == preview.First && existing.FirstPos == Sketcher::PointPos::none
            && preview.FirstPos == Sketcher::PointPos::none;
    }

    if (existing.Type != preview.Type) {
        return false;
    }

    if (singleElementConstraintMatchesPreviewPair(existing, preview)) {
        return true;
    }

    if (preview.Second == Sketcher::GeoEnum::GeoUndef) {
        return existing.Second == Sketcher::GeoEnum::GeoUndef && existing.First == preview.First
            && existing.FirstPos == preview.FirstPos;
    }

    return existing.Second != Sketcher::GeoEnum::GeoUndef
        && isSameRefPairUnordered(
               {existing.First, existing.FirstPos},
               {existing.Second, existing.SecondPos},
               {preview.First, preview.FirstPos},
               {preview.Second, preview.SecondPos}
        );
}

void filterPreviewOptions(const Sketcher::SketchObject& sketch, std::vector<DimensionOption>& options)
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
        return preview
            && std::any_of(constraints.begin(), constraints.end(), [&](const auto* constraint) {
                   return constraint && existingConstraintMatchesPreview(*constraint, *preview);
               });
    });

    options = std::move(unique);
}

}  // namespace SketcherGui::DimensionOptionDetail

namespace SketcherGui
{

using DimensionOptionDetail::buildDirectDistanceOption;
using DimensionOptionDetail::buildLineOptions;
using DimensionOptionDetail::buildPointAxisOptions;
using DimensionOptionDetail::buildRoundOptions;
using DimensionOptionDetail::buildSinglePointOptions;
using DimensionOptionDetail::buildTwoLineOptions;
using DimensionOptionDetail::buildTwoPointOptions;
using DimensionOptionDetail::filterPreviewOptions;
using DimensionOptionDetail::isAxisGeoId;
using DimensionOptionDetail::isLineGeometry;
using DimensionOptionDetail::isLineLikeSelection;
using DimensionOptionDetail::isPointReference;
using DimensionOptionDetail::isRoundGeometry;

std::vector<DimensionOption> buildDimensionOptions(
    Sketcher::SketchObject* sketch,
    const std::vector<DimensionReference>& selectionRefs
)
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
        else if (!isAxisGeoId(item.GeoId)) {
            const Part::Geometry* geometry = sketch->getGeometry(item.GeoId);
            if (isLineGeometry(geometry)) {
                options = buildLineOptions(sketch, item.GeoId);
            }
            else if (isRoundGeometry(geometry)) {
                options = buildRoundOptions(sketch, item.GeoId);
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
                if (isAxisGeoId(second.GeoId)) {
                    options = buildPointAxisOptions(sketch, first, second.GeoId);
                }
                else {
                    const Part::Geometry* geometry = sketch->getGeometry(second.GeoId);
                    if (isLineGeometry(geometry) || isRoundGeometry(geometry)) {
                        setDistanceOption({first, second});
                    }
                }
            }
            else if (isLineLikeSelection(*sketch, first) && isLineLikeSelection(*sketch, second)) {
                options = buildTwoLineOptions(sketch, first.GeoId, second.GeoId);
            }
            else {
                const Part::Geometry* firstGeometry = isAxisGeoId(first.GeoId)
                    ? nullptr
                    : sketch->getGeometry(first.GeoId);
                const Part::Geometry* secondGeometry = isAxisGeoId(second.GeoId)
                    ? nullptr
                    : sketch->getGeometry(second.GeoId);
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

namespace DimensionOptionDetail
{

void canonicalizeLinearConstraint(
    const Sketcher::SketchObject& sketch,
    const DimensionOption& option,
    Sketcher::Constraint& constraint
)
{
    if (option.refs.size() < 2 || !isPointReference(option.refs[0])
        || !isPointReference(option.refs[1])) {
        return;
    }

    const auto segment = directDistanceSegment(sketch, option.refs[0], option.refs[1]);
    if (!segment) {
        return;
    }

    bool swap = false;
    if (option.constraintType == Sketcher::DistanceX || option.constraintType == Sketcher::DistanceY) {
        swap = shouldSwapProjectedEndpoints(segment->a, segment->b, option.constraintType);
    }
    else if (option.constraintType == Sketcher::Distance) {
        swap = shouldSwapDirectLinearEndpoints(segment->a, segment->b);
    }

    if (!swap) {
        return;
    }

    std::swap(constraint.First, constraint.Second);
    std::swap(constraint.FirstPos, constraint.SecondPos);
}

}  // namespace DimensionOptionDetail

std::unique_ptr<Sketcher::Constraint> buildDimensionConstraint(
    const Sketcher::SketchObject& sketch,
    const DimensionOption& option
)
{
    if (!DimensionOptionDetail::isValidDimensionOption(option)) {
        return {};
    }

    auto constraint = std::make_unique<Sketcher::Constraint>();
    constraint->Type = option.constraintType;
    constraint->setValue(option.previewValue);
    constraint->isDriving = true;
    if (!option.refs.empty()) {
        constraint->First = option.refs[0].GeoId;
        constraint->FirstPos = option.refs[0].Pos;
    }
    if (option.refs.size() > 1) {
        constraint->Second = option.refs[1].GeoId;
        constraint->SecondPos = option.refs[1].Pos;
    }
    if (option.refs.size() > 2) {
        constraint->Third = option.refs[2].GeoId;
        constraint->ThirdPos = option.refs[2].Pos;
    }

    if (option.constraintType == Sketcher::Radius || option.constraintType == Sketcher::Diameter) {
        double angle = 0.0;
        bool isArc = false;
        if (!option.refs.empty()) {
            const Part::Geometry* geometry = sketch.getGeometry(option.refs.front().GeoId);
            if (const auto* arc = freecad_cast<const Part::GeomArcOfCircle*>(geometry)) {
                double startAngle = 0.0;
                double endAngle = 0.0;
                arc->getRange(startAngle, endAngle, /*emulateCCW=*/true);
                angle = 0.5 * (startAngle + endAngle);
                isArc = true;
            }
        }
        if (isArc) {
            constraint->LabelPosition = option.constraintType == Sketcher::Diameter
                ? angle + 2.0 * kDimensionOptionAngleOffset
                : angle + kDimensionOptionAngleOffset;
        }
        else {
            constraint->LabelPosition = option.constraintType == Sketcher::Diameter
                ? angle + kDimensionOptionAngleOffset
                : angle;
        }
    }
    else if (option.constraintType == Sketcher::Angle && option.refs.size() == 1) {
        const Part::Geometry* geometry = sketch.getGeometry(option.refs.front().GeoId);
        if (const auto* arc = freecad_cast<const Part::GeomArcOfCircle*>(geometry)) {
            double startAngle = 0.0;
            double endAngle = 0.0;
            arc->getRange(startAngle, endAngle, /*emulateCCW=*/true);
            constraint->LabelPosition = 0.5 * (startAngle + endAngle) - kDimensionOptionAngleOffset;
        }
    }
    DimensionOptionDetail::canonicalizeLinearConstraint(sketch, option, *constraint);
    return constraint;
}

}  // namespace SketcherGui
