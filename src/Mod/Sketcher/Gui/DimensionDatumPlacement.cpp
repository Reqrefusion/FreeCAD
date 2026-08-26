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

#include "DimensionDatumPlacement.h"

#include <cmath>
#include <limits>
#include <memory>
#include <numbers>
#include <optional>
#include <utility>

#include <Base/Precision.h>
#include <Base/Vector3D.h>
#include <Mod/Part/App/Geometry.h>
#include <Mod/Sketcher/App/Constraint.h>
#include <Mod/Sketcher/App/SketchObject.h>

#include "DimensionOption.h"
#include "Utils.h"
#include "ViewProviderSketchGeometryExtension.h"

namespace SketcherGui::DimensionDatumPlacementDetail
{

using Sketcher::getRadiusCenterCircleArc;
using Sketcher::isArcOfCircle;
using Sketcher::isCircleOrArc;
using Sketcher::isLineSegment;

std::optional<Base::Vector3d> projectPointOntoLine(
    const Base::Vector3d& point,
    const Part::GeomLineSegment& line
)
{
    const Base::Vector3d start = line.getStartPoint();
    const Base::Vector3d direction = line.getEndPoint() - start;
    if (direction.Length() <= Base::Precision::Confusion()) {
        // A collapsed line has no stable perpendicular projection.
        return std::nullopt;
    }

    Base::Vector3d projected;
    projected.ProjectToLine(point - start, direction);
    projected += point;
    return projected;
}

Base::Vector3d radialDirection(
    const Base::Vector3d& from,
    Base::Vector3d fallbackLineDirection = Base::Vector3d()
)
{
    Base::Vector3d direction = from;
    if (direction.Length() <= Base::Precision::Confusion()) {
        direction = fallbackLineDirection;
        if (direction.Length() <= Base::Precision::Confusion()) {
            return Base::Vector3d(1.0, 0.0, 0.0);
        }
        direction.RotateZ(std::numbers::pi / 2.0);
    }
    return direction.Normalize();
}

double weightRepresentationFactor(const Part::Geometry& geometry)
{
    if (!geometry.hasExtension(ViewProviderSketchGeometryExtension::getClassTypeId())) {
        return 1.0;
    }

    const auto extension = std::static_pointer_cast<const ViewProviderSketchGeometryExtension>(
        geometry.getExtension(ViewProviderSketchGeometryExtension::getClassTypeId()).lock()
    );
    return extension ? extension->getRepresentationFactor() : 1.0;
}

std::optional<DimensionDatumEndpoints> pointGeometryEndpoints(
    const Base::Vector3d& point,
    const Part::Geometry& geometry
)
{
    if (isLineSegment(geometry)) {
        const auto& line = static_cast<const Part::GeomLineSegment&>(geometry);
        if (const auto projected = projectPointOntoLine(point, line)) {
            return DimensionDatumEndpoints {point, *projected};
        }
        return std::nullopt;
    }

    if (isCircleOrArc(geometry)) {
        const auto [radius, center] = getRadiusCenterCircleArc(&geometry);
        return DimensionDatumEndpoints {point, center + radius * radialDirection(point - center)};
    }
    return std::nullopt;
}

std::optional<DimensionDatumEndpoints> roundLineEndpoints(
    const Part::Geometry& round,
    const Part::GeomLineSegment& line
)
{
    const auto [radius, center] = getRadiusCenterCircleArc(&round);
    const auto projected = projectPointOntoLine(center, line);
    if (!projected) {
        return std::nullopt;
    }

    const Base::Vector3d lineDirection = line.getEndPoint() - line.getStartPoint();
    const Base::Vector3d roundPoint = center
        + radius * radialDirection(*projected - center, lineDirection);
    return DimensionDatumEndpoints {*projected, roundPoint};
}

std::optional<DimensionDatumEndpoints> twoGeometryEndpoints(
    const Sketcher::SketchObject& sketch,
    const Sketcher::Constraint& constraint
)
{
    const Part::Geometry* firstGeometry = sketch.getGeometry(constraint.First);
    const Part::Geometry* secondGeometry = sketch.getGeometry(constraint.Second);
    if (!firstGeometry || !secondGeometry) {
        return std::nullopt;
    }

    if (constraint.FirstPos != Sketcher::PointPos::none) {
        const Base::Vector3d point = sketch.getPoint(constraint.First, constraint.FirstPos);
        return pointGeometryEndpoints(point, *secondGeometry);
    }
    if (isCircleOrArc(*firstGeometry) && isLineSegment(*secondGeometry)) {
        return roundLineEndpoints(
            *firstGeometry,
            static_cast<const Part::GeomLineSegment&>(*secondGeometry)
        );
    }
    if (isLineSegment(*firstGeometry) && isCircleOrArc(*secondGeometry)) {
        return roundLineEndpoints(
            *secondGeometry,
            static_cast<const Part::GeomLineSegment&>(*firstGeometry)
        );
    }
    if (isCircleOrArc(*firstGeometry) && isCircleOrArc(*secondGeometry)) {
        DimensionDatumEndpoints endpoints;
        GetCirclesMinimalDistance(firstGeometry, secondGeometry, endpoints.first, endpoints.second);
        return endpoints;
    }
    return std::nullopt;
}

double defaultRoundAngle(const Part::Geometry& geometry, Sketcher::ConstraintType type)
{
    if (isArcOfCircle(geometry)) {
        const auto& arc = static_cast<const Part::GeomArcOfCircle&>(geometry);
        double startAngle = 0.0;
        double endAngle = 0.0;
        arc.getRange(startAngle, endAngle, /*emulateCCW=*/true);
        return (startAngle + endAngle) / 2.0;
    }
    return type == Sketcher::Diameter ? std::numbers::pi / 4.0 : 0.0;
}

std::optional<DimensionDatumEndpoints> singleGeometryEndpoints(
    const Part::Geometry& geometry,
    const Sketcher::Constraint& constraint,
    const Base::Vector2d& labelPosition
)
{
    if (isLineSegment(geometry)) {
        const auto& line = static_cast<const Part::GeomLineSegment&>(geometry);
        return DimensionDatumEndpoints {line.getStartPoint(), line.getEndPoint()};
    }
    if (!isCircleOrArc(geometry)) {
        return std::nullopt;
    }

    const auto [radius, center] = getRadiusCenterCircleArc(&geometry);
    const double defaultAngle = defaultRoundAngle(geometry, constraint.Type);
    const Base::Vector3d centerToLabel = Base::Vector3d(labelPosition.x, labelPosition.y, 0.0)
        - center;
    double angle = constraint.LabelPosition;
    if (isArcOfCircle(geometry) && isAutoDatumLabelPosition(angle)) {
        angle = defaultAngle;
    }
    else if (centerToLabel.Length() > Base::Precision::Confusion()) {
        angle = std::atan2(centerToLabel.y, centerToLabel.x);
    }
    else if (isAutoDatumLabelPosition(angle)) {
        angle = defaultAngle;
    }

    const Base::Vector3d radial(std::cos(angle), std::sin(angle), 0.0);
    Base::Vector3d first = center;
    if (constraint.Type == Sketcher::Diameter) {
        first -= radius * radial;
    }
    const double displayRadius = constraint.Type == Sketcher::Weight
        ? radius * weightRepresentationFactor(geometry)
        : radius;
    return DimensionDatumEndpoints {first, center + displayRadius * radial};
}

std::optional<DimensionDatumEndpoints> resolveDatumEndpoints(
    const Sketcher::SketchObject& sketch,
    const Sketcher::Constraint& constraint,
    const Base::Vector2d& labelPosition
)
{
    if (constraint.SecondPos != Sketcher::PointPos::none) {
        return DimensionDatumEndpoints {
            sketch.getPoint(constraint.First, constraint.FirstPos),
            sketch.getPoint(constraint.Second, constraint.SecondPos),
        };
    }
    if (constraint.Second != Sketcher::GeoEnum::GeoUndef) {
        return twoGeometryEndpoints(sketch, constraint);
    }
    if (constraint.FirstPos != Sketcher::PointPos::none) {
        return DimensionDatumEndpoints {
            Base::Vector3d(),
            sketch.getPoint(constraint.First, constraint.FirstPos),
        };
    }
    if (constraint.First == Sketcher::GeoEnum::GeoUndef) {
        return std::nullopt;
    }

    const Part::Geometry* geometry = sketch.getGeometry(constraint.First);
    return geometry ? singleGeometryEndpoints(*geometry, constraint, labelPosition) : std::nullopt;
}

bool applyArcLengthPlacement(
    const Part::GeomArcOfCircle& arc,
    Sketcher::Constraint& constraint,
    const Base::Vector2d& labelPosition
)
{
    const double angle = defaultRoundAngle(arc, constraint.Type);
    const Base::Vector2d direction(std::cos(angle), std::sin(angle));
    const Base::Vector3d center = arc.getCenter();
    constraint.LabelDistance = (labelPosition - Base::Vector2d(center.x, center.y)) * direction;
    return true;
}

bool applyLinearPlacement(
    const Sketcher::SketchObject& sketch,
    Sketcher::Constraint& constraint,
    const Base::Vector2d& labelPosition,
    double labelOffset
)
{
    if (constraint.Type == Sketcher::Distance && constraint.Second == Sketcher::GeoEnum::GeoUndef
        && constraint.FirstPos == Sketcher::PointPos::none) {
        const Part::Geometry* geometry = sketch.getGeometry(constraint.First);
        if (geometry && isArcOfCircle(*geometry)) {
            return applyArcLengthPlacement(
                static_cast<const Part::GeomArcOfCircle&>(*geometry),
                constraint,
                labelPosition
            );
        }
    }

    const auto endpoints = resolveDatumEndpoints(sketch, constraint, labelPosition);
    if (!endpoints) {
        return false;
    }

    const Base::Vector3d datumDirection = endpoints->second - endpoints->first;
    if (datumDirection.Length() <= Base::Precision::Confusion()) {
        return false;
    }

    const Base::Vector3d label(labelPosition.x, labelPosition.y, 0.0);
    Base::Vector3d direction;
    if (constraint.Type == Sketcher::Distance || constraint.Type == Sketcher::Radius
        || constraint.Type == Sketcher::Diameter || constraint.Type == Sketcher::Weight) {
        direction = datumDirection;
        direction.Normalize();
    }
    else if (constraint.Type == Sketcher::DistanceX) {
        direction = Base::Vector3d(
            datumDirection.x >= std::numeric_limits<float>::epsilon() ? 1.0 : -1.0,
            0.0,
            0.0
        );
    }
    else if (constraint.Type == Sketcher::DistanceY) {
        direction = Base::Vector3d(
            0.0,
            datumDirection.y >= std::numeric_limits<float>::epsilon() ? 1.0 : -1.0,
            0.0
        );
    }
    else {
        return false;
    }

    if (constraint.Type == Sketcher::Radius || constraint.Type == Sketcher::Diameter
        || constraint.Type == Sketcher::Weight) {
        const Base::Vector3d labelVector = label - endpoints->second;
        double distance = labelVector * direction;
        if (distance > labelOffset) {
            distance -= labelOffset;
        }
        constraint.LabelDistance = distance;
        constraint.LabelPosition = std::atan2(direction.y, direction.x);
        return true;
    }

    const Base::Vector3d normal(-direction.y, direction.x, 0.0);
    constraint.LabelDistance = (label - endpoints->second) * normal - labelOffset;
    constraint.LabelPosition = (label - (endpoints->second + endpoints->first) / 2.0) * direction;
    return true;
}

bool applyAnglePlacement(
    const Sketcher::SketchObject& sketch,
    Sketcher::Constraint& constraint,
    const Base::Vector2d& labelPosition
)
{
    Base::Vector3d vertex;
    constexpr double angleLabelDistanceScale = 0.5;

    if (constraint.Second != Sketcher::GeoEnum::GeoUndef) {
        if (constraint.Third == Sketcher::GeoEnum::GeoUndef) {
            const Part::Geometry* firstGeometry = sketch.getGeometry(constraint.First);
            const Part::Geometry* secondGeometry = sketch.getGeometry(constraint.Second);
            if (!firstGeometry || !secondGeometry || !isLineSegment(*firstGeometry)
                || !isLineSegment(*secondGeometry)) {
                return false;
            }

            const auto& firstLine = static_cast<const Part::GeomLineSegment&>(*firstGeometry);
            const auto& secondLine = static_cast<const Part::GeomLineSegment&>(*secondGeometry);
            Base::Vector2d firstStart(firstLine.getStartPoint().x, firstLine.getStartPoint().y);
            Base::Vector2d firstEnd(firstLine.getEndPoint().x, firstLine.getEndPoint().y);
            Base::Vector2d secondStart(secondLine.getStartPoint().x, secondLine.getStartPoint().y);
            Base::Vector2d secondEnd(secondLine.getEndPoint().x, secondLine.getEndPoint().y);

            if (constraint.FirstPos == Sketcher::PointPos::end) {
                std::swap(firstStart, firstEnd);
            }
            if (constraint.SecondPos == Sketcher::PointPos::end) {
                std::swap(secondStart, secondEnd);
            }

            Base::Vector2d intersection;
            if (!Base::Line2d(firstStart, firstEnd)
                     .Intersect(Base::Line2d(secondStart, secondEnd), intersection)) {
                return false;
            }
            vertex = Base::Vector3d(intersection.x, intersection.y, 0.0);
        }
        else {
            vertex = sketch.getPoint(constraint.Third, constraint.ThirdPos);
        }
    }
    else if (constraint.First != Sketcher::GeoEnum::GeoUndef) {
        const Part::Geometry* geometry = sketch.getGeometry(constraint.First);
        if (!geometry) {
            return false;
        }
        if (isLineSegment(*geometry)) {
            const auto& line = static_cast<const Part::GeomLineSegment&>(*geometry);
            vertex = (line.getEndPoint() + line.getStartPoint()) / 2.0;
        }
        else if (isArcOfCircle(*geometry)) {
            const auto& arc = static_cast<const Part::GeomArcOfCircle&>(*geometry);
            double startAngle = 0.0;
            double endAngle = 0.0;
            arc.getRange(startAngle, endAngle, /*emulateCCW=*/true);
            const double middleAngle = (startAngle + endAngle) / 2.0;
            const Base::Vector2d arcDirection(std::cos(middleAngle), std::sin(middleAngle));
            const Base::Vector3d center = arc.getCenter();
            const Base::Vector2d centerToLabel = labelPosition - Base::Vector2d(center.x, center.y);
            constraint.LabelDistance = angleLabelDistanceScale * (centerToLabel * arcDirection);
            return true;
        }
        else {
            return false;
        }
    }
    else {
        return false;
    }

    const Base::Vector3d label(labelPosition.x, labelPosition.y, 0.0);
    constraint.LabelDistance = angleLabelDistanceScale * (label - vertex).Length();
    return true;
}

}  // namespace SketcherGui::DimensionDatumPlacementDetail

namespace SketcherGui
{

std::optional<DimensionDatumEndpoints> resolveDimensionDatumEndpoints(
    const Sketcher::SketchObject& sketch,
    const Sketcher::Constraint& constraint,
    const Base::Vector2d& labelPosition
)
{
    return DimensionDatumPlacementDetail::resolveDatumEndpoints(sketch, constraint, labelPosition);
}

bool prepareDimensionDatumPlacement(
    const Sketcher::SketchObject& sketch,
    Sketcher::Constraint& constraint,
    const Base::Vector2d& labelPosition,
    double labelOffset
)
{
    if (constraint.Type == Sketcher::Distance || constraint.Type == Sketcher::DistanceX
        || constraint.Type == Sketcher::DistanceY || constraint.Type == Sketcher::Radius
        || constraint.Type == Sketcher::Diameter || constraint.Type == Sketcher::Weight) {
        return DimensionDatumPlacementDetail::applyLinearPlacement(
            sketch,
            constraint,
            labelPosition,
            labelOffset
        );
    }
    if (constraint.Type == Sketcher::Angle) {
        return DimensionDatumPlacementDetail::applyAnglePlacement(sketch, constraint, labelPosition);
    }
    return false;
}

}  // namespace SketcherGui
