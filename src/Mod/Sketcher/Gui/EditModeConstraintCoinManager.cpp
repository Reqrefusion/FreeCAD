// SPDX-License-Identifier: LGPL-2.1-or-later

/***************************************************************************
 *   Copyright (c) 2021 Abdullah Tahiri <abdullah.tahiri.yo@gmail.com>     *
 *                                                                         *
 *   This file is part of the FreeCAD CAx development system.              *
 *                                                                         *
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Library General Public           *
 *   License as published by the Free Software Foundation; either          *
 *   version 2 of the License, or (at your option) any later version.      *
 *                                                                         *
 *   This library  is distributed in the hope that it will be useful,      *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU Library General Public License for more details.                  *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public     *
 *   License along with this library; see the file COPYING.LIB. If not,    *
 *   write to the Free Software Foundation, Inc., 59 Temple Place,         *
 *   Suite 330, Boston, MA  02111-1307, USA                                *
 *                                                                         *
 ***************************************************************************/

#include <FCConfig.h>

#include <QApplication>
#include <QPainter>
#include <QFontInfo>
#include <QLineF>
#include <QLocale>
#include <algorithm>
#include <QRegularExpression>
#include <Bnd_Box.hxx>
#include <limits>
#include <numeric>
#include <memory>
#include <map>
#include <numbers>
#include <stack>
#include <cmath>

#include <Inventor/SbImage.h>
#include <Inventor/SbVec3f.h>
#include <Inventor/SoPickedPoint.h>
#include <Inventor/SoPath.h>
#include <Inventor/actions/SoRayPickAction.h>
#include <Inventor/nodes/SoAnnotation.h>
#include <Inventor/nodes/SoBaseColor.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoDepthBuffer.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include <Inventor/nodes/SoGroup.h>
#include <Inventor/nodes/SoImage.h>
#include <Inventor/nodes/SoLineSet.h>
#include <Inventor/nodes/SoInfo.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoMaterialBinding.h>
#include <Inventor/nodes/SoPickStyle.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/nodes/SoText2.h>
#include <Inventor/nodes/SoTranslation.h>

#include <BRepBndLib.hxx>

#include <Base/Converter.h>
#include <Base/Exception.h>
#include <Base/Tools.h>
#include <Base/UnitsApi.h>
#include <Base/Vector3D.h>
#include <Gui/BitmapFactory.h>
#include <Gui/Inventor/SmSwitchboard.h>
#include <Gui/SoDatumLabel.h>
#include <Gui/Tools.h>
#include <Gui/Utilities.h>
#include <Mod/Part/App/Geometry.h>
#include <Mod/Sketcher/App/Constraint.h>
#include <Mod/Sketcher/App/GeoEnum.h>
#include <Mod/Sketcher/App/GeoList.h>
#include <Mod/Sketcher/App/SketchObject.h>
#include <Mod/Sketcher/App/GeometryFacade.h>
#include <Mod/Sketcher/App/SolverGeometryExtension.h>

#include "EditModeConstraintCoinManager.h"
#include "DimensionConstraintBuilder.h"
#include "DimensionGeometry.h"
#include "SoZoomTranslation.h"
#include "Utils.h"
#include "ViewProviderSketch.h"
#include "ViewProviderSketchCoinAttorney.h"


using namespace Gui;
using namespace SketcherGui;
using namespace Sketcher;

namespace SketcherGui::EditModeConstraintPreviewDetail {

constexpr int kBasePreviewHitTolerancePx = 10;
constexpr int kBasePreviewLineHitTolerancePx = 12;
constexpr int kPreviewArcSamples = 12;

using namespace DimensionGeometry;


QFont previewFont(const DrawingParameters& drawingParameters)
{
    QFont font = QApplication::font();
    int defaultPx = QFontInfo(font).pixelSize();
    if (defaultPx <= 0) {
        defaultPx = QFontMetrics(font).height();
    }
    font.setPixelSize(static_cast<int>(std::max(defaultPx,
        static_cast<int>(std::lround(drawingParameters.constraintIconSize)))));
    font.setBold(true);
    return font;
}


QSize previewLabelSize(const DrawingParameters& drawingParameters,
                       const Gui::SoDatumLabel& datum)
{
    const QFontMetrics metrics(previewFont(drawingParameters));
    const QString text = datum.string.getNum() > 0
        ? QString::fromUtf8(datum.string[0].getString())
        : QString();
    const QRect textRect = metrics.boundingRect(text);
    const int defaultPx = std::max(12, metrics.height());
    const int width = std::max(defaultPx * 3, textRect.width() + 16);
    const int height = std::max(defaultPx + 8, textRect.height() + 8);
    return QSize(width, height);
}

Base::Vector2d previewLabelCenterSketch(const DrawingParameters& drawingParameters,
                                        const DimensionCandidate& candidate,
                                        const Gui::SoDatumLabel& datum)
{
    Base::Vector2d center = candidate.labelPos;
    const QSize labelSize = previewLabelSize(drawingParameters, datum);
    const double imgWidth = static_cast<double>(labelSize.width());
    const double imgHeight = static_cast<double>(labelSize.height());

    const int pointCount = datum.pnts.getNum();
    const auto* verts = datum.pnts.getValues(0);
    const double param1 = static_cast<double>(datum.param1.getValue());
    const double param2 = static_cast<double>(datum.param2.getValue());

    if (pointCount >= 2) {
        const Base::Vector2d p1(verts[0][0], verts[0][1]);
        const Base::Vector2d p2(verts[1][0], verts[1][1]);

        if (datum.datumtype.getValue() == Gui::SoDatumLabel::DISTANCE
            || datum.datumtype.getValue() == Gui::SoDatumLabel::DISTANCEX
            || datum.datumtype.getValue() == Gui::SoDatumLabel::DISTANCEY) {
            Base::Vector2d dir(0.0, 0.0);
            if (datum.datumtype.getValue() == Gui::SoDatumLabel::DISTANCE) {
                dir = normalized(Base::Vector2d(p2.x - p1.x, p2.y - p1.y), Base::Vector2d(1.0, 0.0));
            }
            else if (datum.datumtype.getValue() == Gui::SoDatumLabel::DISTANCEX) {
                dir = Base::Vector2d((p2.x - p1.x >= 0.0) ? 1.0 : -1.0, 0.0);
            }
            else {
                dir = Base::Vector2d(0.0, (p2.y - p1.y >= 0.0) ? 1.0 : -1.0);
            }

            const Base::Vector2d normal(-dir.y, dir.x);
            const double normproj12 = (p2.x - p1.x) * normal.x + (p2.y - p1.y) * normal.y;
            const Base::Vector2d p1proj(p1.x + normproj12 * normal.x,
                                        p1.y + normproj12 * normal.y);
            const Base::Vector2d mid((p1proj.x + p2.x) * 0.5, (p1proj.y + p2.y) * 0.5);
            center = Base::Vector2d(mid.x + normal.x * param1 + dir.x * param2,
                                    mid.y + normal.y * param1 + dir.y * param2);
        }
        else if (datum.datumtype.getValue() == Gui::SoDatumLabel::RADIUS
                 || datum.datumtype.getValue() == Gui::SoDatumLabel::DIAMETER) {
            const Base::Vector2d dir = normalized(Base::Vector2d(p2.x - p1.x, p2.y - p1.y),
                                                  Base::Vector2d(1.0, 0.0));
            center = Base::Vector2d(p2.x + dir.x * param1,
                                    p2.y + dir.y * param1);
        }
    }

    if (datum.datumtype.getValue() == Gui::SoDatumLabel::ANGLE && pointCount >= 1) {
        const Base::Vector2d vertex(verts[0][0], verts[0][1]);
        const double midAngle = param2 + static_cast<double>(datum.param3.getValue()) * 0.5;
        center = Base::Vector2d(vertex.x + std::cos(midAngle) * (2.0 * param1),
                                vertex.y + std::sin(midAngle) * (2.0 * param1));
    }
    else if (datum.datumtype.getValue() == Gui::SoDatumLabel::ARCLENGTH && pointCount >= 3) {
        const Base::Vector2d ctr(verts[0][0], verts[0][1]);
        const double startAngle = static_cast<double>(datum.param2.getValue());
        const double range = static_cast<double>(datum.param3.getValue());
        const double midAngle = startAngle + range * 0.5;
        const double labelRadius = std::max(1.0, std::abs(param1));
        center = Base::Vector2d(ctr.x + std::cos(midAngle) * labelRadius,
                                ctr.y + std::sin(midAngle) * labelRadius);
    }

    return center;
}

double previewLabelAngleRadians(const Gui::SoDatumLabel& datum)
{
    const int pointCount = datum.pnts.getNum();
    if (pointCount < 2) {
        return 0.0;
    }

    const auto* verts = datum.pnts.getValues(0);
    const Base::Vector2d p1(verts[0][0], verts[0][1]);
    const Base::Vector2d p2(verts[1][0], verts[1][1]);
    Base::Vector2d dir(1.0, 0.0);

    constexpr double angleSlack = std::numbers::pi / 12.0;
    if (datum.datumtype.getValue() == Gui::SoDatumLabel::DISTANCE) {
        dir = normalized(Base::Vector2d(p2.x - p1.x, p2.y - p1.y), Base::Vector2d(1.0, 0.0));
    }
    else if (datum.datumtype.getValue() == Gui::SoDatumLabel::DISTANCEX) {
        dir = Base::Vector2d((p2.x - p1.x >= 0.0) ? 1.0 : -1.0, 0.0);
    }
    else if (datum.datumtype.getValue() == Gui::SoDatumLabel::DISTANCEY) {
        dir = Base::Vector2d(0.0, (p2.y - p1.y >= 0.0) ? 1.0 : -1.0);
    }
    else if (datum.datumtype.getValue() == Gui::SoDatumLabel::ARCLENGTH && pointCount >= 3) {
        const double startAngle = static_cast<double>(datum.param2.getValue());
        const double range = static_cast<double>(datum.param3.getValue());
        const double midAngle = startAngle + range * 0.5;
        dir = range >= 0.0 ? Base::Vector2d(-std::sin(midAngle), std::cos(midAngle))
                           : Base::Vector2d(std::sin(midAngle), -std::cos(midAngle));
    }
    else if (datum.datumtype.getValue() == Gui::SoDatumLabel::RADIUS
             || datum.datumtype.getValue() == Gui::SoDatumLabel::DIAMETER) {
        dir = normalized(Base::Vector2d(p2.x - p1.x, p2.y - p1.y), Base::Vector2d(1.0, 0.0));
    }
    else {
        return 0.0;
    }

    double angle = std::atan2(dir.y, dir.x);
    if (angle > std::numbers::pi / 2.0 + angleSlack) {
        angle -= std::numbers::pi;
    }
    else if (angle <= -std::numbers::pi / 2.0 + angleSlack) {
        angle += std::numbers::pi;
    }
    return angle;
}

QPoint previewLabelCenter(const ViewProviderSketch& viewProvider,
                          const DrawingParameters& drawingParameters,
                          const DimensionCandidate& candidate,
                          const Gui::SoDatumLabel& datum)
{
    return viewProvider.projectSketchPointToScreen(
        previewLabelCenterSketch(drawingParameters, candidate, datum));
}

QRect previewLabelRect(const ViewProviderSketch& viewProvider,
                       const DrawingParameters& drawingParameters,
                       const DimensionCandidate& candidate,
                       const Gui::SoDatumLabel& datum)
{
    const QPoint pt = previewLabelCenter(viewProvider, drawingParameters, candidate, datum);
    const QSize baseSize = previewLabelSize(drawingParameters, datum);
    const double angle = previewLabelAngleRadians(datum);
    const double c = std::abs(std::cos(angle));
    const double s = std::abs(std::sin(angle));
    const int width = static_cast<int>(std::ceil(baseSize.width() * c + baseSize.height() * s)) + 6;
    const int height = static_cast<int>(std::ceil(baseSize.width() * s + baseSize.height() * c)) + 6;
    return QRect(pt.x() - width / 2, pt.y() - height / 2, width, height);
}

int previewLabelHitDistance(const ViewProviderSketch& viewProvider,
                            const DrawingParameters& drawingParameters,
                            const DimensionCandidate& candidate,
                            const Gui::SoDatumLabel& datum,
                            const QPoint& screenPos,
                            int tolerancePx)
{
    const QRect labelRect = previewLabelRect(viewProvider, drawingParameters, candidate, datum)
                                .adjusted(-tolerancePx,
                                          -tolerancePx,
                                          tolerancePx,
                                          tolerancePx);
    const int dx = screenPos.x() < labelRect.left() ? labelRect.left() - screenPos.x()
                   : screenPos.x() > labelRect.right() ? screenPos.x() - labelRect.right()
                                                      : 0;
    const int dy = screenPos.y() < labelRect.top() ? labelRect.top() - screenPos.y()
                   : screenPos.y() > labelRect.bottom() ? screenPos.y() - labelRect.bottom()
                                                       : 0;
    return dx * dx + dy * dy;
}



int squaredDistance(const QPoint& a, const QPoint& b)
{
    const int dx = a.x() - b.x();
    const int dy = a.y() - b.y();
    return dx * dx + dy * dy;
}

double distancePointToSegment(const QPoint& p, const QPoint& a, const QPoint& b)
{
    const double ax = static_cast<double>(a.x());
    const double ay = static_cast<double>(a.y());
    const double bx = static_cast<double>(b.x());
    const double by = static_cast<double>(b.y());
    const double px = static_cast<double>(p.x());
    const double py = static_cast<double>(p.y());
    const double vx = bx - ax;
    const double vy = by - ay;
    const double wx = px - ax;
    const double wy = py - ay;
    const double vv = vx * vx + vy * vy;
    if (vv < kEpsilon) {
        return std::sqrt((px - ax) * (px - ax) + (py - ay) * (py - ay));
    }

    const double t = std::clamp((wx * vx + wy * vy) / vv, 0.0, 1.0);
    const double cx = ax + t * vx;
    const double cy = ay + t * vy;
    const double dx = px - cx;
    const double dy = py - cy;
    return std::sqrt(dx * dx + dy * dy);
}

int distancePointToRect(const QPoint& p, const QRect& rect)
{
    const int dx = p.x() < rect.left() ? rect.left() - p.x()
                   : p.x() > rect.right() ? p.x() - rect.right()
                                         : 0;
    const int dy = p.y() < rect.top() ? rect.top() - p.y()
                   : p.y() > rect.bottom() ? p.y() - rect.bottom()
                                          : 0;
    return dx * dx + dy * dy;
}

struct PreviewSegment
{
    QPoint a;
    QPoint b;
};

void appendArcSamples(std::vector<QPoint>& points,
                      const QPoint& center,
                      const QPoint& startPoint,
                      double rangeRadians);

void appendPreviewSegment(std::vector<PreviewSegment>& segments,
                          const QPoint& a,
                          const QPoint& b)
{
    if (a == b) {
        return;
    }

    segments.push_back(PreviewSegment {a, b});
}

bool previewPointsNear(const QPoint& a, const QPoint& b, int tolerancePx = 2)
{
    return std::abs(a.x() - b.x()) <= tolerancePx && std::abs(a.y() - b.y()) <= tolerancePx;
}

bool previewSegmentsIntersect(const PreviewSegment& lhs, const PreviewSegment& rhs)
{
    if (previewPointsNear(lhs.a, rhs.a) || previewPointsNear(lhs.a, rhs.b)
        || previewPointsNear(lhs.b, rhs.a) || previewPointsNear(lhs.b, rhs.b)) {
        return false;
    }

    QPointF intersectionPoint;
    const QLineF first(lhs.a, lhs.b);
    const QLineF second(rhs.a, rhs.b);
    if (first.intersects(second, &intersectionPoint) != QLineF::BoundedIntersection) {
        return false;
    }

    const QPoint roundedIntersection(static_cast<int>(std::lround(intersectionPoint.x())),
                                     static_cast<int>(std::lround(intersectionPoint.y())));
    if (previewPointsNear(roundedIntersection, lhs.a) || previewPointsNear(roundedIntersection, lhs.b)
        || previewPointsNear(roundedIntersection, rhs.a)
        || previewPointsNear(roundedIntersection, rhs.b)) {
        return false;
    }

    return true;
}

bool previewSegmentIntersectsRect(const PreviewSegment& segment, const QRect& rect)
{
    if (rect.contains(segment.a) || rect.contains(segment.b)) {
        return true;
    }

    const QPoint topLeft = rect.topLeft();
    const QPoint topRight(rect.right(), rect.top());
    const QPoint bottomLeft(rect.left(), rect.bottom());
    const QPoint bottomRight = rect.bottomRight();
    const PreviewSegment edges[] {{topLeft, topRight},
                                  {topRight, bottomRight},
                                  {bottomRight, bottomLeft},
                                  {bottomLeft, topLeft}};
    return std::any_of(std::begin(edges), std::end(edges), [&](const PreviewSegment& edge) {
        return previewSegmentsIntersect(segment, edge);
    });
}

void appendSourceGeometrySegments(const ViewProviderSketch& viewProvider,
                                const DimensionCandidate& candidate,
                                std::vector<PreviewSegment>& segments)
{
    auto* sketch = viewProvider.getSketchObject();
    if (!sketch) {
        return;
    }

    auto appendSketchSegment = [&](const Base::Vector2d& a, const Base::Vector2d& b) {
        const QPoint screenA = viewProvider.projectSketchPointToScreen(a);
        const QPoint screenB = viewProvider.projectSketchPointToScreen(b);
        appendPreviewSegment(segments, screenA, screenB);
    };

    switch (candidate.semantic) {
        case DimensionSemantic::ProjectedX:
        case DimensionSemantic::ProjectedY:
        case DimensionSemantic::DirectLength:
        case DimensionSemantic::DirectDistance: {
            if (candidate.refs.size() < 2) {
                break;
            }
            const auto segment = directDistanceSegment(*sketch, candidate.refs[0], candidate.refs[1]);
            if (!segment.valid) {
                break;
            }
            appendSketchSegment(segment.a, segment.b);
            break;
        }
        default:
            break;
    }
}

void appendDatumSegments(const ViewProviderSketch& viewProvider,
                         const DrawingParameters& drawingParameters,
                         const Gui::SoDatumLabel& datum,
                         std::vector<PreviewSegment>& segments)
{
    const int pointCount = datum.pnts.getNum();
    const auto* verts = datum.pnts.getValues(0);
    const QSize labelSize = previewLabelSize(drawingParameters, datum);
    const double imgWidth = static_cast<double>(labelSize.width());
    const double imgHeight = static_cast<double>(labelSize.height());

    const auto toScreenSketchPoint = [&](const Base::Vector2d& point) {
        return viewProvider.projectSketchPointToScreen(point);
    };
    const auto appendSketchSegment = [&](const Base::Vector2d& a, const Base::Vector2d& b) {
        appendPreviewSegment(segments, toScreenSketchPoint(a), toScreenSketchPoint(b));
    };
    const auto appendArrowSegments = [&](const Base::Vector2d& base,
                                         const Base::Vector2d& dir,
                                         double width,
                                         double length) {
        const Base::Vector2d unitDir = normalized(dir, Base::Vector2d(1.0, 0.0));
        const Base::Vector2d perp(-unitDir.y, unitDir.x);
        const Base::Vector2d tip(base.x + unitDir.x * length, base.y + unitDir.y * length);
        const Base::Vector2d p1(base.x + perp.x * (width * 0.5), base.y + perp.y * (width * 0.5));
        const Base::Vector2d p2(base.x - perp.x * (width * 0.5), base.y - perp.y * (width * 0.5));
        appendSketchSegment(tip, p1);
        appendSketchSegment(tip, p2);
    };
    const auto appendArcSegmentsSketch = [&](const Base::Vector2d& center,
                                             double radius,
                                             double startAngle,
                                             double endAngle) {
        const double span = endAngle - startAngle;
        const int segmentCount = std::max(6,
                                          std::abs(static_cast<int>(50.0 * span
                                                                    / (2.0 * std::numbers::pi))));
        if (segmentCount < 2 || radius < kEpsilon) {
            return;
        }

        Base::Vector2d previous(center.x + std::cos(startAngle) * radius,
                                center.y + std::sin(startAngle) * radius);
        for (int i = 1; i < segmentCount; ++i) {
            const double t = static_cast<double>(i) / static_cast<double>(segmentCount - 1);
            const double angle = startAngle + span * t;
            const Base::Vector2d current(center.x + std::cos(angle) * radius,
                                         center.y + std::sin(angle) * radius);
            appendSketchSegment(previous, current);
            previous = current;
        }
    };

    switch (datum.datumtype.getValue()) {
        case Gui::SoDatumLabel::DISTANCE:
        case Gui::SoDatumLabel::DISTANCEX:
        case Gui::SoDatumLabel::DISTANCEY: {
            if (pointCount < 2) {
                break;
            }

            const Base::Vector2d p1(verts[0][0], verts[0][1]);
            const Base::Vector2d p2(verts[1][0], verts[1][1]);
            const double length1 = static_cast<double>(datum.param1.getValue());
            const double length2 = static_cast<double>(datum.param2.getValue());

            Base::Vector2d dir(1.0, 0.0);
            if (datum.datumtype.getValue() == Gui::SoDatumLabel::DISTANCE) {
                dir = normalized(Base::Vector2d(p2.x - p1.x, p2.y - p1.y), Base::Vector2d(1.0, 0.0));
            }
            else if (datum.datumtype.getValue() == Gui::SoDatumLabel::DISTANCEX) {
                dir = Base::Vector2d((p2.x - p1.x >= 0.0) ? 1.0 : -1.0, 0.0);
            }
            else {
                dir = Base::Vector2d(0.0, (p2.y - p1.y >= 0.0) ? 1.0 : -1.0);
            }

            const Base::Vector2d normal(-dir.y, dir.x);
            const double normproj12 = (p2.x - p1.x) * normal.x + (p2.y - p1.y) * normal.y;
            const Base::Vector2d p1proj(p1.x + normal.x * normproj12,
                                        p1.y + normal.y * normproj12);
            const Base::Vector2d midpos((p1proj.x + p2.x) * 0.5, (p1proj.y + p2.y) * 0.5);
            const double margin = imgHeight / 3.0;
            const double offset1 = ((length1 + normproj12 < 0.0) ? -1.0 : 1.0) * margin;
            const double offset2 = ((length1 < 0.0) ? -1.0 : 1.0) * margin;
            const Base::Vector2d perp1(p1proj.x + normal.x * (length1 + offset1),
                                       p1proj.y + normal.y * (length1 + offset1));
            const Base::Vector2d perp2(p2.x + normal.x * (length1 + offset2),
                                       p2.y + normal.y * (length1 + offset2));
            Base::Vector2d par1(p1proj.x + normal.x * length1, p1proj.y + normal.y * length1);
            Base::Vector2d par2(midpos.x + normal.x * length1 + dir.x * (length2 - imgWidth / 2.0 - margin),
                                midpos.y + normal.y * length1 + dir.y * (length2 - imgWidth / 2.0 - margin));
            Base::Vector2d par3(midpos.x + normal.x * length1 + dir.x * (length2 + imgWidth / 2.0 + margin),
                                midpos.y + normal.y * length1 + dir.y * (length2 + imgWidth / 2.0 + margin));
            Base::Vector2d par4(p2.x + normal.x * length1, p2.y + normal.y * length1);
            bool flipTriang = false;
            if (((par3.x - par1.x) * dir.x + (par3.y - par1.y) * dir.y)
                > length(Base::Vector2d(par4.x - par1.x, par4.y - par1.y))) {
                const double tmpMargin = imgHeight / 0.75;
                par3 = par4;
                if (((par2.x - par1.x) * dir.x + (par2.y - par1.y) * dir.y)
                    > length(Base::Vector2d(par4.x - par1.x, par4.y - par1.y))) {
                    par3 = par2;
                    par2 = Base::Vector2d(par1.x - dir.x * tmpMargin, par1.y - dir.y * tmpMargin);
                    flipTriang = true;
                }
            }
            else if (((par2.x - par1.x) * dir.x + (par2.y - par1.y) * dir.y) < 0.0) {
                const double tmpMargin = imgHeight / 0.75;
                par2 = par1;
                if (((par3.x - par1.x) * dir.x + (par3.y - par1.y) * dir.y) < 0.0) {
                    par2 = par3;
                    par3 = Base::Vector2d(par4.x + dir.x * tmpMargin, par4.y + dir.y * tmpMargin);
                    flipTriang = true;
                }
            }

            appendSketchSegment(p1proj, perp1);
            appendSketchSegment(p2, perp2);
            appendSketchSegment(par1, par2);
            appendSketchSegment(par3, par4);

            const double arrowWidth = margin * 0.5;
            const double arrowLength = 0.866 * 2.0 * margin;
            const Base::Vector2d arrowDir((flipTriang ? -1.0 : 1.0) * dir.x,
                                          (flipTriang ? -1.0 : 1.0) * dir.y);
            appendArrowSegments(par1, arrowDir, arrowWidth, arrowLength);
            appendArrowSegments(par4, Base::Vector2d(-arrowDir.x, -arrowDir.y), arrowWidth, arrowLength);
            break;
        }
        case Gui::SoDatumLabel::RADIUS:
        case Gui::SoDatumLabel::DIAMETER: {
            if (pointCount < 2) {
                break;
            }

            const Base::Vector2d p1(verts[0][0], verts[0][1]);
            Base::Vector2d p2(verts[1][0], verts[1][1]);
            const Base::Vector2d dir = normalized(Base::Vector2d(p2.x - p1.x, p2.y - p1.y),
                                                  Base::Vector2d(1.0, 0.0));
            const Base::Vector2d normal(-dir.y, dir.x);
            const double margin = imgHeight / 3.0;
            const double arrowWidth = margin * 0.5;
            const double arrowLength = 0.866 * 2.0 * margin;
            const double length1 = static_cast<double>(datum.param1.getValue());
            const bool isDiameter = datum.datumtype.getValue() == Gui::SoDatumLabel::DIAMETER;
            const Base::Vector2d pos(p2.x + dir.x * length1, p2.y + dir.y * length1);
            const Base::Vector2d pnt1(pos.x - dir.x * (margin + imgWidth / 2.0),
                                      pos.y - dir.y * (margin + imgWidth / 2.0));
            const Base::Vector2d pnt2(pos.x + dir.x * (margin + imgWidth / 2.0),
                                      pos.y + dir.y * (margin + imgWidth / 2.0));
            const Base::Vector2d p3(pos.x + dir.x * (imgWidth / 2.0 + margin),
                                    pos.y + dir.y * (imgWidth / 2.0 + margin));
            if (length(Base::Vector2d(p3.x - p1.x, p3.y - p1.y))
                > length(Base::Vector2d(p2.x - p1.x, p2.y - p1.y))) {
                p2 = p3;
            }

            appendSketchSegment(p1, pnt1);
            appendSketchSegment(pnt2, p2);
            appendArrowSegments(Base::Vector2d(verts[1][0], verts[1][1]), Base::Vector2d(-dir.x, -dir.y), arrowWidth, arrowLength);
            if (isDiameter) {
                appendArrowSegments(p1, dir, arrowWidth, arrowLength);
            }

            Base::Vector2d center = p1;
            double radius = length(Base::Vector2d(verts[1][0] - verts[0][0], verts[1][1] - verts[0][1]));
            if (isDiameter) {
                center = Base::Vector2d((verts[0][0] + verts[1][0]) * 0.5,
                                        (verts[0][1] + verts[1][1]) * 0.5);
                radius *= 0.5;
            }

            const double startAngle = static_cast<double>(datum.param3.getValue());
            const double startRange = static_cast<double>(datum.param4.getValue());
            const double endAngle = static_cast<double>(datum.param5.getValue());
            const double endRange = static_cast<double>(datum.param6.getValue());
            if (std::abs(startRange) > kEpsilon) {
                appendArcSegmentsSketch(center, radius, startAngle, startAngle + startRange);
            }
            if (std::abs(endRange) > kEpsilon) {
                appendArcSegmentsSketch(center, radius, endAngle, endAngle + endRange);
            }
            break;
        }
        case Gui::SoDatumLabel::ANGLE: {
            if (pointCount < 1) {
                break;
            }

            const Base::Vector2d p0(verts[0][0], verts[0][1]);
            const double length1 = static_cast<double>(datum.param1.getValue());
            const double startangle = static_cast<double>(datum.param2.getValue());
            const double range = static_cast<double>(datum.param3.getValue());
            const double endangle = startangle + range;
            const double margin = imgHeight / 3.0;
            const double r = 2.0 * length1;
            const double endLineLength1 = std::max(static_cast<double>(datum.param4.getValue()), margin);
            const double endLineLength2 = std::max(static_cast<double>(datum.param5.getValue()), margin);
            const double endLineLength12 = std::max(-static_cast<double>(datum.param4.getValue()), margin);
            const double endLineLength22 = std::max(-static_cast<double>(datum.param5.getValue()), margin);
            const double textMargin = (std::abs(r) > kEpsilon)
                ? std::min(0.2 * std::abs(range), imgWidth / (2.0 * std::abs(r)))
                : 0.0;

            Base::Vector2d v1(std::cos(startangle), std::sin(startangle));
            Base::Vector2d v2(std::cos(endangle), std::sin(endangle));
            double signedTextMargin = textMargin;
            if (range < 0.0 || length1 < 0.0) {
                std::swap(v1, v2);
                signedTextMargin = -signedTextMargin;
            }

            const Base::Vector2d pnt1(p0.x + (r - endLineLength1) * v1.x,
                                      p0.y + (r - endLineLength1) * v1.y);
            const Base::Vector2d pnt2(p0.x + (r + endLineLength12) * v1.x,
                                      p0.y + (r + endLineLength12) * v1.y);
            const Base::Vector2d pnt3(p0.x + (r - endLineLength2) * v2.x,
                                      p0.y + (r - endLineLength2) * v2.y);
            const Base::Vector2d pnt4(p0.x + (r + endLineLength22) * v2.x,
                                      p0.y + (r + endLineLength22) * v2.y);
            appendSketchSegment(pnt1, pnt2);
            appendSketchSegment(pnt3, pnt4);

            appendArcSegmentsSketch(p0,
                                    std::abs(r),
                                    startangle,
                                    startangle + range / 2.0 - signedTextMargin);
            appendArcSegmentsSketch(p0,
                                    std::abs(r),
                                    startangle + range / 2.0 + signedTextMargin,
                                    endangle);

            const double arrowWidth = margin * 0.5;
            const double arrowLength = margin * 2.0;
            const Base::Vector2d startArrowBase(p0.x + r * v1.x, p0.y + r * v1.y);
            const Base::Vector2d endArrowBase(p0.x + r * v2.x, p0.y + r * v2.y);
            appendArrowSegments(startArrowBase, Base::Vector2d(v1.y, -v1.x), arrowWidth, arrowLength);
            appendArrowSegments(endArrowBase, Base::Vector2d(-v2.y, v2.x), arrowWidth, arrowLength);
            break;
        }
        case Gui::SoDatumLabel::ARCLENGTH: {
            if (pointCount < 3) {
                break;
            }

            const Base::Vector2d ctr(verts[0][0], verts[0][1]);
            const Base::Vector2d p1(verts[1][0], verts[1][1]);
            const Base::Vector2d p2(verts[2][0], verts[2][1]);
            const double labelRadius = std::max(1.0, std::abs(static_cast<double>(datum.param1.getValue())));
            const double margin = imgHeight / 3.0;
            const double startangle = static_cast<double>(datum.param2.getValue());
            const double range = static_cast<double>(datum.param3.getValue());
            const double endangle = startangle + range;
            const Base::Vector2d dir1(std::cos(startangle), std::sin(startangle));
            const Base::Vector2d dir2(std::cos(endangle), std::sin(endangle));
            const Base::Vector2d pnt2(ctr.x + dir1.x * labelRadius, ctr.y + dir1.y * labelRadius);
            const Base::Vector2d pnt4(ctr.x + dir2.x * labelRadius, ctr.y + dir2.y * labelRadius);

            appendSketchSegment(p1, pnt2);
            appendSketchSegment(p2, pnt4);
            appendArcSegmentsSketch(ctr, labelRadius, startangle, endangle);

            const double arrowLength = margin * 2.0;
            const double arrowWidth = margin * 0.5;
            const Base::Vector2d startTangent = range >= 0.0
                ? Base::Vector2d(std::sin(startangle), -std::cos(startangle))
                : Base::Vector2d(-std::sin(startangle), std::cos(startangle));
            const Base::Vector2d endTangent = range >= 0.0
                ? Base::Vector2d(-std::sin(endangle), std::cos(endangle))
                : Base::Vector2d(std::sin(endangle), -std::cos(endangle));
            appendArrowSegments(pnt2, startTangent, arrowWidth, arrowLength);
            appendArrowSegments(pnt4, endTangent, arrowWidth, arrowLength);
            break;
        }
        default:
            if (pointCount >= 2) {
                appendPreviewSegment(segments,
                                     viewProvider.projectSketchPointToScreen(Base::Vector2d(verts[0][0], verts[0][1])),
                                     viewProvider.projectSketchPointToScreen(Base::Vector2d(verts[1][0], verts[1][1])));
            }
            break;
    }
}

void appendArcSamples(std::vector<QPoint>& points,
                      const QPoint& center,
                      const QPoint& startPoint,
                      double rangeRadians)
{
    if (points.empty()) {
        return;
    }

    const double radius = std::sqrt(static_cast<double>(EditModeConstraintPreviewDetail::squaredDistance(center, startPoint)));
    if (radius < kEpsilon) {
        return;
    }

    const double startAngle = std::atan2(static_cast<double>(startPoint.y() - center.y()),
                                         static_cast<double>(startPoint.x() - center.x()));
    const int sampleCount = std::max(3, kPreviewArcSamples);
    for (int i = 1; i <= sampleCount; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(sampleCount);
        const double a = startAngle + rangeRadians * t;
        points.emplace_back(static_cast<int>(std::lround(center.x() + std::cos(a) * radius)),
                            static_cast<int>(std::lround(center.y() + std::sin(a) * radius)));
    }
}




} // namespace SketcherGui::EditModeConstraintPreviewDetail

//**************************** EditModeConstraintCoinManager class ******************************

EditModeConstraintCoinManager::EditModeConstraintCoinManager(
    ViewProviderSketch& vp,
    DrawingParameters& drawingParams,
    GeometryLayerParameters& geometryLayerParams,
    ConstraintParameters& constraintParams,
    EditModeScenegraphNodes& editModeScenegraph,
    CoinMapping& coinMap
)
    : viewProvider(vp)
    , drawingParameters(drawingParams)
    , geometryLayerParameters(geometryLayerParams)
    , constraintParameters(constraintParams)
    , editModeScenegraphNodes(editModeScenegraph)
    , coinMapping(coinMap)
{}

EditModeConstraintCoinManager::~EditModeConstraintCoinManager()
{}

void EditModeConstraintCoinManager::updateVirtualSpace()
{
    const std::vector<Sketcher::Constraint*>& constrlist
        = ViewProviderSketchCoinAttorney::getConstraints(viewProvider);

    bool isshownvirtualspace = ViewProviderSketchCoinAttorney::isShownVirtualSpace(viewProvider);

    if (constrlist.size() == vConstrType.size()) {

        editModeScenegraphNodes.constrGroup->enable.setNum(constrlist.size());

        SbBool* sws = editModeScenegraphNodes.constrGroup->enable.startEditing();

        for (size_t i = 0; i < constrlist.size(); i++) {
            sws[i] = !(constrlist[i]->isInVirtualSpace != isshownvirtualspace)
                && constrlist[i]->isVisible;  // XOR of constraint mode and VP mode
        }


        editModeScenegraphNodes.constrGroup->enable.finishEditing();
    }
}

void EditModeConstraintCoinManager::processConstraints(const GeoListFacade& geolistfacade)
{
    using std::numbers::pi;

    const auto& constrlist = ViewProviderSketchCoinAttorney::getConstraints(viewProvider);

    auto zConstrH = ViewProviderSketchCoinAttorney::getViewOrientationFactor(viewProvider)
        * drawingParameters.zConstr;
    // After an undo/redo it can happen that we have an empty geometry list but a non-empty
    // constraint list In this case just ignore the constraints. (See bug #0000421)
    if (geolistfacade.geomlist.size() <= 2 && !constrlist.empty()) {
        rebuildConstraintNodes(geolistfacade);
        return;
    }

    int extGeoCount = geolistfacade.getExternalCount();
    int intGeoCount = geolistfacade.getInternalCount();

    // reset point if the constraint type has changed
Restart:
    // check if a new constraint arrived
    if (constrlist.size() != vConstrType.size()) {
        rebuildConstraintNodes(geolistfacade);
    }

    assert(int(constrlist.size()) == editModeScenegraphNodes.constrGroup->getNumChildren());
    assert(int(vConstrType.size()) == editModeScenegraphNodes.constrGroup->getNumChildren());

    // update the virtual space
    updateVirtualSpace();

    auto getNormal =
        [](const GeoListFacade& geolistfacade, const int geoid, const Base::Vector3d& pointoncurve) {
            auto geom = geolistfacade.getGeometryFromGeoId(geoid);
            auto curve = dynamic_cast<const Part::GeomCurve*>(geom);

            auto line = dynamic_cast<const Part::GeomLineSegment*>(curve);

            if (line) {
                Base::Vector3d linedir = line->getEndPoint() - line->getStartPoint();
                return Base::Vector3d(-linedir.y, linedir.x, 0);
            }
            else {
                Base::Vector3d normal;
                try {
                    if (!(curve && curve->normalAt(pointoncurve, normal))) {
                        normal = Base::Vector3d(1, 0, 0);
                    }
                }
                catch (const Base::CADKernelError&) {
                    normal = Base::Vector3d(1, 0, 0);
                }

                return normal;
            }
        };

    // go through the constraints and update the position
    int i = 0;
    for (std::vector<Sketcher::Constraint*>::const_iterator it = constrlist.begin();
         it != constrlist.end();
         ++it, i++) {
        // check if the type has changed
        if ((*it)->Type != vConstrType[i]) {
            // clearing the type vector will force a rebuild of the visual nodes
            vConstrType.clear();
            // TODO: The 'goto' here is unsafe as it can happen that we cause an endless loop (see
            // bug #0001956).
            goto Restart;
        }
        try {  // because calculateNormalAtPoint, used in there, can throw
            // root separator for this constraint
            SoSeparator* sep = static_cast<SoSeparator*>(
                editModeScenegraphNodes.constrGroup->getChild(i)
            );
            const Constraint* Constr = *it;

            if (Constr->First < -extGeoCount || Constr->First >= intGeoCount
                || (Constr->Second != GeoEnum::GeoUndef
                    && (Constr->Second < -extGeoCount || Constr->Second >= intGeoCount))
                || (Constr->Third != GeoEnum::GeoUndef
                    && (Constr->Third < -extGeoCount || Constr->Third >= intGeoCount))) {
                // Constraint can refer to non-existent geometry during undo/redo
                continue;
            }

            // distinguish different constraint types to build up
            switch (Constr->Type) {
                case Block:
                case Horizontal:  // write the new position of the Horizontal constraint Same as
                                  // vertical position.
                case Vertical:    // write the new position of the Vertical constraint
                {
                    assert(Constr->First >= -extGeoCount && Constr->First < intGeoCount);
                    bool alignment = Constr->Type != Block && Constr->Second != GeoEnum::GeoUndef;

                    // get the geometry
                    const Part::Geometry* geo = geolistfacade.getGeometryFromGeoId(Constr->First);

                    if (!alignment) {
                        // Vertical & Horiz can only be a GeomLineSegment, but Blocked can be
                        // anything.
                        Base::Vector3d midpos;
                        Base::Vector3d dir;
                        Base::Vector3d norm;

                        if (geo->is<Part::GeomLineSegment>()) {
                            const Part::GeomLineSegment* lineSeg
                                = static_cast<const Part::GeomLineSegment*>(geo);

                            // calculate the half distance between the start and endpoint
                            midpos = ((lineSeg->getEndPoint() + lineSeg->getStartPoint()) / 2);

                            // Get a set of vectors perpendicular and tangential to these
                            dir = (lineSeg->getEndPoint() - lineSeg->getStartPoint()).Normalize();

                            norm = Base::Vector3d(-dir.y, dir.x, 0);
                        }
                        else if (geo->is<Part::GeomBSplineCurve>()) {
                            const Part::GeomBSplineCurve* bsp
                                = static_cast<const Part::GeomBSplineCurve*>(geo);
                            midpos = Base::Vector3d(0, 0, 0);

                            std::vector<Base::Vector3d> poles = bsp->getPoles();

                            // Move center of gravity towards start not to collide with bspline
                            // degree information.
                            double ws = 1.0 / poles.size();
                            double w = 1.0;

                            for (std::vector<Base::Vector3d>::iterator it = poles.begin();
                                 it != poles.end();
                                 ++it) {
                                midpos += w * (*it);
                                w -= ws;
                            }

                            midpos /= poles.size();

                            dir = (bsp->getEndPoint() - bsp->getStartPoint()).Normalize();
                            norm = Base::Vector3d(-dir.y, dir.x, 0);
                        }
                        else {
                            double ra = 0, rb = 0;
                            double angle,        // rotation of object as a whole
                                angleplus = 0.;  // arc angle (t parameter for ellipses)

                            if (geo->is<Part::GeomCircle>()) {
                                const Part::GeomCircle* circle
                                    = static_cast<const Part::GeomCircle*>(geo);
                                ra = circle->getRadius();
                                angle = pi / 4;
                                midpos = circle->getCenter();
                            }
                            else if (geo->is<Part::GeomArcOfCircle>()) {
                                const Part::GeomArcOfCircle* arc
                                    = static_cast<const Part::GeomArcOfCircle*>(geo);
                                ra = arc->getRadius();
                                double startangle, endangle;
                                arc->getRange(startangle, endangle, /*emulateCCW=*/true);
                                angle = (startangle + endangle) / 2;
                                midpos = arc->getCenter();
                            }
                            else if (geo->is<Part::GeomEllipse>()) {
                                const Part::GeomEllipse* ellipse
                                    = static_cast<const Part::GeomEllipse*>(geo);
                                ra = ellipse->getMajorRadius();
                                rb = ellipse->getMinorRadius();
                                Base::Vector3d majdir = ellipse->getMajorAxisDir();
                                angle = atan2(majdir.y, majdir.x);
                                angleplus = pi / 4;
                                midpos = ellipse->getCenter();
                            }
                            else if (geo->is<Part::GeomArcOfEllipse>()) {
                                const Part::GeomArcOfEllipse* aoe
                                    = static_cast<const Part::GeomArcOfEllipse*>(geo);
                                ra = aoe->getMajorRadius();
                                rb = aoe->getMinorRadius();
                                double startangle, endangle;
                                aoe->getRange(startangle, endangle, /*emulateCCW=*/true);
                                Base::Vector3d majdir = aoe->getMajorAxisDir();
                                angle = atan2(majdir.y, majdir.x);
                                angleplus = (startangle + endangle) / 2;
                                midpos = aoe->getCenter();
                            }
                            else if (geo->is<Part::GeomArcOfHyperbola>()) {
                                const Part::GeomArcOfHyperbola* aoh
                                    = static_cast<const Part::GeomArcOfHyperbola*>(geo);
                                ra = aoh->getMajorRadius();
                                rb = aoh->getMinorRadius();
                                double startangle, endangle;
                                aoh->getRange(startangle, endangle, /*emulateCCW=*/true);
                                Base::Vector3d majdir = aoh->getMajorAxisDir();
                                angle = atan2(majdir.y, majdir.x);
                                angleplus = (startangle + endangle) / 2;
                                midpos = aoh->getCenter();
                            }
                            else if (geo->is<Part::GeomArcOfParabola>()) {
                                const Part::GeomArcOfParabola* aop
                                    = static_cast<const Part::GeomArcOfParabola*>(geo);
                                ra = aop->getFocal();
                                double startangle, endangle;
                                aop->getRange(startangle, endangle, /*emulateCCW=*/true);
                                Base::Vector3d majdir = -aop->getXAxisDir();
                                angle = atan2(majdir.y, majdir.x);
                                angleplus = (startangle + endangle) / 2;
                                midpos = aop->getFocus();
                            }
                            else {
                                break;
                            }

                            if (geo->is<Part::GeomEllipse>() || geo->is<Part::GeomArcOfEllipse>()
                                || geo->is<Part::GeomArcOfHyperbola>()) {

                                Base::Vector3d majDir, minDir, rvec;
                                majDir = Base::Vector3d(
                                    cos(angle),
                                    sin(angle),
                                    0
                                );  // direction of major axis of ellipse
                                minDir = Base::Vector3d(
                                    -majDir.y,
                                    majDir.x,
                                    0
                                );  // direction of minor axis of ellipse
                                rvec = (ra * cos(angleplus)) * majDir
                                    + (rb * sin(angleplus)) * minDir;
                                midpos += rvec;
                                rvec.Normalize();
                                norm = rvec;
                                dir = Base::Vector3d(
                                    -rvec.y,
                                    rvec.x,
                                    0
                                );  // DeepSOIC: I'm not sure what dir is supposed to mean.
                            }
                            else {
                                norm = Base::Vector3d(cos(angle), sin(angle), 0);
                                dir = Base::Vector3d(-norm.y, norm.x, 0);
                                midpos += ra * norm;
                            }
                        }

                        Base::Vector3d relpos = seekConstraintPosition(norm, 2.5);

                        auto translation = static_cast<SoZoomTranslation*>(sep->getChild(
                            static_cast<int>(ConstraintNodePosition::FirstTranslationIndex)
                        ));

                        translation->abPos = SbVec3f(midpos.x, midpos.y, zConstrH);  // Absolute
                                                                                     // Reference

                        // Reference Position that is scaled according to zoom
                        translation->translation = SbVec3f(relpos.x, relpos.y, 0);
                    }
                    else {
                        assert(Constr->Second >= -extGeoCount && Constr->Second < intGeoCount);
                        assert(
                            Constr->FirstPos != Sketcher::PointPos::none
                            && Constr->SecondPos != Sketcher::PointPos::none
                        );

                        Base::Vector3d midpos1, dir1, norm1;
                        Base::Vector3d midpos2, dir2, norm2;

                        midpos1 = geolistfacade.getPoint(Constr->First, Constr->FirstPos);
                        midpos2 = geolistfacade.getPoint(Constr->Second, Constr->SecondPos);

                        dir1 = (midpos2 - midpos1).Normalize();
                        dir2 = -dir1;
                        norm1 = Base::Vector3d(-dir1.y, dir1.x, 0.);
                        norm2 = norm1;

                        Base::Vector3d relpos1 = seekConstraintPosition(norm1, 4.0);

                        auto translation = static_cast<SoZoomTranslation*>(sep->getChild(
                            static_cast<int>(ConstraintNodePosition::FirstTranslationIndex)
                        ));

                        translation->abPos = SbVec3f(midpos1.x, midpos1.y, zConstrH);
                        translation->translation = SbVec3f(relpos1.x, relpos1.y, 0);

                        Base::Vector3d relpos2 = seekConstraintPosition(norm2, 4.0);
                        Base::Vector3d secondPos = midpos2 - midpos1;

                        translation = static_cast<SoZoomTranslation*>(sep->getChild(
                            static_cast<int>(ConstraintNodePosition::SecondTranslationIndex)
                        ));

                        translation->abPos = SbVec3f(secondPos.x, secondPos.y, zConstrH);
                        translation->translation
                            = SbVec3f(relpos2.x - relpos1.x, relpos2.y - relpos1.y, 0);
                    }
                } break;
                case Perpendicular: {
                    assert(Constr->First >= -extGeoCount && Constr->First < intGeoCount);
                    assert(Constr->Second >= -extGeoCount && Constr->Second < intGeoCount);
                    // get the geometry
                    const Part::Geometry* geo1 = geolistfacade.getGeometryFromGeoId(Constr->First);
                    const Part::Geometry* geo2 = geolistfacade.getGeometryFromGeoId(Constr->Second);
                    Base::Vector3d midpos1, dir1, norm1;
                    Base::Vector3d midpos2, dir2, norm2;
                    bool twoIcons = false;  // a very local flag. It's set to true to indicate that
                                            // the second dir+norm are valid and should be used

                    if (Constr->Third != GeoEnum::GeoUndef ||            // perpty via point
                        Constr->FirstPos != Sketcher::PointPos::none) {  // endpoint-to-curve or
                                                                         // endpoint-to-endpoint perpty

                        int ptGeoId;
                        Sketcher::PointPos ptPosId;
                        do {  // dummy loop to use break =) Maybe goto?
                            ptGeoId = Constr->First;
                            ptPosId = Constr->FirstPos;
                            if (ptPosId != Sketcher::PointPos::none) {
                                break;
                            }
                            ptGeoId = Constr->Second;
                            ptPosId = Constr->SecondPos;
                            if (ptPosId != Sketcher::PointPos::none) {
                                break;
                            }
                            ptGeoId = Constr->Third;
                            ptPosId = Constr->ThirdPos;
                            if (ptPosId != Sketcher::PointPos::none) {
                                break;
                            }
                            assert(0);  // no point found!
                        } while (false);

                        midpos1 = geolistfacade.getPoint(ptGeoId, ptPosId);

                        norm1 = getNormal(geolistfacade, Constr->Second, midpos1);

                        // TODO: Check the method above. This was the old one making use of the
                        // solver.
                        // norm1 = getSolvedSketch().calculateNormalAtPoint(Constr->Second,
                        // midpos1.x, midpos1.y);

                        norm1.Normalize();
                        dir1 = norm1;
                        dir1.RotateZ(-pi / 2.0);
                    }
                    else if (Constr->FirstPos == Sketcher::PointPos::none) {

                        if (geo1->is<Part::GeomLineSegment>()) {
                            const Part::GeomLineSegment* lineSeg1
                                = static_cast<const Part::GeomLineSegment*>(geo1);
                            midpos1 = ((lineSeg1->getEndPoint() + lineSeg1->getStartPoint()) / 2);
                            dir1 = (lineSeg1->getEndPoint() - lineSeg1->getStartPoint()).Normalize();
                            norm1 = Base::Vector3d(-dir1.y, dir1.x, 0.);
                        }
                        else if (geo1->is<Part::GeomArcOfCircle>()) {
                            const Part::GeomArcOfCircle* arc
                                = static_cast<const Part::GeomArcOfCircle*>(geo1);
                            double startangle, endangle, midangle;
                            arc->getRange(startangle, endangle, /*emulateCCW=*/true);
                            midangle = (startangle + endangle) / 2;
                            norm1 = Base::Vector3d(cos(midangle), sin(midangle), 0);
                            dir1 = Base::Vector3d(-norm1.y, norm1.x, 0);
                            midpos1 = arc->getCenter() + arc->getRadius() * norm1;
                        }
                        else if (geo1->is<Part::GeomCircle>()) {
                            const Part::GeomCircle* circle = static_cast<const Part::GeomCircle*>(geo1);
                            norm1 = Base::Vector3d(cos(pi / 4), sin(pi / 4), 0);
                            dir1 = Base::Vector3d(-norm1.y, norm1.x, 0);
                            midpos1 = circle->getCenter() + circle->getRadius() * norm1;
                        }
                        else {
                            break;
                        }

                        if (geo2->is<Part::GeomLineSegment>()) {
                            const Part::GeomLineSegment* lineSeg2
                                = static_cast<const Part::GeomLineSegment*>(geo2);
                            midpos2 = ((lineSeg2->getEndPoint() + lineSeg2->getStartPoint()) / 2);
                            dir2 = (lineSeg2->getEndPoint() - lineSeg2->getStartPoint()).Normalize();
                            norm2 = Base::Vector3d(-dir2.y, dir2.x, 0.);
                        }
                        else if (geo2->is<Part::GeomArcOfCircle>()) {
                            const Part::GeomArcOfCircle* arc
                                = static_cast<const Part::GeomArcOfCircle*>(geo2);
                            double startangle, endangle, midangle;
                            arc->getRange(startangle, endangle, /*emulateCCW=*/true);
                            midangle = (startangle + endangle) / 2;
                            norm2 = Base::Vector3d(cos(midangle), sin(midangle), 0);
                            dir2 = Base::Vector3d(-norm2.y, norm2.x, 0);
                            midpos2 = arc->getCenter() + arc->getRadius() * norm2;
                        }
                        else if (geo2->is<Part::GeomCircle>()) {
                            const Part::GeomCircle* circle = static_cast<const Part::GeomCircle*>(geo2);
                            norm2 = Base::Vector3d(cos(pi / 4), sin(pi / 4), 0);
                            dir2 = Base::Vector3d(-norm2.y, norm2.x, 0);
                            midpos2 = circle->getCenter() + circle->getRadius() * norm2;
                        }
                        else {
                            break;
                        }
                        twoIcons = true;
                    }

                    Base::Vector3d relpos1 = seekConstraintPosition(norm1, 4.0);

                    auto translation = static_cast<SoZoomTranslation*>(
                        sep->getChild(static_cast<int>(ConstraintNodePosition::FirstTranslationIndex))
                    );

                    translation->abPos = SbVec3f(midpos1.x, midpos1.y, zConstrH);
                    translation->translation = SbVec3f(relpos1.x, relpos1.y, 0);

                    if (twoIcons) {
                        Base::Vector3d relpos2 = seekConstraintPosition(norm2, 4.0);
                        Base::Vector3d secondPos = midpos2 - midpos1;
                        auto translation = static_cast<SoZoomTranslation*>(sep->getChild(
                            static_cast<int>(ConstraintNodePosition::SecondTranslationIndex)
                        ));
                        translation->abPos = SbVec3f(secondPos.x, secondPos.y, zConstrH);
                        translation->translation
                            = SbVec3f(relpos2.x - relpos1.x, relpos2.y - relpos1.y, 0);
                    }

                } break;
                case Parallel:
                case Equal: {
                    assert(Constr->First >= -extGeoCount && Constr->First < intGeoCount);
                    assert(Constr->Second >= -extGeoCount && Constr->Second < intGeoCount);
                    // get the geometry
                    const Part::Geometry* geo1 = geolistfacade.getGeometryFromGeoId(Constr->First);
                    const Part::Geometry* geo2 = geolistfacade.getGeometryFromGeoId(Constr->Second);

                    Base::Vector3d midpos1, dir1, norm1;
                    Base::Vector3d midpos2, dir2, norm2;
                    if (!geo1->is<Part::GeomLineSegment>() || !geo2->is<Part::GeomLineSegment>()) {
                        if (Constr->Type == Equal) {
                            double r1a = 0, r1b = 0, r2a = 0, r2b = 0;
                            double angle1, angle1plus = 0., angle2,
                                           angle2plus = 0.;  // angle1 = rotation of object as a
                                                             // whole; angle1plus = arc angle (t
                                                             // parameter for ellipses).
                            if (geo1->is<Part::GeomCircle>()) {
                                const Part::GeomCircle* circle
                                    = static_cast<const Part::GeomCircle*>(geo1);
                                r1a = circle->getRadius();
                                angle1 = pi / 4;
                                midpos1 = circle->getCenter();
                            }
                            else if (geo1->is<Part::GeomArcOfCircle>()) {
                                const Part::GeomArcOfCircle* arc
                                    = static_cast<const Part::GeomArcOfCircle*>(geo1);
                                r1a = arc->getRadius();
                                double startangle, endangle;
                                arc->getRange(startangle, endangle, /*emulateCCW=*/true);
                                angle1 = (startangle + endangle) / 2;
                                midpos1 = arc->getCenter();
                            }
                            else if (geo1->is<Part::GeomEllipse>()) {
                                const Part::GeomEllipse* ellipse
                                    = static_cast<const Part::GeomEllipse*>(geo1);
                                r1a = ellipse->getMajorRadius();
                                r1b = ellipse->getMinorRadius();
                                Base::Vector3d majdir = ellipse->getMajorAxisDir();
                                angle1 = atan2(majdir.y, majdir.x);
                                angle1plus = pi / 4;
                                midpos1 = ellipse->getCenter();
                            }
                            else if (geo1->is<Part::GeomArcOfEllipse>()) {
                                const Part::GeomArcOfEllipse* aoe
                                    = static_cast<const Part::GeomArcOfEllipse*>(geo1);
                                r1a = aoe->getMajorRadius();
                                r1b = aoe->getMinorRadius();
                                double startangle, endangle;
                                aoe->getRange(startangle, endangle, /*emulateCCW=*/true);
                                Base::Vector3d majdir = aoe->getMajorAxisDir();
                                angle1 = atan2(majdir.y, majdir.x);
                                angle1plus = (startangle + endangle) / 2;
                                midpos1 = aoe->getCenter();
                            }
                            else if (geo1->is<Part::GeomArcOfHyperbola>()) {
                                const Part::GeomArcOfHyperbola* aoh
                                    = static_cast<const Part::GeomArcOfHyperbola*>(geo1);
                                r1a = aoh->getMajorRadius();
                                r1b = aoh->getMinorRadius();
                                double startangle, endangle;
                                aoh->getRange(startangle, endangle, /*emulateCCW=*/true);
                                Base::Vector3d majdir = aoh->getMajorAxisDir();
                                angle1 = atan2(majdir.y, majdir.x);
                                angle1plus = (startangle + endangle) / 2;
                                midpos1 = aoh->getCenter();
                            }
                            else if (geo1->is<Part::GeomArcOfParabola>()) {
                                const Part::GeomArcOfParabola* aop
                                    = static_cast<const Part::GeomArcOfParabola*>(geo1);
                                r1a = aop->getFocal();
                                double startangle, endangle;
                                aop->getRange(startangle, endangle, /*emulateCCW=*/true);
                                Base::Vector3d majdir = -aop->getXAxisDir();
                                angle1 = atan2(majdir.y, majdir.x);
                                angle1plus = (startangle + endangle) / 2;
                                midpos1 = aop->getFocus();
                            }
                            else {
                                break;
                            }

                            if (geo2->is<Part::GeomCircle>()) {
                                const Part::GeomCircle* circle
                                    = static_cast<const Part::GeomCircle*>(geo2);
                                r2a = circle->getRadius();
                                angle2 = pi / 4;
                                midpos2 = circle->getCenter();
                            }
                            else if (geo2->is<Part::GeomArcOfCircle>()) {
                                const Part::GeomArcOfCircle* arc
                                    = static_cast<const Part::GeomArcOfCircle*>(geo2);
                                r2a = arc->getRadius();
                                double startangle, endangle;
                                arc->getRange(startangle, endangle, /*emulateCCW=*/true);
                                angle2 = (startangle + endangle) / 2;
                                midpos2 = arc->getCenter();
                            }
                            else if (geo2->is<Part::GeomEllipse>()) {
                                const Part::GeomEllipse* ellipse
                                    = static_cast<const Part::GeomEllipse*>(geo2);
                                r2a = ellipse->getMajorRadius();
                                r2b = ellipse->getMinorRadius();
                                Base::Vector3d majdir = ellipse->getMajorAxisDir();
                                angle2 = atan2(majdir.y, majdir.x);
                                angle2plus = pi / 4;
                                midpos2 = ellipse->getCenter();
                            }
                            else if (geo2->is<Part::GeomArcOfEllipse>()) {
                                const Part::GeomArcOfEllipse* aoe
                                    = static_cast<const Part::GeomArcOfEllipse*>(geo2);
                                r2a = aoe->getMajorRadius();
                                r2b = aoe->getMinorRadius();
                                double startangle, endangle;
                                aoe->getRange(startangle, endangle, /*emulateCCW=*/true);
                                Base::Vector3d majdir = aoe->getMajorAxisDir();
                                angle2 = atan2(majdir.y, majdir.x);
                                angle2plus = (startangle + endangle) / 2;
                                midpos2 = aoe->getCenter();
                            }
                            else if (geo2->is<Part::GeomArcOfHyperbola>()) {
                                const Part::GeomArcOfHyperbola* aoh
                                    = static_cast<const Part::GeomArcOfHyperbola*>(geo2);
                                r2a = aoh->getMajorRadius();
                                r2b = aoh->getMinorRadius();
                                double startangle, endangle;
                                aoh->getRange(startangle, endangle, /*emulateCCW=*/true);
                                Base::Vector3d majdir = aoh->getMajorAxisDir();
                                angle2 = atan2(majdir.y, majdir.x);
                                angle2plus = (startangle + endangle) / 2;
                                midpos2 = aoh->getCenter();
                            }
                            else if (geo2->is<Part::GeomArcOfParabola>()) {
                                const Part::GeomArcOfParabola* aop
                                    = static_cast<const Part::GeomArcOfParabola*>(geo2);
                                r2a = aop->getFocal();
                                double startangle, endangle;
                                aop->getRange(startangle, endangle, /*emulateCCW=*/true);
                                Base::Vector3d majdir = -aop->getXAxisDir();
                                angle2 = atan2(majdir.y, majdir.x);
                                angle2plus = (startangle + endangle) / 2;
                                midpos2 = aop->getFocus();
                            }
                            else {
                                break;
                            }

                            if (geo1->is<Part::GeomEllipse>() || geo1->is<Part::GeomArcOfEllipse>()
                                || geo1->is<Part::GeomArcOfHyperbola>()) {

                                Base::Vector3d majDir, minDir, rvec;
                                majDir = Base::Vector3d(
                                    cos(angle1),
                                    sin(angle1),
                                    0
                                );  // direction of major axis of ellipse
                                minDir = Base::Vector3d(
                                    -majDir.y,
                                    majDir.x,
                                    0
                                );  // direction of minor axis of ellipse
                                rvec = (r1a * cos(angle1plus)) * majDir
                                    + (r1b * sin(angle1plus)) * minDir;
                                midpos1 += rvec;
                                rvec.Normalize();
                                norm1 = rvec;
                                dir1 = Base::Vector3d(
                                    -rvec.y,
                                    rvec.x,
                                    0
                                );  // DeepSOIC: I'm not sure what dir is supposed to mean.
                            }
                            else {
                                norm1 = Base::Vector3d(cos(angle1), sin(angle1), 0);
                                dir1 = Base::Vector3d(-norm1.y, norm1.x, 0);
                                midpos1 += r1a * norm1;
                            }


                            if (geo2->is<Part::GeomEllipse>() || geo2->is<Part::GeomArcOfEllipse>()
                                || geo2->is<Part::GeomArcOfHyperbola>()) {

                                Base::Vector3d majDir, minDir, rvec;
                                majDir = Base::Vector3d(
                                    cos(angle2),
                                    sin(angle2),
                                    0
                                );  // direction of major axis of ellipse
                                minDir = Base::Vector3d(
                                    -majDir.y,
                                    majDir.x,
                                    0
                                );  // direction of minor axis of ellipse
                                rvec = (r2a * cos(angle2plus)) * majDir
                                    + (r2b * sin(angle2plus)) * minDir;
                                midpos2 += rvec;
                                rvec.Normalize();
                                norm2 = rvec;
                                dir2 = Base::Vector3d(-rvec.y, rvec.x, 0);
                            }
                            else {
                                norm2 = Base::Vector3d(cos(angle2), sin(angle2), 0);
                                dir2 = Base::Vector3d(-norm2.y, norm2.x, 0);
                                midpos2 += r2a * norm2;
                            }
                        }
                        else {  // Parallel can only apply to a GeomLineSegment
                            break;
                        }
                    }
                    else {
                        const Part::GeomLineSegment* lineSeg1
                            = static_cast<const Part::GeomLineSegment*>(geo1);
                        const Part::GeomLineSegment* lineSeg2
                            = static_cast<const Part::GeomLineSegment*>(geo2);

                        // calculate the half distance between the start and endpoint
                        midpos1 = ((lineSeg1->getEndPoint() + lineSeg1->getStartPoint()) / 2);
                        midpos2 = ((lineSeg2->getEndPoint() + lineSeg2->getStartPoint()) / 2);
                        // Get a set of vectors perpendicular and tangential to these
                        dir1 = (lineSeg1->getEndPoint() - lineSeg1->getStartPoint()).Normalize();
                        dir2 = (lineSeg2->getEndPoint() - lineSeg2->getStartPoint()).Normalize();
                        norm1 = Base::Vector3d(-dir1.y, dir1.x, 0.);
                        norm2 = Base::Vector3d(-dir2.y, dir2.x, 0.);
                    }

                    Base::Vector3d relpos1 = seekConstraintPosition(norm1, 4.0);
                    Base::Vector3d relpos2 = seekConstraintPosition(norm2, 4.0);

                    auto translation = static_cast<SoZoomTranslation*>(
                        sep->getChild(static_cast<int>(ConstraintNodePosition::FirstTranslationIndex))
                    );

                    translation->abPos = SbVec3f(midpos1.x, midpos1.y, zConstrH);  // Absolute Reference

                    // Reference Position that is scaled according to zoom
                    translation->translation = SbVec3f(relpos1.x, relpos1.y, 0);

                    Base::Vector3d secondPos = midpos2 - midpos1;

                    translation = static_cast<SoZoomTranslation*>(sep->getChild(
                        static_cast<int>(ConstraintNodePosition::SecondTranslationIndex)
                    ));

                    translation->abPos = SbVec3f(secondPos.x, secondPos.y, zConstrH);  // Absolute
                                                                                       // Reference

                    // Reference Position that is scaled according to zoom
                    translation->translation = SbVec3f(relpos2.x - relpos1.x, relpos2.y - relpos1.y, 0);

                } break;
                case Text:
                case Group: {
                    if (Constr->isElementsEmpty()) {
                        break;  // Nothing to do if the group is empty
                    }

                    Bnd_Box totalBBox;
                    int elementIndex = 0;
                    while (Constr->hasElement(elementIndex)) {
                        auto element = Constr->getElement(elementIndex);
                        if (element.GeoId < -extGeoCount || element.GeoId >= intGeoCount) {
                            elementIndex++;
                            continue;
                        }
                        const Part::Geometry* geo = geolistfacade.getGeometryFromGeoId(element.GeoId);
                        if (!geo) {
                            elementIndex++;
                            continue;
                        }
                        TopoDS_Shape shape = geo->toShape();
                        if (!shape.IsNull()) {
                            BRepBndLib::Add(shape, totalBBox, false);
                        }
                        elementIndex++;
                    }

                    if (!totalBBox.HasFinitePart() || totalBBox.IsVoid()) {
                        // If no valid box, hide the geometry by setting all points to the origin.
                        SoCoordinate3* coords = static_cast<SoCoordinate3*>(sep->getChild(2));

                        // Use startEditing() to get a writable pointer to the internal array.
                        SbVec3f* points = coords->point.startEditing();
                        for (int j = 0; j < 5; ++j) {
                            points[j].setValue(0.0f, 0.0f, 0.0f);
                        }
                        coords->point.finishEditing();
                        break;
                    }

                    // 1. Get the original min/max points and dimensions
                    gp_Pnt min_pnt_orig = totalBBox.CornerMin();
                    gp_Pnt max_pnt_orig = totalBBox.CornerMax();
                    double width = max_pnt_orig.X() - min_pnt_orig.X();
                    double height = max_pnt_orig.Y() - min_pnt_orig.Y();

                    // 2. Calculate the offset amount
                    // Using the average of width and height is a good heuristic for a uniform
                    // offset.
                    double offset = (width + height) / 2.0 * 0.05;  // 5% of the average dimension

                    // 3. Create new, "inflated" corner points by applying the offset
                    gp_Pnt min_pnt(
                        min_pnt_orig.X() - offset,
                        min_pnt_orig.Y() - offset,
                        min_pnt_orig.Z()
                    );
                    gp_Pnt max_pnt(
                        max_pnt_orig.X() + offset,
                        max_pnt_orig.Y() + offset,
                        max_pnt_orig.Z()
                    );

                    // 4. Define the 4 corners of the rectangle using the inflated points
                    SbVec3f p0(min_pnt.X(), min_pnt.Y(), zConstrH);  // bottom-left
                    SbVec3f p1(max_pnt.X(), min_pnt.Y(), zConstrH);  // bottom-right
                    SbVec3f p2(max_pnt.X(), max_pnt.Y(), zConstrH);  // top-right
                    SbVec3f p3(min_pnt.X(), max_pnt.Y(), zConstrH);  // top-left

                    // 3. Get the SoCoordinate3 node we created in rebuildConstraintNodes
                    //    Index 0: SoMaterial, Index 1: SoDrawStyle, Index 2: SoCoordinate3
                    SoCoordinate3* coords = static_cast<SoCoordinate3*>(sep->getChild(2));

                    // 4. Update the points in the node to draw the rectangle
                    SbVec3f* points = coords->point.startEditing();
                    points[0] = p0;
                    points[1] = p1;
                    points[2] = p2;
                    points[3] = p3;
                    points[4] = p0;  // Repeat the first point to close the loop
                    coords->point.finishEditing();

                } break;
                case Distance:
                case DistanceX:
                case DistanceY: {
                    assert(Constr->First >= -extGeoCount && Constr->First < intGeoCount);

                    double helperStartAngle1 = 0.;  // for arc helpers
                    double helperStartAngle2 = 0.;
                    double helperRange1 = 0.;
                    double helperRange2 = 0.;
                    double radius1 = 0.;
                    double radius2 = 0.;
                    Base::Vector3d center1(0., 0., 0.);
                    Base::Vector3d center2(0., 0., 0.);

                    int numPoints = 2;

                    // pnt1 will be initialized to (0,0,0) if only one point is given
                    auto pnt1 = geolistfacade.getPoint(Constr->First, Constr->FirstPos);

                    Base::Vector3d pnt2(0., 0., 0.);

                    if (Constr->SecondPos != Sketcher::PointPos::none) {
                        // point to point distance
                        pnt2 = geolistfacade.getPoint(Constr->Second, Constr->SecondPos);
                    }
                    else if (Constr->Second != GeoEnum::GeoUndef) {
                        auto geo1 = geolistfacade.getGeometryFromGeoId(Constr->First);
                        auto geo2 = geolistfacade.getGeometryFromGeoId(Constr->Second);
                        if (isLineSegment(*geo2)) {
                            // NOLINTNEXTLINE
                            auto lineSeg = static_cast<const Part::GeomLineSegment*>(geo2);
                            Base::Vector3d l2p1 = lineSeg->getStartPoint();
                            Base::Vector3d l2p2 = lineSeg->getEndPoint();

                            if (Constr->FirstPos != Sketcher::PointPos::none) {
                                // point to line distance
                                // calculate the projection of p1 onto lineSeg
                                pnt2.ProjectToLine(pnt1 - l2p1, l2p2 - l2p1);
                                pnt2 += pnt1;
                            }
                            else if (isCircleOrArc(*geo1)) {
                                // circular to line distance
                                auto [radius, ct] = getRadiusCenterCircleArc(geo1);
                                // project the center on the line (translated to origin)
                                pnt1.ProjectToLine(ct - l2p1, l2p2 - l2p1);
                                Base::Vector3d dir = pnt1;
                                dir.Normalize();
                                pnt1 += ct;
                                pnt2 = ct + dir * radius;
                            }
                        }
                        else if (isCircleOrArc(*geo2)) {
                            if (Constr->FirstPos != Sketcher::PointPos::none) {
                                // point to circular distance
                                auto [rad, ct] = getRadiusCenterCircleArc(geo2);

                                Base::Vector3d v = pnt1 - ct;
                                v = v.Normalize();
                                pnt2 = ct + rad * v;
                            }
                            else if (isCircleOrArc(*geo1)) {
                                // circular to circular distance
                                GetCirclesMinimalDistance(geo1, geo2, pnt1, pnt2);
                            }
                        }
                        else {
                            break;
                        }
                    }
                    else if (Constr->FirstPos != Sketcher::PointPos::none) {
                        // one point distance
                        pnt1 = Base::Vector3d(0., 0., 0.);
                        pnt2 = geolistfacade.getPoint(Constr->First, Constr->FirstPos);
                    }
                    else if (Constr->First != GeoEnum::GeoUndef) {
                        auto geo = geolistfacade.getGeometryFromGeoId(Constr->First);
                        if (isLineSegment(*geo)) {
                            // segment distance
                            auto lineSeg = static_cast<const Part::GeomLineSegment*>(geo);
                            pnt1 = lineSeg->getStartPoint();
                            pnt2 = lineSeg->getEndPoint();
                        }
                        else if (isArcOfCircle(*geo)) {
                            // arc length
                            auto arc = static_cast<const Part::GeomArcOfCircle*>(geo);
                            int index = static_cast<int>(ConstraintNodePosition::DatumLabelIndex);
                            auto* asciiText = static_cast<Gui::SoDatumLabel*>(sep->getChild(index));
                            center1 = arc->getCenter();
                            pnt1 = arc->getStartPoint();
                            pnt2 = arc->getEndPoint();

                            double startAngle = 0.0;
                            double range = 0.0;
                            if (!SketcherGui::DimensionGeometry::resolveArcLengthDatumSweep(*arc,
                                                                                                Constr->LabelDistance,
                                                                                                startAngle,
                                                                                                range)) {
                                break;
                            }

                            asciiText->datumtype = Gui::SoDatumLabel::ARCLENGTH;
                            asciiText->param1 = Constr->LabelDistance;
                            asciiText->param2 = static_cast<float>(startAngle);
                            asciiText->param3 = static_cast<float>(range);
                            asciiText->string = SbString(
                                getPresentationString(Constr, "◠ ").toUtf8().constData()
                            );
                            asciiText->strikethrough = !Constr->isActive;

                            asciiText->pnts.setNum(3);
                            SbVec3f* verts = asciiText->pnts.startEditing();
                            verts[0] = SbVec3f(center1.x, center1.y, center1.z);
                            verts[1] = SbVec3f(pnt1.x, pnt1.y, pnt1.z);
                            verts[2] = SbVec3f(pnt2.x, pnt2.y, pnt2.z);
                            asciiText->pnts.finishEditing();
                            break;
                        }
                        else {
                            break;
                        }
                    }
                    else {
                        break;
                    }

                    int index = static_cast<int>(ConstraintNodePosition::DatumLabelIndex);
                    auto* asciiText = static_cast<Gui::SoDatumLabel*>(sep->getChild(index));  // NOLINT

                    // Get presentation string (w/o units if option is set)
                    asciiText->string = SbString(getPresentationString(Constr).toUtf8().constData());
                    asciiText->strikethrough = !Constr->isActive;

                    if (Constr->Type == Distance) {
                        asciiText->datumtype = Gui::SoDatumLabel::DISTANCE;
                    }
                    else if (Constr->Type == DistanceX) {
                        asciiText->datumtype = Gui::SoDatumLabel::DISTANCEX;
                    }
                    else if (Constr->Type == DistanceY) {
                        asciiText->datumtype = Gui::SoDatumLabel::DISTANCEY;
                    }

                    // Check if arc helpers are needed
                    if (Constr->Second != GeoEnum::GeoUndef) {
                        auto geo1 = geolistfacade.getGeometryFromGeoId(Constr->First);
                        auto geo2 = geolistfacade.getGeometryFromGeoId(Constr->Second);

                        if (isArcOfCircle(*geo1) && Constr->FirstPos == Sketcher::PointPos::none) {
                            auto arc = static_cast<const Part::GeomArcOfCircle*>(geo1);  // NOLINT
                            radius1 = arc->getRadius();
                            center1 = arc->getCenter();

                            double angle = toVector2d(
                                               isLineSegment(*geo2) ? pnt2 - center1 : pnt1 - center1
                            )
                                               .Angle();
                            double startAngle, endAngle;
                            arc->getRange(startAngle, endAngle, /*emulateCCW=*/true);

                            findHelperAngles(helperStartAngle1, helperRange1, angle, startAngle, endAngle);

                            if (helperRange1 != 0.) {
                                // We override to draw the full helper as it does not look good
                                // otherwise We still use findHelperAngles before to find if helper
                                // is needed.
                                helperStartAngle1 = endAngle;
                                helperRange1 = 2 * std::numbers::pi - (endAngle - startAngle);

                                numPoints++;
                            }
                        }
                        if (isArcOfCircle(*geo2) && Constr->SecondPos == Sketcher::PointPos::none) {
                            auto arc = static_cast<const Part::GeomArcOfCircle*>(geo2);  // NOLINT
                            radius2 = arc->getRadius();
                            center2 = arc->getCenter();

                            double angle = toVector2d(pnt2 - center2).Angle();  // between -pi and pi
                            double startAngle, endAngle;  // between 0 and 2*pi
                            arc->getRange(startAngle, endAngle, /*emulateCCW=*/true);

                            findHelperAngles(helperStartAngle2, helperRange2, angle, startAngle, endAngle);

                            if (helperRange2 != 0.) {
                                helperStartAngle2 = endAngle;
                                helperRange2 = 2 * std::numbers::pi - (endAngle - startAngle);

                                numPoints++;
                            }
                        }
                    }

                    // Assign the Datum Points
                    asciiText->pnts.setNum(numPoints);
                    SbVec3f* verts = asciiText->pnts.startEditing();

                    verts[0] = SbVec3f(pnt1.x, pnt1.y, zConstrH);
                    verts[1] = SbVec3f(pnt2.x, pnt2.y, zConstrH);

                    if (numPoints > 2) {
                        if (helperRange1 != 0.) {
                            verts[2] = SbVec3f(center1.x, center1.y, zConstrH);
                            asciiText->param3 = helperStartAngle1;
                            asciiText->param4 = helperRange1;
                            asciiText->param5 = radius1;
                        }
                        else {
                            verts[2] = SbVec3f(center2.x, center2.y, zConstrH);
                            asciiText->param3 = helperStartAngle2;
                            asciiText->param4 = helperRange2;
                            asciiText->param5 = radius2;
                        }
                        if (numPoints > 3) {
                            verts[3] = SbVec3f(center2.x, center2.y, zConstrH);
                            asciiText->param6 = helperStartAngle2;
                            asciiText->param7 = helperRange2;
                            asciiText->param8 = radius2;
                        }
                        else {
                            asciiText->param6 = 0.;
                            asciiText->param7 = 0.;
                            asciiText->param8 = 0.;
                        }
                    }
                    else {
                        asciiText->param3 = 0.;
                        asciiText->param4 = 0.;
                        asciiText->param5 = 0.;
                    }

                    asciiText->pnts.finishEditing();

                    // Assign the Label Distance
                    asciiText->param1 = Constr->LabelDistance;
                    asciiText->param2 = Constr->LabelPosition;
                } break;
                case PointOnObject:
                case Tangent:
                case SnellsLaw: {
                    assert(Constr->First >= -extGeoCount && Constr->First < intGeoCount);
                    assert(Constr->Second >= -extGeoCount && Constr->Second < intGeoCount);

                    Base::Vector3d pos, relPos;
                    if (
                        Constr->Type == PointOnObject || Constr->Type == SnellsLaw
                        || (Constr->Type == Tangent && Constr->Third != GeoEnum::GeoUndef)
                        ||  // Tangency via point
                        (Constr->Type == Tangent
                         && Constr->FirstPos != Sketcher::PointPos::none)  // endpoint-to-curve or
                                                                           // endpoint-to-endpoint
                                                                           // tangency
                    ) {

                        // find the point of tangency/point that is on object
                        // just any point among first/second/third should be OK
                        int ptGeoId;
                        Sketcher::PointPos ptPosId;
                        do {  // dummy loop to use break =) Maybe goto?
                            ptGeoId = Constr->First;
                            ptPosId = Constr->FirstPos;
                            if (ptPosId != Sketcher::PointPos::none) {
                                break;
                            }
                            ptGeoId = Constr->Second;
                            ptPosId = Constr->SecondPos;
                            if (ptPosId != Sketcher::PointPos::none) {
                                break;
                            }
                            ptGeoId = Constr->Third;
                            ptPosId = Constr->ThirdPos;
                            if (ptPosId != Sketcher::PointPos::none) {
                                break;
                            }
                            assert(0);  // no point found!
                        } while (false);

                        pos = geolistfacade.getPoint(ptGeoId, ptPosId);
                        auto norm = getNormal(geolistfacade, Constr->Second, pos);

                        // TODO: Check substitution
                        // Base::Vector3d norm =
                        // getSolvedSketch().calculateNormalAtPoint(Constr->Second, pos.x, pos.y);
                        norm.Normalize();
                        Base::Vector3d dir = norm;
                        dir.RotateZ(-pi / 2.0);

                        relPos = seekConstraintPosition(norm, 2.5);

                        auto translation = static_cast<SoZoomTranslation*>(sep->getChild(
                            static_cast<int>(ConstraintNodePosition::FirstTranslationIndex)
                        ));

                        translation->abPos = SbVec3f(pos.x, pos.y, zConstrH);  // Absolute Reference
                        translation->translation = SbVec3f(relPos.x, relPos.y, 0);
                    }
                    else if (Constr->Type == Tangent) {
                        // get the geometry
                        const Part::Geometry* geo1 = geolistfacade.getGeometryFromGeoId(Constr->First);
                        const Part::Geometry* geo2 = geolistfacade.getGeometryFromGeoId(Constr->Second);

                        if (geo1->is<Part::GeomLineSegment>() && geo2->is<Part::GeomLineSegment>()) {
                            const Part::GeomLineSegment* lineSeg1
                                = static_cast<const Part::GeomLineSegment*>(geo1);
                            const Part::GeomLineSegment* lineSeg2
                                = static_cast<const Part::GeomLineSegment*>(geo2);
                            // tangency between two lines
                            Base::Vector3d midpos1
                                = ((lineSeg1->getEndPoint() + lineSeg1->getStartPoint()) / 2);
                            Base::Vector3d midpos2
                                = ((lineSeg2->getEndPoint() + lineSeg2->getStartPoint()) / 2);
                            Base::Vector3d dir1
                                = (lineSeg1->getEndPoint() - lineSeg1->getStartPoint()).Normalize();
                            Base::Vector3d dir2
                                = (lineSeg2->getEndPoint() - lineSeg2->getStartPoint()).Normalize();
                            Base::Vector3d norm1 = Base::Vector3d(-dir1.y, dir1.x, 0.f);
                            Base::Vector3d norm2 = Base::Vector3d(-dir2.y, dir2.x, 0.f);

                            Base::Vector3d relpos1 = seekConstraintPosition(norm1, 4.0);
                            Base::Vector3d relpos2 = seekConstraintPosition(norm2, 4.0);

                            auto translation = static_cast<SoZoomTranslation*>(sep->getChild(
                                static_cast<int>(ConstraintNodePosition::FirstTranslationIndex)
                            ));

                            translation->abPos = SbVec3f(midpos1.x, midpos1.y, zConstrH);  // Absolute
                                                                                           // Reference
                            translation->translation = SbVec3f(relpos1.x, relpos1.y, 0);

                            Base::Vector3d secondPos = midpos2 - midpos1;

                            translation = static_cast<SoZoomTranslation*>(sep->getChild(
                                static_cast<int>(ConstraintNodePosition::SecondTranslationIndex)
                            ));

                            translation->abPos = SbVec3f(
                                secondPos.x,
                                secondPos.y,
                                zConstrH
                            );  // Absolute Reference
                            translation->translation
                                = SbVec3f(relpos2.x - relpos1.x, relpos2.y - relpos1.y, 0);

                            break;
                        }
                        else if (geo2->is<Part::GeomLineSegment>()) {
                            std::swap(geo1, geo2);
                        }

                        if (geo1->is<Part::GeomLineSegment>()) {
                            const Part::GeomLineSegment* lineSeg
                                = static_cast<const Part::GeomLineSegment*>(geo1);
                            Base::Vector3d dir
                                = (lineSeg->getEndPoint() - lineSeg->getStartPoint()).Normalize();
                            Base::Vector3d norm(-dir.y, dir.x, 0);
                            if (geo2->is<Part::GeomCircle>()) {
                                const Part::GeomCircle* circle
                                    = static_cast<const Part::GeomCircle*>(geo2);
                                // tangency between a line and a circle
                                float length = (circle->getCenter() - lineSeg->getStartPoint()) * dir;

                                pos = lineSeg->getStartPoint() + dir * length;
                                relPos = norm * 1;  // TODO Huh?
                            }
                            else if (
                                geo2->is<Part::GeomEllipse>() || geo2->is<Part::GeomArcOfEllipse>()
                            ) {

                                Base::Vector3d center;
                                if (geo2->is<Part::GeomEllipse>()) {
                                    const Part::GeomEllipse* ellipse
                                        = static_cast<const Part::GeomEllipse*>(geo2);
                                    center = ellipse->getCenter();
                                }
                                else {
                                    const Part::GeomArcOfEllipse* aoc
                                        = static_cast<const Part::GeomArcOfEllipse*>(geo2);
                                    center = aoc->getCenter();
                                }

                                // tangency between a line and an ellipse
                                float length = (center - lineSeg->getStartPoint()) * dir;

                                pos = lineSeg->getStartPoint() + dir * length;
                                relPos = norm * 1;
                            }
                            else if (geo2->is<Part::GeomArcOfCircle>()) {
                                const Part::GeomArcOfCircle* arc
                                    = static_cast<const Part::GeomArcOfCircle*>(geo2);
                                // tangency between a line and an arc
                                float length = (arc->getCenter() - lineSeg->getStartPoint()) * dir;

                                pos = lineSeg->getStartPoint() + dir * length;
                                relPos = norm * 1;  // TODO Huh?
                            }
                        }

                        if (geo1->is<Part::GeomCircle>() && geo2->is<Part::GeomCircle>()) {
                            const Part::GeomCircle* circle1 = static_cast<const Part::GeomCircle*>(
                                geo1
                            );
                            const Part::GeomCircle* circle2 = static_cast<const Part::GeomCircle*>(
                                geo2
                            );
                            // tangency between two circles
                            Base::Vector3d dir
                                = (circle2->getCenter() - circle1->getCenter()).Normalize();
                            pos = circle1->getCenter() + dir * circle1->getRadius();
                            relPos = dir * 1;
                        }
                        else if (geo2->is<Part::GeomCircle>()) {
                            std::swap(geo1, geo2);
                        }

                        if (geo1->is<Part::GeomCircle>() && geo2->is<Part::GeomArcOfCircle>()) {
                            const Part::GeomCircle* circle = static_cast<const Part::GeomCircle*>(geo1);
                            const Part::GeomArcOfCircle* arc
                                = static_cast<const Part::GeomArcOfCircle*>(geo2);
                            // tangency between a circle and an arc
                            Base::Vector3d dir = (arc->getCenter() - circle->getCenter()).Normalize();
                            pos = circle->getCenter() + dir * circle->getRadius();
                            relPos = dir * 1;
                        }
                        else if (
                            geo1->is<Part::GeomArcOfCircle>() && geo2->is<Part::GeomArcOfCircle>()
                        ) {
                            const Part::GeomArcOfCircle* arc1
                                = static_cast<const Part::GeomArcOfCircle*>(geo1);
                            const Part::GeomArcOfCircle* arc2
                                = static_cast<const Part::GeomArcOfCircle*>(geo2);
                            // tangency between two arcs
                            Base::Vector3d dir = (arc2->getCenter() - arc1->getCenter()).Normalize();
                            pos = arc1->getCenter() + dir * arc1->getRadius();
                            relPos = dir * 1;
                        }
                        auto translation = static_cast<SoZoomTranslation*>(sep->getChild(
                            static_cast<int>(ConstraintNodePosition::FirstTranslationIndex)
                        ));

                        translation->abPos = SbVec3f(pos.x, pos.y, zConstrH);  // Absolute Reference
                        translation->translation = SbVec3f(relPos.x, relPos.y, 0);
                    }
                } break;
                case Symmetric: {
                    assert(Constr->First >= -extGeoCount && Constr->First < intGeoCount);
                    assert(Constr->Second >= -extGeoCount && Constr->Second < intGeoCount);

                    Base::Vector3d pnt1 = geolistfacade.getPoint(Constr->First, Constr->FirstPos);
                    Base::Vector3d pnt2 = geolistfacade.getPoint(Constr->Second, Constr->SecondPos);

                    SbVec3f p1(pnt1.x, pnt1.y, zConstrH);
                    SbVec3f p2(pnt2.x, pnt2.y, zConstrH);
                    SbVec3f dir = (p2 - p1);
                    dir.normalize();
                    SbVec3f norm(-dir[1], dir[0], 0);

                    Gui::SoDatumLabel* asciiText = static_cast<Gui::SoDatumLabel*>(
                        sep->getChild(static_cast<int>(ConstraintNodePosition::DatumLabelIndex))
                    );
                    asciiText->datumtype = Gui::SoDatumLabel::SYMMETRIC;

                    asciiText->pnts.setNum(2);
                    SbVec3f* verts = asciiText->pnts.startEditing();

                    verts[0] = p1;
                    verts[1] = p2;

                    asciiText->pnts.finishEditing();

                    auto translation = static_cast<SoTranslation*>(
                        sep->getChild(static_cast<int>(ConstraintNodePosition::FirstTranslationIndex))
                    );

                    translation->translation = (p1 + p2) / 2;
                } break;
                case Angle: {
                    assert(Constr->First >= -extGeoCount && Constr->First < intGeoCount);
                    assert(
                        (Constr->Second >= -extGeoCount && Constr->Second < intGeoCount)
                        || Constr->Second == GeoEnum::GeoUndef
                    );

                    SbVec3f p0;
                    double distance = Constr->LabelDistance;
                    double startangle, range;
                    double endLineLength1 = 0.0;
                    double endLineLength2 = 0.0;
                    if (Constr->Second != GeoEnum::GeoUndef) {
                        Base::Vector3d dir1, dir2;
                        if (Constr->Third == GeoEnum::GeoUndef) {  // angle between two lines
                            const Part::Geometry* geo1 = geolistfacade.getGeometryFromGeoId(
                                Constr->First
                            );
                            const Part::Geometry* geo2 = geolistfacade.getGeometryFromGeoId(
                                Constr->Second
                            );
                            if (!isLineSegment(*geo1) || !isLineSegment(*geo2)) {
                                break;
                            }
                            auto* line1 = static_cast<const Part::GeomLineSegment*>(geo1);
                            auto* line2 = static_cast<const Part::GeomLineSegment*>(geo2);

                            bool flip1 = (Constr->FirstPos == PointPos::end);
                            bool flip2 = (Constr->SecondPos == PointPos::end);
                            dir1 = (flip1 ? -1. : 1.)
                                * (line1->getEndPoint() - line1->getStartPoint()).Normalize();
                            dir2 = (flip2 ? -1. : 1.)
                                * (line2->getEndPoint() - line2->getStartPoint()).Normalize();
                            Base::Vector3d pnt1 = flip1 ? line1->getEndPoint()
                                                        : line1->getStartPoint();
                            Base::Vector3d pnt2 = flip2 ? line2->getEndPoint()
                                                        : line2->getStartPoint();
                            Base::Vector3d pnt12 = flip1 ? line1->getStartPoint()
                                                         : line1->getEndPoint();
                            Base::Vector3d pnt22 = flip2 ? line2->getStartPoint()
                                                         : line2->getEndPoint();

                            // line-line intersection
                            Base::Vector3d intersection;
                            {
                                double det = dir1.x * dir2.y - dir1.y * dir2.x;
                                if ((det > 0 ? det : -det) < 1e-10) {
                                    // lines are coincident (or parallel) and in this case the
                                    // center of the point pairs with the shortest distance is
                                    // used
                                    Base::Vector3d p1[2], p2[2];
                                    p1[0] = line1->getStartPoint();
                                    p1[1] = line1->getEndPoint();
                                    p2[0] = line2->getStartPoint();
                                    p2[1] = line2->getEndPoint();
                                    double length = std::numeric_limits<double>::max();
                                    for (int i = 0; i <= 1; i++) {
                                        for (int j = 0; j <= 1; j++) {
                                            double tmp = (p2[j] - p1[i]).Length();
                                            if (tmp < length) {
                                                length = tmp;
                                                double x = (p2[j].x + p1[i].x) / 2;
                                                double y = (p2[j].y + p1[i].y) / 2;
                                                intersection = Base::Vector3d(x, y, 0.);
                                            }
                                        }
                                    }
                                }
                                else {
                                    double c1 = dir1.y * pnt1.x - dir1.x * pnt1.y;
                                    double c2 = dir2.y * pnt2.x - dir2.x * pnt2.y;
                                    double x = (dir1.x * c2 - dir2.x * c1) / det;
                                    double y = (dir1.y * c2 - dir2.y * c1) / det;
                                    intersection = Base::Vector3d(x, y, 0.);
                                }
                            }
                            p0.setValue(intersection.x, intersection.y, 0.);

                            range = Constr->getValue();  // WYSIWYG
                            startangle = atan2(dir1.y, dir1.x);
                            Base::Vector3d vl1 = dir1 * 2 * distance - (pnt1 - intersection);
                            Base::Vector3d vl2 = dir2 * 2 * distance - (pnt2 - intersection);
                            Base::Vector3d vl12 = dir1 * 2 * distance - (pnt12 - intersection);
                            Base::Vector3d vl22 = dir2 * 2 * distance - (pnt22 - intersection);

                            endLineLength1 = vl12.Dot(dir1) > 0 ? vl12.Length()
                                : vl1.Dot(dir1) < 0             ? -vl1.Length()
                                                                : 0.0;
                            endLineLength2 = vl22.Dot(dir2) > 0 ? vl22.Length()
                                : vl2.Dot(dir2) < 0             ? -vl2.Length()
                                                                : 0.0;
                        }
                        else {  // angle-via-point
                            Base::Vector3d p = geolistfacade.getPoint(Constr->Third, Constr->ThirdPos);
                            p0 = SbVec3f(p.x, p.y, 0);
                            dir1 = getNormal(geolistfacade, Constr->First, p);
                            // TODO: Check
                            // dir1 = getSolvedSketch().calculateNormalAtPoint(Constr->First,
                            // p.x, p.y);
                            dir1.RotateZ(-pi / 2);  // convert to vector of tangency by rotating
                            dir2 = getNormal(geolistfacade, Constr->Second, p);
                            // TODO: Check
                            // dir2 = getSolvedSketch().calculateNormalAtPoint(Constr->Second,
                            // p.x, p.y);
                            dir2.RotateZ(-pi / 2);

                            startangle = atan2(dir1.y, dir1.x);
                            range = atan2(
                                dir1.x * dir2.y - dir1.y * dir2.x,
                                dir1.x * dir2.x + dir1.y * dir2.y
                            );
                        }
                    }
                    else if (Constr->First != GeoEnum::GeoUndef) {
                        const Part::Geometry* geo = geolistfacade.getGeometryFromGeoId(Constr->First);
                        if (geo->is<Part::GeomLineSegment>()) {
                            auto* lineSeg = static_cast<const Part::GeomLineSegment*>(geo);
                            p0 = Base::convertTo<SbVec3f>(
                                (lineSeg->getEndPoint() + lineSeg->getStartPoint()) / 2
                            );
                            double l1 = 2 * distance
                                - (lineSeg->getEndPoint() - lineSeg->getStartPoint()).Length() / 2;
                            endLineLength1 = 2 * distance;
                            endLineLength2 = l1 > 0. ? l1 : 0.;

                            Base::Vector3d dir = lineSeg->getEndPoint() - lineSeg->getStartPoint();
                            startangle = 0.;
                            range = atan2(dir.y, dir.x);
                        }
                        else if (geo->is<Part::GeomArcOfCircle>()) {
                            auto* arc = static_cast<const Part::GeomArcOfCircle*>(geo);
                            p0 = Base::convertTo<SbVec3f>(arc->getCenter());

                            endLineLength1 = 2 * distance - arc->getRadius();
                            endLineLength2 = endLineLength1;

                            double endangle;
                            arc->getRange(startangle, endangle, /*emulateCCWXY=*/true);
                            range = endangle - startangle;
                        }
                        else {
                            break;
                        }
                    }
                    else {
                        break;
                    }

                    Gui::SoDatumLabel* asciiText = static_cast<Gui::SoDatumLabel*>(
                        sep->getChild(static_cast<int>(ConstraintNodePosition::DatumLabelIndex))
                    );
                    asciiText->string = SbString(getPresentationString(Constr).toUtf8().constData());
                    asciiText->strikethrough = !Constr->isActive;
                    asciiText->datumtype = Gui::SoDatumLabel::ANGLE;
                    asciiText->param1 = distance;
                    asciiText->param2 = startangle;
                    asciiText->param3 = range;
                    asciiText->param4 = endLineLength1;
                    asciiText->param5 = endLineLength2;

                    asciiText->pnts.setNum(2);
                    SbVec3f* verts = asciiText->pnts.startEditing();

                    verts[0] = p0;

                    asciiText->pnts.finishEditing();

                } break;
                case Diameter: {
                    assert(Constr->First >= -extGeoCount && Constr->First < intGeoCount);

                    Base::Vector3d pnt1(0., 0., 0.), pnt2(0., 0., 0.);
                    double startHelperAngle = 0.;
                    double startHelperRange = 0.;
                    double endHelperAngle = 0.;
                    double endHelperRange = 0.;

                    if (Constr->First == GeoEnum::GeoUndef) {
                        break;
                    }

                    const Part::Geometry* geo = geolistfacade.getGeometryFromGeoId(Constr->First);

                    if (geo->is<Part::GeomArcOfCircle>()) {
                        auto* arc = static_cast<const Part::GeomArcOfCircle*>(geo);
                        double radius = arc->getRadius();
                        double angle = (double)Constr->LabelPosition;  // between -pi and pi
                        double startAngle, endAngle;                   // between 0 and 2*pi
                        arc->getRange(startAngle, endAngle, /*emulateCCW=*/true);

                        if (angle == 10) {
                            angle = (startAngle + endAngle) / 2;
                        }

                        findHelperAngles(startHelperAngle, startHelperRange, angle, startAngle, endAngle);

                        findHelperAngles(endHelperAngle, endHelperRange, angle + pi, startAngle, endAngle);

                        Base::Vector3d center = arc->getCenter();
                        pnt1 = center - radius * Base::Vector3d(cos(angle), sin(angle), 0.);
                        pnt2 = center + radius * Base::Vector3d(cos(angle), sin(angle), 0.);
                    }
                    else if (geo->is<Part::GeomCircle>()) {
                        auto* circle = static_cast<const Part::GeomCircle*>(geo);
                        double radius = circle->getRadius();
                        double angle = (double)Constr->LabelPosition;
                        if (angle == 10) {
                            angle = 0;
                        }
                        Base::Vector3d center = circle->getCenter();
                        pnt1 = center - radius * Base::Vector3d(cos(angle), sin(angle), 0.);
                        pnt2 = center + radius * Base::Vector3d(cos(angle), sin(angle), 0.);
                    }
                    else {
                        break;
                    }

                    SbVec3f p1(pnt1.x, pnt1.y, zConstrH);
                    SbVec3f p2(pnt2.x, pnt2.y, zConstrH);

                    Gui::SoDatumLabel* asciiText = static_cast<Gui::SoDatumLabel*>(
                        sep->getChild(static_cast<int>(ConstraintNodePosition::DatumLabelIndex))
                    );

                    // Get display string with units hidden if so requested
                    asciiText->string = SbString(
                        getPresentationString(Constr, "⌀").toUtf8().constData()
                    );
                    asciiText->strikethrough = !Constr->isActive;

                    asciiText->datumtype = Gui::SoDatumLabel::DIAMETER;
                    asciiText->param1 = Constr->LabelDistance;
                    asciiText->param2 = Constr->LabelPosition;
                    asciiText->param3 = static_cast<float>(startHelperAngle);
                    asciiText->param4 = static_cast<float>(startHelperRange);
                    asciiText->param5 = static_cast<float>(endHelperAngle);
                    asciiText->param6 = static_cast<float>(endHelperRange);

                    asciiText->pnts.setNum(2);
                    SbVec3f* verts = asciiText->pnts.startEditing();

                    verts[0] = p1;
                    verts[1] = p2;

                    asciiText->pnts.finishEditing();
                } break;
                case Weight:
                case Radius: {
                    assert(Constr->First >= -extGeoCount && Constr->First < intGeoCount);

                    Base::Vector3d pnt1(0., 0., 0.), pnt2(0., 0., 0.);
                    double helperStartAngle = 0.;
                    double helperRange = 0.;

                    if (Constr->First == GeoEnum::GeoUndef) {
                        break;
                    }
                    const Part::Geometry* geo = geolistfacade.getGeometryFromGeoId(Constr->First);

                    if (geo->is<Part::GeomArcOfCircle>()) {
                        auto* arc = static_cast<const Part::GeomArcOfCircle*>(geo);
                        double radius = arc->getRadius();
                        double angle = (double)Constr->LabelPosition;  // between -pi and pi
                        double startAngle, endAngle;                   // between 0 and 2*pi
                        arc->getRange(startAngle, endAngle, /*emulateCCW=*/true);

                        if (angle == 10) {
                            angle = (startAngle + endAngle) / 2;
                        }

                        findHelperAngles(helperStartAngle, helperRange, angle, startAngle, endAngle);

                        pnt1 = arc->getCenter();
                        pnt2 = pnt1 + radius * Base::Vector3d(cos(angle), sin(angle), 0.);
                    }
                    else if (geo->is<Part::GeomCircle>()) {
                        auto* circle = static_cast<const Part::GeomCircle*>(geo);
                        auto gf = GeometryFacade::getFacade(geo);

                        double radius;

                        radius = circle->getRadius();

                        double angle = (double)Constr->LabelPosition;
                        if (angle == 10) {
                            angle = 0;
                        }
                        pnt1 = circle->getCenter();
                        pnt2 = pnt1 + radius * Base::Vector3d(cos(angle), sin(angle), 0.);
                    }
                    else {
                        break;
                    }

                    SbVec3f p1(pnt1.x, pnt1.y, zConstrH);
                    SbVec3f p2(pnt2.x, pnt2.y, zConstrH);

                    Gui::SoDatumLabel* asciiText = static_cast<Gui::SoDatumLabel*>(
                        sep->getChild(static_cast<int>(ConstraintNodePosition::DatumLabelIndex))
                    );

                    // Get display string with units hidden if so requested
                    if (Constr->Type == Weight) {
                        asciiText->string = SbString(
                            QString::number(Constr->getValue()).toStdString().c_str()
                        );
                    }
                    else {
                        asciiText->string = SbString(
                            getPresentationString(Constr, "R").toUtf8().constData()
                        );
                        asciiText->strikethrough = !Constr->isActive;
                    }

                    asciiText->datumtype = Gui::SoDatumLabel::RADIUS;
                    asciiText->param1 = Constr->LabelDistance;
                    asciiText->param2 = Constr->LabelPosition;
                    asciiText->param3 = helperStartAngle;
                    asciiText->param4 = helperRange;

                    asciiText->pnts.setNum(2);
                    SbVec3f* verts = asciiText->pnts.startEditing();

                    verts[0] = p1;
                    verts[1] = p2;

                    asciiText->pnts.finishEditing();
                } break;
                case Coincident:  // nothing to do for coincident
                case None:
                case InternalAlignment:
                case NumConstraintTypes:
                    break;
            }
        }
        catch (Base::Exception& e) {
            Base::Console().developerError(
                "EditModeConstraintCoinManager",
                "Exception during draw: %s\n",
                e.what()
            );
            e.reportException();
        }
        catch (...) {
            Base::Console().developerError(
                "EditModeConstraintCoinManager",
                "Exception during draw: unknown\n"
            );
        }
    }
}

void EditModeConstraintCoinManager::findHelperAngles(
    double& helperStartAngle,
    double& helperRange,
    double angle,
    double startAngle,
    double endAngle
) const
{
    using std::numbers::pi;

    double margin = 0.2;  // about 10deg
    if (angle < 0) {
        angle = angle + 2 * pi;
    }
    // endAngle can be more than 2*pi as its startAngle + arcAngle
    if (endAngle > 2 * pi && angle < endAngle - 2 * pi) {
        angle = angle + 2 * pi;
    }
    if (!(angle > startAngle && angle < endAngle)) {
        if ((angle < startAngle && startAngle - angle < angle + 2 * pi - endAngle)
            || (angle > endAngle && startAngle + 2 * pi - angle < angle - endAngle)) {
            if (angle > startAngle) {
                angle -= 2 * pi;
            }
            helperStartAngle = angle - margin;
            helperRange = startAngle - angle + margin;
        }
        else {
            if (angle < endAngle) {
                angle += 2 * pi;
            }
            helperStartAngle = endAngle;
            helperRange = angle - endAngle + margin;
        }
    }
}

Base::Vector3d EditModeConstraintCoinManager::seekConstraintPosition(
    const Base::Vector3d& norm,
    float step
)
{
    return norm * 0.5f * step;
}

void EditModeConstraintCoinManager::updateConstraintColor(
    const std::vector<Sketcher::Constraint*>& constraints
)
{
    // Because coincident constraints are selected using the point color, we need to edit the point
    // materials.

    std::vector<int> PtNum;
    std::vector<SbColor*> pcolor;  // point color
    std::vector<std::vector<int>> CurvNum;
    std::vector<std::vector<SbColor*>> color;  // curve color

    for (int l = 0; l < geometryLayerParameters.getCoinLayerCount(); l++) {
        PtNum.push_back(editModeScenegraphNodes.PointsMaterials[l]->diffuseColor.getNum());
        pcolor.push_back(editModeScenegraphNodes.PointsMaterials[l]->diffuseColor.startEditing());
        CurvNum.emplace_back();
        color.emplace_back();
        for (int t = 0; t < geometryLayerParameters.getSubLayerCount(); t++) {
            CurvNum[l].push_back(editModeScenegraphNodes.CurvesMaterials[l][t]->diffuseColor.getNum());
            color[l].push_back(
                editModeScenegraphNodes.CurvesMaterials[l][t]->diffuseColor.startEditing()
            );
        }
    }

    int maxNumberOfConstraints = std::min(
        editModeScenegraphNodes.constrGroup->getNumChildren(),
        static_cast<int>(constraints.size())
    );

    // colors of the constraints
    for (int i = 0; i < maxNumberOfConstraints; i++) {
        SoSeparator* s = static_cast<SoSeparator*>(editModeScenegraphNodes.constrGroup->getChild(i));

        // Check Constraint Type
        Sketcher::Constraint* constraint = constraints[i];
        ConstraintType type = constraint->Type;

        // It may happen that color updates are triggered by programmatic selection changes before a
        // command final update. Then constraints may have been changed and the color will be
        // updated as part
        if (type != vConstrType[i]) {
            break;
        }

        bool hasDatumLabel
            = (type == Sketcher::Angle || type == Sketcher::Radius || type == Sketcher::Diameter
               || type == Sketcher::Weight || type == Sketcher::Symmetric || type == Sketcher::Distance
               || type == Sketcher::DistanceX || type == Sketcher::DistanceY);

        // Non DatumLabel Nodes will have a material excluding coincident
        bool hasMaterial = false;

        SoMaterial* m = nullptr;
        if (!hasDatumLabel && type != Sketcher::Coincident && type != Sketcher::InternalAlignment) {
            int matIndex = static_cast<int>(ConstraintNodePosition::MaterialIndex);
            if (matIndex < s->getNumChildren()) {
                hasMaterial = true;
                m = static_cast<SoMaterial*>(s->getChild(matIndex));
            }
        }

        auto selectpoint = [this, pcolor, PtNum](int geoid, Sketcher::PointPos pos) {
            if (geoid >= 0) {
                auto multifieldIndex = coinMapping.getIndexLayer(geoid, pos);

                if (multifieldIndex != MultiFieldId::Invalid) {
                    int index = multifieldIndex.fieldIndex;
                    int layer = multifieldIndex.layerId;
                    if (layer < static_cast<int>(PtNum.size()) && index >= 0 && index < PtNum[layer]) {
                        pcolor[layer][index] = drawingParameters.SelectColor;
                    }
                }
            }
        };

        auto selectline = [this, color, CurvNum](int geoid) {
            if (geoid >= 0) {
                auto multifieldIndex = coinMapping.getIndexLayer(geoid, Sketcher::PointPos::none);

                if (multifieldIndex != MultiFieldId::Invalid) {
                    int index = multifieldIndex.fieldIndex;
                    int layer = multifieldIndex.layerId;
                    int sublayer = multifieldIndex.geoTypeId;
                    if (layer < static_cast<int>(CurvNum.size())
                        && sublayer < static_cast<int>(CurvNum[layer].size()) && index >= 0
                        && index < CurvNum[layer][sublayer]) {
                        color[layer][sublayer][index] = drawingParameters.SelectColor;
                    }
                }
            }
        };


        if (ViewProviderSketchCoinAttorney::isConstraintSelected(viewProvider, i)) {
            if (hasDatumLabel) {
                Gui::SoDatumLabel* l = static_cast<Gui::SoDatumLabel*>(
                    s->getChild(static_cast<int>(ConstraintNodePosition::DatumLabelIndex))
                );
                l->textColor = drawingParameters.SelectColor;
            }
            else if (hasMaterial) {
                m->diffuseColor = drawingParameters.SelectColor;
            }
            else if (type == Sketcher::Coincident) {
                selectpoint(constraint->First, constraint->FirstPos);
                selectpoint(constraint->Second, constraint->SecondPos);
            }
            else if (type == Sketcher::InternalAlignment) {
                switch (constraint->AlignmentType) {
                    case EllipseMajorDiameter:
                    case EllipseMinorDiameter:
                    case HyperbolaMajor:
                    case HyperbolaMinor:
                    case ParabolaFocalAxis: {
                        selectline(constraint->First);
                    } break;
                    case EllipseFocus1:
                    case EllipseFocus2:
                    case HyperbolaFocus:
                    case ParabolaFocus:
                    case BSplineControlPoint:
                    case BSplineKnotPoint: {
                        selectpoint(constraint->First, constraint->FirstPos);
                    } break;
                    default:
                        break;
                }
            }
        }
        else if (ViewProviderSketchCoinAttorney::isConstraintPreselected(viewProvider, i)) {
            if (hasDatumLabel) {
                Gui::SoDatumLabel* l = static_cast<Gui::SoDatumLabel*>(
                    s->getChild(static_cast<int>(ConstraintNodePosition::DatumLabelIndex))
                );
                l->textColor = drawingParameters.PreselectColor;
            }
            else if (hasMaterial) {
                m->diffuseColor = drawingParameters.PreselectColor;
            }
        }
        else {
            bool isActive = ViewProviderSketchCoinAttorney::isConstraintActiveInSketch(
                viewProvider,
                constraint
            );
            if (hasDatumLabel) {
                Gui::SoDatumLabel* l = static_cast<Gui::SoDatumLabel*>(
                    s->getChild(static_cast<int>(ConstraintNodePosition::DatumLabelIndex))
                );

                l->textColor = isActive
                    ? ViewProviderSketchCoinAttorney::constraintHasExpression(viewProvider, i)
                        ? drawingParameters.ExprBasedConstrDimColor
                        : (constraint->isDriving ? drawingParameters.ConstrDimColor
                                                 : drawingParameters.NonDrivingConstrDimColor)
                    : drawingParameters.DeactivatedConstrDimColor;
            }
            else if (hasMaterial) {
                m->diffuseColor = isActive
                    ? (constraint->isDriving ? drawingParameters.ConstrDimColor
                                             : drawingParameters.NonDrivingConstrDimColor)
                    : drawingParameters.DeactivatedConstrDimColor;
            }
        }
    }

    for (int l = 0; l < geometryLayerParameters.getCoinLayerCount(); l++) {
        editModeScenegraphNodes.PointsMaterials[l]->diffuseColor.finishEditing();
        for (int t = 0; t < geometryLayerParameters.getSubLayerCount(); t++) {
            editModeScenegraphNodes.CurvesMaterials[l][t]->diffuseColor.finishEditing();
        }
    }
}

void EditModeConstraintCoinManager::rebuildConstraintNodes()
{
    auto geolistfacade = ViewProviderSketchCoinAttorney::getGeoListFacade(viewProvider);

    rebuildConstraintNodes(geolistfacade);
}

void EditModeConstraintCoinManager::setConstraintSelectability(bool enabled /* = true */)
{
    if (enabled) {
        editModeScenegraphNodes.constrGrpSelect->style.setValue(SoPickStyle::SHAPE);
    }
    else {
        editModeScenegraphNodes.constrGrpSelect->style.setValue(SoPickStyle::UNPICKABLE);
    }
}

void EditModeConstraintCoinManager::rebuildConstraintNodes(const GeoListFacade& geolistfacade)
{
    const std::vector<Sketcher::Constraint*>& constrlist
        = ViewProviderSketchCoinAttorney::getConstraints(viewProvider);

    // clean up
    Gui::coinRemoveAllChildren(editModeScenegraphNodes.constrGroup);

    vConstrType.clear();

    // Get sketch normal
    Base::Vector3d RN(0, 0, 1);

    // move to position of Sketch
    Base::Placement Plz = ViewProviderSketchCoinAttorney::getEditingPlacement(viewProvider);
    Base::Rotation tmp(Plz.getRotation());
    tmp.multVec(RN, RN);
    Plz.setRotation(tmp);

    SbVec3f norm(RN.x, RN.y, RN.z);

    rebuildConstraintNodes(geolistfacade, constrlist, norm);
}

void EditModeConstraintCoinManager::rebuildConstraintNodes(
    const GeoListFacade& geolistfacade,
    const std::vector<Sketcher::Constraint*> constrlist,
    SbVec3f norm
)
{

    for (std::vector<Sketcher::Constraint*>::const_iterator it = constrlist.begin();
         it != constrlist.end();
         ++it) {
        // root separator for one constraint
        SoSeparator* sep = new SoSeparator();
        sep->ref();
        // no caching for frequently-changing data structures
        sep->renderCaching = SoSeparator::OFF;

        // every constrained visual node gets its own material for preselection and selection
        SoMaterial* mat = new SoMaterial;
        mat->ref();
        bool isActive = ViewProviderSketchCoinAttorney::isConstraintActiveInSketch(viewProvider, *it);
        mat->diffuseColor = isActive
            ? ((*it)->isDriving ? drawingParameters.ConstrDimColor
                                : drawingParameters.NonDrivingConstrDimColor)
            : drawingParameters.DeactivatedConstrDimColor;


        // distinguish different constraint types to build up
        switch ((*it)->Type) {
            case Distance:
            case DistanceX:
            case DistanceY:
            case Radius:
            case Diameter:
            case Weight:
            case Angle: {
                Gui::SoDatumLabel* text = new Gui::SoDatumLabel();
                text->norm.setValue(norm);
                text->string = "";
                text->textColor = isActive
                    ? ((*it)->isDriving ? drawingParameters.ConstrDimColor
                                        : drawingParameters.NonDrivingConstrDimColor)
                    : drawingParameters.DeactivatedConstrDimColor;
                text->size.setValue(drawingParameters.labelFontSize);
                text->lineWidth = 2 * drawingParameters.pixelScalingFactor;
                text->useAntialiasing = false;
                sep->addChild(text);
                editModeScenegraphNodes.constrGroup->addChild(sep);
                vConstrType.push_back((*it)->Type);
                // nodes not needed
                sep->unref();
                mat->unref();
                continue;  // jump to next constraint
            } break;
            case Horizontal:
            case Vertical:
            case Block: {
                // #define CONSTRAINT_SEPARATOR_INDEX_MATERIAL_OR_DATUMLABEL 0
                sep->addChild(mat);
                // #define CONSTRAINT_SEPARATOR_INDEX_FIRST_TRANSLATION 1
                sep->addChild(new SoZoomTranslation());
                // #define CONSTRAINT_SEPARATOR_INDEX_FIRST_ICON 2
                sep->addChild(new SoImage());
                // #define CONSTRAINT_SEPARATOR_INDEX_FIRST_CONSTRAINTID 3
                sep->addChild(new SoInfo());
                // #define CONSTRAINT_SEPARATOR_INDEX_SECOND_TRANSLATION 4
                sep->addChild(new SoZoomTranslation());
                // #define CONSTRAINT_SEPARATOR_INDEX_SECOND_ICON 5
                sep->addChild(new SoImage());
                // #define CONSTRAINT_SEPARATOR_INDEX_SECOND_CONSTRAINTID 6
                sep->addChild(new SoInfo());

                // remember the type of this constraint node
                vConstrType.push_back((*it)->Type);
            } break;
            case Group:
            case Text: {
                // For a group, we will draw a dashed rectangle.
                // We need a Material, a DrawStyle, Coordinates, and a LineSet.

                // 1. Material (for color, re-using the one already created)
                sep->addChild(mat);

                // 2. DrawStyle (to make the line dashed)
                SoDrawStyle* drawStyle = new SoDrawStyle();
                drawStyle->linePattern = 0x0F0F;  // A standard 50% dashed pattern
                sep->addChild(drawStyle);

                // 3. Coordinates (for the 4 corners + 1 to close the loop)
                SoCoordinate3* coords = new SoCoordinate3();
                coords->point.setNum(5);  // Pre-allocate 5 points for a closed rectangle
                sep->addChild(coords);

                // 4. LineSet (to connect the coordinates)
                SoLineSet* lineSet = new SoLineSet();
                lineSet->numVertices.set1Value(0, 5);  // A single polyline of 5 vertices
                sep->addChild(lineSet);

                vConstrType.push_back((*it)->Type);
            } break;
            case Coincident:  // no visual for coincident so far
                vConstrType.push_back(Coincident);
                break;
            case Parallel:
            case Perpendicular:
            case Equal: {
                // #define CONSTRAINT_SEPARATOR_INDEX_MATERIAL_OR_DATUMLABEL 0
                sep->addChild(mat);
                // #define CONSTRAINT_SEPARATOR_INDEX_FIRST_TRANSLATION 1
                sep->addChild(new SoZoomTranslation());
                // #define CONSTRAINT_SEPARATOR_INDEX_FIRST_ICON 2
                sep->addChild(new SoImage());
                // #define CONSTRAINT_SEPARATOR_INDEX_FIRST_CONSTRAINTID 3
                sep->addChild(new SoInfo());
                // #define CONSTRAINT_SEPARATOR_INDEX_SECOND_TRANSLATION 4
                sep->addChild(new SoZoomTranslation());
                // #define CONSTRAINT_SEPARATOR_INDEX_SECOND_ICON 5
                sep->addChild(new SoImage());
                // #define CONSTRAINT_SEPARATOR_INDEX_SECOND_CONSTRAINTID 6
                sep->addChild(new SoInfo());

                // remember the type of this constraint node
                vConstrType.push_back((*it)->Type);
            } break;
            case PointOnObject:
            case Tangent:
            case SnellsLaw: {
                // #define CONSTRAINT_SEPARATOR_INDEX_MATERIAL_OR_DATUMLABEL 0
                sep->addChild(mat);
                // #define CONSTRAINT_SEPARATOR_INDEX_FIRST_TRANSLATION 1
                sep->addChild(new SoZoomTranslation());
                // #define CONSTRAINT_SEPARATOR_INDEX_FIRST_ICON 2
                sep->addChild(new SoImage());
                // #define CONSTRAINT_SEPARATOR_INDEX_FIRST_CONSTRAINTID 3
                sep->addChild(new SoInfo());

                if ((*it)->Type == Tangent) {
                    const Part::Geometry* geo1 = geolistfacade.getGeometryFromGeoId((*it)->First);
                    const Part::Geometry* geo2 = geolistfacade.getGeometryFromGeoId((*it)->Second);
                    if (!geo1 || !geo2) {
                        Base::Console().developerWarning(
                            "EditModeConstraintCoinManager",
                            "Tangent constraint references non-existing geometry\n"
                        );
                    }
                    else if (geo1->is<Part::GeomLineSegment>() && geo2->is<Part::GeomLineSegment>()) {
                        // #define CONSTRAINT_SEPARATOR_INDEX_SECOND_TRANSLATION 4
                        sep->addChild(new SoZoomTranslation());
                        // #define CONSTRAINT_SEPARATOR_INDEX_SECOND_ICON 5
                        sep->addChild(new SoImage());
                        // #define CONSTRAINT_SEPARATOR_INDEX_SECOND_CONSTRAINTID 6
                        sep->addChild(new SoInfo());
                    }
                }

                vConstrType.push_back((*it)->Type);
            } break;
            case Symmetric: {
                Gui::SoDatumLabel* arrows = new Gui::SoDatumLabel();
                arrows->norm.setValue(norm);
                arrows->string = "";
                arrows->textColor = drawingParameters.ConstrDimColor;
                arrows->lineWidth = 2 * drawingParameters.pixelScalingFactor;

                // #define CONSTRAINT_SEPARATOR_INDEX_MATERIAL_OR_DATUMLABEL 0
                sep->addChild(arrows);
                // #define CONSTRAINT_SEPARATOR_INDEX_FIRST_TRANSLATION 1
                sep->addChild(new SoTranslation());
                // #define CONSTRAINT_SEPARATOR_INDEX_FIRST_ICON 2
                sep->addChild(new SoImage());
                // #define CONSTRAINT_SEPARATOR_INDEX_FIRST_CONSTRAINTID 3
                sep->addChild(new SoInfo());

                vConstrType.push_back((*it)->Type);
            } break;
            case InternalAlignment: {
                vConstrType.push_back((*it)->Type);
            } break;
            default:
                vConstrType.push_back((*it)->Type);
        }

        editModeScenegraphNodes.constrGroup->addChild(sep);
        // decrement ref counter again
        sep->unref();
        mat->unref();
    }
}

QString EditModeConstraintCoinManager::getPresentationString(
    const Constraint* constraint,
    std::string prefix
)
{
    /**
     * Hide units if
     *  - user has requested it,
     *  - is being displayed in the base units, -and-
     *  - the schema being used has a clear base unit in the first place.
     *
     * Remove unit string if expected unit string matches actual unit string
     * Example code from: Mod/TechDraw/App/DrawViewDimension.cpp:372
     *
     * Hide the default length unit
     */
    auto fixValueStr = [&](const QString& valueStr, const auto& unitStr) -> std::optional<QString> {
        if (!constraintParameters.bHideUnits || constraint->Type == Sketcher::Angle) {
            return std::nullopt;
        }

        const auto baseUnitStr {Base::UnitsApi::getBasicLengthUnit()};
        if (baseUnitStr.empty() || baseUnitStr != unitStr) {
            return std::nullopt;
        }

        // trailing space or non-dig
        const QRegularExpression rxUnits {QString::fromUtf8(" \\D*$")};
        auto vStr = valueStr;
        vStr.remove(rxUnits);
        return {vStr};
    };

    // Get the current value string including units.
    //
    // Smart-dimension preview must use the same visible precision as the datum dialog,
    // otherwise the preview text can be much longer than the value that will actually be
    // accepted when the user simply presses OK. That creates visual drift between preview
    // and committed constraint, especially for radius/diameter labels.
    double factor {};
    std::string unitStr;  // the actual unit string
    auto presentationValue = constraint->getPresentationValue();
    auto quantityFormat = presentationValue.getFormat();
    quantityFormat.setPrecision(std::max(0, Base::UnitsApi::getDecimals()));
    presentationValue.setFormat(quantityFormat);
    const auto constrPresValue {presentationValue.getUserString(factor, unitStr)};
    auto valueStr = QString::fromStdString(constrPresValue);

    auto normalizeScientificNotation = [](const QString& input, int decimals) -> QString {
        static const QRegularExpression rx {
            QString::fromUtf8(R"(^\s*([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)(.*)$)")
        };

        const auto match = rx.match(input);
        if (!match.hasMatch()) {
            return input;
        }

        const QString numericPart = match.captured(1);
        if (!numericPart.contains(QRegularExpression(QString::fromUtf8("[eE]")))) {
            return input;
        }

        bool ok = false;
        const double numericValue = QLocale::c().toDouble(numericPart, &ok);
        if (!ok) {
            return input;
        }

        QString formatted = QString::number(numericValue, 'f', std::max(0, decimals));
        formatted.remove(QRegularExpression(QString::fromUtf8("0+$")));
        formatted.remove(QRegularExpression(QString::fromUtf8("\\.$")));
        return formatted + match.captured(2);
    };

    valueStr = normalizeScientificNotation(valueStr, Base::UnitsApi::getDecimals());
    auto fixedValueStr = fixValueStr(valueStr, unitStr).value_or(valueStr);
    if (!prefix.empty()) {
        fixedValueStr.prepend(QString::fromStdString(prefix));
    }

    if (constraintParameters.bShowDimensionalName && !constraint->Name.empty()) {
        /**
         * Create the representation string from the user defined format string
         * Format options are:
         * %N - the constraint name parameter
         * %V - the value of the dimensional constraint, including any unit characters
         */
        auto sDimFmt {constraintParameters.sDimensionalStringFormat};
        if (!sDimFmt.contains(QLatin1String("%V"))
            && !sDimFmt.contains(QLatin1String("%N"))) {  // using default format "%N = %V"

            fixedValueStr = QString::fromStdString(constraint->Name) + QString::fromLatin1(" = ")
                + fixedValueStr;
        }
        else {
            sDimFmt.replace(QLatin1String("%N"), QString::fromStdString(constraint->Name));
            sDimFmt.replace(QLatin1String("%V"), fixedValueStr);
            fixedValueStr = sDimFmt;
        }
    }

    int constraintIndex = -1;
    const auto& constrlist = ViewProviderSketchCoinAttorney::getConstraints(viewProvider);
    auto it = std::find(constrlist.begin(), constrlist.end(), constraint);
    if (it != constrlist.end()) {
        constraintIndex = std::distance(constrlist.begin(), it);
        if (ViewProviderSketchCoinAttorney::constraintHasExpression(viewProvider, constraintIndex)) {
            fixedValueStr += QStringLiteral(" (ƒ𝑥)");
        }
    }

    if (!constraint->isDriving) {
        fixedValueStr = QStringLiteral("(") + fixedValueStr + QStringLiteral(")");
    }

    return fixedValueStr;
}

std::set<int> EditModeConstraintCoinManager::detectPreselectionConstr(const SoPickedPoint* Point)
{
    std::set<int> constrIndices;
    SoPath* path = Point->getPath();

    // The picked node must be a child of the main constraint group.
    SoNode* tailFather2 = path->getNode(path->getLength() - 3);
    if (tailFather2 != editModeScenegraphNodes.constrGroup) {
        return constrIndices;
    }

    SoNode* tail = path->getTail();  // This is the SoImage or SoDatumLabel node that was picked.
    SoSeparator* sep = static_cast<SoSeparator*>(path->getNode(path->getLength() - 2));

    for (int childIdx = 0; childIdx < sep->getNumChildren(); ++childIdx) {
        if (tail == sep->getChild(childIdx) && dynamic_cast<SoImage*>(tail)) {
            // The SoInfo node with the ID always follows the SoImage node.
            if (childIdx + 1 < sep->getNumChildren()) {
                SoInfo* constrIdsNode = dynamic_cast<SoInfo*>(sep->getChild(childIdx + 1));
                if (!constrIdsNode) {
                    continue;
                }

                QString constrIdsStr = QString::fromLatin1(
                    constrIdsNode->string.getValue().getString()
                );

                if (combinedConstrBoxes.count(constrIdsStr)) {

                    // 1. Get the icon group size in device independent pixels.
                    SbVec3s iconGroupSize = getDisplayedSize(static_cast<SoImage*>(tail));

                    // 2. Get the icon group's absolute world position from its
                    // SoZoomTranslation node.
                    SoZoomTranslation* translation = nullptr;
                    SoNode* firstTransNode = sep->getChild(
                        static_cast<int>(ConstraintNodePosition::FirstTranslationIndex)
                    );
                    if (dynamic_cast<SoZoomTranslation*>(firstTransNode)) {
                        translation = static_cast<SoZoomTranslation*>(firstTransNode);
                    }
                    if (!translation) {
                        continue;
                    }

                    SbVec3f absPos = translation->abPos.getValue();
                    SbVec3f trans = translation->translation.getValue();
                    float scaleFactor = translation->getScaleFactor();

                    // If this is the second icon in a pair, add its relative translation.
                    if (int secondIndex = static_cast<int>(ConstraintNodePosition::SecondIconIndex);
                        secondIndex < sep->getNumChildren()) {
                        SoNode* secondIconNode = sep->getChild(secondIndex);
                        if (tail == secondIconNode) {
                            auto translation2 = static_cast<SoZoomTranslation*>(sep->getChild(
                                static_cast<int>(ConstraintNodePosition::SecondTranslationIndex)
                            ));
                            absPos += translation2->abPos.getValue();
                            trans += translation2->translation.getValue();
                            scaleFactor = translation2->getScaleFactor();
                        }
                    }

                    // 3. Calculate the icon's center in world coordinates.
                    SbVec3f iconGroupWorldPos = absPos + scaleFactor * trans;

                    // 4. Project both the icon's center and the picked point to screen coordinates
                    // (device independent pixels). This is the key: both points are now in the same
                    // coordinate system.
                    SbVec2f iconGroupScreenCenter = ViewProviderSketchCoinAttorney::getScreenCoordinates(
                        viewProvider,
                        SbVec2f(iconGroupWorldPos[0], iconGroupWorldPos[1])
                    );

                    SbVec2f cursorScreenPos = ViewProviderSketchCoinAttorney::getScreenCoordinates(
                        viewProvider,
                        SbVec2f(Point->getPoint()[0], Point->getPoint()[1])
                    );

                    // 5. Calculate cursor position relative to the icon group's top-left corner.
                    //    - QRect/QImage assumes a top-left origin (Y increases downwards).
                    //    - Coin3D screen coordinates have a bottom-left origin (Y increases
                    //    upwards).
                    //    - We must flip the Y-axis for the check.
                    int relativeX = static_cast<int>(
                        cursorScreenPos[0] - iconGroupScreenCenter[0] + iconGroupSize[0] / 2.0f
                    );
                    int relativeY = static_cast<int>(
                        iconGroupScreenCenter[1] - cursorScreenPos[1] + iconGroupSize[1] / 2.0f
                    );

                    // 6. Perform the hit test on each icon in the group.
                    for (const auto& boxInfo : combinedConstrBoxes[constrIdsStr]) {
                        if (boxInfo.first.contains(relativeX, relativeY)) {
                            constrIndices.insert(boxInfo.second.begin(), boxInfo.second.end());
                        }
                    }
                }
                else {
                    // Simple, non-merged icon.
                    QStringList constrIdStrings = constrIdsStr.split(QStringLiteral(","));
                    for (const QString& id : constrIdStrings) {
                        constrIndices.insert(id.toInt());
                    }
                }

                return constrIndices;
            }
        }
    }

    // Handle selection of datum labels (e.g., radius, distance dimensions).
    if (dynamic_cast<Gui::SoDatumLabel*>(tail)) {
        for (int i = 0; i < editModeScenegraphNodes.constrGroup->getNumChildren(); ++i) {
            if (editModeScenegraphNodes.constrGroup->getChild(i) == sep) {
                constrIndices.insert(i);
                break;
            }
        }
    }

    return constrIndices;
}

SbVec3s EditModeConstraintCoinManager::getDisplayedSize(const SoImage* iconPtr) const
{
    SbVec3s iconSize = iconPtr->image.getValue().getSize();

    if (iconPtr->width.getValue() != -1) {
        iconSize[0] = iconPtr->width.getValue();
    }
    if (iconPtr->height.getValue() != -1) {
        iconSize[1] = iconPtr->height.getValue();
    }
    return iconSize;
}

// public function that triggers drawing of most constraint icons
void EditModeConstraintCoinManager::drawConstraintIcons()
{
    auto geolistfacade = ViewProviderSketchCoinAttorney::getGeoListFacade(viewProvider);

    drawConstraintIcons(geolistfacade);
}

void EditModeConstraintCoinManager::drawConstraintIcons(const GeoListFacade& geolistfacade)
{
    const std::vector<Sketcher::Constraint*>& constraints
        = ViewProviderSketchCoinAttorney::getConstraints(viewProvider);

    std::vector<constrIconQueueItem> iconQueue;

    int maxNumberOfConstraints = std::min(
        editModeScenegraphNodes.constrGroup->getNumChildren(),
        static_cast<int>(constraints.size())
    );

    for (int constrId = 0; constrId < maxNumberOfConstraints; ++constrId) {
        Sketcher::Constraint* constraint = constraints[constrId];

        // Check if Icon Should be created
        bool multipleIcons = false;

        QString icoType = iconTypeFromConstraint(constraint);
        if (icoType.isEmpty()) {
            continue;
        }

        if (constraint->Type != vConstrType[constrId]) {
            break;
        }

        switch (constraint->Type) {

            case Tangent: {  // second icon is available only for collinear line segments
                const Part::Geometry* geo1 = geolistfacade.getGeometryFromGeoId(constraint->First);
                const Part::Geometry* geo2 = geolistfacade.getGeometryFromGeoId(constraint->Second);
                if (geo1 && geo1->is<Part::GeomLineSegment>() && geo2
                    && geo2->is<Part::GeomLineSegment>()) {
                    multipleIcons = true;
                }
            } break;
            case Horizontal:
            case Vertical: {  // second icon is available only for point alignment
                if (constraint->Second != GeoEnum::GeoUndef
                    && constraint->FirstPos != Sketcher::PointPos::none
                    && constraint->SecondPos != Sketcher::PointPos::none) {
                    multipleIcons = true;
                }
            } break;
            case Parallel:
                multipleIcons = true;
                break;
            case Perpendicular:
                // second icon is available only when there is no common point
                if (constraint->FirstPos == Sketcher::PointPos::none
                    && constraint->Third == GeoEnum::GeoUndef) {
                    multipleIcons = true;
                }
                break;
            case Equal:
                multipleIcons = true;
                break;
            default:
                break;
        }

        // Double-check that we can safely access the Inventor nodes
        if (constrId >= editModeScenegraphNodes.constrGroup->getNumChildren()) {
            Base::Console().developerWarning(
                "EditModeConstraintManager",
                "Can't update constraint icons because view is not in sync with sketch\n"
            );
            break;
        }

        // Find the Constraint Icon SoImage Node
        SoSeparator* sep = static_cast<SoSeparator*>(
            editModeScenegraphNodes.constrGroup->getChild(constrId)
        );
        int numChildren = sep->getNumChildren();

        SbVec3f absPos;
        // Somewhat hacky - we use SoZoomTranslations for most types of icon,
        // but symmetry icons use SoTranslations...
        SoTranslation* translationPtr = static_cast<SoTranslation*>(
            sep->getChild(static_cast<int>(ConstraintNodePosition::FirstTranslationIndex))
        );

        if (dynamic_cast<SoZoomTranslation*>(translationPtr)) {
            absPos = static_cast<SoZoomTranslation*>(translationPtr)->abPos.getValue();
        }
        else {
            absPos = translationPtr->translation.getValue();
        }

        SoImage* coinIconPtr = dynamic_cast<SoImage*>(
            sep->getChild(static_cast<int>(ConstraintNodePosition::FirstIconIndex))
        );
        SoInfo* infoPtr = static_cast<SoInfo*>(
            sep->getChild(static_cast<int>(ConstraintNodePosition::FirstConstraintIdIndex))
        );

        constrIconQueueItem thisIcon;
        thisIcon.type = icoType;
        thisIcon.constraintId = constrId;
        thisIcon.position = absPos;
        thisIcon.destination = coinIconPtr;
        thisIcon.infoPtr = infoPtr;
        thisIcon.visible = (constraint->isInVirtualSpace
                            == ViewProviderSketchCoinAttorney::isShownVirtualSpace(viewProvider))
            && constraint->isVisible;

        if (constraint->Type == Symmetric) {
            Base::Vector3d startingpoint
                = geolistfacade.getPoint(constraint->First, constraint->FirstPos);
            Base::Vector3d endpoint = geolistfacade.getPoint(constraint->Second, constraint->SecondPos);

            SbVec3f pos0(startingpoint.x, startingpoint.y, startingpoint.z);
            SbVec3f pos1(endpoint.x, endpoint.y, endpoint.z);

            thisIcon.iconRotation
                = ViewProviderSketchCoinAttorney::getRotation(viewProvider, pos0, pos1);
        }
        else {
            thisIcon.iconRotation = 0;
        }

        if (multipleIcons) {
            if (constraint->Name.empty()) {
                thisIcon.label = QString::number(constrId + 1);
            }
            else {
                thisIcon.label = QString::fromUtf8(constraint->Name.c_str());
            }
            iconQueue.push_back(thisIcon);

            // Note that the second translation is meant to be applied after the first.
            // So, to get the position of the second icon, we add the two translations together
            //
            // See note ~30 lines up.
            if (numChildren > static_cast<int>(ConstraintNodePosition::SecondConstraintIdIndex)) {
                translationPtr = static_cast<SoTranslation*>(
                    sep->getChild(static_cast<int>(ConstraintNodePosition::SecondTranslationIndex))
                );
                if (dynamic_cast<SoZoomTranslation*>(translationPtr)) {
                    thisIcon.position
                        += static_cast<SoZoomTranslation*>(translationPtr)->abPos.getValue();
                }
                else {
                    thisIcon.position += translationPtr->translation.getValue();
                }

                thisIcon.destination = dynamic_cast<SoImage*>(
                    sep->getChild(static_cast<int>(ConstraintNodePosition::SecondIconIndex))
                );
                thisIcon.infoPtr = static_cast<SoInfo*>(
                    sep->getChild(static_cast<int>(ConstraintNodePosition::SecondConstraintIdIndex))
                );
            }
        }
        else {
            if (constraint->Name.empty()) {
                thisIcon.label = QString();
            }
            else {
                thisIcon.label = QString::fromUtf8(constraint->Name.c_str());
            }
        }

        iconQueue.push_back(thisIcon);
    }

    combineConstraintIcons(iconQueue);
}

void EditModeConstraintCoinManager::combineConstraintIcons(IconQueue iconQueue)
{
    // getScaleFactor gives us a ratio of pixels per some kind of real units
    float scale = ViewProviderSketchCoinAttorney::getScaleFactor(viewProvider);
    float maxDistSquared = pow(scale, 2);

    // There's room for optimisation here; we could reuse the combined icons...
    combinedConstrBoxes.clear();

    // Grid size needs to be slightly larger than the max merge distance to ensure
    // we catch neighbors.
    float gridSize = std::max(1.0f, std::abs(scale) * 1.1f);

    // 2. FILTERING & PREPARATION
    // We create a list of valid indices.
    std::vector<int> validIndices;
    validIndices.reserve(iconQueue.size());

    for (size_t i = 0; i < iconQueue.size(); ++i) {
        if (!iconQueue[i].visible) {
            clearCoinImage(iconQueue[i].destination);
            continue;
        }

        // Symmetric constraints render alone (original logic)
        if (iconQueue[i].type == QStringLiteral("Constraint_Symmetric")) {
            drawTypicalConstraintIcon(iconQueue[i]);
            continue;
        }

        validIndices.push_back(static_cast<int>(i));
    }

    // 3. SPATIAL HASHING (The Grid)
    // Map: Pair(GridX, GridY) -> List of indices in iconQueue
    std::map<std::pair<int, int>, std::vector<int>> grid;

    for (int idx : validIndices) {
        const auto& icon = iconQueue[idx];
        int gx = static_cast<int>(std::floor(icon.position[0] / gridSize));
        int gy = static_cast<int>(std::floor(icon.position[1] / gridSize));
        grid[{gx, gy}].push_back(idx);
    }

    // 4. CLUSTERING (Reversed Iteration)
    std::vector<bool> processed(iconQueue.size(), false);

    for (auto it = validIndices.rbegin(); it != validIndices.rend(); ++it) {
        int startIdx = *it;
        if (processed[startIdx]) {
            continue;
        }

        // Start a new group with the "Anchor"
        IconQueue thisGroup;
        thisGroup.reserve(5);

        // Stack for recursive search (Chain reaction)
        std::stack<int> searchStack;
        searchStack.push(startIdx);
        processed[startIdx] = true;

        while (!searchStack.empty()) {
            int currentIdx = searchStack.top();
            searchStack.pop();
            thisGroup.push_back(iconQueue[currentIdx]);

            // Check neighbors of current icon
            const auto& currentIcon = iconQueue[currentIdx];
            int cx = static_cast<int>(std::floor(currentIcon.position[0] / gridSize));
            int cy = static_cast<int>(std::floor(currentIcon.position[1] / gridSize));

            // Check 3x3 grid neighborhood
            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    auto gridIt = grid.find({cx + dx, cy + dy});
                    if (gridIt == grid.end()) {
                        continue;
                    }

                    // Iterate over potential matches in this cell
                    for (int neighborIdx : gridIt->second) {
                        if (processed[neighborIdx]) {
                            continue;
                        }

                        const auto& otherIcon = iconQueue[neighborIdx];

                        float distSq = pow(currentIcon.position[0] - otherIcon.position[0], 2)
                            + pow(currentIcon.position[1] - otherIcon.position[1], 2);

                        if (distSq <= maxDistSquared) {
                            processed[neighborIdx] = true;
                            searchStack.push(neighborIdx);
                        }
                    }
                }
            }
        }

        // 5. DRAW
        if (thisGroup.size() == 1) {
            drawTypicalConstraintIcon(thisGroup[0]);
        }
        else {
            drawMergedConstraintIcons(thisGroup);
        }
    }
}

void EditModeConstraintCoinManager::drawMergedConstraintIcons(IconQueue iconQueue)
{
    for (IconQueue::iterator i = iconQueue.begin(); i != iconQueue.end(); ++i) {
        clearCoinImage(i->destination);
    }

    QImage compositeIcon;
    SoImage* thisDest = iconQueue[0].destination;
    SoInfo* thisInfo = iconQueue[0].infoPtr;

    // Tracks all constraint IDs that are combined into this icon
    QString idString;
    int lastVPad = 0;

    QStringList labels;
    std::vector<int> ids;
    QString thisType;
    QColor iconColor;
    QList<QColor> labelColors;
    int maxColorPriority;
    double iconRotation;

    ConstrIconBBVec boundingBoxes;
    while (!iconQueue.empty()) {
        IconQueue::iterator i = iconQueue.begin();

        labels.clear();
        labels.append(i->label);

        ids.clear();
        ids.push_back(i->constraintId);

        thisType = i->type;
        iconColor = constrColor(i->constraintId);
        labelColors.clear();
        labelColors.append(iconColor);
        iconRotation = i->iconRotation;

        maxColorPriority = constrColorPriority(i->constraintId);

        if (idString.length()) {
            idString.append(QStringLiteral(","));
        }
        idString.append(QString::number(i->constraintId));

        i = iconQueue.erase(i);
        while (i != iconQueue.end()) {
            if (i->type != thisType) {
                ++i;
                continue;
            }

            labels.append(i->label);
            ids.push_back(i->constraintId);
            labelColors.append(constrColor(i->constraintId));

            if (constrColorPriority(i->constraintId) > maxColorPriority) {
                maxColorPriority = constrColorPriority(i->constraintId);
                iconColor = constrColor(i->constraintId);
            }

            idString.append(QStringLiteral(",") + QString::number(i->constraintId));

            i = iconQueue.erase(i);
        }

        // To be inserted into edit->combinedConstBoxes
        std::vector<QRect> boundingBoxesVec;
        int oldHeight = 0;

        // Render the icon here.
        if (compositeIcon.isNull()) {
            compositeIcon = renderConstrIcon(
                thisType,
                iconColor,
                labels,
                labelColors,
                iconRotation,
                &boundingBoxesVec,
                &lastVPad
            );
        }
        else {
            int thisVPad;
            QImage partialIcon = renderConstrIcon(
                thisType,
                iconColor,
                labels,
                labelColors,
                iconRotation,
                &boundingBoxesVec,
                &thisVPad
            );

            // Stack vertically for now.  Down the road, it might make sense
            // to figure out the best orientation automatically.
            oldHeight = compositeIcon.height();

            // This is overkill for the currently used (20 July 2014) font,
            // since it always seems to have the same vertical pad, but this
            // might not always be the case.  The 3 pixel buffer might need
            // to vary depending on font size too...
            oldHeight -= std::max(lastVPad - 3, 0);

            compositeIcon = compositeIcon.copy(
                0,
                0,
                std::max(partialIcon.width(), compositeIcon.width()),
                partialIcon.height() + compositeIcon.height()
            );

            QPainter qp(&compositeIcon);
            qp.drawImage(0, oldHeight, partialIcon);

            lastVPad = thisVPad;
        }

        // Add bounding boxes for the icon we just rendered to boundingBoxes
        std::vector<int>::iterator id = ids.begin();
        std::set<int> nextIds;
        for (std::vector<QRect>::iterator bb = boundingBoxesVec.begin(); bb != boundingBoxesVec.end();
             ++bb) {
            nextIds.clear();

            if (bb == boundingBoxesVec.begin()) {
                // The first bounding box is for the icon at left, so assign
                // all IDs for that type of constraint to the icon.
                for (std::vector<int>::iterator j = ids.begin(); j != ids.end(); ++j) {
                    nextIds.insert(*j);
                }
            }
            else {
                nextIds.insert(*(id++));
            }

            ConstrIconBB newBB(bb->adjusted(0, oldHeight, 0, oldHeight), nextIds);

            boundingBoxes.push_back(newBB);
        }
    }

    combinedConstrBoxes[idString] = boundingBoxes;
    thisInfo->string.setValue(idString.toLatin1().data());
    sendConstraintIconToCoin(compositeIcon, thisDest);
}


/// Note: labels, labelColors, and boundingBoxes are all
/// assumed to be the same length.
QImage EditModeConstraintCoinManager::renderConstrIcon(
    const QString& type,
    const QColor& iconColor,
    const QStringList& labels,
    const QList<QColor>& labelColors,
    double iconRotation,
    std::vector<QRect>* boundingBoxes,
    int* vPad
)
{
    // Constants to help create constraint icons
    QString joinStr = QStringLiteral(", ");

    QPixmap pxMap;
    std::stringstream constraintName;
    constraintName << type.toLatin1().data()
                   << drawingParameters.constraintIconSize;  // allow resizing by embedding size
    if (!Gui::BitmapFactory().findPixmapInCache(constraintName.str().c_str(), pxMap)) {
        pxMap = Gui::BitmapFactory().pixmapFromSvg(
            type.toLatin1().data(),
            QSizeF(drawingParameters.constraintIconSize, drawingParameters.constraintIconSize)
        );
        Gui::BitmapFactory().addPixmapToCache(
            constraintName.str().c_str(),
            pxMap
        );  // Cache for speed, avoiding pixmapFromSvg
    }
    QImage icon = pxMap.toImage();
    // The pixmap was already scaled so we don't need to scale the image
    icon.setDevicePixelRatio(1.0f);

    QFont font = EditModeConstraintPreviewDetail::previewFont(drawingParameters);
    QFontMetrics qfm = QFontMetrics(font);

    int labelWidth = qfm.boundingRect(labels.join(joinStr)).width();
    // See Qt docs on qRect::bottom() for explanation of the +1
    int pxBelowBase = qfm.boundingRect(labels.join(joinStr)).bottom() + 1;

    if (vPad) {
        *vPad = pxBelowBase;
    }

    QTransform rotation;
    rotation.rotate(iconRotation);

    QImage roticon = icon.transformed(rotation);
    QImage image = roticon.copy(0, 0, roticon.width() + labelWidth, roticon.height() + pxBelowBase);

    // Make a bounding box for the icon
    if (boundingBoxes) {
        boundingBoxes->push_back(QRect(0, 0, roticon.width(), roticon.height()));
    }

    // Render the Icons
    QPainter qp(&image);
    qp.setCompositionMode(QPainter::CompositionMode_SourceIn);
    qp.fillRect(roticon.rect(), iconColor);

    // Render constraint label if necessary
    if (!labels.join(QString()).isEmpty()) {
        qp.setCompositionMode(QPainter::CompositionMode_SourceOver);
        qp.setFont(font);

        int cursorOffset = 0;

        // In Python: "for label, color in zip(labels, labelColors):"
        QStringList::const_iterator labelItr;
        QString labelStr;
        QList<QColor>::const_iterator colorItr;
        QRect labelBB;
        for (labelItr = labels.begin(), colorItr = labelColors.begin();
             labelItr != labels.end() && colorItr != labelColors.end();
             ++labelItr, ++colorItr) {

            qp.setPen(*colorItr);

            if (labelItr + 1 == labels.end()) {  // if this is the last label
                labelStr = *labelItr;
            }
            else {
                labelStr = *labelItr + joinStr;
            }

            // Note: text can sometimes draw to the left of the starting
            //       position, eg italic fonts.  Check QFontMetrics
            //       documentation for more info, but be mindful if the
            //       icon.width() is ever very small (or removed).
            qp.drawText(icon.width() + cursorOffset, icon.height(), labelStr);

            if (boundingBoxes) {
                labelBB = qfm.boundingRect(labelStr);
                labelBB.moveTo(icon.width() + cursorOffset, icon.height() - qfm.height() + pxBelowBase);
                boundingBoxes->push_back(labelBB);
            }

            cursorOffset += Gui::QtTools::horizontalAdvance(qfm, labelStr);
        }
    }

    return image;
}

void EditModeConstraintCoinManager::drawTypicalConstraintIcon(const constrIconQueueItem& i)
{
    QColor color = constrColor(i.constraintId);

    QImage image
        = renderConstrIcon(i.type, color, QStringList(i.label), QList<QColor>() << color, i.iconRotation);

    i.infoPtr->string.setValue(QString::number(i.constraintId).toLatin1().data());
    sendConstraintIconToCoin(image, i.destination);
}

QString EditModeConstraintCoinManager::iconTypeFromConstraint(Constraint* constraint)
{
    /*! TODO: Consider pushing this functionality up into Constraint
     *
     Abdullah: Please, don't. An icon is visualisation information and
     does not belong in App, but in Gui. Rather consider refactoring it
     in a separate class dealing with visualisation of constraints.*/

    switch (constraint->Type) {
        case Horizontal:
            return QStringLiteral("Constraint_Horizontal");
        case Vertical:
            return QStringLiteral("Constraint_Vertical");
        case PointOnObject:
            return QStringLiteral("Constraint_PointOnObject");
        case Tangent:
            return QStringLiteral("Constraint_Tangent");
        case Parallel:
            return QStringLiteral("Constraint_Parallel");
        case Perpendicular:
            return QStringLiteral("Constraint_Perpendicular");
        case Equal:
            return QStringLiteral("Constraint_EqualLength");
        case Symmetric:
            return QStringLiteral("Constraint_Symmetric");
        case SnellsLaw:
            return QStringLiteral("Constraint_SnellsLaw");
        case Block:
            return QStringLiteral("Constraint_Block");
        default:
            return QString();
    }
}

void EditModeConstraintCoinManager::sendConstraintIconToCoin(const QImage& icon, SoImage* soImagePtr)
{
    SoSFImage icondata = SoSFImage();

    Gui::BitmapFactory().convert(icon, icondata);

    SbVec2s iconSize(icon.width(), icon.height());

    int four = 4;
    soImagePtr->image.setValue(iconSize, 4, icondata.getValue(iconSize, four));

    // Set Image Alignment to Center
    soImagePtr->vertAlignment = SoImage::HALF;
    soImagePtr->horAlignment = SoImage::CENTER;
}

void EditModeConstraintCoinManager::clearCoinImage(SoImage* soImagePtr)
{
    soImagePtr->setToDefaults();
}

QColor EditModeConstraintCoinManager::constrColor(int constraintId)
{
    auto toQColor = [](auto sbcolor) -> QColor {
        return QColor((int)(sbcolor[0] * 255.0f), (int)(sbcolor[1] * 255.0f), (int)(sbcolor[2] * 255.0f));
    };

    const auto constraints = ViewProviderSketchCoinAttorney::getConstraints(viewProvider);
    bool isActive = ViewProviderSketchCoinAttorney::isConstraintActiveInSketch(
        viewProvider,
        constraints[constraintId]
    );

    if (ViewProviderSketchCoinAttorney::isConstraintPreselected(viewProvider, constraintId)) {
        return toQColor(drawingParameters.PreselectColor);
    }
    else if (ViewProviderSketchCoinAttorney::isConstraintSelected(viewProvider, constraintId)) {
        return toQColor(drawingParameters.SelectColor);
    }
    else if (!isActive) {
        return toQColor(drawingParameters.DeactivatedConstrDimColor);
    }
    else if (!constraints[constraintId]->isDriving) {
        return toQColor(drawingParameters.NonDrivingConstrDimColor);
    }
    else {
        return toQColor(drawingParameters.ConstrIcoColor);
    }
}

int EditModeConstraintCoinManager::constrColorPriority(int constraintId)
{
    if (ViewProviderSketchCoinAttorney::isConstraintPreselected(viewProvider, constraintId)) {
        return 3;
    }
    else if (ViewProviderSketchCoinAttorney::isConstraintSelected(viewProvider, constraintId)) {
        return 2;
    }
    else {
        return 1;
    }
}

int EditModeConstraintCoinManager::previewHitTolerancePx() const
{
    const int fontBased = std::max(8, ViewProviderSketchCoinAttorney::defaultApplicationFontSizePixels(viewProvider) / 2);
    const int scaledBase = static_cast<int>(std::lround(EditModeConstraintPreviewDetail::kBasePreviewHitTolerancePx
                                                        * drawingParameters.pixelScalingFactor));
    return std::max(fontBased, scaledBase);
}

int EditModeConstraintCoinManager::previewLineHitTolerancePx() const
{
    const int fontBased = std::max(10, static_cast<int>(std::lround(
                                      0.75 * ViewProviderSketchCoinAttorney::defaultApplicationFontSizePixels(viewProvider))));
    const int scaledBase = static_cast<int>(std::lround(EditModeConstraintPreviewDetail::kBasePreviewLineHitTolerancePx
                                                        * drawingParameters.pixelScalingFactor));
    return std::max(fontBased, scaledBase);
}

void EditModeConstraintCoinManager::setDimensionCandidates(
    const std::vector<DimensionCandidate>& candidates)
{
    smartDimensionCandidates = candidates;

    std::erase_if(smartDimensionCandidates, [&](const DimensionCandidate& candidate) {
        Gui::SoDatumLabel* preview = preparePreviewDatum(candidate);
        if (preview) {
            preview->unref();
            return false;
        }
        return true;
    });

    if (smartDimensionActiveCandidate >= static_cast<int>(smartDimensionCandidates.size())) {
        smartDimensionActiveCandidate = -1;
    }

    // Once the user grabs a preview, keep every candidate exactly where it was. Re-running the
    // global layout optimizer on every mouse-move makes the dragged preview feel sticky and also
    // causes neighboring previews to jump around as the cursor passes over them. During an active
    // drag we only want the selected candidate's explicit labelPos update to propagate.
    if (smartDimensionActiveCandidate < 0) {
        optimizeSmartDimensionPreviewLayout(smartDimensionCandidates);
    }

    rebuildSmartDimensionNodes();
}

std::vector<DimensionCandidate>
EditModeConstraintCoinManager::buildSmartDimensionLayoutVariants(
    const DimensionCandidate& candidate) const
{
    std::vector<DimensionCandidate> variants;
    variants.reserve(8);

    const auto pushUnique = [&](const Base::Vector2d& labelPos) {
        constexpr double kMinVariantDistance = 0.25;
        for (const auto& existing : variants) {
            const double dx = existing.labelPos.x - labelPos.x;
            const double dy = existing.labelPos.y - labelPos.y;
            if ((dx * dx + dy * dy) <= (kMinVariantDistance * kMinVariantDistance)) {
                return;
            }
        }

        auto variant = candidate;
        variant.labelPos = labelPos;
        variants.push_back(std::move(variant));
    };

    pushUnique(candidate.labelPos);

    auto* sketch = viewProvider.getSketchObject();
    if (!sketch) {
        return variants;
    }

    using namespace DimensionGeometry;

    const auto rotated = [](const Base::Vector2d& v, double angle) {
        const double c = std::cos(angle);
        const double s = std::sin(angle);
        return Base::Vector2d(v.x * c - v.y * s, v.x * s + v.y * c);
    };

    switch (candidate.semantic) {
        case DimensionSemantic::ProjectedX:
        case DimensionSemantic::ProjectedY:
        case DimensionSemantic::DirectLength:
        case DimensionSemantic::DirectDistance: {
            if (candidate.refs.size() < 2) {
                break;
            }

            const auto frame = makeLinearPreviewFrame(*sketch,
                                                      candidate.refs[0],
                                                      candidate.refs[1],
                                                      candidate.semantic);
            if (!frame.valid) {
                break;
            }

            double signedNormal = 0.0;
            double along = 0.0;
            decomposeLinearLabelPosition(frame, candidate.labelPos, signedNormal, along);
            const double preferredSide = signedNormal < -kEpsilon ? -1.0 : 1.0;
            const double offset = std::max(6.0,
                                           std::abs(signedNormal) > kEpsilon ? std::abs(signedNormal) : 8.0);
            const double slide = std::max(6.0, std::abs(along) + 6.0);

            const auto makeVariant = [&](double slideOffset, double offsetScale = 1.0) {
                pushUnique(composeLinearLabelPosition(frame,
                                                      preferredSide * offset * offsetScale,
                                                      along + slideOffset));
            };

            // Keep linear previews on the canonical exterior side that was already chosen in
            // makeStableLinearLabelPos(). Sliding along the measured direction is enough to
            // resolve most overlaps without reintroducing the side-flip regressions from v112/v113.
            makeVariant(0.0);
            makeVariant(slide);
            makeVariant(-slide);
            makeVariant(0.0, 1.35);

            if (candidate.semantic == DimensionSemantic::DirectLength
                || candidate.semantic == DimensionSemantic::DirectDistance) {
                makeVariant(slide * 0.5, 1.15);
                makeVariant(-slide * 0.5, 1.15);
            }
            break;
        }
        case DimensionSemantic::Radius:
        case DimensionSemantic::Diameter: {
            if (candidate.refs.empty()) {
                break;
            }

            const auto placement = describeRoundPlacement(sketch->getGeometry(candidate.refs.front().geoId),
                                                          candidate.semantic);
            if (!placement.valid) {
                break;
            }

            // Keep the semantic slot angle fixed for round previews. Radius and diameter already
            // receive different preferred angles in describeRoundPlacement(); allowing the local
            // preview optimizer to sweep both candidates through a shared set of angle offsets
            // lets them drift back into each other, especially on arcs where both are clamped into
            // the same usable angular interval. Only vary label distance here.
            const double baseAngle = placement.preferredAngle;
            const double baseDistance = std::max(6.0,
                                                 roundConstraintLabelDistance(placement,
                                                                              candidate.labelPos));
            if (candidate.semantic == DimensionSemantic::Diameter) {
                static constexpr double diameterDistanceOffsets[] {0.0, 8.0, 14.0, 20.0};
                for (double distanceOffset : diameterDistanceOffsets) {
                    pushUnique(roundLabelPosition(placement,
                                                  baseAngle,
                                                  baseDistance + distanceOffset));
                }
            }
            else {
                static constexpr double radiusDistanceOffsets[] {0.0, 4.0, 8.0, 12.0};
                for (double distanceOffset : radiusDistanceOffsets) {
                    pushUnique(roundLabelPosition(placement,
                                                  baseAngle,
                                                  baseDistance + distanceOffset));
                }
            }
            break;
        }
        case DimensionSemantic::ArcLength: {
            if (candidate.refs.empty()) {
                break;
            }

            ArcLengthPlacementData placement;
            if (!describeArcLengthPlacement(sketch->getGeometry(candidate.refs.front().geoId),
                                            placement)) {
                break;
            }

            const double baseRadius = std::max(placement.radius + 6.0,
                                               arcLengthLabelRadius(placement, candidate.labelPos));
            static constexpr double offsets[] {0.0, 6.0, 12.0, -6.0, 18.0};
            for (double offset : offsets) {
                const double radius = std::max(1.0, baseRadius + offset);
                pushUnique(Base::Vector2d(placement.center.x + placement.visualDir.x * radius,
                                          placement.center.y + placement.visualDir.y * radius));
            }
            break;
        }
        case DimensionSemantic::Angle: {
            if (candidate.refs.size() < 2) {
                break;
            }

            const Base::Vector2d vertex = midpoint(sketchPoint(*sketch, candidate.refs[0]),
                                                   sketchPoint(*sketch, candidate.refs[1]));
            Base::Vector2d dir(candidate.labelPos.x - vertex.x, candidate.labelPos.y - vertex.y);
            dir = normalized(dir, Base::Vector2d(1.0, 0.0));
            const double distance = std::max(10.0,
                                             length(Base::Vector2d(candidate.labelPos.x - vertex.x,
                                                                   candidate.labelPos.y - vertex.y)));
            static constexpr double angleOffsets[] {0.0, 0.25, -0.25};
            static constexpr double distanceOffsets[] {0.0, 6.0, 12.0};
            for (double a : angleOffsets) {
                for (double d : distanceOffsets) {
                    const Base::Vector2d rotatedDir = rotated(dir, a);
                    pushUnique(Base::Vector2d(vertex.x + rotatedDir.x * (distance + d),
                                              vertex.y + rotatedDir.y * (distance + d)));
                }
            }
            break;
        }
        case DimensionSemantic::Unknown:
        default:
            break;
    }

    return variants;
}

void EditModeConstraintCoinManager::optimizeSmartDimensionPreviewLayout(
    std::vector<DimensionCandidate>& candidates) const
{
    if (candidates.empty()) {
        return;
    }

    auto* sketch = viewProvider.getSketchObject();
    if (!sketch) {
        return;
    }

    using namespace DimensionGeometry;

    struct CandidateLayoutOption
    {
        DimensionCandidate candidate;
        double displacementPenalty {0.0};
        double sourceCrossingPenalty {0.0};
        double sideFlipPenalty {0.0};
        double compactnessPenalty {0.0};
    };

    // Keep every preview independently stable. The earlier global optimizer looked at the full
    // candidate set and happily moved the surviving previews when one sibling candidate was added,
    // removed, hovered or committed. That made round/arc previews jump around even though the
    // underlying geometry never changed. Here each candidate still evaluates a few local variants,
    // but the final choice depends only on that candidate and its source geometry.
    for (auto& candidate : candidates) {
        std::vector<CandidateLayoutOption> options;
        options.reserve(8);

        for (const auto& variant : buildSmartDimensionLayoutVariants(candidate)) {
            Gui::SoDatumLabel* preview = preparePreviewDatum(variant);
            if (!preview) {
                continue;
            }

            CandidateLayoutOption option;
            option.candidate = variant;

            std::vector<EditModeConstraintPreviewDetail::PreviewSegment> segments;
            std::vector<EditModeConstraintPreviewDetail::PreviewSegment> sourceSegments;
            EditModeConstraintPreviewDetail::appendDatumSegments(viewProvider,
                                                                 drawingParameters,
                                                                 *preview,
                                                                 segments);
            EditModeConstraintPreviewDetail::appendSourceGeometrySegments(viewProvider,
                                                                          variant,
                                                                          sourceSegments);
            for (const auto& previewSegment : segments) {
                for (const auto& sourceSegment : sourceSegments) {
                    if (EditModeConstraintPreviewDetail::previewSegmentsIntersect(previewSegment,
                                                                                 sourceSegment)) {
                        option.sourceCrossingPenalty += 9000.0;
                    }
                }
            }

            const double dx = variant.labelPos.x - candidate.labelPos.x;
            const double dy = variant.labelPos.y - candidate.labelPos.y;
            option.displacementPenalty = dx * dx + dy * dy;

            if (candidate.refs.size() >= 2
                && (candidate.semantic == DimensionSemantic::ProjectedX
                    || candidate.semantic == DimensionSemantic::ProjectedY
                    || candidate.semantic == DimensionSemantic::DirectLength
                    || candidate.semantic == DimensionSemantic::DirectDistance)) {
                const auto frame = makeLinearPreviewFrame(*sketch,
                                                          candidate.refs[0],
                                                          candidate.refs[1],
                                                          candidate.semantic);
                if (frame.valid) {
                    double originalSignedNormal = 0.0;
                    double originalAlong = 0.0;
                    double variantSignedNormal = 0.0;
                    double variantAlong = 0.0;
                    decomposeLinearLabelPosition(frame,
                                                 candidate.labelPos,
                                                 originalSignedNormal,
                                                 originalAlong);
                    decomposeLinearLabelPosition(frame,
                                                 variant.labelPos,
                                                 variantSignedNormal,
                                                 variantAlong);
                    if (originalSignedNormal * variantSignedNormal < -kEpsilon) {
                        option.sideFlipPenalty += (candidate.semantic == DimensionSemantic::DirectLength
                                                       || candidate.semantic == DimensionSemantic::DirectDistance)
                            ? 2500.0
                            : 3000.0;
                    }

                    const double bboxMinX = std::min(frame.a.x, frame.b.x);
                    const double bboxMaxX = std::max(frame.a.x, frame.b.x);
                    const double bboxMinY = std::min(frame.a.y, frame.b.y);
                    const double bboxMaxY = std::max(frame.a.y, frame.b.y);
                    switch (candidate.semantic) {
                        case DimensionSemantic::ProjectedX: {
                            const double preferredY = bboxMinY - 8.0;
                            const double yDelta = variant.labelPos.y - preferredY;
                            option.compactnessPenalty += yDelta * yDelta * 3.0;
                            break;
                        }
                        case DimensionSemantic::ProjectedY: {
                            const Base::Vector2d stableNormal = stableNormalFromLine(frame.a, frame.b);
                            const double preferredX = stableNormal.x >= 0.0 ? (bboxMinX - 8.0)
                                                                             : (bboxMaxX + 8.0);
                            const double xDelta = variant.labelPos.x - preferredX;
                            option.compactnessPenalty += xDelta * xDelta * 3.0;
                            break;
                        }
                        case DimensionSemantic::DirectLength:
                        case DimensionSemantic::DirectDistance: {
                            const Base::Vector2d stableNormal = stableNormalFromLine(frame.a, frame.b);
                            const Base::Vector2d center = midpoint(frame.a, frame.b);
                            const Base::Vector2d preferred(center.x + stableNormal.x * 10.0,
                                                           center.y + stableNormal.y * 10.0);
                            const double px = variant.labelPos.x - preferred.x;
                            const double py = variant.labelPos.y - preferred.y;
                            option.compactnessPenalty += (px * px + py * py) * 0.75;
                            break;
                        }
                        default:
                            break;
                    }
                }
            }

            options.push_back(std::move(option));
            preview->unref();
        }

        if (options.empty()) {
            continue;
        }

        const auto bestIt = std::min_element(options.begin(),
                                             options.end(),
                                             [](const CandidateLayoutOption& lhs,
                                                const CandidateLayoutOption& rhs) {
            const double lhsScore = lhs.displacementPenalty + lhs.sourceCrossingPenalty
                + lhs.sideFlipPenalty + lhs.compactnessPenalty;
            const double rhsScore = rhs.displacementPenalty + rhs.sourceCrossingPenalty
                + rhs.sideFlipPenalty + rhs.compactnessPenalty;
            if (lhsScore != rhsScore) {
                return lhsScore < rhsScore;
            }
            return lhs.displacementPenalty < rhs.displacementPenalty;
        });
        candidate = bestIt->candidate;
    }
}

void EditModeConstraintCoinManager::clearDimensionCandidates()
{
    smartDimensionCandidates.clear();
    smartDimensionActiveCandidate = -1;
    rebuildSmartDimensionNodes();
}

void EditModeConstraintCoinManager::setSmartDimensionActiveCandidate(int index)
{
    if (index < 0 || index >= static_cast<int>(smartDimensionCandidates.size())) {
        index = -1;
    }

    if (smartDimensionActiveCandidate == index) {
        return;
    }

    smartDimensionActiveCandidate = index;
    rebuildSmartDimensionNodes();
}

Gui::SoDatumLabel*
EditModeConstraintCoinManager::preparePreviewDatum(
    const DimensionCandidate& candidate) const
{
    auto* sketch = viewProvider.getSketchObject();
    if (!sketch) {
        return nullptr;
    }

    auto constraint = buildDimensionConstraint(*sketch, candidate);
    if (!constraint) {
        return nullptr;
    }

    constraint->isActive = true;

    auto* datum = new Gui::SoDatumLabel();
    datum->ref();
    if (!configurePreviewDatumLabel(candidate, *constraint, *datum)) {
        datum->unref();
        return nullptr;
    }

    return datum;
}

bool EditModeConstraintCoinManager::resolveDimensionCandidate(
    int index,
    DimensionCandidate& candidate) const
{
    if (index < 0 || index >= static_cast<int>(smartDimensionCandidates.size())) {
        return false;
    }

    candidate = smartDimensionCandidates[index];
    if (isLinearDimensionSemantic(candidate.semantic)) {
        return true;
    }

    Gui::SoDatumLabel* preview = preparePreviewDatum(candidate);
    if (!preview) {
        return false;
    }

    const SbVec3f center = preview->getLabelTextCenter();
    preview->unref();
    candidate.labelPos = Base::Vector2d(center[0], center[1]);
    return true;
}

void EditModeConstraintCoinManager::initializePreviewDatumStyle(
    Gui::SoDatumLabel& datum,
    const Sketcher::Constraint& constraint) const
{
    Base::Vector3d sketchNormal(0.0, 0.0, 1.0);
    Base::Placement placement = ViewProviderSketchCoinAttorney::getEditingPlacement(viewProvider);
    Base::Rotation rotation(placement.getRotation());
    rotation.multVec(sketchNormal, sketchNormal);

    datum.norm.setValue(SbVec3f(sketchNormal.x, sketchNormal.y, sketchNormal.z));
    datum.size.setValue(drawingParameters.labelFontSize);
    // Keep previews visually closer to FreeCAD's temporary on-screen dimension feedback while
    // sketching. That feedback is calmer than the final dimensional-constraint styling, so use a
    // thinner stroke and the coordinate/cursor text palette as the normal preview tone.
    datum.lineWidth = 1 * drawingParameters.pixelScalingFactor;
    datum.useAntialiasing = false;
    datum.strikethrough = !constraint.isActive;

    datum.textColor = constraint.isActive
        ? drawingParameters.CursorTextColor
        : drawingParameters.DeactivatedConstrDimColor;
}

int EditModeConstraintCoinManager::pickDimensionCandidate(const QPoint& screenPos) const
{
    if (smartDimensionRoot && !smartDimensionCandidates.empty()) {
        auto rayPickAction = ViewProviderSketchCoinAttorney::getRayPickAction(viewProvider);
        if (rayPickAction) {
            const auto clampCoord = [](int value) {
                return static_cast<short>(std::clamp(value,
                                                     static_cast<int>(std::numeric_limits<short>::min()),
                                                     static_cast<int>(std::numeric_limits<short>::max())));
            };

            rayPickAction->setPoint(SbVec2s(clampCoord(screenPos.x()), clampCoord(screenPos.y())));
            rayPickAction->setRadius(static_cast<float>(previewLineHitTolerancePx()));
            rayPickAction->apply(smartDimensionRoot);

            if (const SoPickedPoint* pickedPoint = rayPickAction->getPickedPoint()) {
                SoPath* path = pickedPoint->getPath();
                if (path && path->getLength() >= 2
                    && dynamic_cast<Gui::SoDatumLabel*>(path->getTail())) {
                    if (auto* sep = dynamic_cast<SoSeparator*>(path->getNode(path->getLength() - 2))) {
                        for (int i = 0; i < smartDimensionRoot->getNumChildren(); ++i) {
                            if (smartDimensionRoot->getChild(i) == sep) {
                                return i;
                            }
                        }
                    }
                }
            }
        }
    }

    int bestIndex = -1;
    int bestDistance = std::numeric_limits<int>::max();

    for (int i = 0; i < static_cast<int>(smartDimensionCandidates.size()); ++i) {
        Gui::SoDatumLabel* preview = preparePreviewDatum(smartDimensionCandidates[i]);
        if (!preview) {
            continue;
        }

        const int labelHitDistance = EditModeConstraintPreviewDetail::previewLabelHitDistance(
            viewProvider,
            drawingParameters,
            smartDimensionCandidates[i],
            *preview,
            screenPos,
            previewHitTolerancePx());
        preview->unref();

        if (labelHitDistance < bestDistance
            || (labelHitDistance == bestDistance && labelHitDistance <= 0 && bestIndex > i)) {
            bestDistance = labelHitDistance;
            bestIndex = i;
        }
    }

    if (bestIndex >= 0 && bestDistance <= 0) {
        return bestIndex;
    }

    bestIndex = -1;
    bestDistance = std::numeric_limits<int>::max();

    for (int i = 0; i < static_cast<int>(smartDimensionCandidates.size()); ++i) {
        const int hitDistance = candidateHitDistance(smartDimensionCandidates[i], screenPos);
        if (hitDistance > previewLineHitTolerancePx()) {
            continue;
        }

        if (hitDistance < bestDistance || (hitDistance == bestDistance && bestIndex > i)) {
            bestDistance = hitDistance;
            bestIndex = i;
        }
    }

    return bestIndex;
}

int EditModeConstraintCoinManager::candidateHitDistance(
    const DimensionCandidate& candidate, const QPoint& screenPos) const
{
    Gui::SoDatumLabel* preview = preparePreviewDatum(candidate);
    if (!preview) {
        return std::numeric_limits<int>::max();
    }

    int bestDistance2 = EditModeConstraintPreviewDetail::previewLabelHitDistance(
        viewProvider,
        drawingParameters,
        candidate,
        *preview,
        screenPos,
        previewHitTolerancePx());
    {
        const int datumDistance = datumHitDistance(*preview, screenPos);
        const int datumDistance2 = datumDistance * datumDistance;
        if (datumDistance2 < bestDistance2) {
            bestDistance2 = datumDistance2;
        }
    }

    const int result = static_cast<int>(std::lround(std::sqrt(static_cast<double>(bestDistance2))));
    preview->unref();
    return result;
}

int EditModeConstraintCoinManager::datumHitDistance(const Gui::SoDatumLabel& datum,
                                                    const QPoint& screenPos) const
{
    int bestDistance2 = std::numeric_limits<int>::max();
    std::vector<EditModeConstraintPreviewDetail::PreviewSegment> segments;
    EditModeConstraintPreviewDetail::appendDatumSegments(viewProvider,
                                                         drawingParameters,
                                                         datum,
                                                         segments);

    for (const auto& segment : segments) {
        const int d2 = static_cast<int>(std::lround(
            std::pow(EditModeConstraintPreviewDetail::distancePointToSegment(screenPos,
                                                                             segment.a,
                                                                             segment.b),
                     2.0)));
        if (d2 < bestDistance2) {
            bestDistance2 = d2;
        }
    }

    if (bestDistance2 == std::numeric_limits<int>::max()) {
        return std::numeric_limits<int>::max() / 2;
    }

    return static_cast<int>(std::lround(std::sqrt(static_cast<double>(bestDistance2))));
}

bool EditModeConstraintCoinManager::configurePreviewDatumLabel(
    const DimensionCandidate& candidate,
    const Sketcher::Constraint& constraint,
    Gui::SoDatumLabel& datum) const
{
    auto* sketch = viewProvider.getSketchObject();
    if (!sketch) {
        return false;
    }

    initializePreviewDatumStyle(datum, constraint);

    const auto zConstrH = ViewProviderSketchCoinAttorney::getViewOrientationFactor(viewProvider)
        * drawingParameters.zConstr;
    datum.string = SbString(
        const_cast<EditModeConstraintCoinManager*>(this)->getPresentationString(&constraint)
            .toUtf8()
            .constData());

    if (candidate.semantic == DimensionSemantic::Radius
        || candidate.semantic == DimensionSemantic::Diameter) {
        const Part::Geometry* geo = sketch->getGeometry(constraint.First);
        if (!geo) {
            return false;
        }

        Base::Vector3d pnt1(0., 0., 0.);
        Base::Vector3d pnt2(0., 0., 0.);
        double helperStartAngle = 0.;
        double helperRange = 0.;
        double startHelperAngle = 0.;
        double startHelperRange = 0.;
        double endHelperAngle = 0.;
        double endHelperRange = 0.;
        const double angle = static_cast<double>(constraint.LabelPosition);

        if (candidate.semantic == DimensionSemantic::Radius) {
            if (geo->is<Part::GeomArcOfCircle>()) {
                auto* arc = static_cast<const Part::GeomArcOfCircle*>(geo);
                double startAngle, endAngle;
                arc->getRange(startAngle, endAngle, /*emulateCCW=*/true);
                findHelperAngles(helperStartAngle, helperRange, angle, startAngle, endAngle);
                pnt1 = arc->getCenter();
                pnt2 = pnt1 + arc->getRadius() * Base::Vector3d(std::cos(angle), std::sin(angle), 0.0);
            }
            else if (geo->is<Part::GeomCircle>()) {
                auto* circle = static_cast<const Part::GeomCircle*>(geo);
                pnt1 = circle->getCenter();
                pnt2 = pnt1 + circle->getRadius() * Base::Vector3d(std::cos(angle), std::sin(angle), 0.0);
            }
            else {
                return false;
            }

            datum.string = SbString(
                const_cast<EditModeConstraintCoinManager*>(this)
                    ->getPresentationString(&constraint, "R")
                    .toUtf8()
                    .constData());
            datum.datumtype = Gui::SoDatumLabel::RADIUS;
            datum.param1 = constraint.LabelDistance;
            datum.param2 = constraint.LabelPosition;
            datum.param3 = helperStartAngle;
            datum.param4 = helperRange;
        }
        else {
            if (geo->is<Part::GeomArcOfCircle>()) {
                auto* arc = static_cast<const Part::GeomArcOfCircle*>(geo);
                double startAngle, endAngle;
                arc->getRange(startAngle, endAngle, /*emulateCCW=*/true);
                findHelperAngles(startHelperAngle, startHelperRange, angle, startAngle, endAngle);
                findHelperAngles(endHelperAngle,
                                 endHelperRange,
                                 angle + 3.14159265358979323846,
                                 startAngle,
                                 endAngle);
                const auto center = arc->getCenter();
                pnt1 = center - arc->getRadius() * Base::Vector3d(std::cos(angle), std::sin(angle), 0.0);
                pnt2 = center + arc->getRadius() * Base::Vector3d(std::cos(angle), std::sin(angle), 0.0);
            }
            else if (geo->is<Part::GeomCircle>()) {
                auto* circle = static_cast<const Part::GeomCircle*>(geo);
                const auto center = circle->getCenter();
                pnt1 = center - circle->getRadius() * Base::Vector3d(std::cos(angle), std::sin(angle), 0.0);
                pnt2 = center + circle->getRadius() * Base::Vector3d(std::cos(angle), std::sin(angle), 0.0);
            }
            else {
                return false;
            }

            datum.string = SbString(
                const_cast<EditModeConstraintCoinManager*>(this)
                    ->getPresentationString(&constraint, "⌀")
                    .toUtf8()
                    .constData());
            datum.datumtype = Gui::SoDatumLabel::DIAMETER;
            datum.param1 = constraint.LabelDistance;
            datum.param2 = constraint.LabelPosition;
            datum.param3 = static_cast<float>(startHelperAngle);
            datum.param4 = static_cast<float>(startHelperRange);
            datum.param5 = static_cast<float>(endHelperAngle);
            datum.param6 = static_cast<float>(endHelperRange);
        }

        datum.pnts.setNum(2);
        SbVec3f* verts = datum.pnts.startEditing();
        verts[0] = SbVec3f(pnt1.x, pnt1.y, zConstrH);
        verts[1] = SbVec3f(pnt2.x, pnt2.y, zConstrH);
        datum.pnts.finishEditing();
        return true;
    }

    if (candidate.semantic == DimensionSemantic::Angle) {
        auto angleTangentAtPoint = [](const Part::Geometry* geo,
                                      Sketcher::PointPos pos,
                                      const Base::Vector3d& point) -> std::optional<Base::Vector3d> {
            if (!geo) {
                return std::nullopt;
            }

            if (geo->is<Part::GeomLineSegment>()) {
                auto* line = static_cast<const Part::GeomLineSegment*>(geo);
                Base::Vector3d dir = line->getEndPoint() - line->getStartPoint();
                if (dir.Length() <= Precision::Confusion()) {
                    return std::nullopt;
                }
                dir.Normalize();
                if (pos == Sketcher::PointPos::end) {
                    dir *= -1.0;
                }
                return dir;
            }

            (void)point;
            return std::nullopt;
        };

        const double distance = constraint.LabelDistance;
        double startAngle = 0.0;
        double range = 0.0;
        double endLineLength1 = 0.0;
        double endLineLength2 = 0.0;
        Base::Vector3d anchor(0.0, 0.0, zConstrH);

        if (constraint.Second != Sketcher::GeoEnum::GeoUndef) {
            const Part::Geometry* geo1 = sketch->getGeometry(constraint.First);
            const Part::Geometry* geo2 = sketch->getGeometry(constraint.Second);
            if (!geo1 || !geo2) {
                return false;
            }

            if (constraint.Third == Sketcher::GeoEnum::GeoUndef) {
                if (!geo1->is<Part::GeomLineSegment>() || !geo2->is<Part::GeomLineSegment>()) {
                    return false;
                }

                auto* line1 = static_cast<const Part::GeomLineSegment*>(geo1);
                auto* line2 = static_cast<const Part::GeomLineSegment*>(geo2);
                const bool flip1 = (constraint.FirstPos == Sketcher::PointPos::end);
                const bool flip2 = (constraint.SecondPos == Sketcher::PointPos::end);

                Base::Vector3d dir1 = (flip1 ? -1.0 : 1.0)
                    * (line1->getEndPoint() - line1->getStartPoint()).Normalize();
                Base::Vector3d dir2 = (flip2 ? -1.0 : 1.0)
                    * (line2->getEndPoint() - line2->getStartPoint()).Normalize();
                Base::Vector3d pnt1 = flip1 ? line1->getEndPoint() : line1->getStartPoint();
                Base::Vector3d pnt2 = flip2 ? line2->getEndPoint() : line2->getStartPoint();
                Base::Vector3d pnt12 = flip1 ? line1->getStartPoint() : line1->getEndPoint();
                Base::Vector3d pnt22 = flip2 ? line2->getStartPoint() : line2->getEndPoint();

                Base::Vector3d intersection;
                const double det = dir1.x * dir2.y - dir1.y * dir2.x;
                if ((det > 0 ? det : -det) < 1e-10) {
                    Base::Vector3d p1[2] = {line1->getStartPoint(), line1->getEndPoint()};
                    Base::Vector3d p2[2] = {line2->getStartPoint(), line2->getEndPoint()};
                    double shortest = std::numeric_limits<double>::max();
                    for (int i = 0; i <= 1; ++i) {
                        for (int j = 0; j <= 1; ++j) {
                            const double candidateDistance = (p2[j] - p1[i]).Length();
                            if (candidateDistance < shortest) {
                                shortest = candidateDistance;
                                intersection = Base::Vector3d((p2[j].x + p1[i].x) * 0.5,
                                                              (p2[j].y + p1[i].y) * 0.5,
                                                              0.0);
                            }
                        }
                    }
                }
                else {
                    const double c1 = dir1.y * pnt1.x - dir1.x * pnt1.y;
                    const double c2 = dir2.y * pnt2.x - dir2.x * pnt2.y;
                    intersection = Base::Vector3d((dir1.x * c2 - dir2.x * c1) / det,
                                                  (dir1.y * c2 - dir2.y * c1) / det,
                                                  0.0);
                }

                anchor = Base::Vector3d(intersection.x, intersection.y, zConstrH);
                startAngle = std::atan2(dir1.y, dir1.x);
                range = constraint.getValue();
                const Base::Vector3d vl1 = dir1 * 2.0 * distance - (pnt1 - intersection);
                const Base::Vector3d vl2 = dir2 * 2.0 * distance - (pnt2 - intersection);
                const Base::Vector3d vl12 = dir1 * 2.0 * distance - (pnt12 - intersection);
                const Base::Vector3d vl22 = dir2 * 2.0 * distance - (pnt22 - intersection);
                endLineLength1 = vl12.Dot(dir1) > 0.0 ? vl12.Length()
                               : vl1.Dot(dir1) < 0.0 ? -vl1.Length()
                                                     : 0.0;
                endLineLength2 = vl22.Dot(dir2) > 0.0 ? vl22.Length()
                               : vl2.Dot(dir2) < 0.0 ? -vl2.Length()
                                                     : 0.0;
            }
            else {
                const Base::Vector3d point = sketch->getPoint(constraint.Third, constraint.ThirdPos);
                const auto dir1Opt = angleTangentAtPoint(geo1, constraint.FirstPos, point);
                const auto dir2Opt = angleTangentAtPoint(geo2, constraint.SecondPos, point);
                if (!dir1Opt || !dir2Opt) {
                    return false;
                }

                Base::Vector3d dir1 = *dir1Opt;
                Base::Vector3d dir2 = *dir2Opt;
                anchor = Base::Vector3d(point.x, point.y, zConstrH);
                startAngle = std::atan2(dir1.y, dir1.x);
                range = std::atan2(dir1.x * dir2.y - dir1.y * dir2.x,
                                   dir1.x * dir2.x + dir1.y * dir2.y);
            }
        }
        else if (constraint.First != Sketcher::GeoEnum::GeoUndef) {
            const Part::Geometry* geo = sketch->getGeometry(constraint.First);
            if (!geo) {
                return false;
            }

            if (geo->is<Part::GeomLineSegment>()) {
                auto* lineSeg = static_cast<const Part::GeomLineSegment*>(geo);
                anchor = Base::Vector3d((lineSeg->getEndPoint().x + lineSeg->getStartPoint().x) / 2.0,
                                        (lineSeg->getEndPoint().y + lineSeg->getStartPoint().y) / 2.0,
                                        zConstrH);
                const double l1 = 2.0 * distance
                    - (lineSeg->getEndPoint() - lineSeg->getStartPoint()).Length() / 2.0;
                endLineLength1 = 2.0 * distance;
                endLineLength2 = l1 > 0.0 ? l1 : 0.0;

                const Base::Vector3d dir = lineSeg->getEndPoint() - lineSeg->getStartPoint();
                startAngle = 0.0;
                range = std::atan2(dir.y, dir.x);
            }
            else if (geo->is<Part::GeomArcOfCircle>()) {
                auto* arc = static_cast<const Part::GeomArcOfCircle*>(geo);
                anchor = Base::Vector3d(arc->getCenter().x, arc->getCenter().y, zConstrH);
                endLineLength1 = 2.0 * distance - arc->getRadius();
                endLineLength2 = endLineLength1;

                double endAngle = 0.0;
                arc->getRange(startAngle, endAngle, /*emulateCCWXY=*/true);
                range = endAngle - startAngle;
            }
            else {
                return false;
            }
        }
        else {
            return false;
        }

        datum.datumtype = Gui::SoDatumLabel::ANGLE;
        datum.param1 = constraint.LabelDistance;
        datum.param2 = static_cast<float>(startAngle);
        datum.param3 = static_cast<float>(range);
        datum.param4 = static_cast<float>(endLineLength1);
        datum.param5 = static_cast<float>(endLineLength2);
        datum.pnts.setNum(2);
        SbVec3f* verts = datum.pnts.startEditing();
        verts[0] = SbVec3f(anchor.x, anchor.y, anchor.z);
        datum.pnts.finishEditing();
        return true;
    }

    double helperStartAngle1 = 0.;
    double helperStartAngle2 = 0.;
    double helperRange1 = 0.;
    double helperRange2 = 0.;
    double radius1 = 0.;
    double radius2 = 0.;
    Base::Vector3d center1(0., 0., 0.);
    Base::Vector3d center2(0., 0., 0.);

    int numPoints = 2;
    Base::Vector3d pnt1 = sketch->getPoint(constraint.First, constraint.FirstPos);
    Base::Vector3d pnt2(0., 0., 0.);

    if (constraint.SecondPos != Sketcher::PointPos::none) {
        pnt2 = sketch->getPoint(constraint.Second, constraint.SecondPos);
    }
    else if (constraint.Second != Sketcher::GeoEnum::GeoUndef) {
        auto geo1 = sketch->getGeometry(constraint.First);
        auto geo2 = sketch->getGeometry(constraint.Second);
        if (!geo2) {
            return false;
        }

        if (isLineSegment(*geo2)) {
            auto lineSeg = static_cast<const Part::GeomLineSegment*>(geo2);
            Base::Vector3d l2p1 = lineSeg->getStartPoint();
            Base::Vector3d l2p2 = lineSeg->getEndPoint();

            if (constraint.FirstPos != Sketcher::PointPos::none) {
                pnt2.ProjectToLine(pnt1 - l2p1, l2p2 - l2p1);
                pnt2 += pnt1;
            }
            else if (geo1 && isCircleOrArc(*geo1)) {
                auto [radius, ct] = getRadiusCenterCircleArc(geo1);
                pnt1.ProjectToLine(ct - l2p1, l2p2 - l2p1);
                Base::Vector3d dir = pnt1;
                dir.Normalize();
                pnt1 += ct;
                pnt2 = ct + dir * radius;
            }
            else {
                return false;
            }
        }
        else if (isCircleOrArc(*geo2)) {
            if (constraint.FirstPos != Sketcher::PointPos::none) {
                auto [rad, ct] = getRadiusCenterCircleArc(geo2);
                Base::Vector3d v = pnt1 - ct;
                v = v.Normalize();
                pnt2 = ct + rad * v;
            }
            else if (geo1 && isCircleOrArc(*geo1)) {
                GetCirclesMinimalDistance(geo1, geo2, pnt1, pnt2);
            }
            else {
                return false;
            }
        }
        else {
            return false;
        }
    }
    else if (constraint.FirstPos != Sketcher::PointPos::none) {
        pnt1 = Base::Vector3d(0., 0., 0.);
        pnt2 = sketch->getPoint(constraint.First, constraint.FirstPos);
    }
    else if (constraint.First != Sketcher::GeoEnum::GeoUndef) {
        auto geo = sketch->getGeometry(constraint.First);
        if (!geo) {
            return false;
        }

        if (isLineSegment(*geo)) {
            auto lineSeg = static_cast<const Part::GeomLineSegment*>(geo);
            pnt1 = lineSeg->getStartPoint();
            pnt2 = lineSeg->getEndPoint();
        }
        else if (isArcOfCircle(*geo)) {
            auto arc = static_cast<const Part::GeomArcOfCircle*>(geo);
            center1 = arc->getCenter();
            pnt1 = arc->getStartPoint();
            pnt2 = arc->getEndPoint();

            double startAngle = 0.0;
            double range = 0.0;
            if (!SketcherGui::DimensionGeometry::resolveArcLengthDatumSweep(*arc,
                                                                                constraint.LabelDistance,
                                                                                startAngle,
                                                                                range)) {
                return false;
            }

            datum.datumtype = Gui::SoDatumLabel::ARCLENGTH;
            datum.param1 = constraint.LabelDistance;
            datum.param2 = static_cast<float>(startAngle);
            datum.param3 = static_cast<float>(range);
            datum.string = SbString(
                const_cast<EditModeConstraintCoinManager*>(this)
                    ->getPresentationString(&constraint, "◠ ")
                    .toUtf8()
                    .constData());
            datum.strikethrough = !constraint.isActive;

            datum.pnts.setNum(3);
            SbVec3f* verts = datum.pnts.startEditing();
            verts[0] = SbVec3f(center1.x, center1.y, zConstrH);
            verts[1] = SbVec3f(pnt1.x, pnt1.y, zConstrH);
            verts[2] = SbVec3f(pnt2.x, pnt2.y, zConstrH);
            datum.pnts.finishEditing();
            return true;
        }
        else {
            return false;
        }
    }
    else {
        return false;
    }

    if (constraint.Type == Sketcher::ConstraintType::Distance) {
        datum.datumtype = Gui::SoDatumLabel::DISTANCE;
    }
    else if (constraint.Type == Sketcher::ConstraintType::DistanceX) {
        datum.datumtype = Gui::SoDatumLabel::DISTANCEX;
    }
    else {
        datum.datumtype = Gui::SoDatumLabel::DISTANCEY;
    }

    if (constraint.Second != Sketcher::GeoEnum::GeoUndef) {
        auto geo1 = sketch->getGeometry(constraint.First);
        auto geo2 = sketch->getGeometry(constraint.Second);

        if (geo1 && isArcOfCircle(*geo1) && constraint.FirstPos == Sketcher::PointPos::none) {
            auto arc = static_cast<const Part::GeomArcOfCircle*>(geo1);
            radius1 = arc->getRadius();
            center1 = arc->getCenter();

            double angle = toVector2d(isLineSegment(*geo2) ? pnt2 - center1 : pnt1 - center1).Angle();
            double startAngle, endAngle;
            arc->getRange(startAngle, endAngle, /*emulateCCW=*/true);
            findHelperAngles(helperStartAngle1, helperRange1, angle, startAngle, endAngle);

            if (helperRange1 != 0.) {
                helperStartAngle1 = endAngle;
                helperRange1 = 2 * std::numbers::pi - (endAngle - startAngle);
                numPoints++;
            }
        }
        if (geo2 && isArcOfCircle(*geo2) && constraint.SecondPos == Sketcher::PointPos::none) {
            auto arc = static_cast<const Part::GeomArcOfCircle*>(geo2);
            radius2 = arc->getRadius();
            center2 = arc->getCenter();

            double angle = toVector2d(pnt2 - center2).Angle();
            double startAngle, endAngle;
            arc->getRange(startAngle, endAngle, /*emulateCCW=*/true);
            findHelperAngles(helperStartAngle2, helperRange2, angle, startAngle, endAngle);

            if (helperRange2 != 0.) {
                helperStartAngle2 = endAngle;
                helperRange2 = 2 * std::numbers::pi - (endAngle - startAngle);
                numPoints++;
            }
        }
    }

    datum.pnts.setNum(numPoints);
    SbVec3f* verts = datum.pnts.startEditing();
    verts[0] = SbVec3f(pnt1.x, pnt1.y, zConstrH);
    verts[1] = SbVec3f(pnt2.x, pnt2.y, zConstrH);

    if (numPoints > 2) {
        if (helperRange1 != 0.) {
            verts[2] = SbVec3f(center1.x, center1.y, zConstrH);
            datum.param3 = helperStartAngle1;
            datum.param4 = helperRange1;
            datum.param5 = radius1;
        }
        else {
            verts[2] = SbVec3f(center2.x, center2.y, zConstrH);
            datum.param3 = helperStartAngle2;
            datum.param4 = helperRange2;
            datum.param5 = radius2;
        }
        if (numPoints > 3) {
            verts[3] = SbVec3f(center2.x, center2.y, zConstrH);
            datum.param6 = helperStartAngle2;
            datum.param7 = helperRange2;
            datum.param8 = radius2;
        }
        else {
            datum.param6 = 0.;
            datum.param7 = 0.;
            datum.param8 = 0.;
        }
    }
    else {
        datum.param3 = 0.;
        datum.param4 = 0.;
        datum.param5 = 0.;
        datum.param6 = 0.;
        datum.param7 = 0.;
        datum.param8 = 0.;
    }

    datum.pnts.finishEditing();
    datum.param1 = constraint.LabelDistance;
    datum.param2 = constraint.LabelPosition;
    return true;
}

void EditModeConstraintCoinManager::ensureSmartDimensionRoot()
{
    if (smartDimensionRoot) {
        return;
    }

    if (!editModeScenegraphNodes.EditRoot) {
        return;
    }

    smartDimensionRoot = new SoSeparator;
    smartDimensionRoot->setName("SmartDimensionRoot");

    int insertIndex = -1;
    for (int i = 0; i < editModeScenegraphNodes.EditRoot->getNumChildren(); ++i) {
        if (editModeScenegraphNodes.EditRoot->getChild(i) == editModeScenegraphNodes.constrGroup) {
            insertIndex = i + 1;
            break;
        }
    }

    if (insertIndex >= 0) {
        editModeScenegraphNodes.EditRoot->insertChild(smartDimensionRoot, insertIndex);
    }
    else {
        editModeScenegraphNodes.EditRoot->addChild(smartDimensionRoot);
    }
}

void EditModeConstraintCoinManager::rebuildSmartDimensionNodes()
{
    ensureSmartDimensionRoot();
    if (!smartDimensionRoot) {
        return;
    }

    smartDimensionRoot->removeAllChildren();

    for (int i = 0; i < static_cast<int>(smartDimensionCandidates.size()); ++i) {
        Gui::SoDatumLabel* preview = preparePreviewDatum(smartDimensionCandidates[i]);
        if (!preview) {
            continue;
        }

        auto* sep = new SoSeparator;
        sep->renderCaching = SoSeparator::OFF;

        auto* pickStyle = new SoPickStyle;
        pickStyle->style = SoPickStyle::SHAPE;
        sep->addChild(pickStyle);

        sep->addChild(editModeScenegraphNodes.ConstraintDrawStyle);

        if (i == smartDimensionActiveCandidate) {
            // Hovered preview should follow the same global preselection color language used by
            // the rest of Sketcher/FreeCAD, while the normal state keeps the pale temporary style.
            preview->textColor = drawingParameters.PreselectColor;
        }
        sep->addChild(preview);
        preview->unref();

        smartDimensionRoot->addChild(sep);
    }
}

SoSeparator* EditModeConstraintCoinManager::getConstraintIdSeparator(int i)
{
    return dynamic_cast<SoSeparator*>(editModeScenegraphNodes.constrGroup->getChild(i));
}

void EditModeConstraintCoinManager::createEditModeInventorNodes()
{
    // group node for the Constraint visual +++++++++++++++++++++++++++++++++++
    SoMaterialBinding* MtlBind = new SoMaterialBinding;
    MtlBind->setName("ConstraintMaterialBinding");
    MtlBind->value = SoMaterialBinding::OVERALL;
    editModeScenegraphNodes.EditRoot->addChild(MtlBind);

    // use small line width for the Constraints
    editModeScenegraphNodes.ConstraintDrawStyle = new SoDrawStyle;
    editModeScenegraphNodes.ConstraintDrawStyle->setName("ConstraintDrawStyle");
    editModeScenegraphNodes.ConstraintDrawStyle->lineWidth = 1 * drawingParameters.pixelScalingFactor;
    editModeScenegraphNodes.EditRoot->addChild(editModeScenegraphNodes.ConstraintDrawStyle);

    // add the group where all the constraints has its SoSeparator
    editModeScenegraphNodes.constrGrpSelect = new SoPickStyle();  // used to toggle constraints
                                                                  // selectability
    editModeScenegraphNodes.constrGrpSelect->style.setValue(SoPickStyle::SHAPE);
    editModeScenegraphNodes.EditRoot->addChild(editModeScenegraphNodes.constrGrpSelect);
    setConstraintSelectability();  // Ensure default value;

    // disable depth testing for constraint icons so they render ON TOP of geometry lines
    // check issues #25840 and #11603
    SoDepthBuffer* constrDepthOff = new SoDepthBuffer();
    constrDepthOff->test.setValue(false);
    editModeScenegraphNodes.EditRoot->addChild(constrDepthOff);

    editModeScenegraphNodes.constrGroup = new SmSwitchboard();
    editModeScenegraphNodes.constrGroup->setName("ConstraintGroup");
    editModeScenegraphNodes.EditRoot->addChild(editModeScenegraphNodes.constrGroup);

    // re-enable depth testing for the rest of the nodes
    SoDepthBuffer* constrDepthOn = new SoDepthBuffer();
    constrDepthOn->test.setValue(true);
    editModeScenegraphNodes.EditRoot->addChild(constrDepthOn);

    SoPickStyle* ps = new SoPickStyle();  // used to following nodes aren't impacted
    ps->style.setValue(SoPickStyle::SHAPE);
    editModeScenegraphNodes.EditRoot->addChild(ps);
}
