/***************************************************************************
 *   Copyright (c) 2022 WandererFan <wandererfan@gmail.com>                *
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

#include "PreCompiled.h"
#ifndef _PreComp_
# include <gp_Ax3.hxx>
# include <gp_Trsf.hxx>
# include <gp_Vec.hxx>
#endif

#include <Base/Console.h>
#include <Base/Converter.h>
#include <Base/Tools.h>

#include "DimensionGeometry.h"
#include "DrawUtil.h"
#include "DrawViewPart.h"
#include "DrawViewDetail.h"


using namespace TechDraw;
using DU = DrawUtil;

pointPair::pointPair(const pointPair& pp)
{
    first(pp.first());
    second(pp.second());
    overrideFirst(pp.extensionLineFirst());
    overrideSecond(pp.extensionLineSecond());
}

// set extension line points from argument base points
void pointPair::setExtensionLine(const pointPair& pp){
    overrideFirst(pp.first());
    overrideSecond(pp.second());
}

//move the points by offset
void pointPair::move(const Base::Vector3d& offset)
{
    m_first = m_first - offset;
    m_second = m_second - offset;
    m_overrideFirst = m_overrideFirst - offset;
    m_overrideSecond = m_overrideSecond - offset;
}

//move the points by factor
void pointPair::scale(double factor)
{
    m_first = m_first * factor;
    m_second = m_second * factor;
    m_overrideFirst = m_overrideFirst * factor;
    m_overrideSecond = m_overrideSecond * factor;
}

// project the points onto the dvp's paper plane.
void pointPair::project(const DrawViewPart* dvp)
{
    auto detailView = dynamic_cast<const DrawViewDetail*>(dvp);
    if (detailView) {
        m_first = detailView->mapPoint3dToDetail(m_first) * detailView->getScale();
        m_second = detailView->mapPoint3dToDetail(m_second) * detailView->getScale();
        m_overrideFirst = detailView->mapPoint3dToDetail(m_overrideFirst) * detailView->getScale();
        m_overrideSecond = detailView->mapPoint3dToDetail(m_overrideSecond) * detailView->getScale();
        return;
    }
    m_first = dvp->projectPoint(m_first) * dvp->getScale();
    m_second = dvp->projectPoint(m_second) * dvp->getScale();
    m_overrideFirst = dvp->projectPoint(m_overrideFirst) * dvp->getScale();
    m_overrideSecond = dvp->projectPoint(m_overrideSecond) * dvp->getScale();
}

// map the points onto the dvp's XY coordinate system
// this routine is no longer needed since we now use the hlr projector instead
// of "projectToPlane" from Vector3d
void pointPair::mapToPage(const DrawViewPart* dvp)
{
    gp_Trsf xOXYZ;
    gp_Ax3 OXYZ;
    xOXYZ.SetTransformation(OXYZ, gp_Ax3(dvp->getRotatedCS()));

    gp_Vec gvFirst = Base::convertTo<gp_Vec>(m_first).Transformed(xOXYZ);
    m_first = Base::convertTo<Base::Vector3d>(gvFirst);
    gp_Vec gvSecond = Base::convertTo<gp_Vec>(m_second).Transformed(xOXYZ);
    m_second = Base::convertTo<Base::Vector3d>(gvSecond);
}

// this routine is no longer needed since we now use the dvp's projectPoint
// which performs Y inversion by default
void pointPair::invertY()
{
    m_first = DU::invertY(m_first);
    m_second = DU::invertY(m_second);
}

void pointPair::dump(const std::string& text) const
{
    Base::Console().message("pointPair - %s\n", text.c_str());
    Base::Console().message("pointPair - first: %s  second: %s\n",
                            DU::formatVector(first()).c_str(), DU::formatVector(second()).c_str());
}

//! return unscaled, unrotated version of this pointPair.  caller is responsible for
//! for ensuring this pointPair is in scaled, rotated form before calling this method.
pointPair pointPair::toCanonicalForm(DrawViewPart* dvp) const
{
    pointPair result;
    // invert points?
    result.m_first = CosmeticVertex::makeCanonicalPoint(dvp, m_first);
    result.m_second = CosmeticVertex::makeCanonicalPoint(dvp, m_second);
    result.m_overrideFirst = CosmeticVertex::makeCanonicalPoint(dvp, m_overrideFirst);
    result.m_overrideSecond = CosmeticVertex::makeCanonicalPoint(dvp, m_overrideSecond);
    return result;
}

//! return scaled and rotated version of this pointPair.  caller is responsible for
//! for ensuring this pointPair is in canonical form before calling this method.
pointPair pointPair::toDisplayForm(DrawViewPart* dvp) const
{
    pointPair result;

    // invert points?
    result.m_first = m_first * dvp->getScale();
    result.m_second = m_second * dvp->getScale();
    result.m_overrideFirst = m_overrideFirst * dvp->getScale();
    result.m_overrideSecond = m_overrideSecond * dvp->getScale();
    auto rotationDeg = dvp->Rotation.getValue();
    if (rotationDeg != 0.0) {
        auto rotationRad = Base::toRadians(rotationDeg);
        result.m_first.RotateZ(rotationRad);
        result.m_second.RotateZ(rotationRad);
        result.m_overrideFirst.RotateZ(rotationRad);
        result.m_overrideSecond.RotateZ(rotationRad);
    }
    return result;
}



anglePoints::anglePoints()
{
    m_ends.first(Base::Vector3d(0.0, 0.0, 0.0));
    m_ends.second(Base::Vector3d(0.0, 0.0, 0.0));
    m_vertex = Base::Vector3d(0.0, 0.0, 0.0);
}

anglePoints::anglePoints(const anglePoints& ap) : m_ends(ap.ends()), m_vertex(ap.vertex()) {}

anglePoints& anglePoints::operator=(const anglePoints& ap)
{
    m_ends = ap.ends();
    m_vertex = ap.vertex();
    return *this;
}

// move the points by offset
void anglePoints::move(const Base::Vector3d& offset)
{
    m_ends.move(offset);
    m_vertex = m_vertex - offset;
}

// project the points onto the dvp's paper plane.
void anglePoints::project(const DrawViewPart* dvp)
{
    m_ends.project(dvp);
    m_vertex = dvp->projectPoint(m_vertex) * dvp->getScale();
}

// map the points onto the dvp's XY coordinate system
// obsolete. see above.
void anglePoints::mapToPage(const DrawViewPart* dvp)
{
    m_ends.mapToPage(dvp);

    gp_Trsf xOXYZ;
    gp_Ax3 OXYZ;
    xOXYZ.SetTransformation(OXYZ, gp_Ax3(dvp->getRotatedCS()));
    gp_Vec gvVertex = Base::convertTo<gp_Vec>(m_vertex).Transformed(xOXYZ);
    m_vertex = Base::convertTo<Base::Vector3d>(gvVertex);
}

// map the points onto the coordinate system used for drawing where -Y direction is "up"
// obsolete. see above
void anglePoints::invertY()
{
    m_ends.invertY();
    m_vertex = DU::invertY(m_vertex);
}

//! return unscaled, unrotated version of this anglePoints.  caller is responsible for
//! for ensuring this anglePoints is in scaled, rotated form before calling this method.
anglePoints anglePoints::toCanonicalForm(DrawViewPart* dvp) const
{
    anglePoints result;
    result.m_ends = m_ends.toCanonicalForm(dvp);
    result.m_vertex = CosmeticVertex::makeCanonicalPoint(dvp, m_vertex);
    return result;
}

//! return scaled and rotated version of this anglePoints.  caller is responsible for
//! for ensuring this anglePoints is in canonical form before calling this method.
anglePoints anglePoints::toDisplayForm(DrawViewPart* dvp) const
{
    anglePoints result;
    result.m_ends = m_ends.toDisplayForm(dvp);
    result.m_vertex = m_vertex * dvp->getScale();
    auto rotationDeg = dvp->Rotation.getValue();
    if (rotationDeg != 0.0) {
        auto rotationRad = Base::toRadians(rotationDeg);
        result.m_vertex.RotateZ(rotationRad);
    }
    return result;
}
void anglePoints::dump(const std::string& text) const
{
    Base::Console().message("anglePoints - %s\n", text.c_str());
    Base::Console().message("anglePoints - ends - first: %s  second: %s\n",
                            DU::formatVector(first()).c_str(), DU::formatVector(second()).c_str());
    Base::Console().message("anglePoints - vertex: %s\n", DU::formatVector(vertex()).c_str());
}

arcPoints::arcPoints() :
    isArc(false),
    radius(0.0),
    arcCW(false)
{
    center = Base::Vector3d(0.0, 0.0, 0.0);
    onCurve.first(Base::Vector3d(0.0, 0.0, 0.0));
    onCurve.second(Base::Vector3d(0.0, 0.0, 0.0));
    arcEnds.first(Base::Vector3d(0.0, 0.0, 0.0));
    arcEnds.second(Base::Vector3d(0.0, 0.0, 0.0));
    midArc = Base::Vector3d(0.0, 0.0, 0.0);
}

arcPoints& arcPoints::operator=(const arcPoints& ap)
{
    isArc = ap.isArc;
    radius = ap.radius;
    center = ap.center;
    onCurve = ap.onCurve;
    arcEnds = ap.arcEnds;
    midArc = ap.midArc;
    arcCW = ap.arcCW;
    return *this;
}

void arcPoints::move(const Base::Vector3d& offset)
{
    center = center - offset;
    onCurve.first(onCurve.first() - offset);
    onCurve.second(onCurve.second() - offset);
    arcEnds.first(arcEnds.first() - offset);
    arcEnds.second(arcEnds.second() - offset);
    midArc = midArc - offset;
}

void arcPoints::project(const DrawViewPart* dvp)
{
    radius = radius * dvp->getScale();
    center = dvp->projectPoint(center) * dvp->getScale();
    onCurve.first(dvp->projectPoint(onCurve.first()) * dvp->getScale());
    onCurve.second(dvp->projectPoint(onCurve.second()) * dvp->getScale());
    arcEnds.first(dvp->projectPoint(arcEnds.first()) * dvp->getScale());
    arcEnds.second(dvp->projectPoint(arcEnds.second()) * dvp->getScale());
    midArc = dvp->projectPoint(midArc) * dvp->getScale();
}

// obsolete. see above
void arcPoints::mapToPage(const DrawViewPart* dvp)
{
    gp_Trsf xOXYZ;
    gp_Ax3 OXYZ;
    xOXYZ.SetTransformation(OXYZ, gp_Ax3(dvp->getRotatedCS()));

    gp_Vec gvCenter = Base::convertTo<gp_Vec>(center).Transformed(xOXYZ);
    center = Base::convertTo<Base::Vector3d>(gvCenter);
    gp_Vec gvOnCurve1 = Base::convertTo<gp_Vec>(onCurve.first()).Transformed(xOXYZ);
    onCurve.first(Base::convertTo<Base::Vector3d>(gvOnCurve1));
    gp_Vec gvOnCurve2 = Base::convertTo<gp_Vec>(onCurve.second()).Transformed(xOXYZ);
    onCurve.second(Base::convertTo<Base::Vector3d>(gvOnCurve2));
    gp_Vec gvArcEnds1 = Base::convertTo<gp_Vec>(arcEnds.first()).Transformed(xOXYZ);
    arcEnds.first(Base::convertTo<Base::Vector3d>(gvArcEnds1));
    gp_Vec gvArcEnds2 = Base::convertTo<gp_Vec>(arcEnds.second()).Transformed(xOXYZ);
    arcEnds.second(Base::convertTo<Base::Vector3d>(gvArcEnds2));
    gp_Vec gvMidArc = Base::convertTo<gp_Vec>(midArc).Transformed(xOXYZ);
    midArc = Base::convertTo<Base::Vector3d>(gvMidArc);
}

// obsolete. see above
void arcPoints::invertY()
{
    center = DU::invertY(center);
    onCurve.invertY();
    arcEnds.invertY();
    midArc = DU::invertY(midArc);
}

//! return scaled and rotated version of this arcPoints.  caller is responsible for
//! for ensuring this arcPoints is in canonical form before calling this method.
arcPoints arcPoints::toDisplayForm(DrawViewPart* dvp) const
{
    arcPoints result;
    result.onCurve = onCurve.toDisplayForm(dvp);
    result.arcEnds = arcEnds.toDisplayForm(dvp);
    result.center = center * dvp->getScale();
    result.midArc = midArc * dvp->getScale();
    result.radius = radius * dvp->getScale();
    auto rotationDeg = dvp->Rotation.getValue();
    if (rotationDeg != 0.0) {
        auto rotationRad = Base::toRadians(rotationDeg);
        result.center.RotateZ(rotationRad);
        result.midArc.RotateZ(rotationRad);
    }
    return result;
}

//! return unscaled, unrotated version of this arcPoints.  caller is responsible for
//! for ensuring this arcPoints is in scaled, rotated form before calling this method.
arcPoints arcPoints::toCanonicalForm(DrawViewPart* dvp) const
{
    arcPoints result;
    result.onCurve = onCurve.toCanonicalForm(dvp);
    result.arcEnds = arcEnds.toCanonicalForm(dvp);
    result.center = CosmeticVertex::makeCanonicalPoint(dvp, center);
    result.midArc = CosmeticVertex::makeCanonicalPoint(dvp, midArc);
    result.radius = radius / dvp->getScale();
    return result;
}


void arcPoints::dump(const std::string& text) const
{
    Base::Console().message("arcPoints - %s\n", text.c_str());
    Base::Console().message("arcPoints - radius: %.3f center: %s\n", radius,
                            DrawUtil::formatVector(center).c_str());
    Base::Console().message("arcPoints - isArc: %d arcCW: %d\n", isArc, arcCW);
    Base::Console().message("arcPoints - onCurve: %s  %s\n",
                            DrawUtil::formatVector(onCurve.first()).c_str(),
                            DrawUtil::formatVector(onCurve.second()).c_str());
    Base::Console().message("arcPoints - arcEnds: %s  %s\n",
                            DrawUtil::formatVector(arcEnds.first()).c_str(),
                            DrawUtil::formatVector(arcEnds.second()).c_str());
    Base::Console().message("arcPoints - midArc: %s\n", DrawUtil::formatVector(midArc).c_str());
}


areaPoint::areaPoint() :
    area(0.0),
    actualArea(0.0),
    center(Base::Vector3d())
{
}

areaPoint& areaPoint::operator=(const areaPoint& ap)
{
    area = ap.area;
    center = ap.center;
    actualArea = ap.actualArea;
    return *this;
}

void areaPoint::move(const Base::Vector3d& offset)
{
    center = center - offset;
}

void areaPoint::project(const DrawViewPart* dvp)
{
    center = dvp->projectPoint(center) * dvp->getScale();
}

void areaPoint::invertY()
{
    center = DU::invertY(center);
}

void areaPoint::dump(const std::string& text) const
{
    Base::Console().message("areaPoint - %s\n", text.c_str());
    Base::Console().message("areaPoint - area: %.3f center: %s\n", area,
        DrawUtil::formatVector(center).c_str());
}
