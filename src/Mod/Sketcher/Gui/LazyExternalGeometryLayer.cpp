// SPDX-License-Identifier: LGPL-2.1-or-later

/***************************************************************************
 *   Copyright (c) 2026                                                     *
 *                                                                         *
 *   This file is part of the FreeCAD CAx development system.              *
 *                                                                         *
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Library General Public           *
 *   License as published by the Free Software Foundation; either          *
 *   version 2 of the License, or (at your option) any later version.      *
 ***************************************************************************/

#include <FCConfig.h>

#include <algorithm>

#include <Inventor/details/SoLineDetail.h>
#include <Inventor/details/SoPointDetail.h>
#include <Inventor/nodes/SoCoordinate3.h>
#include <Inventor/nodes/SoDrawStyle.h>
#include <Inventor/nodes/SoLineSet.h>
#include <Inventor/nodes/SoMarkerSet.h>
#include <Inventor/nodes/SoMaterial.h>
#include <Inventor/nodes/SoSeparator.h>
#include <Inventor/SoPickedPoint.h>

#include <TopAbs_ShapeEnum.hxx>

#include <Base/Tools.h>
#include <Gui/Inventor/MarkerBitmaps.h>
#include <Mod/Part/App/Geometry.h>
#include <Mod/Part/App/TopoShape.h>
#include <Mod/Sketcher/App/SketchObject.h>

#include "EditModeCoinManagerParameters.h"
#include "LazyExternalGeometryLayer.h"

using namespace SketcherGui;

namespace
{
constexpr double LazyLayerZOffset = 0.5;

bool isEdgeSubName(const std::string& subName)
{
    return subName.size() > 4 && subName.substr(0, 4) == "Edge";
}

bool isVertexSubName(const std::string& subName)
{
    return subName.size() > 6 && subName.substr(0, 6) == "Vertex";
}

void appendSampledCurve(const Part::Geometry* geometry,
                        std::vector<Base::Vector3d>& curveCoords,
                        std::vector<int32_t>& curveVertexCounts,
                        int segments)
{
    if (auto line = freecad_cast<const Part::GeomLineSegment*>(geometry)) {
        curveCoords.push_back(line->getStartPoint());
        curveCoords.push_back(line->getEndPoint());
        curveVertexCounts.push_back(2);
        return;
    }

    if (auto curve = freecad_cast<const Part::GeomCurve*>(geometry)) {
        int sampleCount = std::max(segments, 8);
        double first = curve->getFirstParameter();
        double last = curve->getLastParameter();
        double step = (last - first) / sampleCount;

        for (int i = 0; i < sampleCount; ++i) {
            curveCoords.push_back(curve->value(first + i * step));
        }
        curveCoords.push_back(curve->value(last));
        curveVertexCounts.push_back(sampleCount + 1);
    }
}
}  // namespace

LazyExternalGeometryLayer::LazyExternalGeometryLayer() = default;

LazyExternalGeometryLayer::~LazyExternalGeometryLayer()
{
    detach();
}

void LazyExternalGeometryLayer::attachTo(SoSeparator* editRoot)
{
    if (parentRoot == editRoot && root) {
        return;
    }

    detach();
    parentRoot = editRoot;
    createNodes();

    if (parentRoot && root) {
        parentRoot->addChild(root);
    }
}

void LazyExternalGeometryLayer::detach()
{
    if (parentRoot && root) {
        parentRoot->removeChild(root);
    }

    parentRoot = nullptr;
    resetNodes();
}

void LazyExternalGeometryLayer::resetNodes()
{
    if (root) {
        root->unref();
    }

    root = nullptr;
    material = nullptr;
    pointCoordinates = nullptr;
    curveCoordinates = nullptr;
    pointDrawStyle = nullptr;
    curveDrawStyle = nullptr;
    pointSet = nullptr;
    curveSet = nullptr;
}

void LazyExternalGeometryLayer::createNodes()
{
    root = new SoSeparator;
    root->ref();
    root->setName("Sketch_LazyExternalGeometryRoot");
    root->renderCaching = SoSeparator::OFF;

    material = new SoMaterial;
    material->setName("LazyExternalGeometryMaterial");
    root->addChild(material);

    pointCoordinates = new SoCoordinate3;
    pointCoordinates->setName("LazyExternalGeometryPointCoordinates");
    root->addChild(pointCoordinates);

    pointDrawStyle = new SoDrawStyle;
    pointDrawStyle->setName("LazyExternalGeometryPointDrawStyle");
    root->addChild(pointDrawStyle);

    pointSet = new SoMarkerSet;
    pointSet->setName("LazyExternalGeometryPointSet");
    root->addChild(pointSet);

    curveCoordinates = new SoCoordinate3;
    curveCoordinates->setName("LazyExternalGeometryCurveCoordinates");
    root->addChild(curveCoordinates);

    curveDrawStyle = new SoDrawStyle;
    curveDrawStyle->setName("LazyExternalGeometryCurveDrawStyle");
    root->addChild(curveDrawStyle);

    curveSet = new SoLineSet;
    curveSet->setName("LazyExternalGeometryCurveSet");
    root->addChild(curveSet);
}

void LazyExternalGeometryLayer::clear()
{
    elements.clear();
    pointIndexToElementId.clear();
    curveIndexToElementId.clear();
    nextId = 1;

    if (pointCoordinates) {
        pointCoordinates->point.setNum(0);
    }
    if (curveCoordinates) {
        curveCoordinates->point.setNum(0);
    }
    if (curveSet) {
        curveSet->numVertices.setNum(0);
    }
}

void LazyExternalGeometryLayer::rebuildFromSupport(Sketcher::SketchObject* sketch)
{
    clear();

    if (!sketch) {
        return;
    }

    App::DocumentObject* support = sketch->AttachmentSupport.getValue();
    if (!support) {
        return;
    }

    try {
        auto shape = Part::Feature::getTopoShape(
            support,
            Part::ShapeOption::ResolveLink | Part::ShapeOption::Transform
        );

        const int edgeCount = shape.countSubShapes(TopAbs_EDGE);
        for (int i = 1; i <= edgeCount; ++i) {
            addReference(sketch, support, "Edge" + std::to_string(i), false, false);
        }

        const int vertexCount = shape.countSubShapes(TopAbs_VERTEX);
        for (int i = 1; i <= vertexCount; ++i) {
            addReference(sketch, support, "Vertex" + std::to_string(i), false, false);
        }
    }
    catch (const Base::Exception&) {
        clear();
    }
}

int LazyExternalGeometryLayer::addReference(Sketcher::SketchObject* sketch,
                                            App::DocumentObject* sourceObject,
                                            const std::string& subName,
                                            bool defining,
                                            bool intersection)
{
    if (!sketch || !sourceObject) {
        return -1;
    }

    const bool isEdge = isEdgeSubName(subName);
    const bool isVertex = isVertexSubName(subName);
    if (!isEdge && !isVertex) {
        return -1;
    }

    if (const Element* existing = getElementBySource(sourceObject, subName, intersection)) {
        return existing->id;
    }

    auto preview = sketch->buildExternalGeometryPreview(sourceObject, subName.c_str(), intersection);
    if (preview.empty()) {
        return -1;
    }

    Element element;
    element.id = nextId++;
    element.sourceObject = sourceObject;
    element.subName = subName;
    element.type = isEdge ? ElementType::Edge : ElementType::Vertex;
    element.defining = defining;
    element.intersection = intersection;
    element.geometry = std::move(preview);

    const int id = element.id;
    elements.emplace_back(std::move(element));
    return id;
}

const LazyExternalGeometryLayer::Element* LazyExternalGeometryLayer::getElement(int id) const
{
    auto it = std::find_if(elements.begin(), elements.end(), [id](const Element& element) {
        return element.id == id;
    });

    return it == elements.end() ? nullptr : &(*it);
}

const LazyExternalGeometryLayer::Element* LazyExternalGeometryLayer::getElementBySource(
    App::DocumentObject* sourceObject,
    const std::string& subName,
    bool intersection) const
{
    auto it = std::find_if(elements.begin(), elements.end(), [&](const Element& element) {
        return element.sourceObject == sourceObject && element.subName == subName
            && element.intersection == intersection;
    });

    return it == elements.end() ? nullptr : &(*it);
}

bool LazyExternalGeometryLayer::empty() const
{
    return elements.empty();
}

const std::vector<LazyExternalGeometryLayer::Element>& LazyExternalGeometryLayer::getElements() const
{
    return elements;
}

void LazyExternalGeometryLayer::appendGeometryToCoin(const Element& element,
                                                     std::vector<Base::Vector3d>& points,
                                                     std::vector<Base::Vector3d>& curveCoords,
                                                     std::vector<int32_t>& curveVertexCounts)
{
    for (const auto& geometry : element.geometry) {
        if (!geometry) {
            continue;
        }

        if (auto point = freecad_cast<const Part::GeomPoint*>(geometry.get())) {
            pointIndexToElementId.push_back(element.id);
            points.push_back(point->getPoint());
            continue;
        }

        const auto curvesBefore = curveVertexCounts.size();
        appendSampledCurve(geometry.get(), curveCoords, curveVertexCounts, 40);
        const auto curvesAfter = curveVertexCounts.size();
        for (auto i = curvesBefore; i < curvesAfter; ++i) {
            const int segmentCount = std::max<int>(0, curveVertexCounts[i] - 1);
            for (int segment = 0; segment < segmentCount; ++segment) {
                curveIndexToElementId.push_back(element.id);
            }
        }
    }
}

void LazyExternalGeometryLayer::draw(const DrawingParameters& parameters, int viewOrientationFactor)
{
    if (!root) {
        return;
    }

    pointIndexToElementId.clear();
    curveIndexToElementId.clear();

    std::vector<Base::Vector3d> points;
    std::vector<Base::Vector3d> curveCoords;
    std::vector<int32_t> curveVertexCounts;

    for (const auto& element : elements) {
        appendGeometryToCoin(element, points, curveCoords, curveVertexCounts);
    }

    const float pointz = viewOrientationFactor * parameters.zLowPoints * LazyLayerZOffset;
    const float linez = viewOrientationFactor * parameters.zLowLines * LazyLayerZOffset;

    material->diffuseColor = DrawingParameters::CurveExternalColor;
    pointDrawStyle->pointSize = 8 * parameters.pixelScalingFactor;
    pointSet->markerIndex = Gui::Inventor::MarkerBitmaps::getMarkerIndex(
        "CIRCLE_FILLED",
        parameters.markerSize
    );

    curveDrawStyle->lineWidth = parameters.ExternalWidth * parameters.pixelScalingFactor;
    curveDrawStyle->linePattern = parameters.ExternalPattern;
    curveDrawStyle->linePatternScaleFactor = 2;

    pointCoordinates->point.setNum(points.size());
    SbVec3f* pverts = pointCoordinates->point.startEditing();
    int i = 0;
    for (const auto& point : points) {
        pverts[i++].setValue(point.x, point.y, pointz);
    }
    pointCoordinates->point.finishEditing();

    curveCoordinates->point.setNum(curveCoords.size());
    curveSet->numVertices.setNum(curveVertexCounts.size());

    SbVec3f* cverts = curveCoordinates->point.startEditing();
    i = 0;
    for (const auto& point : curveCoords) {
        cverts[i++].setValue(point.x, point.y, linez);
    }
    curveCoordinates->point.finishEditing();

    int32_t* vertexCounts = curveSet->numVertices.startEditing();
    i = 0;
    for (auto count : curveVertexCounts) {
        vertexCounts[i++] = count;
    }
    curveSet->numVertices.finishEditing();
}

bool LazyExternalGeometryLayer::isLazyPointNode(const SoNode* node) const
{
    return node && node == pointSet;
}

bool LazyExternalGeometryLayer::isLazyCurveNode(const SoNode* node) const
{
    return node && node == curveSet;
}

int LazyExternalGeometryLayer::getPickedPointElementId(const SoPickedPoint* pickedPoint) const
{
    if (!pickedPoint || !pointSet) {
        return -1;
    }

    const SoDetail* detail = pickedPoint->getDetail(pointSet);
    if (!detail || detail->getTypeId() != SoPointDetail::getClassTypeId()) {
        return -1;
    }

    int index = static_cast<const SoPointDetail*>(detail)->getCoordinateIndex();
    if (index < 0 || index >= static_cast<int>(pointIndexToElementId.size())) {
        return -1;
    }

    return pointIndexToElementId[index];
}

int LazyExternalGeometryLayer::getPickedCurveElementId(const SoPickedPoint* pickedPoint) const
{
    if (!pickedPoint || !curveSet) {
        return -1;
    }

    const SoDetail* detail = pickedPoint->getDetail(curveSet);
    if (!detail || detail->getTypeId() != SoLineDetail::getClassTypeId()) {
        return -1;
    }

    int index = static_cast<const SoLineDetail*>(detail)->getLineIndex();
    if (index < 0 || index >= static_cast<int>(curveIndexToElementId.size())) {
        return -1;
    }

    return curveIndexToElementId[index];
}
