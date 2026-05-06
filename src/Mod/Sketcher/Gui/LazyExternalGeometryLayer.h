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

#pragma once

#include <memory>
#include <set>
#include <string>
#include <vector>

#include <Base/Vector3D.h>

class SoCoordinate3;
class SoDrawStyle;
class SoLineSet;
class SoMarkerSet;
class SoMaterial;
class SoNode;
class SoPickedPoint;
class SoSeparator;

namespace App
{
class DocumentObject;
}

namespace Part
{
class Geometry;
}

namespace Sketcher
{
class SketchObject;
}

namespace SketcherGui
{

struct DrawingParameters;

/** Edit-session-only virtual external geometry layer.
 *
 * The layer owns no document state. It keeps projected previews of source Edge/Vertex references so
 * they can participate in edit-mode hover/preselection and the generic constraint-pick sequence. A
 * reference is materialized with SketchObject::addExternal() only by the constraint commit path.
 */
class LazyExternalGeometryLayer
{
public:
    enum class ElementType
    {
        Edge,
        Vertex
    };

    struct Element
    {
        int id = -1;
        App::DocumentObject* sourceObject = nullptr;
        std::string subName;
        ElementType type = ElementType::Edge;
        bool defining = false;
        bool intersection = false;
        std::vector<std::unique_ptr<Part::Geometry>> geometry;
    };

    LazyExternalGeometryLayer();
    ~LazyExternalGeometryLayer();

    LazyExternalGeometryLayer(const LazyExternalGeometryLayer&) = delete;
    LazyExternalGeometryLayer& operator=(const LazyExternalGeometryLayer&) = delete;

    void attachTo(SoSeparator* editRoot);
    void detach();

    void clear();
    void rebuildFromSupport(Sketcher::SketchObject* sketch);

    int addReference(Sketcher::SketchObject* sketch,
                     App::DocumentObject* sourceObject,
                     const std::string& subName,
                     bool defining,
                     bool intersection);

    const Element* getElement(int id) const;
    const Element* getElementBySource(App::DocumentObject* sourceObject,
                                      const std::string& subName,
                                      bool intersection) const;

    bool empty() const;
    const std::vector<Element>& getElements() const;

    void setPreselectedElement(int id);
    void clearPreselectedElement();
    void setSelectedElement(int id, bool selected);
    void clearSelectedElements();

    void draw(const DrawingParameters& parameters, int viewOrientationFactor);

    bool isLazyPointNode(const SoNode* node) const;
    bool isLazyCurveNode(const SoNode* node) const;
    int getPickedPointElementId(const SoPickedPoint* pickedPoint) const;
    int getPickedCurveElementId(const SoPickedPoint* pickedPoint) const;

private:
    void createNodes();
    void resetNodes();
    void appendGeometryToCoin(const Element& element,
                              std::vector<Base::Vector3d>& points,
                              std::vector<Base::Vector3d>& curveCoords,
                              std::vector<int32_t>& curveVertexCounts,
                              bool updatePickMap);
    bool isVisibleElement(const Element& element) const;

private:
    SoSeparator* parentRoot = nullptr;
    SoSeparator* root = nullptr;
    SoMaterial* pickMaterial = nullptr;
    SoMaterial* highlightMaterial = nullptr;
    SoCoordinate3* pointCoordinates = nullptr;
    SoCoordinate3* curveCoordinates = nullptr;
    SoCoordinate3* highlightPointCoordinates = nullptr;
    SoCoordinate3* highlightCurveCoordinates = nullptr;
    SoDrawStyle* pointDrawStyle = nullptr;
    SoDrawStyle* curveDrawStyle = nullptr;
    SoDrawStyle* highlightPointDrawStyle = nullptr;
    SoDrawStyle* highlightCurveDrawStyle = nullptr;
    SoMarkerSet* pointSet = nullptr;
    SoLineSet* curveSet = nullptr;
    SoMarkerSet* highlightPointSet = nullptr;
    SoLineSet* highlightCurveSet = nullptr;

    std::vector<Element> elements;
    std::vector<int> pointIndexToElementId;
    std::vector<int> curveIndexToElementId;
    std::set<int> preselectedElementIds;
    std::set<int> selectedElementIds;
    int nextId = 1;
};

}  // namespace SketcherGui
