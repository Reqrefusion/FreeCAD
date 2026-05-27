// SPDX-License-Identifier: LGPL-2.1-or-later

/***************************************************************************
 *   Copyright (c) 2026 FreeCAD contributors                               *
 *                                                                         *
 *   This file is part of the FreeCAD CAx development system.              *
 *                                                                         *
 *   This library is free software; you can redistribute it and/or         *
 *   modify it under the terms of the GNU Lesser General Public            *
 *   License as published by the Free Software Foundation; either          *
 *   version 2.1 of the License, or (at your option) any later version.    *
 *                                                                         *
 ***************************************************************************/

#include <algorithm>
#include <cmath>
#include <exception>
#include <limits>
#include <numeric>
#include <set>
#include <sstream>
#include <vector>

#include <Approx_ParametrizationType.hxx>
#include <BRepAdaptor_CompCurve.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepExtrema_DistShapeShape.hxx>
#include <GeomAPI_PointsToBSplineSurface.hxx>
#include <GeomAPI_ProjectPointOnSurf.hxx>
#include <GeomAbs_Shape.hxx>
#include <Geom_BSplineSurface.hxx>
#include <Geom_Surface.hxx>
#include <Precision.hxx>
#include <Standard_Failure.hxx>
#include <TColgp_Array2OfPnt.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Pnt.hxx>

#include <Mod/Part/App/PartFeature.h>
#include <Mod/Part/App/TopoShape.h>

#include "FeatureThroughCurveMesh.h"


using namespace Surface;

namespace
{

const App::PropertyFloatConstraint::Constraints ToleranceRange = {0.0, 100.0, 0.001};
const App::PropertyIntegerConstraint::Constraints SamplesRange = {4, 200, 1};
constexpr double ParameterEpsilon = 1.0e-9;

void raiseFailure(const std::string& message)
{
    throw Standard_Failure(message.c_str());
}

double clamp01(double value)
{
    return std::max(0.0, std::min(1.0, value));
}

gp_Pnt lerp(const gp_Pnt& p1, const gp_Pnt& p2, double t)
{
    return gp_Pnt(
        p1.X() + t * (p2.X() - p1.X()),
        p1.Y() + t * (p2.Y() - p1.Y()),
        p1.Z() + t * (p2.Z() - p1.Z())
    );
}

gp_Pnt addSubtract(const gp_Pnt& p1, const gp_Pnt& p2, const gp_Pnt& p3)
{
    return gp_Pnt(
        p1.X() + p2.X() - p3.X(),
        p1.Y() + p2.Y() - p3.Y(),
        p1.Z() + p2.Z() - p3.Z()
    );
}

gp_Pnt blendGordonContributions(
    const gp_Pnt& primary,
    const gp_Pnt& cross,
    const gp_Pnt& net,
    double primaryWeight,
    double crossWeight
)
{
    const double netWeight = 1.0 - primaryWeight - crossWeight;
    return gp_Pnt(
        primaryWeight * primary.X() + crossWeight * cross.X() + netWeight * net.X(),
        primaryWeight * primary.Y() + crossWeight * cross.Y() + netWeight * net.Y(),
        primaryWeight * primary.Z() + crossWeight * cross.Z() + netWeight * net.Z()
    );
}

gp_Pnt midpoint(const gp_Pnt& p1, const gp_Pnt& p2)
{
    return lerp(p1, p2, 0.5);
}

struct CurveShape
{
    TopoDS_Shape shape;
    bool reversed {false};

    bool isEdge() const
    {
        return !shape.IsNull() && shape.ShapeType() == TopAbs_EDGE;
    }

    bool isWire() const
    {
        return !shape.IsNull() && shape.ShapeType() == TopAbs_WIRE;
    }

    double firstParameter() const
    {
        if (isEdge()) {
            BRepAdaptor_Curve curve(TopoDS::Edge(shape));
            return curve.FirstParameter();
        }
        if (isWire()) {
            BRepAdaptor_CompCurve curve(TopoDS::Wire(shape), Standard_True);
            return curve.FirstParameter();
        }
        raiseFailure("Curve mesh input is not an edge or wire.");
        return 0.0;
    }

    double lastParameter() const
    {
        if (isEdge()) {
            BRepAdaptor_Curve curve(TopoDS::Edge(shape));
            return curve.LastParameter();
        }
        if (isWire()) {
            BRepAdaptor_CompCurve curve(TopoDS::Wire(shape), Standard_True);
            return curve.LastParameter();
        }
        raiseFailure("Curve mesh input is not an edge or wire.");
        return 1.0;
    }

    double rawParameterFromNormalized(double normalized) const
    {
        const double first = firstParameter();
        const double last = lastParameter();
        return first + clamp01(normalized) * (last - first);
    }

    double rawNormalizedFromParameter(double parameter) const
    {
        const double first = firstParameter();
        const double last = lastParameter();
        if (std::abs(last - first) <= ParameterEpsilon) {
            return 0.0;
        }
        return clamp01((parameter - first) / (last - first));
    }

    double orientedNormalizedFromRaw(double rawNormalized) const
    {
        return reversed ? 1.0 - clamp01(rawNormalized) : clamp01(rawNormalized);
    }

    gp_Pnt valueAtRawNormalized(double normalized) const
    {
        const double parameter = rawParameterFromNormalized(normalized);
        if (isEdge()) {
            BRepAdaptor_Curve curve(TopoDS::Edge(shape));
            return curve.Value(parameter);
        }
        if (isWire()) {
            BRepAdaptor_CompCurve curve(TopoDS::Wire(shape), Standard_True);
            return curve.Value(parameter);
        }
        raiseFailure("Curve mesh input is not an edge or wire.");
        return gp_Pnt();
    }

    gp_Pnt valueAtOrientedNormalized(double normalized) const
    {
        const double rawNormalized = reversed ? 1.0 - clamp01(normalized) : clamp01(normalized);
        return valueAtRawNormalized(rawNormalized);
    }
};

struct IntersectionInfo
{
    gp_Pnt point;
    double primaryRawParam {0.0};
    double crossRawParam {0.0};
    double gap {0.0};
};

struct LinkKey
{
    App::DocumentObject* object {nullptr};
    std::string subName;

    bool operator<(const LinkKey& other) const
    {
        if (object != other.object) {
            return object < other.object;
        }
        return subName < other.subName;
    }
};

std::vector<LinkKey> flattenedKeys(const App::PropertyLinkSubList& links)
{
    std::vector<LinkKey> keys;
    for (const auto& value : links.getSubListValues()) {
        App::DocumentObject* object = value.first;
        if (value.second.empty()) {
            keys.push_back({object, std::string()});
            continue;
        }
        for (const std::string& subName : value.second) {
            keys.push_back({object, subName});
        }
    }
    return keys;
}

struct FamilyReferenceSummary
{
    std::set<LinkKey> keys;
    std::set<App::DocumentObject*> wholeObjects;
    std::set<App::DocumentObject*> subElementObjects;
};

FamilyReferenceSummary validateFamilyReferences(
    const App::PropertyLinkSubList& links,
    App::DocumentObject* owner,
    const char* familyName
)
{
    FamilyReferenceSummary summary;
    for (const LinkKey& key : flattenedKeys(links)) {
        if (!key.object) {
            std::ostringstream str;
            str << "Through Curve Mesh contains an empty " << familyName << " curve reference.";
            raiseFailure(str.str());
        }
        if (key.object == owner) {
            std::ostringstream str;
            str << "Through Curve Mesh cannot reference itself in the " << familyName << " curve list.";
            raiseFailure(str.str());
        }
        if (!summary.keys.insert(key).second) {
            std::ostringstream str;
            str << "Through Curve Mesh contains duplicate " << familyName << " curve references.";
            raiseFailure(str.str());
        }

        if (key.subName.empty()) {
            summary.wholeObjects.insert(key.object);
        }
        else {
            summary.subElementObjects.insert(key.object);
        }
    }

    for (App::DocumentObject* object : summary.wholeObjects) {
        if (summary.subElementObjects.count(object) != 0) {
            std::ostringstream str;
            str << "Through Curve Mesh cannot mix whole-object and subelement references "
                << "for the same object in the " << familyName << " curve list.";
            raiseFailure(str.str());
        }
    }

    return summary;
}

void validateInputReferences(
    const App::PropertyLinkSubList& primaryLinks,
    const App::PropertyLinkSubList& crossLinks,
    App::DocumentObject* owner
)
{
    const FamilyReferenceSummary primary = validateFamilyReferences(primaryLinks, owner, "primary");
    const FamilyReferenceSummary cross = validateFamilyReferences(crossLinks, owner, "cross");

    for (const LinkKey& key : cross.keys) {
        if (primary.keys.count(key) != 0) {
            raiseFailure("The same curve cannot be used as both a primary and a cross curve.");
        }
    }

    for (App::DocumentObject* object : primary.wholeObjects) {
        if (cross.wholeObjects.count(object) != 0 || cross.subElementObjects.count(object) != 0) {
            raiseFailure("A whole-object curve reference cannot be mixed with cross-family references to the same object.");
        }
    }
    for (App::DocumentObject* object : cross.wholeObjects) {
        if (primary.subElementObjects.count(object) != 0) {
            raiseFailure("A whole-object curve reference cannot be mixed with cross-family references to the same object.");
        }
    }
}


bool containsForbiddenTopology(const TopoDS_Shape& shape)
{
    if (shape.IsNull()) {
        return false;
    }
    if (shape.ShapeType() == TopAbs_FACE || shape.ShapeType() == TopAbs_SHELL || shape.ShapeType() == TopAbs_SOLID) {
        return true;
    }
    for (TopExp_Explorer xp(shape, TopAbs_FACE); xp.More(); xp.Next()) {
        return true;
    }
    for (TopExp_Explorer xp(shape, TopAbs_SHELL); xp.More(); xp.Next()) {
        return true;
    }
    for (TopExp_Explorer xp(shape, TopAbs_SOLID); xp.More(); xp.Next()) {
        return true;
    }
    return false;
}

std::vector<TopoDS_Edge> collectEdgesFromCurveShape(const TopoDS_Shape& shape)
{
    std::vector<TopoDS_Edge> edges;
    if (shape.IsNull()) {
        return edges;
    }

    if (containsForbiddenTopology(shape)) {
        raiseFailure("Through Curve Mesh inputs must be edges, wires, or compounds of edges/wires; faces and solids are not accepted.");
    }

    if (shape.ShapeType() == TopAbs_EDGE) {
        edges.push_back(TopoDS::Edge(shape));
        return edges;
    }

    for (TopExp_Explorer xp(shape, TopAbs_EDGE); xp.More(); xp.Next()) {
        edges.push_back(TopoDS::Edge(xp.Current()));
    }
    return edges;
}

TopoDS_Shape makeSingleCurveShape(const TopoDS_Shape& shape)
{
    if (shape.IsNull()) {
        raiseFailure("Through Curve Mesh input did not resolve to a curve. Select an edge, a wire, or a connected selection family.");
    }

    if (containsForbiddenTopology(shape)) {
        raiseFailure("Through Curve Mesh inputs must be edges, wires, or compounds of edges/wires; faces and solids are not accepted.");
    }

    if (shape.ShapeType() == TopAbs_EDGE || shape.ShapeType() == TopAbs_WIRE) {
        return shape;
    }

    std::vector<TopoDS_Edge> edges = collectEdgesFromCurveShape(shape);
    if (edges.empty()) {
        raiseFailure("Through Curve Mesh input did not resolve to a curve. Select an edge, a wire, or a connected selection family.");
    }
    if (edges.size() == 1) {
        return edges.front();
    }

    BRepBuilderAPI_MakeWire wireBuilder;
    for (const TopoDS_Edge& edge : edges) {
        wireBuilder.Add(edge);
    }

    if (!wireBuilder.IsDone()) {
        raiseFailure("Through Curve Mesh selection family must form one connected wire. Split disconnected selections into separate curves.");
    }

    return wireBuilder.Wire();
}

TopoDS_Shape makeSingleCurveShape(
    const Part::TopoShape& topo,
    const std::vector<std::string>& subNames
)
{
    if (subNames.empty()) {
        return makeSingleCurveShape(topo.getShape());
    }

    if (subNames.size() == 1) {
        return makeSingleCurveShape(topo.getSubShape(subNames.front().c_str()));
    }

    BRepBuilderAPI_MakeWire wireBuilder;
    bool addedAny = false;
    for (const std::string& subName : subNames) {
        TopoDS_Shape subShape = topo.getSubShape(subName.c_str());
        std::vector<TopoDS_Edge> edges = collectEdgesFromCurveShape(subShape);
        if (edges.empty()) {
            std::ostringstream str;
            str << "Through Curve Mesh selection family contains a non-curve subelement: " << subName;
            raiseFailure(str.str());
        }
        for (const TopoDS_Edge& edge : edges) {
            wireBuilder.Add(edge);
            addedAny = true;
        }
    }

    if (!addedAny) {
        raiseFailure("Through Curve Mesh selection family did not resolve to a curve.");
    }
    if (!wireBuilder.IsDone()) {
        raiseFailure("Through Curve Mesh selection family must form one connected wire. Split disconnected selections into separate curves.");
    }

    return wireBuilder.Wire();
}

TopoDS_Shape makeSingleCurveShape(
    const std::vector<App::DocumentObject*>& objects,
    const std::vector<std::string>& subNames,
    std::size_t begin,
    std::size_t count
)
{
    if (count == 0 || begin + count > objects.size() || begin + count > subNames.size()) {
        raiseFailure("Through Curve Mesh has an invalid curve-family grouping.");
    }

    if (count == 1) {
        App::DocumentObject* object = objects[begin];
        if (!object || !object->isDerivedFrom<Part::Feature>()) {
            raiseFailure("Through Curve Mesh inputs must be Part::Feature objects.");
        }
        const Part::TopoShape& topo = static_cast<Part::Feature*>(object)->Shape.getShape();
        std::vector<std::string> singleSubNames;
        if (!subNames[begin].empty()) {
            singleSubNames.push_back(subNames[begin]);
        }
        return makeSingleCurveShape(topo, singleSubNames);
    }

    BRepBuilderAPI_MakeWire wireBuilder;
    bool addedAny = false;
    for (std::size_t index = begin; index < begin + count; ++index) {
        App::DocumentObject* object = objects[index];
        if (!object || !object->isDerivedFrom<Part::Feature>()) {
            raiseFailure("Through Curve Mesh inputs must be Part::Feature objects.");
        }

        const Part::TopoShape& topo = static_cast<Part::Feature*>(object)->Shape.getShape();
        TopoDS_Shape shape = subNames[index].empty()
            ? topo.getShape()
            : topo.getSubShape(subNames[index].c_str());
        const std::vector<TopoDS_Edge> edges = collectEdgesFromCurveShape(shape);
        if (edges.empty()) {
            std::ostringstream str;
            str << "Through Curve Mesh selection family contains a non-curve entry: "
                << subNames[index];
            raiseFailure(str.str());
        }
        for (const TopoDS_Edge& edge : edges) {
            wireBuilder.Add(edge);
            addedAny = true;
        }
    }

    if (!addedAny) {
        raiseFailure("Through Curve Mesh selection family did not resolve to a curve.");
    }
    if (!wireBuilder.IsDone()) {
        raiseFailure("Through Curve Mesh selection family must form one connected wire. Split disconnected selections into separate rows.");
    }

    return wireBuilder.Wire();
}

std::vector<std::size_t> normalizedGroupSizes(
    const App::PropertyIntegerList& groupSizes,
    std::size_t referenceCount
)
{
    std::vector<std::size_t> groups;
    const std::vector<long> values = groupSizes.getValues();
    if (values.empty()) {
        groups.assign(referenceCount, 1);
        return groups;
    }

    std::size_t total = 0;
    groups.reserve(values.size());
    for (long value : values) {
        if (value <= 0) {
            raiseFailure("Through Curve Mesh has an invalid curve-family group size.");
        }
        groups.push_back(static_cast<std::size_t>(value));
        total += static_cast<std::size_t>(value);
    }

    if (total != referenceCount) {
        raiseFailure("Through Curve Mesh curve-family grouping is inconsistent with the linked curves.");
    }
    return groups;
}

std::vector<CurveShape> collectCurves(
    const App::PropertyLinkSubList& links,
    const App::PropertyIntegerList& groupSizes
)
{
    std::vector<CurveShape> curves;
    std::vector<App::DocumentObject*> objects = links.getValues();
    std::vector<std::string> subNames = links.getSubValues();
    if (objects.size() != subNames.size()) {
        raiseFailure("Through Curve Mesh has inconsistent curve references.");
    }

    const std::vector<std::size_t> groups = normalizedGroupSizes(groupSizes, objects.size());
    std::size_t offset = 0;
    curves.reserve(groups.size());
    for (std::size_t groupSize : groups) {
        curves.push_back({makeSingleCurveShape(objects, subNames, offset, groupSize), false});
        offset += groupSize;
    }

    return curves;
}

double distanceSquaredAt(const CurveShape& curve, const gp_Pnt& point, double normalized)
{
    return curve.valueAtRawNormalized(normalized).SquareDistance(point);
}

double nearestRawNormalizedParameter(const CurveShape& curve, const gp_Pnt& point)
{
    constexpr int coarseSamples = 80;
    double best = 0.0;
    double bestDistance = distanceSquaredAt(curve, point, best);

    for (int i = 1; i <= coarseSamples; ++i) {
        const double parameter = static_cast<double>(i) / static_cast<double>(coarseSamples);
        const double dist = distanceSquaredAt(curve, point, parameter);
        if (dist < bestDistance) {
            best = parameter;
            bestDistance = dist;
        }
    }

    double lower = std::max(0.0, best - 1.0 / static_cast<double>(coarseSamples));
    double upper = std::min(1.0, best + 1.0 / static_cast<double>(coarseSamples));

    for (int i = 0; i < 45; ++i) {
        const double m1 = lower + (upper - lower) / 3.0;
        const double m2 = upper - (upper - lower) / 3.0;
        if (distanceSquaredAt(curve, point, m1) < distanceSquaredAt(curve, point, m2)) {
            upper = m2;
        }
        else {
            lower = m1;
        }
    }

    return clamp01(0.5 * (lower + upper));
}

IntersectionInfo curveIntersection(
    const CurveShape& primary,
    const CurveShape& cross,
    double tolerance,
    int primaryIndex,
    int crossIndex
)
{
    BRepExtrema_DistShapeShape distance(primary.shape, cross.shape);
    distance.Perform();

    if (!distance.IsDone() || distance.NbSolution() < 1) {
        std::ostringstream str;
        str << "Failed to compute intersection between primary curve " << primaryIndex
            << " and cross curve " << crossIndex << ".";
        raiseFailure(str.str());
    }

    int bestIndex = 0;
    int acceptedSolutions = 0;
    double bestGap = std::numeric_limits<double>::max();
    for (int index = 1; index <= distance.NbSolution(); ++index) {
        const double gap = distance.PointOnShape1(index).Distance(distance.PointOnShape2(index));
        if (gap < bestGap) {
            bestGap = gap;
            bestIndex = index;
        }
        if (gap <= tolerance) {
            ++acceptedSolutions;
        }
    }

    if (bestIndex == 0 || bestGap > tolerance) {
        std::ostringstream str;
        str << "Primary curve " << primaryIndex << " and cross curve " << crossIndex
            << " do not intersect within tolerance. Gap: " << bestGap;
        raiseFailure(str.str());
    }

    if (acceptedSolutions != 1) {
        std::ostringstream str;
        str << "Primary curve " << primaryIndex << " and cross curve " << crossIndex
            << " must have exactly one intersection within tolerance. Found "
            << acceptedSolutions << ".";
        raiseFailure(str.str());
    }

    const gp_Pnt p1 = distance.PointOnShape1(bestIndex);
    const gp_Pnt p2 = distance.PointOnShape2(bestIndex);
    const gp_Pnt point = midpoint(p1, p2);

    return {
        point,
        nearestRawNormalizedParameter(primary, point),
        nearestRawNormalizedParameter(cross, point),
        bestGap
    };
}

std::vector<std::vector<IntersectionInfo>> computeIntersectionGrid(
    const std::vector<CurveShape>& primaryCurves,
    const std::vector<CurveShape>& crossCurves,
    double tolerance
)
{
    std::vector<std::vector<IntersectionInfo>> grid;
    grid.reserve(primaryCurves.size());

    for (std::size_t i = 0; i < primaryCurves.size(); ++i) {
        std::vector<IntersectionInfo> row;
        row.reserve(crossCurves.size());
        for (std::size_t j = 0; j < crossCurves.size(); ++j) {
            row.push_back(curveIntersection(
                primaryCurves[i],
                crossCurves[j],
                tolerance,
                static_cast<int>(i + 1),
                static_cast<int>(j + 1)
            ));
        }
        grid.push_back(row);
    }

    return grid;
}

template <typename T>
void reorderVector(std::vector<T>& values, const std::vector<std::size_t>& order)
{
    std::vector<T> copy;
    copy.reserve(values.size());
    for (std::size_t index : order) {
        copy.push_back(values[index]);
    }
    values.swap(copy);
}

std::vector<std::size_t> sortedIndices(const std::vector<double>& values)
{
    std::vector<std::size_t> order(values.size());
    std::iota(order.begin(), order.end(), 0);
    std::stable_sort(order.begin(), order.end(), [&values](std::size_t lhs, std::size_t rhs) {
        return values[lhs] < values[rhs];
    });
    return order;
}

void sortCurveFamilies(
    std::vector<CurveShape>& primaryCurves,
    std::vector<CurveShape>& crossCurves,
    double tolerance
)
{
    std::vector<std::vector<IntersectionInfo>> grid = computeIntersectionGrid(
        primaryCurves,
        crossCurves,
        tolerance
    );

    std::vector<double> primaryOrder(primaryCurves.size(), 0.0);
    for (std::size_t i = 0; i < primaryCurves.size(); ++i) {
        for (std::size_t j = 0; j < crossCurves.size(); ++j) {
            primaryOrder[i] += grid[i][j].crossRawParam;
        }
        primaryOrder[i] /= static_cast<double>(crossCurves.size());
    }
    reorderVector(primaryCurves, sortedIndices(primaryOrder));

    grid = computeIntersectionGrid(primaryCurves, crossCurves, tolerance);

    std::vector<double> crossOrder(crossCurves.size(), 0.0);
    for (std::size_t j = 0; j < crossCurves.size(); ++j) {
        for (std::size_t i = 0; i < primaryCurves.size(); ++i) {
            crossOrder[j] += grid[i][j].primaryRawParam;
        }
        crossOrder[j] /= static_cast<double>(primaryCurves.size());
    }
    reorderVector(crossCurves, sortedIndices(crossOrder));
}

bool isDecreasing(const std::vector<double>& parameters)
{
    if (parameters.size() < 2) {
        return false;
    }
    return parameters.back() < parameters.front();
}

void orientCurves(
    std::vector<CurveShape>& primaryCurves,
    std::vector<CurveShape>& crossCurves,
    const std::vector<std::vector<IntersectionInfo>>& grid
)
{
    for (std::size_t i = 0; i < primaryCurves.size(); ++i) {
        std::vector<double> parameters;
        parameters.reserve(crossCurves.size());
        for (std::size_t j = 0; j < crossCurves.size(); ++j) {
            parameters.push_back(grid[i][j].primaryRawParam);
        }
        primaryCurves[i].reversed = isDecreasing(parameters);
    }

    for (std::size_t j = 0; j < crossCurves.size(); ++j) {
        std::vector<double> parameters;
        parameters.reserve(primaryCurves.size());
        for (std::size_t i = 0; i < primaryCurves.size(); ++i) {
            parameters.push_back(grid[i][j].crossRawParam);
        }
        crossCurves[j].reversed = isDecreasing(parameters);
    }
}

void ensureStrictlyIncreasing(const std::vector<double>& values, const char* name)
{
    for (std::size_t i = 1; i < values.size(); ++i) {
        if (values[i] <= values[i - 1] + ParameterEpsilon) {
            std::ostringstream str;
            str << name << " are not strictly increasing. Check curve order and intersections.";
            raiseFailure(str.str());
        }
    }
}

std::vector<double> chordNodesU(const std::vector<std::vector<IntersectionInfo>>& grid)
{
    const std::size_t uCount = grid.size();
    const std::size_t vCount = grid.front().size();
    std::vector<double> accum(uCount, 0.0);
    int validRows = 0;

    for (std::size_t j = 0; j < vCount; ++j) {
        std::vector<double> distances(uCount, 0.0);
        for (std::size_t i = 1; i < uCount; ++i) {
            distances[i] = distances[i - 1] + grid[i - 1][j].point.Distance(grid[i][j].point);
        }
        if (distances.back() <= Precision::Confusion()) {
            continue;
        }
        for (std::size_t i = 0; i < uCount; ++i) {
            accum[i] += distances[i] / distances.back();
        }
        ++validRows;
    }

    if (validRows == 0) {
        raiseFailure("Primary curve nodes are degenerate.");
    }

    for (double& value : accum) {
        value /= static_cast<double>(validRows);
    }
    accum.front() = 0.0;
    accum.back() = 1.0;
    ensureStrictlyIncreasing(accum, "Primary mesh nodes");
    return accum;
}

std::vector<double> chordNodesV(const std::vector<std::vector<IntersectionInfo>>& grid)
{
    const std::size_t uCount = grid.size();
    const std::size_t vCount = grid.front().size();
    std::vector<double> accum(vCount, 0.0);
    int validColumns = 0;

    for (std::size_t i = 0; i < uCount; ++i) {
        std::vector<double> distances(vCount, 0.0);
        for (std::size_t j = 1; j < vCount; ++j) {
            distances[j] = distances[j - 1] + grid[i][j - 1].point.Distance(grid[i][j].point);
        }
        if (distances.back() <= Precision::Confusion()) {
            continue;
        }
        for (std::size_t j = 0; j < vCount; ++j) {
            accum[j] += distances[j] / distances.back();
        }
        ++validColumns;
    }

    if (validColumns == 0) {
        raiseFailure("Cross curve nodes are degenerate.");
    }

    for (double& value : accum) {
        value /= static_cast<double>(validColumns);
    }
    accum.front() = 0.0;
    accum.back() = 1.0;
    ensureStrictlyIncreasing(accum, "Cross mesh nodes");
    return accum;
}

double interpolateScalar(const std::vector<double>& x, const std::vector<double>& y, double value)
{
    if (x.size() != y.size() || x.empty()) {
        raiseFailure("Invalid interpolation data.");
    }
    if (value <= x.front()) {
        return y.front();
    }
    if (value >= x.back()) {
        return y.back();
    }

    auto upper = std::upper_bound(x.begin(), x.end(), value);
    const std::size_t index = static_cast<std::size_t>(std::distance(x.begin(), upper) - 1);
    const double span = x[index + 1] - x[index];
    const double local = span <= ParameterEpsilon ? 0.0 : (value - x[index]) / span;
    return y[index] + local * (y[index + 1] - y[index]);
}

gp_Pnt interpolatePoints(const std::vector<gp_Pnt>& points, const std::vector<double>& coordinates, double value)
{
    if (points.size() != coordinates.size() || points.empty()) {
        raiseFailure("Invalid point interpolation data.");
    }
    if (value <= coordinates.front()) {
        return points.front();
    }
    if (value >= coordinates.back()) {
        return points.back();
    }

    auto upper = std::upper_bound(coordinates.begin(), coordinates.end(), value);
    const std::size_t index = static_cast<std::size_t>(std::distance(coordinates.begin(), upper) - 1);
    const double span = coordinates[index + 1] - coordinates[index];
    const double local = span <= ParameterEpsilon ? 0.0 : (value - coordinates[index]) / span;
    return lerp(points[index], points[index + 1], local);
}

std::vector<std::vector<double>> orientedPrimaryParams(
    const std::vector<CurveShape>& primaryCurves,
    const std::vector<std::vector<IntersectionInfo>>& grid
)
{
    std::vector<std::vector<double>> params(primaryCurves.size());
    for (std::size_t i = 0; i < primaryCurves.size(); ++i) {
        params[i].reserve(grid[i].size());
        for (const IntersectionInfo& info : grid[i]) {
            params[i].push_back(primaryCurves[i].orientedNormalizedFromRaw(info.primaryRawParam));
        }
        ensureStrictlyIncreasing(params[i], "Primary curve intersection parameters");
    }
    return params;
}

std::vector<std::vector<double>> orientedCrossParams(
    const std::vector<CurveShape>& crossCurves,
    const std::vector<std::vector<IntersectionInfo>>& grid
)
{
    const std::size_t uCount = grid.size();
    const std::size_t vCount = grid.front().size();
    std::vector<std::vector<double>> params(vCount, std::vector<double>(uCount));

    for (std::size_t j = 0; j < vCount; ++j) {
        for (std::size_t i = 0; i < uCount; ++i) {
            params[j][i] = crossCurves[j].orientedNormalizedFromRaw(grid[i][j].crossRawParam);
        }
        ensureStrictlyIncreasing(params[j], "Cross curve intersection parameters");
    }
    return params;
}

gp_Pnt evaluatePrimaryContribution(
    const std::vector<CurveShape>& primaryCurves,
    const std::vector<double>& uNodes,
    const std::vector<double>& vNodes,
    const std::vector<std::vector<double>>& primaryParams,
    double u,
    double v
)
{
    std::vector<gp_Pnt> points;
    points.reserve(primaryCurves.size());
    for (std::size_t i = 0; i < primaryCurves.size(); ++i) {
        const double localParam = interpolateScalar(vNodes, primaryParams[i], v);
        points.push_back(primaryCurves[i].valueAtOrientedNormalized(localParam));
    }
    return interpolatePoints(points, uNodes, u);
}

gp_Pnt evaluateCrossContribution(
    const std::vector<CurveShape>& crossCurves,
    const std::vector<double>& uNodes,
    const std::vector<double>& vNodes,
    const std::vector<std::vector<double>>& crossParams,
    double u,
    double v
)
{
    std::vector<gp_Pnt> points;
    points.reserve(crossCurves.size());
    for (std::size_t j = 0; j < crossCurves.size(); ++j) {
        const double localParam = interpolateScalar(uNodes, crossParams[j], u);
        points.push_back(crossCurves[j].valueAtOrientedNormalized(localParam));
    }
    return interpolatePoints(points, vNodes, v);
}

gp_Pnt evaluateGridContribution(
    const std::vector<std::vector<IntersectionInfo>>& grid,
    const std::vector<double>& uNodes,
    const std::vector<double>& vNodes,
    double u,
    double v
)
{
    std::vector<gp_Pnt> rowValues;
    rowValues.reserve(grid.size());
    for (const auto& row : grid) {
        std::vector<gp_Pnt> rowPoints;
        rowPoints.reserve(row.size());
        for (const IntersectionInfo& info : row) {
            rowPoints.push_back(info.point);
        }
        rowValues.push_back(interpolatePoints(rowPoints, vNodes, v));
    }
    return interpolatePoints(rowValues, uNodes, u);
}

GeomAbs_Shape continuityForDegree(Standard_Integer degree)
{
    if (degree >= 3) {
        return GeomAbs_C2;
    }
    if (degree == 2) {
        return GeomAbs_C1;
    }
    return GeomAbs_C0;
}

double pointSurfaceDistance(const gp_Pnt& point, const Handle(Geom_Surface)& surface)
{
    GeomAPI_ProjectPointOnSurf projector(point, surface);
    if (projector.NbPoints() < 1) {
        raiseFailure("Failed to project a curve sample onto the through-curve mesh surface.");
    }
    return projector.LowerDistance();
}

double curveSurfaceDeviation(
    const CurveShape& curve,
    const Handle(Geom_Surface)& surface,
    int samples
)
{
    double maxDeviation = 0.0;
    const int count = std::max(samples, 4);
    for (int index = 0; index < count; ++index) {
        const double parameter = static_cast<double>(index) / static_cast<double>(count - 1);
        const gp_Pnt point = curve.valueAtOrientedNormalized(parameter);
        maxDeviation = std::max(maxDeviation, pointSurfaceDistance(point, surface));
    }
    return maxDeviation;
}

double computeMaxDeviation(
    const std::vector<CurveShape>& primaryCurves,
    const std::vector<CurveShape>& crossCurves,
    const Handle(Geom_Surface)& surface,
    int samples
)
{
    double maxDeviation = 0.0;
    for (const CurveShape& curve : primaryCurves) {
        maxDeviation = std::max(maxDeviation, curveSurfaceDeviation(curve, surface, samples));
    }
    for (const CurveShape& curve : crossCurves) {
        maxDeviation = std::max(maxDeviation, curveSurfaceDeviation(curve, surface, samples));
    }
    return maxDeviation;
}

}  // namespace

const char* Surface::ThroughCurveMesh::EmphasisEnums[] = {"Both", "Primary", "Cross", nullptr};

PROPERTY_SOURCE(Surface::ThroughCurveMesh, Part::Spline)

ThroughCurveMesh::ThroughCurveMesh()
{
    ADD_PROPERTY_TYPE(PrimaryCurves, (nullptr), "Through Curve Mesh", App::Prop_None, "Primary curve rows");
    ADD_PROPERTY_TYPE(PrimaryCurveGroupSizes, (-1), "Through Curve Mesh", App::Prop_Hidden, "Internal primary curve-row grouping");
    ADD_PROPERTY_TYPE(CrossCurves, (nullptr), "Through Curve Mesh", App::Prop_None, "Cross curve rows");
    ADD_PROPERTY_TYPE(CrossCurveGroupSizes, (-1), "Through Curve Mesh", App::Prop_Hidden, "Internal cross curve-row grouping");
    ADD_PROPERTY_TYPE(Tolerance, (0.1), "Through Curve Mesh", App::Prop_None, "Intersection tolerance");
    ADD_PROPERTY_TYPE(PositionTolerance, (0.1), "Through Curve Mesh", App::Prop_Hidden, "Internal fitting tolerance used by the through-curve mesh preview");
    ADD_PROPERTY_TYPE(Samples, (24), "Through Curve Mesh", App::Prop_None, "Number of samples in each surface direction");
    ADD_PROPERTY_TYPE(AutoSort, (false), "Through Curve Mesh", App::Prop_Hidden, "Automatically sort curve families before building the surface");
    ADD_PROPERTY(Emphasis, ((long)0));
    Emphasis.setEnums(EmphasisEnums);
    ADD_PROPERTY_TYPE(MaxDeviation, (0.0), "Through Curve Mesh", App::Prop_ReadOnly, "Maximum measured deviation from input curves");

    PrimaryCurves.setScope(App::LinkScope::Global);
    CrossCurves.setScope(App::LinkScope::Global);
    PrimaryCurveGroupSizes.setSize(0);
    CrossCurveGroupSizes.setSize(0);
    Tolerance.setConstraints(&ToleranceRange);
    PositionTolerance.setConstraints(&ToleranceRange);
    Samples.setConstraints(&SamplesRange);
}

short ThroughCurveMesh::mustExecute() const
{
    if (PrimaryCurves.isTouched() || PrimaryCurveGroupSizes.isTouched() || CrossCurves.isTouched() || CrossCurveGroupSizes.isTouched() || Tolerance.isTouched() || PositionTolerance.isTouched() || Samples.isTouched() || AutoSort.isTouched() || Emphasis.isTouched()) {
        return 1;
    }
    return Spline::mustExecute();
}

App::DocumentObjectExecReturn* ThroughCurveMesh::execute()
{
    Shape.setValue(TopoDS_Shape());
    MaxDeviation.setValue(0.0);

    try {
        validateInputReferences(PrimaryCurves, CrossCurves, this);
        std::vector<CurveShape> primaryCurves = collectCurves(PrimaryCurves, PrimaryCurveGroupSizes);
        std::vector<CurveShape> crossCurves = collectCurves(CrossCurves, CrossCurveGroupSizes);

        if (primaryCurves.size() < 2) {
            return new App::DocumentObjectExecReturn("At least two primary curves are required.");
        }
        if (crossCurves.size() < 2) {
            return new App::DocumentObjectExecReturn("At least two cross curves are required.");
        }

        const double intersectionTolerance = std::max(Tolerance.getValue(), Precision::Confusion());
        const double positionTolerance = std::max(PositionTolerance.getValue(), Precision::Confusion());
        const int sampleCount = std::max<int>(Samples.getValue(), 4);
        double primaryWeight = 1.0;
        double crossWeight = 1.0;
        switch (Emphasis.getValue()) {
            case 1:  // Primary
                crossWeight = 0.35;
                break;
            case 2:  // Cross
                primaryWeight = 0.35;
                break;
            default:
                break;
        }

        if (AutoSort.getValue()) {
            sortCurveFamilies(primaryCurves, crossCurves, intersectionTolerance);
        }
        std::vector<std::vector<IntersectionInfo>> grid = computeIntersectionGrid(
            primaryCurves,
            crossCurves,
            intersectionTolerance
        );
        orientCurves(primaryCurves, crossCurves, grid);

        const std::vector<double> uNodes = chordNodesU(grid);
        const std::vector<double> vNodes = chordNodesV(grid);
        const std::vector<std::vector<double>> primaryParams = orientedPrimaryParams(primaryCurves, grid);
        const std::vector<std::vector<double>> crossParams = orientedCrossParams(crossCurves, grid);

        TColgp_Array2OfPnt points(1, sampleCount, 1, sampleCount);
        for (int uIndex = 0; uIndex < sampleCount; ++uIndex) {
            const double u = static_cast<double>(uIndex) / static_cast<double>(sampleCount - 1);
            for (int vIndex = 0; vIndex < sampleCount; ++vIndex) {
                const double v = static_cast<double>(vIndex) / static_cast<double>(sampleCount - 1);
                const gp_Pnt primary = evaluatePrimaryContribution(
                    primaryCurves,
                    uNodes,
                    vNodes,
                    primaryParams,
                    u,
                    v
                );
                const gp_Pnt cross = evaluateCrossContribution(
                    crossCurves,
                    uNodes,
                    vNodes,
                    crossParams,
                    u,
                    v
                );
                const gp_Pnt net = evaluateGridContribution(grid, uNodes, vNodes, u, v);
                points(uIndex + 1, vIndex + 1) = blendGordonContributions(
                    primary,
                    cross,
                    net,
                    primaryWeight,
                    crossWeight
                );
            }
        }

        const Standard_Integer maxDegree = std::min<Standard_Integer>(5, sampleCount - 1);
        const Standard_Integer minDegree = std::min<Standard_Integer>(3, maxDegree);

        GeomAPI_PointsToBSplineSurface surfaceBuilder;
        surfaceBuilder.Init(
            points,
            Approx_ChordLength,
            minDegree,
            maxDegree,
            continuityForDegree(maxDegree),
            positionTolerance
        );

        Handle(Geom_BSplineSurface) surface = surfaceBuilder.Surface();
        if (surface.IsNull()) {
            return new App::DocumentObjectExecReturn("Failed to create a surface from the curve mesh.");
        }

        const double maxDeviation = computeMaxDeviation(primaryCurves, crossCurves, surface, sampleCount);
        MaxDeviation.setValue(maxDeviation);

        BRepBuilderAPI_MakeFace mkFace(surface, Precision::Confusion());
        if (!mkFace.IsDone()) {
            return new App::DocumentObjectExecReturn("Failed to build a face from the through-curve mesh surface.");
        }
        Shape.setValue(mkFace.Face());
    }
    catch (const Standard_Failure& e) {
        return new App::DocumentObjectExecReturn(e.GetMessageString());
    }
    catch (const std::exception& e) {
        return new App::DocumentObjectExecReturn(e.what());
    }
    catch (...) {
        return new App::DocumentObjectExecReturn("Failed to create through-curve mesh surface.");
    }

    return StdReturn;
}
