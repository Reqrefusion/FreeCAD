// SPDX-License-Identifier: LGPL-2.1-or-later
/***************************************************************************
 *   Copyright (c) 2026 FreeCAD contributors                               *
 *                                                                         *
 *   This file is part of the FreeCAD CAx development system.              *
 *                                                                         *
 *   This library is free software; you can redistribute it and/or modify   *
 *   it under the terms of the GNU Library General Public License as        *
 *   published by the Free Software Foundation; either version 2 of the     *
 *   License, or (at your option) any later version.                       *
 *                                                                         *
 *   This library is distributed in the hope that it will be useful,        *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of         *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU       *
 *   Library General Public License for more details.                      *
 *                                                                         *
 *   You should have received a copy of the GNU Library General Public      *
 *   License along with this library; see the file COPYING.LIB. If not,    *
 *   write to the Free Software Foundation, Inc., 59 Temple Place,          *
 *   Suite 330, Boston, MA 02111-1307, USA                                 *
 ***************************************************************************/

#include "PreCompiled.h"

#include <algorithm>
#include <cmath>
#include <exception>
#include <string>
#include <vector>

#include <Bnd_Box.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <BRepBndLib.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepLib.hxx>
#include <BRepTools.hxx>
#include <BRep_Builder.hxx>
#include <BRep_Tool.hxx>
#include <Geom2dAPI_PointsToBSpline.hxx>
#include <Geom2d_BSplineCurve.hxx>
#include <GeomAbs_Shape.hxx>
#include <GeomAPI_PointsToBSpline.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_Geometry.hxx>
#include <Geom_Surface.hxx>
#include <Precision.hxx>
#include <Standard_Failure.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <TColgp_Array1OfPnt2d.hxx>
#include <TopAbs.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Compound.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Wire.hxx>
#include <gp_Pnt.hxx>
#include <gp_Pnt2d.hxx>
#include <gp_Vec.hxx>

#include "FeatureWrap.h"

using namespace Surface;

namespace
{

const App::PropertyIntegerConstraint::Constraints SampleRange = {4, 2000, 1};
const App::PropertyFloatConstraint::Constraints ToleranceRange = {1e-9, 10.0, 1e-4};

struct SourceRange
{
    double xmin {0.0};
    double xmax {0.0};
    double ymin {0.0};
    double ymax {0.0};
    bool degenerateX {false};
    bool degenerateY {false};

    double width() const
    {
        return xmax - xmin;
    }

    double height() const
    {
        return ymax - ymin;
    }
};

struct TargetRange
{
    double umin {0.0};
    double umax {0.0};
    double vmin {0.0};
    double vmax {0.0};
};

static bool isFaceSubName(const std::string& sub)
{
    return sub.rfind("Face", 0) == 0;
}

static double clamp01(double value)
{
    return std::max(0.0, std::min(1.0, value));
}

static int splineDegree(int samples)
{
    return std::max(1, std::min(8, samples - 1));
}

static GeomAbs_Shape continuityForDegree(int degree)
{
    if (degree >= 3) {
        return GeomAbs_C2;
    }
    if (degree == 2) {
        return GeomAbs_C1;
    }
    return GeomAbs_C0;
}

static TopoDS_Shape makeCompoundFromSubShapes(Part::Feature* feature,
                                               const std::vector<std::string>& subValues)
{
    if (subValues.empty()) {
        return feature->Shape.getValue();
    }

    BRep_Builder builder;
    TopoDS_Compound compound;
    builder.MakeCompound(compound);

    bool added = false;
    for (const std::string& sub : subValues) {
        TopoDS_Shape subShape = feature->Shape.getShape().getSubShape(sub.c_str());
        if (!subShape.IsNull()) {
            builder.Add(compound, subShape);
            added = true;
        }
    }

    if (added) {
        return compound;
    }

    return TopoDS_Shape();
}

static bool getSourceRange(const TopoDS_Shape& shape, SourceRange& range)
{
    Bnd_Box box;
    BRepBndLib::Add(shape, box);

    if (box.IsVoid()) {
        return false;
    }

    double zmin = 0.0;
    double zmax = 0.0;
    box.Get(range.xmin, range.ymin, zmin, range.xmax, range.ymax, zmax);

    range.degenerateX = std::abs(range.width()) <= Precision::Confusion();
    range.degenerateY = std::abs(range.height()) <= Precision::Confusion();

    // A single point cannot define a 2D wrap domain. A horizontal or vertical
    // line can still be mapped, using the middle of the collapsed direction.
    return !(range.degenerateX && range.degenerateY);
}

static gp_Pnt2d mapSourcePointToUV(const gp_Pnt& p,
                                   const SourceRange& sourceRange,
                                   const TargetRange& targetRange,
                                   bool reverseU,
                                   bool reverseV,
                                   bool swapUV)
{
    double su = sourceRange.degenerateX
        ? 0.5
        : (p.X() - sourceRange.xmin) / sourceRange.width();
    double sv = sourceRange.degenerateY
        ? 0.5
        : (p.Y() - sourceRange.ymin) / sourceRange.height();

    su = clamp01(su);
    sv = clamp01(sv);

    if (swapUV) {
        std::swap(su, sv);
    }
    if (reverseU) {
        su = 1.0 - su;
    }
    if (reverseV) {
        sv = 1.0 - sv;
    }

    const double u = targetRange.umin + su * (targetRange.umax - targetRange.umin);
    const double v = targetRange.vmin + sv * (targetRange.vmax - targetRange.vmin);
    return gp_Pnt2d(u, v);
}

static Handle(Geom_Surface) getLocatedSurface(const TopoDS_Face& face)
{
    TopLoc_Location loc;
    Handle(Geom_Surface) surface = BRep_Tool::Surface(face, loc);
    if (surface.IsNull()) {
        return surface;
    }

    if (!loc.IsIdentity()) {
        Handle(Geom_Geometry) transformed = surface->Transformed(loc.Transformation());
        surface = Handle(Geom_Surface)::DownCast(transformed);
    }

    return surface;
}

static TargetRange getTargetRange(const TopoDS_Face& face)
{
    TargetRange range;
    BRepTools::UVBounds(face, range.umin, range.umax, range.vmin, range.vmax);
    return range;
}

static bool isValidTargetRange(const TargetRange& range)
{
    return std::isfinite(range.umin) && std::isfinite(range.umax)
        && std::isfinite(range.vmin) && std::isfinite(range.vmax)
        && std::abs(range.umax - range.umin) > Precision::Confusion()
        && std::abs(range.vmax - range.vmin) > Precision::Confusion();
}

static TopoDS_Edge makeEdgeOnSurface(const TopoDS_Edge& sourceEdge,
                                     const Handle(Geom_Surface)& surface,
                                     const SourceRange& sourceRange,
                                     const TargetRange& targetRange,
                                     int samples,
                                     double tolerance,
                                     bool reverseU,
                                     bool reverseV,
                                     bool swapUV)
{
    BRepAdaptor_Curve curve(sourceEdge);
    const double first = curve.FirstParameter();
    const double last = curve.LastParameter();

    TColgp_Array1OfPnt2d uvPoints(1, samples);
    for (int i = 1; i <= samples; ++i) {
        const double t = first + (last - first) * static_cast<double>(i - 1)
            / static_cast<double>(samples - 1);
        uvPoints.SetValue(i,
                          mapSourcePointToUV(curve.Value(t),
                                             sourceRange,
                                             targetRange,
                                             reverseU,
                                             reverseV,
                                             swapUV));
    }

    const int degree = splineDegree(samples);
    Geom2dAPI_PointsToBSpline interpolator(uvPoints,
                                           std::min(3, degree),
                                           degree,
                                           continuityForDegree(degree),
                                           tolerance);
    if (!interpolator.IsDone()) {
        throw Standard_Failure("Could not approximate the 2D curve on the target surface.");
    }
    Handle(Geom2d_BSplineCurve) c2d = interpolator.Curve();
    if (c2d.IsNull()) {
        throw Standard_Failure("Could not create a 2D BSpline curve.");
    }

    BRepBuilderAPI_MakeEdge edgeMaker(c2d,
                                      surface,
                                      c2d->FirstParameter(),
                                      c2d->LastParameter());
    if (!edgeMaker.IsDone()) {
        throw Standard_Failure("Could not create an edge on the target surface.");
    }

    TopoDS_Edge edge = edgeMaker.Edge();
    if (edge.IsNull() || !BRepLib::BuildCurve3d(edge, tolerance)) {
        throw Standard_Failure("Could not build the 3D curve for the wrapped edge.");
    }

    return edge;
}

static TopoDS_Edge makeOffsetEdge(const TopoDS_Edge& sourceEdge,
                                  const Handle(Geom_Surface)& surface,
                                  const SourceRange& sourceRange,
                                  const TargetRange& targetRange,
                                  int samples,
                                  double tolerance,
                                  double offset,
                                  bool flipNormal,
                                  bool reverseU,
                                  bool reverseV,
                                  bool swapUV)
{
    BRepAdaptor_Curve curve(sourceEdge);
    const double first = curve.FirstParameter();
    const double last = curve.LastParameter();

    TColgp_Array1OfPnt points(1, samples);
    for (int i = 1; i <= samples; ++i) {
        const double t = first + (last - first) * static_cast<double>(i - 1)
            / static_cast<double>(samples - 1);

        const gp_Pnt2d uv = mapSourcePointToUV(curve.Value(t),
                                               sourceRange,
                                               targetRange,
                                               reverseU,
                                               reverseV,
                                               swapUV);

        gp_Pnt p;
        gp_Vec du;
        gp_Vec dv;
        surface->D1(uv.X(), uv.Y(), p, du, dv);

        gp_Vec normal = du.Crossed(dv);
        if (normal.SquareMagnitude() > Precision::SquareConfusion()) {
            normal.Normalize();
            if (flipNormal) {
                normal.Reverse();
            }
            p.Translate(normal * offset);
        }

        points.SetValue(i, p);
    }

    const int degree = splineDegree(samples);
    GeomAPI_PointsToBSpline interpolator(points,
                                         std::min(3, degree),
                                         degree,
                                         continuityForDegree(degree),
                                         tolerance);
    if (!interpolator.IsDone()) {
        throw Standard_Failure("Could not approximate the offset curve.");
    }
    Handle(Geom_BSplineCurve) c3d = interpolator.Curve();
    if (c3d.IsNull()) {
        throw Standard_Failure("Could not create an offset BSpline curve.");
    }

    BRepBuilderAPI_MakeEdge edgeMaker(c3d,
                                      c3d->FirstParameter(),
                                      c3d->LastParameter());
    if (!edgeMaker.IsDone()) {
        throw Standard_Failure("Could not create an offset edge.");
    }

    return edgeMaker.Edge();
}

static TopoDS_Edge wrapEdge(const TopoDS_Edge& sourceEdge,
                            const Handle(Geom_Surface)& surface,
                            const SourceRange& sourceRange,
                            const TargetRange& targetRange,
                            int samples,
                            double tolerance,
                            double offset,
                            bool flipNormal,
                            bool reverseU,
                            bool reverseV,
                            bool swapUV)
{
    if (std::abs(offset) <= Precision::Confusion()) {
        return makeEdgeOnSurface(sourceEdge,
                                 surface,
                                 sourceRange,
                                 targetRange,
                                 samples,
                                 tolerance,
                                 reverseU,
                                 reverseV,
                                 swapUV);
    }

    return makeOffsetEdge(sourceEdge,
                          surface,
                          sourceRange,
                          targetRange,
                          samples,
                          tolerance,
                          offset,
                          flipNormal,
                          reverseU,
                          reverseV,
                          swapUV);
}

static bool addWrappedWireOrFace(BRep_Builder& builder,
                                 TopoDS_Compound& compound,
                                 const TopoDS_Wire& sourceWire,
                                 const Handle(Geom_Surface)& surface,
                                 const SourceRange& sourceRange,
                                 const TargetRange& targetRange,
                                 int samples,
                                 double tolerance,
                                 double offset,
                                 bool flipNormal,
                                 bool reverseU,
                                 bool reverseV,
                                 bool swapUV,
                                 bool createFaces)
{
    std::vector<TopoDS_Edge> wrappedEdges;

    for (TopExp_Explorer edgeExp(sourceWire, TopAbs_EDGE); edgeExp.More(); edgeExp.Next()) {
        wrappedEdges.push_back(wrapEdge(TopoDS::Edge(edgeExp.Current()),
                                        surface,
                                        sourceRange,
                                        targetRange,
                                        samples,
                                        tolerance,
                                        offset,
                                        flipNormal,
                                        reverseU,
                                        reverseV,
                                        swapUV));
    }

    if (wrappedEdges.empty()) {
        return false;
    }

    BRepBuilderAPI_MakeWire wireMaker;
    for (const TopoDS_Edge& edge : wrappedEdges) {
        wireMaker.Add(edge);
    }

    if (!wireMaker.IsDone()) {
        for (const TopoDS_Edge& edge : wrappedEdges) {
            builder.Add(compound, edge);
        }
        return true;
    }

    TopoDS_Wire wrappedWire = wireMaker.Wire();

    if (createFaces && std::abs(offset) <= Precision::Confusion() && wrappedWire.Closed()) {
        BRepBuilderAPI_MakeFace faceMaker(surface, wrappedWire, true);
        if (faceMaker.IsDone()) {
            builder.Add(compound, faceMaker.Face());
            return true;
        }
    }

    builder.Add(compound, wrappedWire);
    return true;
}

}  // namespace

PROPERTY_SOURCE(Surface::Wrap, Part::Spline)

Wrap::Wrap()
{
    ADD_PROPERTY_TYPE(Source, (nullptr), "Wrap", App::Prop_None, "Source sketch, wire, edge, or shape");
    Source.setScope(App::LinkScope::Global);

    ADD_PROPERTY_TYPE(Face, (nullptr), "Wrap", App::Prop_None, "Target face");
    Face.setScope(App::LinkScope::Global);

    ADD_PROPERTY_TYPE(Samples, (24), "Wrap", App::Prop_None, "Samples per source edge");
    Samples.setConstraints(&SampleRange);

    ADD_PROPERTY_TYPE(Tolerance, (1e-4), "Wrap", App::Prop_None, "Curve approximation tolerance");
    Tolerance.setConstraints(&ToleranceRange);

    ADD_PROPERTY_TYPE(Offset, (0.0), "Wrap", App::Prop_None, "Normal offset from target surface");
    ADD_PROPERTY_TYPE(ReverseU, (false), "Mapping", App::Prop_None, "Reverse target U direction");
    ADD_PROPERTY_TYPE(ReverseV, (false), "Mapping", App::Prop_None, "Reverse target V direction");
    ADD_PROPERTY_TYPE(SwapUV, (false), "Mapping", App::Prop_None, "Swap mapped U and V axes");
    ADD_PROPERTY_TYPE(CreateFaces, (false), "Result", App::Prop_None, "Create faces from closed wrapped wires");
}

short Wrap::mustExecute() const
{
    if (Source.isTouched() || Face.isTouched() || Samples.isTouched() || Tolerance.isTouched()
        || Offset.isTouched() || ReverseU.isTouched() || ReverseV.isTouched() || SwapUV.isTouched()
        || CreateFaces.isTouched()) {
        return 1;
    }

    return 0;
}

App::DocumentObjectExecReturn* Wrap::execute()
{
    App::DocumentObject* sourceObj = Source.getValue();
    if (!sourceObj || !sourceObj->isDerivedFrom<Part::Feature>()) {
        return new App::DocumentObjectExecReturn("Source must be a Part shape, Sketch, Wire, or Edge object.");
    }

    App::DocumentObject* faceObj = Face.getValue();
    if (!faceObj || !faceObj->isDerivedFrom<Part::Feature>()) {
        return new App::DocumentObjectExecReturn("No target face linked.");
    }

    const auto& faceSubValues = Face.getSubValues();
    if (faceSubValues.size() != 1 || !isFaceSubName(faceSubValues[0])) {
        return new App::DocumentObjectExecReturn("Target must be exactly one face.");
    }

    try {
        auto* sourceFeature = static_cast<Part::Feature*>(sourceObj);
        auto* faceFeature = static_cast<Part::Feature*>(faceObj);

        TopoDS_Shape sourceShape = makeCompoundFromSubShapes(sourceFeature, Source.getSubValues());
        if (sourceShape.IsNull()) {
            return new App::DocumentObjectExecReturn("Source shape is null or selected source sub-shapes are invalid.");
        }

        TopoDS_Shape faceShape = faceFeature->Shape.getShape().getSubShape(faceSubValues[0].c_str());
        if (faceShape.IsNull() || faceShape.ShapeType() != TopAbs_FACE) {
            return new App::DocumentObjectExecReturn("Linked target sub-shape is not a face.");
        }

        const TopoDS_Face targetFace = TopoDS::Face(faceShape);
        Handle(Geom_Surface) surface = getLocatedSurface(targetFace);
        if (surface.IsNull()) {
            return new App::DocumentObjectExecReturn("Target face has no valid surface.");
        }

        SourceRange sourceRange;
        if (!getSourceRange(sourceShape, sourceRange)) {
            return new App::DocumentObjectExecReturn(
                "Source XY bounding box is too small. The source must span at least one of X or Y directions."
            );
        }

        TargetRange targetRange = getTargetRange(targetFace);
        if (!isValidTargetRange(targetRange)) {
            return new App::DocumentObjectExecReturn("Target face has an invalid or unbounded UV range.");
        }

        const int samples = static_cast<int>(Samples.getValue());
        const double tolerance = Tolerance.getValue();
        const double offset = Offset.getValue();
        const bool flipNormal = targetFace.Orientation() == TopAbs_REVERSED;
        const bool reverseU = ReverseU.getValue();
        const bool reverseV = ReverseV.getValue();
        const bool swapUV = SwapUV.getValue();
        const bool createFaces = CreateFaces.getValue();

        BRep_Builder builder;
        TopoDS_Compound compound;
        builder.MakeCompound(compound);

        bool added = false;

        for (TopExp_Explorer wireExp(sourceShape, TopAbs_WIRE); wireExp.More(); wireExp.Next()) {
            added = addWrappedWireOrFace(builder,
                                         compound,
                                         TopoDS::Wire(wireExp.Current()),
                                         surface,
                                         sourceRange,
                                         targetRange,
                                         samples,
                                         tolerance,
                                         offset,
                                         flipNormal,
                                         reverseU,
                                         reverseV,
                                         swapUV,
                                         createFaces) || added;
        }

        if (!added) {
            for (TopExp_Explorer edgeExp(sourceShape, TopAbs_EDGE); edgeExp.More(); edgeExp.Next()) {
                builder.Add(compound,
                            wrapEdge(TopoDS::Edge(edgeExp.Current()),
                                     surface,
                                     sourceRange,
                                     targetRange,
                                     samples,
                                     tolerance,
                                     offset,
                                     flipNormal,
                                     reverseU,
                                     reverseV,
                                     swapUV));
                added = true;
            }
        }

        if (!added) {
            return new App::DocumentObjectExecReturn("Source contains no edges or wires to wrap.");
        }

        Shape.setValue(compound);
        return StdReturn;
    }
    catch (Standard_Failure& e) {
        return new App::DocumentObjectExecReturn(e.GetMessageString());
    }
    catch (const std::exception& e) {
        return new App::DocumentObjectExecReturn(e.what());
    }
}
