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

#pragma once

#include <App/PropertyLinks.h>
#include <App/PropertyStandard.h>
#include <Mod/Part/App/FeaturePartSpline.h>
#include <Mod/Surface/SurfaceGlobal.h>

namespace Surface
{

class SurfaceExport ThroughCurveMesh: public Part::Spline
{
    PROPERTY_HEADER_WITH_OVERRIDE(Surface::ThroughCurveMesh);

public:
    ThroughCurveMesh();

    App::PropertyLinkSubList PrimaryCurves;
    App::PropertyIntegerList PrimaryCurveGroupSizes;
    App::PropertyLinkSubList CrossCurves;
    App::PropertyIntegerList CrossCurveGroupSizes;
    App::PropertyLinkSubList SupportFaces;
    App::PropertyFloatConstraint Tolerance;
    App::PropertyFloatConstraint PositionTolerance;
    App::PropertyIntegerConstraint Samples;
    App::PropertyIntegerConstraint SamplesU;
    App::PropertyIntegerConstraint SamplesV;
    App::PropertyBool AutoSort;
    App::PropertyEnumeration Emphasis;
    App::PropertyEnumeration Construction;
    App::PropertyEnumeration Parameterization;
    App::PropertyEnumeration SurfaceContinuity;
    App::PropertyEnumeration BoundaryContinuity;
    App::PropertyIntegerConstraint MinDegree;
    App::PropertyIntegerConstraint MaxDegree;
    App::PropertyFloatConstraint SmoothLengthWeight;
    App::PropertyFloatConstraint SmoothCurvatureWeight;
    App::PropertyFloatConstraint SmoothTorsionWeight;
    App::PropertyFloat MaxDeviation;
    App::PropertyFloat MaxIntersectionGap;
    App::PropertyFloat ContinuityG0Error;
    App::PropertyFloat ContinuityG1Error;
    App::PropertyFloat ContinuityG2Error;

    short mustExecute() const override;
    App::DocumentObjectExecReturn* execute() override;
    const char* getViewProviderName() const override
    {
        return "SurfaceGui::ViewProviderThroughCurveMesh";
    }
    static const char* EmphasisEnums[];
    static const char* ConstructionEnums[];
    static const char* ParameterizationEnums[];
    static const char* SurfaceContinuityEnums[];
    static const char* BoundaryContinuityEnums[];
};

}  // namespace Surface
