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
    App::PropertyFloatConstraint Tolerance;
    App::PropertyFloatConstraint PositionTolerance;
    App::PropertyIntegerConstraint Samples;
    App::PropertyBool AutoSort;
    App::PropertyEnumeration Emphasis;
    App::PropertyFloat MaxDeviation;

    short mustExecute() const override;
    App::DocumentObjectExecReturn* execute() override;
    const char* getViewProviderName() const override
    {
        return "SurfaceGui::ViewProviderThroughCurveMesh";
    }
    static const char* EmphasisEnums[];
};

}  // namespace Surface
