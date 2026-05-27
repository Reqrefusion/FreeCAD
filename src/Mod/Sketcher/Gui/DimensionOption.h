// SPDX-License-Identifier: LGPL-2.1-or-later

/***************************************************************************
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

#pragma once

#include <Base/Tools2D.h>
#include <Mod/Sketcher/App/GeoEnum.h>

#include <memory>
#include <vector>

namespace Sketcher {
class SketchObject;
class Constraint;
}

namespace SketcherGui {

/// Legacy datum label-position sentinel used to request automatic placement.
inline constexpr double kAutoDatumLabelPosition = 10.0;

[[nodiscard]] inline bool isAutoDatumLabelPosition(double labelPosition)
{
    return labelPosition == kAutoDatumLabelPosition;
}

enum class DimensionSemantic
{
    Unknown,
    DirectLength,
    DirectDistance,
    ProjectedX,
    ProjectedY,
    Radius,
    Diameter,
    Angle,
};

struct DimensionReference
{
    int geoId {-1};
    Sketcher::PointPos posId {Sketcher::PointPos::none};
};

struct DimensionOption
{
    DimensionSemantic semantic {DimensionSemantic::Unknown};
    double previewValue {0.0};

    std::vector<DimensionReference> refs;

    Base::Vector2d labelPos;

    bool hasCustomLabelPos {false};

    bool hasPreparedDatumPlacement {false};

    double preparedLabelDistance {0.0};
    double preparedLabelPosition {0.0};
};

[[nodiscard]] std::unique_ptr<Sketcher::Constraint> buildDimensionConstraint(
    const Sketcher::SketchObject& sketch,
    const DimensionOption& option);

[[nodiscard]] std::vector<DimensionOption> buildDimensionOptions(
    Sketcher::SketchObject* sketch,
    const std::vector<DimensionReference>& selectionRefs);

bool commitDimensionOption(Sketcher::SketchObject& sketch,
                           const DimensionOption& option);

} // namespace SketcherGui
