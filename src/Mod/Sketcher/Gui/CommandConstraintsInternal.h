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

#pragma once

#include <optional>

#include <Base/Tools2D.h>

namespace Gui
{
class Command;
}

namespace Sketcher
{
class SketchObject;
}

namespace SketcherGui::CommandConstraintsInternal
{

enum class DatumPlacementMode
{
    Compute,
    Preserve,
};

void finishDatumConstraint(
    Gui::Command* command,
    Sketcher::SketchObject* sketch,
    bool isDriving = true,
    unsigned int numberOfConstraints = 1,
    std::optional<Base::Vector2d> datumPlacement = std::nullopt,
    DatumPlacementMode placementMode = DatumPlacementMode::Compute
);

bool shouldCreateDrivingDimension(const Sketcher::SketchObject* sketch, int geoId1, int geoId2);

}  // namespace SketcherGui::CommandConstraintsInternal
