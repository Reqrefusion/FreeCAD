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

#include "DimensionOption.h"

#include <optional>
#include <string>

#include <QObject>

#include <Base/Exception.h>
#include <Gui/Command.h>
#include <Gui/CommandT.h>
#include <Gui/Notifications.h>
#include <Mod/Sketcher/App/Constraint.h>
#include <Mod/Sketcher/App/PythonConverter.h>
#include <Mod/Sketcher/App/SketchObject.h>

#include "CommandConstraintsInternal.h"

namespace SketcherGui::DimensionOptionCommandDetail
{

class DimensionOptionCommandProxy final: public Gui::Command
{
public:
    DimensionOptionCommandProxy()
        : Gui::Command("Sketcher_DimensionOptionProxy")
    {}

    const char* className() const override
    {
        return "SketcherGui::DimensionOptionCommandProxy";
    }

    void activated(int) override
    {}

    using Gui::Command::abortCommand;
    using Gui::Command::openCommand;
};

void addConstraint(Sketcher::SketchObject& sketch, const Sketcher::Constraint& constraint)
{
    std::string command = Gui::Command::getObjectCmd(&sketch);
    command += '.';
    command += Sketcher::PythonConverter::convert(&constraint);
    Gui::Command::doCommand(Gui::Command::Doc, "%s", command.c_str());
}

void setDatumPlacement(
    Sketcher::SketchObject& sketch,
    int constraintIndex,
    const DimensionOption::DatumPlacement& placement
)
{
    Gui::cmdAppObjectArgs(&sketch, "setLabelDistance(%d,%.9g)", constraintIndex, placement.labelDistance);
    Gui::cmdAppObjectArgs(&sketch, "setLabelPosition(%d,%.9g)", constraintIndex, placement.labelPosition);
}

}  // namespace SketcherGui::DimensionOptionCommandDetail

namespace SketcherGui
{

bool commitDimensionOption(Sketcher::SketchObject& sketch, const DimensionOption& option)
{
    const int firstGeoId = option.refs.empty() ? Sketcher::GeoEnum::GeoUndef
                                               : option.refs.front().GeoId;
    const int secondGeoId = option.refs.size() > 1 ? option.refs[1].GeoId
                                                   : Sketcher::GeoEnum::GeoUndef;
    const bool isDriving
        = CommandConstraintsInternal::shouldCreateDrivingDimension(&sketch, firstGeoId, secondGeoId);

    DimensionOptionCommandDetail::DimensionOptionCommandProxy command;
    command.openCommand(QT_TRANSLATE_NOOP("Command", "Dimension option"));

    try {
        auto constraint = buildDimensionConstraint(sketch, option);
        if (!constraint) {
            command.abortCommand();
            return false;
        }
        constraint->isDriving = isDriving;

        const auto previousConstraintCount = sketch.Constraints.getValues().size();
        DimensionOptionCommandDetail::addConstraint(sketch, *constraint);

        if (sketch.Constraints.getValues().size() != previousConstraintCount + 1) {
            command.abortCommand();
            return false;
        }
        const auto createdIndex = static_cast<int>(previousConstraintCount);

        if (option.preparedDatumPlacement) {
            DimensionOptionCommandDetail::setDatumPlacement(
                sketch,
                createdIndex,
                *option.preparedDatumPlacement
            );
        }

        const std::optional<Base::Vector2d> datumPlacement = option.preparedDatumPlacement
            ? std::nullopt
            : option.customLabelPosition;
        CommandConstraintsInternal::finishDatumConstraint(
            &command,
            &sketch,
            isDriving,
            1,
            datumPlacement,
            option.preparedDatumPlacement ? CommandConstraintsInternal::DatumPlacementMode::Preserve
                                          : CommandConstraintsInternal::DatumPlacementMode::Compute
        );
        return true;
    }
    catch (const Base::Exception& exception) {
        command.abortCommand();
        Gui::NotifyUserError(
            &sketch,
            QT_TRANSLATE_NOOP("Notifications", "Invalid Constraint"),
            exception.what()
        );
        return false;
    }
}

}  // namespace SketcherGui
