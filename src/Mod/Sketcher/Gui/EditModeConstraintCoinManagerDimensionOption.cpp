// SPDX-License-Identifier: LGPL-2.1-or-later

/***************************************************************************
 *   Copyright (c) 2021 Abdullah Tahiri <abdullah.tahiri.yo@gmail.com>     *
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

#include <FCConfig.h>

#include <memory>
#include <string>

#include <QString>

#include <Inventor/SbVec3f.h>
#include <Inventor/SoPath.h>
#include <Inventor/SoPickedPoint.h>
#include <Inventor/nodes/SoInfo.h>
#include <Inventor/nodes/SoPickStyle.h>
#include <Inventor/nodes/SoSeparator.h>

#include <Base/Placement.h>
#include <Base/Vector3D.h>
#include <Gui/SoDatumLabel.h>
#include <Mod/Sketcher/App/Constraint.h>
#include <Mod/Sketcher/App/SketchObject.h>

#include "EditModeConstraintCoinManagerDimensionOption.h"
#include "CommandConstraints.h"
#include "ViewProviderSketch.h"
#include "ViewProviderSketchCoinAttorney.h"

namespace SketcherGui {
namespace {

std::unique_ptr<Sketcher::Constraint> buildPreparedPreviewConstraint(
    const Sketcher::SketchObject& sketch,
    const DimensionOption& option)
{
    auto constraint = buildDimensionConstraint(sketch, option);
    if (!constraint) {
        return nullptr;
    }

    if (option.hasCustomLabelPos) {
        prepareConstraintForLatestDatumPlacement(sketch, *constraint, option.labelPos);
    }
    else {
        prepareConstraintForLatestDatumPlacement(sketch, *constraint);
    }
    constraint->isActive = true;
    return constraint;
}

} // namespace

const std::vector<DimensionOption>& EditModeConstraintCoinManager::currentDimensionOptions() const
{
    static const std::vector<DimensionOption> emptyOptions;
    return dimensionOptionList ? *dimensionOptionList : emptyOptions;
}

void EditModeConstraintCoinManager::setDimensionOptions(
    const std::vector<DimensionOption>& options)
{
    dimensionOptionList = &options;

    if (dimensionOptionActive >= static_cast<int>(currentDimensionOptions().size())) {
        dimensionOptionActive = -1;
    }

    rebuildDimensionOptionNodes();
}

bool EditModeConstraintCoinManager::setActiveDimensionOption(int index)
{
    if (index < 0 || index >= static_cast<int>(currentDimensionOptions().size())) {
        index = -1;
    }

    if (dimensionOptionActive == index) {
        return false;
    }

    dimensionOptionActive = index;
    rebuildDimensionOptionNodes();
    return true;
}

bool EditModeConstraintCoinManager::preparePreviewDatum(
    const DimensionOption& option,
    std::unique_ptr<Sketcher::Constraint>& constraint,
    Gui::SoDatumLabel*& datum) const
{
    datum = nullptr;
    constraint.reset();

    auto* sketch = viewProvider.getSketchObject();
    if (!sketch) {
        return false;
    }

    constraint = buildPreparedPreviewConstraint(*sketch, option);
    if (!constraint) {
        return false;
    }

    datum = createDimensionDatumLabel(*constraint, true, constraint->isActive);
    datum->ref();
    if (!configurePreviewDatumLabel(*constraint, *datum)) {
        datum->unref();
        datum = nullptr;
        constraint.reset();
        return false;
    }

    return true;
}

bool EditModeConstraintCoinManager::resolveDimensionOption(
    int index,
    DimensionOption& option) const
{
    const auto& options = currentDimensionOptions();
    if (index < 0 || index >= static_cast<int>(options.size())) {
        return false;
    }

    option = options[index];

    std::unique_ptr<Sketcher::Constraint> constraint;
    Gui::SoDatumLabel* preview = nullptr;
    if (!preparePreviewDatum(option, constraint, preview)) {
        return false;
    }

    option.hasPreparedDatumPlacement = true;
    option.preparedLabelDistance = constraint->LabelDistance;
    option.preparedLabelPosition = constraint->LabelPosition;

    const SbVec3f center = preview->getLabelTextCenter();
    preview->unref();
    option.labelPos = Base::Vector2d(center[0], center[1]);
    return true;
}

Gui::SoDatumLabel* EditModeConstraintCoinManager::createDimensionDatumLabel(
    const Sketcher::Constraint& constraint,
    bool preview,
    bool isActive) const
{
    auto* datum = new Gui::SoDatumLabel();

    Base::Vector3d sketchNormal(0.0, 0.0, 1.0);
    Base::Placement placement = ViewProviderSketchCoinAttorney::getEditingPlacement(viewProvider);
    Base::Rotation rotation(placement.getRotation());
    rotation.multVec(sketchNormal, sketchNormal);

    datum->norm.setValue(SbVec3f(sketchNormal.x, sketchNormal.y, sketchNormal.z));
    datum->string = preview
        ? SbString(const_cast<EditModeConstraintCoinManager*>(this)
                       ->getPresentationString(&constraint)
                       .toUtf8()
                       .constData())
        : SbString("");
    datum->textColor = !isActive ? drawingParameters.DeactivatedConstrDimColor
        : preview ? drawingParameters.CursorTextColor
        : (constraint.isDriving ? drawingParameters.ConstrDimColor
                                : drawingParameters.NonDrivingConstrDimColor);

    if (!preview && !drawingParameters.labelFontName.isEmpty()) {
        datum->name.setValue(drawingParameters.labelFontName.toStdString().c_str());
    }
    datum->size.setValue(drawingParameters.labelFontSize);
    datum->lineWidth = (preview ? 1 : 2) * drawingParameters.pixelScalingFactor;
    datum->useAntialiasing = false;
    datum->strikethrough = preview && !isActive;
    return datum;
}

int EditModeConstraintCoinManager::pickedDimensionOption(const SoPickedPoint* point) const
{
    if (!point || !dimensionOptionRoot) {
        return -1;
    }

    SoPath* path = point->getPath();
    if (!path) {
        return -1;
    }

    for (int pathIndex = 0; pathIndex + 1 < path->getLength(); ++pathIndex) {
        if (path->getNode(pathIndex) != dimensionOptionRoot) {
            continue;
        }

        auto* optionSeparator = dynamic_cast<SoSeparator*>(path->getNode(pathIndex + 1));
        if (!optionSeparator) {
            return -1;
        }

        for (int childIndex = 0; childIndex < optionSeparator->getNumChildren(); ++childIndex) {
            auto* optionIdNode = dynamic_cast<SoInfo*>(optionSeparator->getChild(childIndex));
            if (!optionIdNode) {
                continue;
            }

            bool ok = false;
            const int optionIndex = QString::fromLatin1(
                                           optionIdNode->string.getValue().getString())
                                           .toInt(&ok);
            if (ok && optionIndex >= 0
                && optionIndex < static_cast<int>(currentDimensionOptions().size())) {
                return optionIndex;
            }
            return -1;
        }
    }

    return -1;
}

int EditModeConstraintCoinManager::pickDimensionOption(const SoPickedPoint* point) const
{
    return pickedDimensionOption(point);
}

void EditModeConstraintCoinManager::ensureDimensionOptionRoot()
{
    if (dimensionOptionRoot) {
        return;
    }

    if (!editModeScenegraphNodes.EditRoot) {
        return;
    }

    dimensionOptionRoot = new SoSeparator;
    dimensionOptionRoot->setName("DimensionOptionRoot");

    int insertIndex = -1;
    for (int i = 0; i < editModeScenegraphNodes.EditRoot->getNumChildren(); ++i) {
        if (editModeScenegraphNodes.EditRoot->getChild(i) == editModeScenegraphNodes.constrGroup) {
            insertIndex = i + 1;
            break;
        }
    }

    if (insertIndex >= 0) {
        editModeScenegraphNodes.EditRoot->insertChild(dimensionOptionRoot, insertIndex);
    }
    else {
        editModeScenegraphNodes.EditRoot->addChild(dimensionOptionRoot);
    }
}

void EditModeConstraintCoinManager::rebuildDimensionOptionNodes()
{
    ensureDimensionOptionRoot();
    if (!dimensionOptionRoot) {
        return;
    }

    dimensionOptionRoot->removeAllChildren();

    const auto& options = currentDimensionOptions();
    for (int i = 0; i < static_cast<int>(options.size()); ++i) {
        std::unique_ptr<Sketcher::Constraint> constraint;
        Gui::SoDatumLabel* preview = nullptr;
        if (!preparePreviewDatum(options[i], constraint, preview)) {
            continue;
        }

        auto* sep = new SoSeparator;
        sep->renderCaching = SoSeparator::OFF;

        auto* pickStyle = new SoPickStyle;
        pickStyle->style = SoPickStyle::SHAPE;
        sep->addChild(pickStyle);

        sep->addChild(editModeScenegraphNodes.ConstraintDrawStyle);

        if (i == dimensionOptionActive) {
            preview->textColor = drawingParameters.PreselectColor;
        }
        sep->addChild(preview);
        preview->unref();

        auto* optionIdNode = new SoInfo;
        optionIdNode->string = SbString(std::to_string(i).c_str());
        sep->addChild(optionIdNode);

        dimensionOptionRoot->addChild(sep);
    }
}

} // namespace SketcherGui
