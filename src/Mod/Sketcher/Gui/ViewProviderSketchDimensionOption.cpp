// SPDX-License-Identifier: LGPL-2.1-or-later

/***************************************************************************
 *   Copyright (c) 2009 Juergen Riegel <juergen.riegel@web.de>             *
 *   Copyright (c) 2026 Reqrefusion                                        *
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

#include <Inventor/SbLine.h>
#include <Inventor/SoPickedPoint.h>

#include <QApplication>
#include <QCoreApplication>
#include <QEvent>
#include <QMouseEvent>
#include <QScopedValueRollback>

#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <string>
#include <utility>

#include <App/Application.h>
#include <App/Document.h>
#include <Base/Exception.h>
#include <Gui/Selection/Selection.h>
#include <Gui/Selection/SelectionObject.h>
#include <Gui/View3DInventor.h>
#include <Gui/View3DInventorViewer.h>
#include <Mod/Sketcher/App/SketchObject.h>

#include "EditModeCoinManager.h"
#include "Utils.h"
#include "ViewProviderSketch.h"

namespace SketcherGui
{

namespace DimensionOptionInteractionDetail
{
constexpr int viewportMarginPx = 20;

std::optional<DimensionReference> dimensionReferenceFromSubName(
    const Sketcher::SketchObject& sketch,
    const std::string& subName
)
{
    int geoId = Sketcher::GeoEnum::GeoUndef;
    Sketcher::PointPos posId = Sketcher::PointPos::none;
    getIdsFromName(subName, &sketch, geoId, posId);
    if (geoId == Sketcher::GeoEnum::GeoUndef) {
        return std::nullopt;
    }

    const bool isRootPoint = geoId == Sketcher::GeoEnum::RtPnt && posId == Sketcher::PointPos::start;
    const bool isAxis = (geoId == Sketcher::GeoEnum::HAxis || geoId == Sketcher::GeoEnum::VAxis)
        && posId == Sketcher::PointPos::none;
    const bool isGeometry = posId != Sketcher::PointPos::none || sketch.getGeometry(geoId);
    return isRootPoint || isAxis || isGeometry
        ? std::optional<DimensionReference> {DimensionReference {geoId, posId}}
        : std::nullopt;
}
}  // namespace DimensionOptionInteractionDetail

class DimensionOptionReleaseFilter: public QObject
{
public:
    explicit DimensionOptionReleaseFilter(ViewProviderSketch* owner, QObject* parent = nullptr)
        : QObject(parent)
        , owner(owner)
    {}

protected:
    bool eventFilter(QObject* watched, QEvent* event) override
    {
        Q_UNUSED(watched);

        if (!owner || !owner->dimensionOptionInteraction.active
            || owner->dimensionOptionInteraction.finalizing) {
            return QObject::eventFilter(watched, event);
        }

        if (event->type() == QEvent::MouseButtonRelease) {
            auto* mouseEvent = static_cast<QMouseEvent*>(event);
            if (mouseEvent->button() == Qt::LeftButton) {
                owner->finalizeDimensionOptionInteraction();
                return true;
            }
        }
        else if (
            event->type() == QEvent::WindowDeactivate || event->type() == QEvent::ApplicationDeactivate
        ) {
            owner->cancelDimensionOptionInteraction();
        }

        return QObject::eventFilter(watched, event);
    }

private:
    ViewProviderSketch* owner;
};

std::vector<DimensionReference> ViewProviderSketch::getSelectedDimensionOptionRefs() const
{
    std::vector<DimensionReference> items;

    if (!isInEditMode() || !selection.SelConstraintSet.empty()) {
        return items;
    }

    const auto* sketch = getSketchObject();
    if (!sketch) {
        return items;
    }

    // SelectionObject::SubNames follows the global selection sequence, so no parallel order cache
    // is needed in ViewProviderSketch.
    const auto selectedSketches = Gui::Selection().getSelectionEx(
        sketch->getDocument()->getName(),
        Sketcher::SketchObject::getClassTypeId()
    );
    if (selectedSketches.size() != 1 || selectedSketches.front().getObject() != sketch) {
        return items;
    }

    const auto& subNames = selectedSketches.front().getSubNames();
    items.reserve(subNames.size());
    for (const auto& subName : subNames) {
        if (auto reference
            = DimensionOptionInteractionDetail::dimensionReferenceFromSubName(*sketch, subName)) {
            items.push_back(*reference);
        }
    }

    return items;
}

QPoint ViewProviderSketch::projectSketchPointToScreen(const Base::Vector2d& p) const
{
    const SbVec2f screen = getScreenCoordinates(
        SbVec2f(static_cast<float>(p.x), static_cast<float>(p.y))
    );
    return QPoint(static_cast<int>(std::lround(screen[0])), static_cast<int>(std::lround(screen[1])));
}

std::optional<Base::Vector2d> ViewProviderSketch::projectScreenPointToSketch(const QPoint& p) const
{
    auto* view = qobject_cast<Gui::View3DInventor*>(this->getActiveView());
    if (!view || !isInEditMode()) {
        return std::nullopt;
    }

    SbLine line;
    const SbVec2s screenPoint(
        static_cast<short>(std::clamp(
            p.x(),
            static_cast<int>(std::numeric_limits<short>::min()),
            static_cast<int>(std::numeric_limits<short>::max())
        )),
        static_cast<short>(std::clamp(
            p.y(),
            static_cast<int>(std::numeric_limits<short>::min()),
            static_cast<int>(std::numeric_limits<short>::max())
        ))
    );
    if (!getProjectingLine(screenPoint, view->getViewer(), line)) {
        return std::nullopt;
    }

    double x = 0.0;
    double y = 0.0;
    if (!getCoordsOnSketchPlane(line.getPosition(), line.getDirection(), x, y)) {
        return std::nullopt;
    }

    return Base::Vector2d(x, y);
}

Base::Vector2d ViewProviderSketch::clampSketchPointToViewport(const Base::Vector2d& p) const
{
    auto* view = qobject_cast<Gui::View3DInventor*>(this->getActiveView());
    if (!view || !isInEditMode()) {
        return p;
    }

    auto* viewer = view->getViewer();
    auto* widget = viewer ? viewer->getGLWidget() : nullptr;
    if (!widget) {
        return p;
    }

    const int width = widget->width();
    const int height = widget->height();
    if (width <= 0 || height <= 0) {
        return p;
    }

    const int safeMarginX
        = std::clamp(DimensionOptionInteractionDetail::viewportMarginPx, 0, std::max(0, width / 2));
    const int safeMarginY
        = std::clamp(DimensionOptionInteractionDetail::viewportMarginPx, 0, std::max(0, height / 2));

    const QPoint screen = projectSketchPointToScreen(p);
    const QPoint clamped(
        std::clamp(screen.x(), safeMarginX, std::max(safeMarginX, width - safeMarginX)),
        std::clamp(screen.y(), safeMarginY, std::max(safeMarginY, height - safeMarginY))
    );

    if (clamped == screen) {
        return p;
    }

    try {
        return projectScreenPointToSketch(clamped).value_or(p);
    }
    catch (const Base::ZeroDivisionError&) {
        return p;
    }
}

void ViewProviderSketch::setDimensionOptions(const std::vector<DimensionOption>& options)
{
    dimensionOptions = options;
    if (editCoinManager) {
        editCoinManager->setDimensionOptions(dimensionOptions);
    }
    if (auto* view = qobject_cast<Gui::View3DInventor*>(this->getActiveView())) {
        if (auto* viewer = view->getViewer()) {
            viewer->redraw();
        }
    }
}

void ViewProviderSketch::installDimensionOptionReleaseFilter()
{
    if (dimensionOptionReleaseFilter) {
        return;
    }

    auto* app = QCoreApplication::instance();
    if (!app) {
        return;
    }

    auto* filter = new DimensionOptionReleaseFilter(this, app);
    app->installEventFilter(filter);
    dimensionOptionReleaseFilter = filter;
}

void ViewProviderSketch::removeDimensionOptionReleaseFilter()
{
    auto* filter = dimensionOptionReleaseFilter.data();
    if (!filter) {
        return;
    }

    if (auto* app = QCoreApplication::instance()) {
        app->removeEventFilter(filter);
    }

    dimensionOptionReleaseFilter = nullptr;
    filter->deleteLater();
}

void ViewProviderSketch::clearDimensionOptions()
{
    removeDimensionOptionReleaseFilter();
    dimensionOptionInteraction = DimensionOptionInteraction();
    setDimensionOptions({});
    if (editCoinManager) {
        editCoinManager->setActiveDimensionOption(-1);
    }
}

bool ViewProviderSketch::isDimensionOptionPreviewEnabled() const
{
    ParameterGrp::handle hGrp = App::GetApplication().GetParameterGroupByPath(
        "User parameter:BaseApp/Preferences/Mod/Sketcher"
    );
    return hGrp->GetBool("EnableDimensionOptionPreview", true);
}

bool ViewProviderSketch::refreshDimensionOptionPreview()
{
    if (!isDimensionOptionPreviewEnabled() || !isInEditMode() || getSolvedSketch().hasConflicts()) {
        clearDimensionOptions();
        return false;
    }

    if (Mode == STATUS_SKETCH_Drag || Mode == STATUS_SKETCH_DragConstraint
        || Mode == STATUS_SKETCH_UseHandler || Mode == STATUS_SKETCH_StartRubberBand
        || Mode == STATUS_SKETCH_UseRubberBand || Mode == STATUS_SELECT_Constraint
        || Mode == STATUS_SELECT_Wire) {
        clearDimensionOptions();
        return false;
    }

    const auto selectionRefs = getSelectedDimensionOptionRefs();
    if (selectionRefs.empty()) {
        clearDimensionOptions();
        return false;
    }

    auto options = buildDimensionOptions(getSketchObject(), selectionRefs);
    if (options.empty()) {
        clearDimensionOptions();
        return false;
    }

    setDimensionOptions(options);
    if (editCoinManager) {
        editCoinManager->setActiveDimensionOption(-1);
    }
    return true;
}

bool ViewProviderSketch::beginDimensionOptionInteraction(
    const QPoint& screenPos,
    const SoPickedPoint* point
)
{
    if (!isDimensionOptionPreviewEnabled() || !isInEditMode() || Mode != STATUS_NONE
        || getSolvedSketch().hasConflicts() || dimensionOptions.empty()) {
        return false;
    }

    if (preselection.isPreselectPointValid()
        || preselection.PreselectCross == Preselection::Axes::RootPoint) {
        return false;
    }

    if (!editCoinManager) {
        return false;
    }

    const int idx = editCoinManager->pickDimensionOption(point);
    if (idx < 0 || idx >= static_cast<int>(dimensionOptions.size())) {
        return false;
    }

    dimensionOptionInteraction.active = true;
    dimensionOptionInteraction.dragged = false;
    dimensionOptionInteraction.finalizing = false;
    dimensionOptionInteraction.optionIndex = idx;
    dimensionOptionInteraction.pressScreenPos = screenPos;
    installDimensionOptionReleaseFilter();

    editCoinManager->setActiveDimensionOption(idx);
    return true;
}

bool ViewProviderSketch::updateDimensionOptionInteraction(
    const QPoint& screenPos,
    const Base::Vector2d& onSketchPos
)
{
    if (!dimensionOptionInteraction.active || dimensionOptionInteraction.finalizing) {
        return false;
    }

    const int idx = dimensionOptionInteraction.optionIndex;
    if (idx < 0 || idx >= static_cast<int>(dimensionOptions.size())) {
        cancelDimensionOptionInteraction();
        return false;
    }

    const int dragDistance = (screenPos - dimensionOptionInteraction.pressScreenPos).manhattanLength();
    if (!dimensionOptionInteraction.dragged && dragDistance < QApplication::startDragDistance()) {
        return false;
    }

    DimensionOption updated = dimensionOptions[idx];
    updated.customLabelPosition = clampSketchPointToViewport(onSketchPos);
    dimensionOptions[idx] = updated;
    dimensionOptionInteraction.dragged = true;
    setDimensionOptions(dimensionOptions);
    if (editCoinManager) {
        editCoinManager->setActiveDimensionOption(idx);
    }
    return true;
}

bool ViewProviderSketch::finalizeDimensionOptionInteraction()
{
    if (dimensionOptionInteraction.finalizing) {
        return true;
    }

    if (!dimensionOptionInteraction.active) {
        return false;
    }

    const int idx = dimensionOptionInteraction.optionIndex;
    if (idx < 0 || idx >= static_cast<int>(dimensionOptions.size())) {
        cancelDimensionOptionInteraction();
        return false;
    }

    auto* sketch = getSketchObject();
    if (!sketch) {
        cancelDimensionOptionInteraction();
        return false;
    }

    DimensionOption option = dimensionOptions[idx];
    if (editCoinManager) {
        if (auto resolvedOption = editCoinManager->resolveDimensionOption(idx)) {
            option = std::move(*resolvedOption);
        }
    }

    QScopedValueRollback<bool> finalizingGuard(dimensionOptionInteraction.finalizing, true);
    removeDimensionOptionReleaseFilter();
    dimensionOptionInteraction.active = false;
    if (editCoinManager) {
        editCoinManager->setActiveDimensionOption(-1);
    }

    setDimensionOptions({});

    const bool ok = commitDimensionOption(*sketch, option);

    dimensionOptionInteraction = DimensionOptionInteraction();
    if (!ok) {
        refreshDimensionOptionPreview();
    }
    return ok;
}

void ViewProviderSketch::cancelDimensionOptionInteraction()
{
    if (!dimensionOptionInteraction.active || dimensionOptionInteraction.finalizing) {
        return;
    }

    const int index = dimensionOptionInteraction.optionIndex;
    const bool restoreDefaultPlacement = dimensionOptionInteraction.dragged && index >= 0
        && index < static_cast<int>(dimensionOptions.size());
    removeDimensionOptionReleaseFilter();
    dimensionOptionInteraction = DimensionOptionInteraction();
    if (restoreDefaultPlacement) {
        dimensionOptions[index].customLabelPosition.reset();
        setDimensionOptions(dimensionOptions);
    }
    if (editCoinManager) {
        editCoinManager->setActiveDimensionOption(-1);
    }
}

}  // namespace SketcherGui
