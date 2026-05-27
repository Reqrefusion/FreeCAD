// SPDX-License-Identifier: LGPL-2.1-or-later

/***************************************************************************
 *   Copyright (c) 2009 Juergen Riegel <juergen.riegel@web.de>             *
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

#include <QCoreApplication>
#include <QEvent>
#include <QMouseEvent>
#include <algorithm>
#include <cmath>
#include <limits>

#include <App/Application.h>
#include <Base/Exception.h>
#include <Gui/View3DInventor.h>
#include <Gui/View3DInventorViewer.h>
#include <Mod/Sketcher/App/SketchObject.h>

#include "ViewProviderSketch.h"
#include "EditModeCoinManager.h"

namespace SketcherGui {

class DimensionOptionReleaseFilter : public QObject
{
public:
    explicit DimensionOptionReleaseFilter(ViewProviderSketch* owner, QObject* parent = nullptr)
        : QObject(parent), owner(owner)
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
        else if (event->type() == QEvent::WindowDeactivate
                 || event->type() == QEvent::ApplicationDeactivate) {
            owner->cancelDimensionOptionInteraction();
        }

        return QObject::eventFilter(watched, event);
    }

private:
    ViewProviderSketch* owner;
};

class DimensionOptionFinalizingGuard
{
public:
    explicit DimensionOptionFinalizingGuard(ViewProviderSketch& owner)
        : owner(owner)
    {
        owner.dimensionOptionInteraction.finalizing = true;
    }

    ~DimensionOptionFinalizingGuard() noexcept
    {
        owner.dimensionOptionInteraction.finalizing = false;
    }

    DimensionOptionFinalizingGuard(const DimensionOptionFinalizingGuard&) = delete;
    DimensionOptionFinalizingGuard& operator=(const DimensionOptionFinalizingGuard&) = delete;

private:
    ViewProviderSketch& owner;
};


std::vector<DimensionReference> ViewProviderSketch::getSelectedDimensionOptionRefs() const
{
    std::vector<DimensionReference> items;

    if (!isInEditMode()) {
        return items;
    }

    const auto* sketch = getSketchObject();
    if (!sketch) {
        return items;
    }
    items.reserve(selection.SelOrder.size());

    for (const auto& orderedItem : selection.SelOrder) {
        if (orderedItem.kind == Selection::OrderedItem::Kind::Point) {
            if (selection.SelPointSet.find(orderedItem.id) == selection.SelPointSet.end()) {
                continue;
            }

            int geoId = Sketcher::GeoEnum::GeoUndef;
            Sketcher::PointPos posId = Sketcher::PointPos::none;
            if (orderedItem.id == Selection::RootPoint) {
                geoId = Sketcher::GeoEnum::RtPnt;
                posId = Sketcher::PointPos::start;
            }
            else {
                sketch->getGeoVertexIndex(orderedItem.id, geoId, posId);
            }

            const bool validPointRef = geoId == Sketcher::GeoEnum::RtPnt
                || (geoId == Sketcher::GeoEnum::HAxis || geoId == Sketcher::GeoEnum::VAxis)
                || (geoId != Sketcher::GeoEnum::GeoUndef && sketch->getGeometry(geoId));
            if (validPointRef) {
                items.push_back({geoId, posId});
            }
            continue;
        }

        if (selection.SelCurvSet.find(orderedItem.id) == selection.SelCurvSet.end()
            || orderedItem.id == Sketcher::GeoEnum::GeoUndef) {
            continue;
        }

        const bool validGeometry = (orderedItem.id == Sketcher::GeoEnum::HAxis || orderedItem.id == Sketcher::GeoEnum::VAxis)
            || sketch->getGeometry(orderedItem.id);
        if (!validGeometry) {
            continue;
        }

        items.push_back({orderedItem.id, Sketcher::PointPos::none});
    }

    return items;
}

QPoint ViewProviderSketch::projectSketchPointToScreen(const Base::Vector2d& p) const
{
    const SbVec2f screen = getScreenCoordinates(SbVec2f(static_cast<float>(p.x),
                                                        static_cast<float>(p.y)));
    return QPoint(static_cast<int>(std::lround(screen[0])),
                  static_cast<int>(std::lround(screen[1])));
}

Base::Vector2d ViewProviderSketch::projectScreenPointToSketch(const QPoint& p) const
{
    auto* view = qobject_cast<Gui::View3DInventor*>(this->getActiveView());
    if (!view || !isInEditMode()) {
        return Base::Vector2d();
    }

    SbLine line;
    getProjectingLine(SbVec2s(static_cast<short>(std::clamp(p.x(),
                                                            static_cast<int>(std::numeric_limits<short>::min()),
                                                            static_cast<int>(std::numeric_limits<short>::max()))),
                              static_cast<short>(std::clamp(p.y(),
                                                            static_cast<int>(std::numeric_limits<short>::min()),
                                                            static_cast<int>(std::numeric_limits<short>::max())))),
                      view->getViewer(),
                      line);

    double x = 0.0;
    double y = 0.0;
    getCoordsOnSketchPlane(line.getPosition(), line.getDirection(), x, y);
    return Base::Vector2d(x, y);
}

Base::Vector2d ViewProviderSketch::clampSketchPointToViewport(const Base::Vector2d& p, int marginPx) const
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

    const int safeMarginX = std::clamp(marginPx, 0, std::max(0, width / 2));
    const int safeMarginY = std::clamp(marginPx, 0, std::max(0, height / 2));

    const QPoint screen = projectSketchPointToScreen(p);
    const QPoint clamped(std::clamp(screen.x(), safeMarginX, std::max(safeMarginX, width - safeMarginX)),
                         std::clamp(screen.y(), safeMarginY, std::max(safeMarginY, height - safeMarginY)));

    if (clamped == screen) {
        return p;
    }

    try {
        return projectScreenPointToSketch(clamped);
    }
    catch (const Base::ZeroDivisionError&) {
        return p;
    }
}

void ViewProviderSketch::setDimensionOptions(
    const std::vector<DimensionOption>& options)
{
    dimensionOptions = options;
    if (editCoinManager) {
        editCoinManager->setDimensionOptions(dimensionOptions);
    }
    if (auto* view = qobject_cast<Gui::View3DInventor*>(this->getActiveView())) {
        if (auto* viewer = view->getViewer()) {
            const_cast<Gui::View3DInventorViewer*>(viewer)->redraw();
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
    delete filter;
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
        "User parameter:BaseApp/Preferences/Mod/Sketcher");
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

bool ViewProviderSketch::beginDimensionOptionInteraction(const QPoint& screenPos, const SoPickedPoint* point)
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

    DimensionOption resolvedOption = dimensionOptions[idx];
    if (editCoinManager) {
        editCoinManager->resolveDimensionOption(idx, resolvedOption);
    }

    dimensionOptionInteraction.active = true;
    dimensionOptionInteraction.dragged = false;
    dimensionOptionInteraction.finalizing = false;
    dimensionOptionInteraction.optionIndex = idx;
    dimensionOptionInteraction.pressScreenPos = screenPos;
    dimensionOptionInteraction.pressedOption = resolvedOption;
    installDimensionOptionReleaseFilter();

    if (editCoinManager) {
        editCoinManager->setActiveDimensionOption(idx);
    }
    return true;
}

bool ViewProviderSketch::updateDimensionOptionInteraction(const QPoint& screenPos,
                                                       const Base::Vector2d& onSketchPos)
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
    if (!dimensionOptionInteraction.dragged && dragDistance < 4) {
        return false;
    }

    DimensionOption updated = dimensionOptions[idx];
    updated.labelPos = clampSketchPointToViewport(onSketchPos);
    updated.hasCustomLabelPos = true;
    dimensionOptions[idx] = updated;
    dimensionOptionInteraction.pressedOption = updated;
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

    DimensionOption option = dimensionOptionInteraction.dragged
        ? dimensionOptions[idx]
        : dimensionOptionInteraction.pressedOption;
    if (editCoinManager) {
        DimensionOption resolvedOption = option;
        if (editCoinManager->resolveDimensionOption(idx, resolvedOption)) {
            option = resolvedOption;
        }
    }

    DimensionOptionFinalizingGuard finalizingGuard(*this);
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

    removeDimensionOptionReleaseFilter();
    dimensionOptionInteraction = DimensionOptionInteraction();
    if (editCoinManager) {
        editCoinManager->setActiveDimensionOption(-1);
    }
}

} // namespace SketcherGui
