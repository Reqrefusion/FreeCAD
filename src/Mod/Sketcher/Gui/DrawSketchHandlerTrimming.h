// SPDX-License-Identifier: LGPL-2.1-or-later

/***************************************************************************
 *   Copyright (c) 2022 Abdullah Tahiri <abdullah.tahiri.yo@gmail.com>     *
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

#include <QApplication>
#include <Precision.hxx>
#include <Base/Tools.h>

#include <Gui/Notifications.h>
#include <Gui/Selection/SelectionFilter.h>
#include <Gui/Command.h>
#include <Gui/CommandT.h>

#include <Mod/Sketcher/App/SketchObject.h>

#include "DrawSketchControllableHandler.h"
#include "DrawSketchDefaultWidgetController.h"
#include "Utils.h"
#include "ViewProviderSketch.h"


namespace SketcherGui
{

class TrimmingSelection: public Gui::SelectionFilterGate
{
    App::DocumentObject* object;

public:
    explicit TrimmingSelection(App::DocumentObject* obj)
        : Gui::SelectionFilterGate(nullPointer())
        , object(obj)
    {}

    bool allow(App::Document* /*pDoc*/, App::DocumentObject* pObj, const char* sSubName) override
    {
        if (pObj != this->object) {
            return false;
        }
        if (Base::Tools::isNullOrEmpty(sSubName)) {
            return false;
        }
        std::string element(sSubName);
        if (element.substr(0, 4) == "Edge") {
            int GeoId = std::atoi(element.substr(4, 4000).c_str()) - 1;
            Sketcher::SketchObject* Sketch = static_cast<Sketcher::SketchObject*>(object);
            const Part::Geometry* geom = Sketch->getGeometry(GeoId);
            if (geom->isDerivedFrom<Part::GeomTrimmedCurve>() || geom->is<Part::GeomCircle>()
                || geom->is<Part::GeomEllipse>() || geom->is<Part::GeomBSplineCurve>()) {
                // We do not trim internal geometry of complex geometries
                if (Sketcher::GeometryFacade::isInternalType(geom, Sketcher::InternalType::None)) {
                    return true;
                }
            }
        }
        return false;
    }
};

class DrawSketchHandlerTrimming;

using DSHTrimmingController = DrawSketchDefaultWidgetController<
    DrawSketchHandlerTrimming,
    StateMachines::OneSeekEnd,
    /*PAutoConstraintSize=*/0,
    /*OnViewParametersT=*/OnViewParameters<0, 0>,
    /*WidgetParametersT=*/WidgetParameters<0, 0>,
    /*WidgetCheckboxesT=*/WidgetCheckboxes<1, 1>,
    /*WidgetComboboxesT=*/WidgetComboboxes<0, 0>,
    /*WidgetLineEditsT=*/WidgetLineEdits<0, 0>>;

using DSHTrimmingControllerBase = DSHTrimmingController::ControllerBase;
using DrawSketchHandlerTrimmingBase = DrawSketchControllableHandler<DSHTrimmingController>;

class DrawSketchHandlerTrimming: public DrawSketchHandlerTrimmingBase
{
    Q_DECLARE_TR_FUNCTIONS(SketcherGui::DrawSketchHandlerTrimming)

    friend DSHTrimmingController;
    friend DSHTrimmingControllerBase;

public:
    DrawSketchHandlerTrimming()
        : DrawSketchHandlerTrimmingBase()
    {}
    ~DrawSketchHandlerTrimming() override
    {
        Gui::Selection().rmvSelectionGate();
    }

    bool pressButton(Base::Vector2d onSketchPos) override
    {
        mousePressed = true;
        return DrawSketchControllableHandler::pressButton(onSketchPos);
    }

    bool releaseButton(Base::Vector2d onSketchPos) override
    {
        mousePressed = false;
        return DrawSketchControllableHandler::releaseButton(onSketchPos);
    }

    void updateDataAndDrawToPosition(Base::Vector2d onSketchPos) override
    {
        trimPos = onSketchPos;
        geoIdToTrim = getPreselectCurve();

        // Hold-and-drag trim
        if (mousePressed) {
            executeCommands();
            return;
        }

        if (geoIdToTrim < 0) {
            EditMarkers.resize(0);
            drawEditMarkers(EditMarkers, 2);
        }

        auto sk = sketchgui->getObject<Sketcher::SketchObject>();
        int GeoId1, GeoId2;
        Base::Vector3d intersect1, intersect2;
        if (!sk->seekTrimPoints(
                geoIdToTrim,
                Base::Vector3d(onSketchPos.x, onSketchPos.y, 0),
                includeAxes,
                GeoId1,
                intersect1,
                GeoId2,
                intersect2
            )) {
            return;
        }

        EditMarkers.resize(0);

        if (GeoId1 != Sketcher::GeoEnum::GeoUndef) {
            EditMarkers.emplace_back(intersect1.x, intersect1.y);
        }
        else {
            auto start = sk->getPoint(geoIdToTrim, Sketcher::PointPos::start);
            EditMarkers.emplace_back(start.x, start.y);
        }

        if (GeoId2 != Sketcher::GeoEnum::GeoUndef) {
            EditMarkers.emplace_back(intersect2.x, intersect2.y);
        }
        else {
            auto end = sk->getPoint(geoIdToTrim, Sketcher::PointPos::end);
            EditMarkers.emplace_back(end.x, end.y);
        }

        // maker augmented by two sizes (see supported marker sizes)
        drawEditMarkers(EditMarkers, 2);
    }

    bool canGoToNextMode() override
    {
        if (geoIdToTrim < 0) {
            return false;
        }
        const Part::Geometry* geo = sketchgui->getSketchObject()->getGeometry(geoIdToTrim);
        return geo->isDerivedFrom<Part::GeomTrimmedCurve>() || geo->is<Part::GeomCircle>()
            || geo->is<Part::GeomEllipse>() || geo->is<Part::GeomBSplineCurve>();
    }

    void executeCommands() override
    {
        if (geoIdToTrim < 0) {
            return;
        }

        // FIXME: Attempt to avoid double trimming. This messes up the cursor.
        // Possibly `mouseMove` gets triggered after first trim, but before preselection,
        // resulting in another edge being deleted.
        Gui::Selection().rmvPreselect();

        try {
            if (isConstructionMode()) {
                openCommand(QT_TRANSLATE_NOOP("Command", "Trim edge as construction geometry"));
                trimAsConstruction();
            }
            else {
                openCommand(QT_TRANSLATE_NOOP("Command", "Trim edge"));
                Gui::cmdAppObjectArgs(
                    sketchgui->getObject(),
                    "trim(%d,App.Vector(%f,%f,0),%s)",
                    geoIdToTrim,
                    trimPos.x,
                    trimPos.y,
                    includeAxes ? "True" : "False"
                );
            }
            commitCommand();
            tryAutoRecompute(sketchgui->getObject<Sketcher::SketchObject>());
        }
        catch (const Base::Exception&) {
            Gui::NotifyError(
                sketchgui,
                QT_TRANSLATE_NOOP("Notifications", "Error"),
                QT_TRANSLATE_NOOP("Notifications", "Failed to trim edge")
            );
            abortCommand();
        }
    }

private:
    struct GeometryReference
    {
        int geoId = Sketcher::GeoEnum::GeoUndef;
        long geometryId = 0;
    };

    struct GeometrySegment
    {
        int geoId = Sketcher::GeoEnum::GeoUndef;
        Sketcher::PointPos firstPoint = Sketcher::PointPos::none;
        Sketcher::PointPos secondPoint = Sketcher::PointPos::none;
    };

    GeometryReference makeGeometryReference(Sketcher::SketchObject* sketch, int geoId) const
    {
        GeometryReference reference;
        reference.geoId = geoId;
        if (geoId >= 0) {
            reference.geometryId = Sketcher::GeometryFacade::getId(sketch->getGeometry(geoId));
        }
        return reference;
    }

    int resolveGeometryReference(
        Sketcher::SketchObject* sketch,
        const GeometryReference& reference
    ) const
    {
        if (reference.geoId < 0) {
            return reference.geoId;
        }

        for (int geoId = 0; geoId <= sketch->getHighestCurveIndex(); ++geoId) {
            if (Sketcher::GeometryFacade::getId(sketch->getGeometry(geoId))
                == reference.geometryId) {
                return geoId;
            }
        }
        return Sketcher::GeoEnum::GeoUndef;
    }

    bool pointsCoincide(const Base::Vector3d& first, const Base::Vector3d& second) const
    {
        return (first - second).Sqr() <= Precision::SquareConfusion();
    }

    GeometrySegment findGeometrySegment(
        Sketcher::SketchObject* sketch,
        const Base::Vector3d& firstPoint,
        const Base::Vector3d& secondPoint
    ) const
    {
        for (int geoId = 0; geoId <= sketch->getHighestCurveIndex(); ++geoId) {
            const Part::Geometry* geometry = sketch->getGeometry(geoId);
            if (!Sketcher::GeometryFacade::isInternalType(
                    geometry,
                    Sketcher::InternalType::None
                )
                || sketch->isClosedCurve(geometry)) {
                continue;
            }

            const Base::Vector3d startPoint
                = sketch->getPoint(geoId, Sketcher::PointPos::start);
            const Base::Vector3d endPoint
                = sketch->getPoint(geoId, Sketcher::PointPos::end);
            if (pointsCoincide(startPoint, firstPoint)
                && pointsCoincide(endPoint, secondPoint)) {
                return {
                    geoId,
                    Sketcher::PointPos::start,
                    Sketcher::PointPos::end
                };
            }
            if (pointsCoincide(endPoint, firstPoint)
                && pointsCoincide(startPoint, secondPoint)) {
                return {
                    geoId,
                    Sketcher::PointPos::end,
                    Sketcher::PointPos::start
                };
            }
        }
        return {};
    }

    void splitGeometry(int geoId, const Base::Vector3d& point)
    {
        Gui::cmdAppObjectArgs(
            sketchgui->getObject(),
            "split(%d,App.Vector(%f,%f,0))",
            geoId,
            point.x,
            point.y
        );
    }

    void setConstruction(int geoId)
    {
        Gui::cmdAppObjectArgs(sketchgui->getObject(), "setConstruction(%d,True)", geoId);
    }

    void constrainCutPoint(
        int constructionGeoId,
        Sketcher::PointPos constructionPoint,
        int cuttingGeoId
    )
    {
        if (cuttingGeoId == Sketcher::GeoEnum::GeoUndef || cuttingGeoId == constructionGeoId) {
            return;
        }

        Gui::cmdAppObjectArgs(
            sketchgui->getObject(),
            "addConstraint(Sketcher.Constraint('PointOnObject',%d,%d,%d))",
            constructionGeoId,
            static_cast<int>(constructionPoint),
            cuttingGeoId
        );
    }

    void trimAsConstruction()
    {
        auto* sketch = sketchgui->getObject<Sketcher::SketchObject>();

        int cuttingGeoId1 = Sketcher::GeoEnum::GeoUndef;
        int cuttingGeoId2 = Sketcher::GeoEnum::GeoUndef;
        Base::Vector3d cutPoint1;
        Base::Vector3d cutPoint2;
        if (!sketch->seekTrimPoints(
                geoIdToTrim,
                Base::Vector3d(trimPos.x, trimPos.y, 0),
                includeAxes,
                cuttingGeoId1,
                cutPoint1,
                cuttingGeoId2,
                cutPoint2
            )) {
            // Normal trim deletes the whole geometry when no boundaries are found.
            setConstruction(geoIdToTrim);
            return;
        }

        const bool hasFirstCut = cuttingGeoId1 != Sketcher::GeoEnum::GeoUndef;
        const bool hasSecondCut = cuttingGeoId2 != Sketcher::GeoEnum::GeoUndef;
        if ((!hasFirstCut && !hasSecondCut)
            || (hasFirstCut && hasSecondCut && pointsCoincide(cutPoint1, cutPoint2))) {
            setConstruction(geoIdToTrim);
            return;
        }

        const GeometryReference cuttingReference1
            = makeGeometryReference(sketch, cuttingGeoId1);
        const GeometryReference cuttingReference2
            = makeGeometryReference(sketch, cuttingGeoId2);
        const bool isClosed = sketch->isClosedCurve(sketch->getGeometry(geoIdToTrim));
        const Base::Vector3d originalStart
            = sketch->getPoint(geoIdToTrim, Sketcher::PointPos::start);
        const Base::Vector3d originalEnd
            = sketch->getPoint(geoIdToTrim, Sketcher::PointPos::end);

        GeometrySegment constructionSegment;
        if (isClosed) {
            splitGeometry(geoIdToTrim, cutPoint1);
            splitGeometry(geoIdToTrim, cutPoint2);
            constructionSegment = findGeometrySegment(sketch, cutPoint1, cutPoint2);
        }
        else if (hasFirstCut && hasSecondCut) {
            splitGeometry(geoIdToTrim, cutPoint1);
            const GeometrySegment tail
                = findGeometrySegment(sketch, cutPoint1, originalEnd);
            if (tail.geoId == Sketcher::GeoEnum::GeoUndef) {
                throw Base::RuntimeError();
            }
            splitGeometry(tail.geoId, cutPoint2);
            constructionSegment = findGeometrySegment(sketch, cutPoint1, cutPoint2);
        }
        else if (hasFirstCut) {
            splitGeometry(geoIdToTrim, cutPoint1);
            constructionSegment = findGeometrySegment(sketch, cutPoint1, originalEnd);
        }
        else {
            splitGeometry(geoIdToTrim, cutPoint2);
            constructionSegment = findGeometrySegment(sketch, originalStart, cutPoint2);
        }

        if (constructionSegment.geoId == Sketcher::GeoEnum::GeoUndef) {
            throw Base::RuntimeError();
        }
        setConstruction(constructionSegment.geoId);

        if (hasFirstCut) {
            constrainCutPoint(
                constructionSegment.geoId,
                constructionSegment.firstPoint,
                resolveGeometryReference(sketch, cuttingReference1)
            );
        }
        if (hasSecondCut) {
            constrainCutPoint(
                constructionSegment.geoId,
                constructionSegment.secondPoint,
                resolveGeometryReference(sketch, cuttingReference2)
            );
        }
    }

    std::string getToolName() const override
    {
        return "DSH_Trimming";
    }

    QString getCrosshairCursorSVGName() const override
    {
        Gui::Selection().rmvSelectionGate();
        Gui::Selection().addSelectionGate(new TrimmingSelection(sketchgui->getObject()));
        return QStringLiteral("Sketcher_Pointer_Trimming");
    }

    std::unique_ptr<QWidget> createWidget() const override
    {
        return std::make_unique<SketcherToolDefaultWidget>();
    }

    bool isWidgetVisible() const override
    {
        return true;
    };

    QPixmap getToolIcon() const override
    {
        return Gui::BitmapFactory().pixmap(
            isConstructionMode() ? "Sketcher_Trimming_Constr" : "Sketcher_Trimming"
        );
    }

    QString getToolWidgetText() const override
    {
        return QString(tr("Trimming Parameters"));
    }

private:
    std::vector<Base::Vector2d> EditMarkers;
    bool mousePressed = false;
    Base::Vector2d trimPos;
    int geoIdToTrim = Sketcher::GeoEnum::GeoUndef;
    bool includeAxes = false;

public:
    std::list<Gui::InputHint> getToolHints() const override
    {
        using enum Gui::InputHint::UserInput;

        return Gui::lookupHints<SelectMode>(
            state(),
            {{.state = SelectMode::SeekFirst,
              .hints
              = {{tr("%1 pick edge to trim", "Sketcher Trimming: hint"), {MouseLeft}},
                 {tr("%1 toggle include axes as trim boundaries"), {KeyU}}}}}
        );
    }
};

template<>
void DSHTrimmingController::configureToolWidget()
{
    if (!init) {  // Code to be executed only upon initialisation
        toolWidget->setCheckboxLabel(
            WCheckbox::FirstBox,
            QApplication::translate("TaskSketcherTool_c1_trimming", "Include axes (U)")
        );
        toolWidget->setCheckboxToolTip(
            WCheckbox::FirstBox,
            QApplication::translate("TaskSketcherTool_c1_trimming", "Include axes as trim boundaries")
        );
    }
    syncCheckboxToHandler(WCheckbox::FirstBox, handler->includeAxes);
}

template<>
void DSHTrimmingController::adaptDrawingToCheckboxChange(int checkboxindex, bool value)
{
    if (checkboxindex == WCheckbox::FirstBox) {
        handler->includeAxes = value;
    }
    handler->updateCursor();
}
}  // namespace SketcherGui
