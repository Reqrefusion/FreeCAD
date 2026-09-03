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

#include <cmath>

#include <QApplication>
#include <Precision.hxx>
#include <Base/Tools.h>

#include <Gui/Notifications.h>
#include <Gui/Selection/SelectionFilter.h>
#include <Gui/Command.h>
#include <Gui/CommandT.h>

#include <Mod/Sketcher/App/SketchObject.h>
#include <Mod/Sketcher/App/PythonConverter.h>

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
        // The construction result is created under the cursor. Repeating trim while dragging
        // would immediately select that new geometry and process it again.
        mousePressed = !isConstructionMode();
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

    struct GeometryPointReference
    {
        int geoId = Sketcher::GeoEnum::GeoUndef;
        Sketcher::PointPos point = Sketcher::PointPos::none;
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
        return (first - second).Length() < 500 * Precision::Confusion();
    }

    GeometryPointReference findAdjacentGeometryPoint(
        Sketcher::SketchObject* sketch,
        const Base::Vector3d& point,
        int firstExcludedGeoId,
        int secondExcludedGeoId
    ) const
    {
        for (int geoId = 0; geoId <= sketch->getHighestCurveIndex(); ++geoId) {
            if (geoId == firstExcludedGeoId || geoId == secondExcludedGeoId) {
                continue;
            }

            const Part::Geometry* geometry = sketch->getGeometry(geoId);
            if (!Sketcher::GeometryFacade::isInternalType(
                    geometry,
                    Sketcher::InternalType::None
                )
                || sketch->isClosedCurve(geometry)) {
                continue;
            }

            if (pointsCoincide(
                    sketch->getPoint(geoId, Sketcher::PointPos::start),
                    point
                )) {
                return {geoId, Sketcher::PointPos::start};
            }
            if (pointsCoincide(
                    sketch->getPoint(geoId, Sketcher::PointPos::end),
                    point
                )) {
                return {geoId, Sketcher::PointPos::end};
            }
        }
        return {};
    }

    void setConstruction(int geoId)
    {
        Gui::cmdAppObjectArgs(sketchgui->getObject(), "setConstruction(%d,True)", geoId);
    }

    int addConstructionGeometry(std::unique_ptr<Part::Geometry> geometry)
    {
        Sketcher::GeometryFacade::setConstruction(geometry.get(), true);
        std::vector<Part::Geometry*> geometries {geometry.get()};
        const std::string sketchObject = Gui::Command::getObjectCmd(sketchgui->getObject());
        Gui::Command::doCommand(
            Gui::Command::Doc,
            Sketcher::PythonConverter::convert(
                sketchObject,
                geometries,
                Sketcher::PythonConverter::Mode::OmitInternalGeometry
            )
                .c_str()
        );
        return sketchgui->getSketchObject()->getHighestCurveIndex();
    }

    std::unique_ptr<Sketcher::Constraint> makeCoincidentConstraint(
        int firstGeoId,
        Sketcher::PointPos firstPoint,
        int secondGeoId,
        Sketcher::PointPos secondPoint
    ) const
    {
        auto constraint = std::make_unique<Sketcher::Constraint>();
        constraint->Type = Sketcher::Coincident;
        constraint->First = firstGeoId;
        constraint->FirstPos = firstPoint;
        constraint->Second = secondGeoId;
        constraint->SecondPos = secondPoint;
        return constraint;
    }

    std::unique_ptr<Sketcher::Constraint> makeCutConstraint(
        int constructionGeoId,
        Sketcher::PointPos constructionPoint,
        const GeometryPointReference& adjacentPoint,
        int cuttingGeoId,
        const Base::Vector3d& cutPoint
    )
    {
        if (adjacentPoint.geoId != Sketcher::GeoEnum::GeoUndef) {
            return makeCoincidentConstraint(
                constructionGeoId,
                constructionPoint,
                adjacentPoint.geoId,
                adjacentPoint.point
            );
        }

        if (cuttingGeoId == Sketcher::GeoEnum::GeoUndef || cuttingGeoId == constructionGeoId) {
            return {};
        }

        // Negative geometry IDs are axes/external references. They support PointOnObject, but
        // they do not necessarily expose regular curve endpoints through getPoint().
        if (cuttingGeoId >= 0) {
            auto* sketch = sketchgui->getSketchObject();
            if (pointsCoincide(
                    sketch->getPoint(cuttingGeoId, Sketcher::PointPos::start),
                    cutPoint
                )) {
                return makeCoincidentConstraint(
                    constructionGeoId,
                    constructionPoint,
                    cuttingGeoId,
                    Sketcher::PointPos::start
                );
            }
            if (pointsCoincide(
                    sketch->getPoint(cuttingGeoId, Sketcher::PointPos::end),
                    cutPoint
                )) {
                return makeCoincidentConstraint(
                    constructionGeoId,
                    constructionPoint,
                    cuttingGeoId,
                    Sketcher::PointPos::end
                );
            }
        }

        auto constraint = std::make_unique<Sketcher::Constraint>();
        constraint->Type = Sketcher::PointOnObject;
        constraint->First = constructionGeoId;
        constraint->FirstPos = constructionPoint;
        constraint->Second = cuttingGeoId;
        return constraint;
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

        const auto* curve = static_cast<const Part::GeomCurve*>(
            sketch->getGeometry(geoIdToTrim)
        );
        double firstParameter = curve->getFirstParameter();
        double lastParameter = curve->getLastParameter();
        double firstCutParameter = firstParameter;
        double secondCutParameter = lastParameter;
        curve->closestParameter(cutPoint1, firstCutParameter);
        curve->closestParameter(cutPoint2, secondCutParameter);

        bool hasFirstCut = cuttingGeoId1 != Sketcher::GeoEnum::GeoUndef;
        bool hasSecondCut = cuttingGeoId2 != Sketcher::GeoEnum::GeoUndef;
        if (!sketch->isClosedCurve(curve)
            && std::abs(firstCutParameter - firstParameter) < Precision::PApproximation()) {
            hasFirstCut = false;
            cuttingGeoId1 = Sketcher::GeoEnum::GeoUndef;
        }
        if (!sketch->isClosedCurve(curve)
            && std::abs(secondCutParameter - lastParameter) < Precision::PApproximation()) {
            hasSecondCut = false;
            cuttingGeoId2 = Sketcher::GeoEnum::GeoUndef;
        }
        if ((!hasFirstCut && !hasSecondCut)
            || (hasFirstCut && hasSecondCut && pointsCoincide(cutPoint1, cutPoint2))) {
            setConstruction(geoIdToTrim);
            return;
        }

        const GeometryReference cuttingReference1
            = makeGeometryReference(sketch, cuttingGeoId1);
        const GeometryReference cuttingReference2
            = makeGeometryReference(sketch, cuttingGeoId2);
        const double constructionFirstParameter
            = hasFirstCut ? firstCutParameter : firstParameter;
        const double constructionLastParameter
            = hasSecondCut ? secondCutParameter : lastParameter;
        std::unique_ptr<Part::Geometry> constructionGeometry(
            curve->createArc(constructionFirstParameter, constructionLastParameter)
        );
        if (!constructionGeometry) {
            setConstruction(geoIdToTrim);
            return;
        }

        Gui::cmdAppObjectArgs(
            sketchgui->getObject(),
            "trim(%d,App.Vector(%f,%f,0),%s)",
            geoIdToTrim,
            trimPos.x,
            trimPos.y,
            includeAxes ? "True" : "False"
        );

        const int resolvedCuttingGeoId1
            = resolveGeometryReference(sketch, cuttingReference1);
        const int resolvedCuttingGeoId2
            = resolveGeometryReference(sketch, cuttingReference2);
        const GeometryPointReference adjacentPoint1 = hasFirstCut
            ? findAdjacentGeometryPoint(
                  sketch,
                  cutPoint1,
                  resolvedCuttingGeoId1,
                  resolvedCuttingGeoId2
              )
            : GeometryPointReference {};
        const GeometryPointReference adjacentPoint2 = hasSecondCut
            ? findAdjacentGeometryPoint(
                  sketch,
                  cutPoint2,
                  resolvedCuttingGeoId1,
                  resolvedCuttingGeoId2
              )
            : GeometryPointReference {};
        const int constructionGeoId = addConstructionGeometry(std::move(constructionGeometry));

        std::vector<std::unique_ptr<Sketcher::Constraint>> boundaryConstraints;
        if (hasFirstCut) {
            if (auto constraint = makeCutConstraint(
                    constructionGeoId,
                    Sketcher::PointPos::start,
                    adjacentPoint1,
                    resolvedCuttingGeoId1,
                    cutPoint1
                )) {
                boundaryConstraints.push_back(std::move(constraint));
            }
        }
        if (hasSecondCut) {
            if (auto constraint = makeCutConstraint(
                    constructionGeoId,
                    Sketcher::PointPos::end,
                    adjacentPoint2,
                    resolvedCuttingGeoId2,
                    cutPoint2
                )) {
                boundaryConstraints.push_back(std::move(constraint));
            }
        }

        // Normal trim may already preserve the same topological relationship. Diagnose all
        // candidates together and only add the subset that the Sketcher solver finds independent.
        if (boundaryConstraints.empty()
            || !filterRedundantAutoConstraints(boundaryConstraints)) {
            return;
        }
        if (!boundaryConstraints.empty()) {
            addGeneratedAutoConstraints(boundaryConstraints);
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
