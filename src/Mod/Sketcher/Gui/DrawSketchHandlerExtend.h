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

#include <Precision.hxx>

#include <Gui/Notifications.h>
#include <Gui/Selection/SelectionFilter.h>
#include <Gui/Command.h>
#include <Gui/CommandT.h>

#include <Mod/Sketcher/App/SketchObject.h>
#include <Mod/Sketcher/App/PythonConverter.h>

#include "DrawSketchHandler.h"
#include "Utils.h"
#include "ViewProviderSketch.h"
#include "SnapManager.h"


namespace SketcherGui
{

class ExtendSelection: public Gui::SelectionFilterGate
{
    App::DocumentObject* object;

public:
    explicit ExtendSelection(App::DocumentObject* obj)
        : Gui::SelectionFilterGate(nullPointer())
        , object(obj)
        , disabled(false)
    {}

    bool allow(App::Document* /*pDoc*/, App::DocumentObject* pObj, const char* sSubName) override
    {
        if (pObj != this->object) {
            return false;
        }
        if (Base::Tools::isNullOrEmpty(sSubName)) {
            return false;
        }
        if (disabled) {
            return true;
        }
        std::string element(sSubName);
        if (element.substr(0, 4) == "Edge") {
            int GeoId = std::atoi(element.substr(4, 4000).c_str()) - 1;
            Sketcher::SketchObject* Sketch = static_cast<Sketcher::SketchObject*>(object);
            const Part::Geometry* geom = Sketch->getGeometry(GeoId);
            if (geom->is<Part::GeomLineSegment>() || geom->is<Part::GeomArcOfCircle>()) {
                return true;
            }
        }
        return false;
    }

    void setDisabled(bool isDisabled)
    {
        disabled = isDisabled;
    }

protected:
    bool disabled;
};


class DrawSketchHandlerExtend: public DrawSketchHandler
{
    Q_DECLARE_TR_FUNCTIONS(SketcherGui::DrawSketchHandlerExtend)

public:
    DrawSketchHandlerExtend()
        : Mode(STATUS_SEEK_First)
        , EditCurve(2)
        , BaseGeoId(-1)
        , ExtendFromStart(false)
        , SavedExtendFromStart(false)
        , Increment(0)
    {}

    ~DrawSketchHandlerExtend() override
    {
        Gui::Selection().rmvSelectionGate();
    }
    enum SelectMode
    {
        STATUS_SEEK_First,
        STATUS_SEEK_Second,
    };

    void mouseMove(SnapManager::SnapHandle snapHandle) override
    {
        Base::Vector2d onSketchPos = snapHandle.compute();

        using std::numbers::pi;

        if (Mode == STATUS_SEEK_Second) {
            const Part::Geometry* geom = sketchgui->getSketchObject()->getGeometry(BaseGeoId);
            if (geom->is<Part::GeomLineSegment>()) {
                const Part::GeomLineSegment* lineSeg = static_cast<const Part::GeomLineSegment*>(geom);
                // project point to the existing curve
                Base::Vector3d start3d = lineSeg->getStartPoint();
                Base::Vector3d end3d = lineSeg->getEndPoint();

                Base::Vector2d startPoint = Base::Vector2d(start3d.x, start3d.y);
                Base::Vector2d endPoint = Base::Vector2d(end3d.x, end3d.y);
                Base::Vector2d recenteredLine = endPoint - startPoint;
                Base::Vector2d recenteredPoint = onSketchPos - startPoint;
                Base::Vector2d projection;
                projection.ProjectToLine(recenteredPoint, recenteredLine);
                if (recenteredPoint.Length() < recenteredPoint.Distance(recenteredLine)) {
                    EditCurve[0] = startPoint + projection;
                    EditCurve[1] = endPoint;
                }
                else {
                    EditCurve[0] = startPoint;
                    EditCurve[1] = startPoint + projection;
                }
                /**
                 * If in-curve, the intuitive behavior is for the line to shrink an amount from
                 * the original click-point.
                 *
                 * If out-of-curve, the intuitive behavior is for the closest line endpoint to
                 * expand.
                 */
                bool inCurve
                    = (projection.Length() < recenteredLine.Length()
                       && projection.GetAngle(recenteredLine) < 0.1);  // Two possible values here,
                                                                       // pi and 0, but 0.1 is to
                                                                       // avoid floating point
                                                                       // problems.
                if (inCurve) {
                    Increment = SavedExtendFromStart ? -1 * projection.Length()
                                                     : projection.Length() - recenteredLine.Length();
                    ExtendFromStart = SavedExtendFromStart;
                }
                else {
                    ExtendFromStart = onSketchPos.Distance(startPoint)
                        < onSketchPos.Distance(endPoint);
                    Increment = ExtendFromStart ? projection.Length()
                                                : projection.Length() - recenteredLine.Length();
                }
                drawEdit(EditCurve);
            }
            else if (geom->is<Part::GeomArcOfCircle>()) {
                const Part::GeomArcOfCircle* arc = static_cast<const Part::GeomArcOfCircle*>(geom);
                Base::Vector3d center = arc->getCenter();
                double radius = arc->getRadius();

                double start, end;
                arc->getRange(start, end, true);
                double arcAngle = end - start;

                Base::Vector2d angle
                    = Base::Vector2d(onSketchPos.x - center.x, onSketchPos.y - center.y);
                Base::Vector2d startAngle = Base::Vector2d(cos(start), sin(start));
                Base::Vector2d endAngle = Base::Vector2d(cos(end), sin(end));

                Base::Vector2d arcHalf
                    = Base::Vector2d(cos(start + arcAngle / 2.0), sin(start + arcAngle / 2.0));
                double angleToEndAngle = angle.GetAngle(endAngle);
                double angleToStartAngle = angle.GetAngle(startAngle);


                double modStartAngle = start;
                double modArcAngle = end - start;
                bool outOfArc = arcHalf.GetAngle(angle) * 2.0 > arcAngle;
                if (ExtendFromStart) {
                    bool isCCWFromStart = crossProduct(angle, startAngle) < 0;
                    if (outOfArc) {
                        if (isCCWFromStart) {
                            modStartAngle -= 2 * pi - angleToStartAngle;
                            modArcAngle += 2 * pi - angleToStartAngle;
                        }
                        else {
                            modStartAngle -= angleToStartAngle;
                            modArcAngle += angleToStartAngle;
                        }
                    }
                    else {
                        if (isCCWFromStart) {
                            modStartAngle += angleToStartAngle;
                            modArcAngle -= angleToStartAngle;
                        }
                        else {
                            modStartAngle += 2 * pi - angleToStartAngle;
                            modArcAngle -= 2 * pi - angleToStartAngle;
                        }
                    }
                }
                else {
                    bool isCWFromEnd = crossProduct(angle, endAngle) >= 0;
                    if (outOfArc) {
                        if (isCWFromEnd) {
                            modArcAngle += 2 * pi - angleToEndAngle;
                        }
                        else {
                            modArcAngle += angleToEndAngle;
                        }
                    }
                    else {
                        if (isCWFromEnd) {
                            modArcAngle -= angleToEndAngle;
                        }
                        else {
                            modArcAngle -= 2 * pi - angleToEndAngle;
                        }
                    }
                }
                Increment = modArcAngle - (end - start);
                for (int i = 0; i < 31; i++) {
                    double angle = modStartAngle + i * modArcAngle / 30.0;
                    EditCurve[i] = Base::Vector2d(
                        center.x + radius * cos(angle),
                        center.y + radius * sin(angle)
                    );
                }
                drawEdit(EditCurve);
            }
            int curveId = getPreselectCurve();
            if (BaseGeoId != curveId) {
                seekAndRenderAutoConstraint(SugConstr, onSketchPos, Base::Vector2d(0.f, 0.f));
            }
        }
    }

    bool pressButton(Base::Vector2d onSketchPos) override
    {
        Q_UNUSED(onSketchPos);
        return true;
    }

    bool releaseButton(Base::Vector2d onSketchPos) override
    {
        Q_UNUSED(onSketchPos);
        if (Mode == STATUS_SEEK_First) {
            BaseGeoId = getPreselectCurve();
            if (BaseGeoId > -1) {
                const Part::Geometry* geom = sketchgui->getSketchObject()->getGeometry(BaseGeoId);
                if (geom->is<Part::GeomLineSegment>()) {
                    const Part::GeomLineSegment* seg = static_cast<const Part::GeomLineSegment*>(geom);
                    Base::Vector3d start3d = seg->getStartPoint();
                    Base::Vector3d end3d = seg->getEndPoint();
                    Base::Vector2d start = Base::Vector2d(start3d.x, start3d.y);
                    Base::Vector2d end = Base::Vector2d(end3d.x, end3d.y);
                    SavedExtendFromStart = (onSketchPos.Distance(start) < onSketchPos.Distance(end));
                    ExtendFromStart = SavedExtendFromStart;
                    Mode = STATUS_SEEK_Second;
                }
                else if (geom->is<Part::GeomArcOfCircle>()) {
                    const Part::GeomArcOfCircle* arc = static_cast<const Part::GeomArcOfCircle*>(geom);
                    double start, end;
                    arc->getRange(start, end, true);

                    Base::Vector3d center = arc->getCenter();
                    Base::Vector2d angle
                        = Base::Vector2d(onSketchPos.x - center.x, onSketchPos.y - center.y);
                    double angleToStart = angle.GetAngle(Base::Vector2d(cos(start), sin(start)));
                    double angleToEnd = angle.GetAngle(Base::Vector2d(cos(end), sin(end)));
                    ExtendFromStart = (angleToStart < angleToEnd);  // move start point if closer to
                                                                    // angle than end point
                    EditCurve.resize(31);
                    Mode = STATUS_SEEK_Second;
                }
                filterGate->setDisabled(true);
            }
        }
        else if (Mode == STATUS_SEEK_Second) {
            try {
                ExtensionTarget autoConstraintTarget {
                    BaseGeoId,
                    ExtendFromStart ? Sketcher::PointPos::start : Sketcher::PointPos::end
                };
                const bool constructionMode = isConstructionMode();
                if (constructionMode) {
                    openCommand(
                        QT_TRANSLATE_NOOP("Command", "Extend edge with construction geometry")
                    );
                    autoConstraintTarget = extendWithConstruction();
                }
                else {
                    openCommand(QT_TRANSLATE_NOOP("Command", "Extend edge"));
                    Gui::cmdAppObjectArgs(
                        sketchgui->getObject(),
                        "extend(%d, %f, %d)\n",  // GeoId, increment, PointPos
                        BaseGeoId,
                        Increment,
                        ExtendFromStart ? static_cast<int>(Sketcher::PointPos::start)
                                        : static_cast<int>(Sketcher::PointPos::end)
                    );
                }
                commitCommand();

                ParameterGrp::handle hGrp = App::GetApplication().GetParameterGroupByPath(
                    "User parameter:BaseApp/Preferences/Mod/Sketcher"
                );
                bool autoRecompute = hGrp->GetBool("AutoRecompute", false);
                if (autoRecompute) {
                    Gui::Command::updateActive();
                }

                // constrain chosen point
                if (!SugConstr.empty()) {
                    if (constructionMode) {
                        createFilteredAutoConstraints(SugConstr, autoConstraintTarget);
                    }
                    else {
                        createAutoConstraints(
                            SugConstr,
                            autoConstraintTarget.geoId,
                            autoConstraintTarget.point
                        );
                    }
                    SugConstr.clear();
                }
                bool continuousMode = hGrp->GetBool("ContinuousCreationMode", true);

                if (continuousMode) {
                    // This code enables the continuous creation mode.
                    Mode = STATUS_SEEK_First;
                    filterGate->setDisabled(false);
                    EditCurve.clear();
                    drawEdit(EditCurve);
                    EditCurve.resize(2);
                    applyCursor();
                    /* this is ok not to call to purgeHandler
                     * in continuous creation mode because the
                     * handler is destroyed by the quit() method on pressing the
                     * right button of the mouse */
                }
                else {
                    sketchgui->purgeHandler();  // no code after this line, Handler get deleted in
                                                // ViewProvider
                }
            }
            catch (const Base::Exception&) {
                Gui::NotifyError(
                    sketchgui,
                    QT_TRANSLATE_NOOP("Notifications", "Error"),
                    QT_TRANSLATE_NOOP("Notifications", "Failed to extend edge")
                );
                abortCommand();
            }
        }
        else {  // exit extension tool if user clicked on empty space
            BaseGeoId = -1;
            sketchgui->purgeHandler();  // no code after this line, Handler get deleted in ViewProvider
        }

        updateHint();
        return true;
    }

private:
    void activated() override
    {
        Gui::Selection().clearSelection();
        Gui::Selection().rmvSelectionGate();
        filterGate = new ExtendSelection(sketchgui->getObject());
        Gui::Selection().addSelectionGate(filterGate);
    }

    QString getCrosshairCursorSVGName() const override
    {
        return QStringLiteral("Sketcher_Pointer_Extension");
    }

protected:
    SelectMode Mode;
    std::vector<Base::Vector2d> EditCurve;
    int BaseGeoId;
    ExtendSelection* filterGate = nullptr;
    bool ExtendFromStart;  // if true, extend from start, else extend from end (circle only)
    bool SavedExtendFromStart;
    double Increment;
    std::vector<AutoConstraint> SugConstr;

public:
    std::list<Gui::InputHint> getToolHints() const override
    {
        using enum Gui::InputHint::UserInput;

        return Gui::lookupHints<SelectMode>(
            Mode,
            {
                {.state = STATUS_SEEK_First,
                 .hints =
                     {
                         {tr("%1 pick edge to extend", "Sketcher Extend: hint"), {MouseLeft}},
                     }},
                {.state = STATUS_SEEK_Second,
                 .hints =
                     {
                         {tr("%1 set extension length", "Sketcher Extend: hint"), {MouseLeft}},
                     }},
            });
    }

private:
    struct ExtensionTarget
    {
        int geoId;
        Sketcher::PointPos point;
    };

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

    std::unique_ptr<Sketcher::Constraint> makeConstraint(
        Sketcher::ConstraintType type,
        int firstGeoId,
        Sketcher::PointPos firstPoint = Sketcher::PointPos::none,
        int secondGeoId = Sketcher::GeoEnum::GeoUndef,
        Sketcher::PointPos secondPoint = Sketcher::PointPos::none
    )
    {
        auto constraint = std::make_unique<Sketcher::Constraint>();
        constraint->Type = type;
        constraint->First = firstGeoId;
        constraint->FirstPos = firstPoint;
        constraint->Second = secondGeoId;
        constraint->SecondPos = secondPoint;
        return constraint;
    }

    void addFilteredConstraints(
        std::vector<std::unique_ptr<Sketcher::Constraint>>& constraints
    )
    {
        if (constraints.empty() || !filterRedundantAutoConstraints(constraints)) {
            return;
        }
        if (!constraints.empty()) {
            addGeneratedAutoConstraints(constraints);
        }
    }

    void createFilteredAutoConstraints(
        const std::vector<AutoConstraint>& suggestions,
        const ExtensionTarget& target
    )
    {
        std::vector<std::unique_ptr<Sketcher::Constraint>> constraints;
        for (const auto& suggestion : suggestions) {
            if (!generateOneAutoConstraintFromSuggestion(
                    suggestion,
                    target.geoId,
                    target.point,
                    constraints
                )) {
                return;
            }
        }

        if (constraints.empty() || !filterRedundantAutoConstraints(constraints)) {
            return;
        }
        if (constraints.empty()) {
            return;
        }

        openCommand(QT_TRANSLATE_NOOP("Command", "Add Auto-Constraints"));
        addGeneratedAutoConstraints(constraints);
        commitCommand();
    }

    void setConstruction(int geoId)
    {
        Gui::cmdAppObjectArgs(sketchgui->getObject(), "setConstruction(%d,True)", geoId);
    }

    ExtensionTarget createConstructionExtension()
    {
        const Part::Geometry* geometry = sketchgui->getSketchObject()->getGeometry(BaseGeoId);
        if (geometry->is<Part::GeomLineSegment>()) {
            const auto* segment = static_cast<const Part::GeomLineSegment*>(geometry);
            const Base::Vector3d startPoint = segment->getStartPoint();
            const Base::Vector3d endPoint = segment->getEndPoint();
            Base::Vector3d direction = ExtendFromStart ? startPoint - endPoint : endPoint - startPoint;
            if (direction.Length() <= Precision::Confusion()) {
                return {
                    BaseGeoId,
                    ExtendFromStart ? Sketcher::PointPos::start : Sketcher::PointPos::end
                };
            }
            direction.Normalize();

            auto extension = std::make_unique<Part::GeomLineSegment>();
            Sketcher::PointPos extensionSharedPoint;
            Sketcher::PointPos extensionFreePoint;
            Sketcher::PointPos originalPoint;
            if (ExtendFromStart) {
                extension->setPoints(startPoint + direction * Increment, startPoint);
                extensionSharedPoint = Sketcher::PointPos::end;
                extensionFreePoint = Sketcher::PointPos::start;
                originalPoint = Sketcher::PointPos::start;
            }
            else {
                extension->setPoints(endPoint, endPoint + direction * Increment);
                extensionSharedPoint = Sketcher::PointPos::start;
                extensionFreePoint = Sketcher::PointPos::end;
                originalPoint = Sketcher::PointPos::end;
            }

            const int extensionGeoId = addConstructionGeometry(std::move(extension));
            std::vector<std::unique_ptr<Sketcher::Constraint>> constraints;
            constraints.push_back(
                makeConstraint(
                    Sketcher::Coincident,
                    extensionGeoId,
                    extensionSharedPoint,
                    BaseGeoId,
                    originalPoint
                )
            );
            constraints.push_back(
                makeConstraint(
                    Sketcher::Parallel,
                    extensionGeoId,
                    Sketcher::PointPos::none,
                    BaseGeoId
                )
            );
            addFilteredConstraints(constraints);
            return {extensionGeoId, extensionFreePoint};
        }

        const auto* arc = static_cast<const Part::GeomArcOfCircle*>(geometry);
        double startParameter;
        double endParameter;
        arc->getRange(startParameter, endParameter, true);
        const Sketcher::PointPos originalPoint
            = ExtendFromStart ? Sketcher::PointPos::start : Sketcher::PointPos::end;
        std::unique_ptr<Part::Geometry> extension(
            ExtendFromStart ? arc->createArc(startParameter - Increment, startParameter)
                            : arc->createArc(endParameter, endParameter + Increment)
        );
        if (!extension) {
            return {BaseGeoId, originalPoint};
        }
        const int extensionGeoId = addConstructionGeometry(std::move(extension));
        const Sketcher::PointPos extensionSharedPoint
            = ExtendFromStart ? Sketcher::PointPos::end : Sketcher::PointPos::start;
        const Sketcher::PointPos extensionFreePoint
            = ExtendFromStart ? Sketcher::PointPos::start : Sketcher::PointPos::end;

        std::vector<std::unique_ptr<Sketcher::Constraint>> constraints;
        constraints.push_back(
            makeConstraint(
                Sketcher::Coincident,
                extensionGeoId,
                extensionSharedPoint,
                BaseGeoId,
                originalPoint
            )
        );
        constraints.push_back(
            makeConstraint(
                Sketcher::Coincident,
                extensionGeoId,
                Sketcher::PointPos::mid,
                BaseGeoId,
                Sketcher::PointPos::mid
            )
        );
        constraints.push_back(
            makeConstraint(
                Sketcher::Equal,
                extensionGeoId,
                Sketcher::PointPos::none,
                BaseGeoId
            )
        );
        addFilteredConstraints(constraints);
        return {extensionGeoId, extensionFreePoint};
    }

    ExtensionTarget convertShortenedPartToConstruction()
    {
        auto* sketch = sketchgui->getSketchObject();
        const Part::Geometry* geometry = sketch->getGeometry(BaseGeoId);
        std::unique_ptr<Part::Geometry> shortenedPart;
        Sketcher::PointPos constructionSharedPoint;
        const Sketcher::PointPos originalPoint
            = ExtendFromStart ? Sketcher::PointPos::start : Sketcher::PointPos::end;
        const bool isLineSegment = geometry->is<Part::GeomLineSegment>();

        if (isLineSegment) {
            const auto* segment = static_cast<const Part::GeomLineSegment*>(geometry);
            const Base::Vector3d startPoint = segment->getStartPoint();
            const Base::Vector3d endPoint = segment->getEndPoint();
            Base::Vector3d direction = endPoint - startPoint;
            const double newLength = direction.Length() + Increment;
            if (newLength <= Precision::Confusion()) {
                setConstruction(BaseGeoId);
                return {
                    BaseGeoId,
                    ExtendFromStart ? Sketcher::PointPos::start : Sketcher::PointPos::end
                };
            }
            direction.Normalize();
            const Base::Vector3d cutPoint = ExtendFromStart
                ? endPoint - direction * newLength
                : startPoint + direction * newLength;
            auto removedSegment = std::make_unique<Part::GeomLineSegment>();
            if (ExtendFromStart) {
                removedSegment->setPoints(startPoint, cutPoint);
                constructionSharedPoint = Sketcher::PointPos::end;
            }
            else {
                removedSegment->setPoints(cutPoint, endPoint);
                constructionSharedPoint = Sketcher::PointPos::start;
            }
            shortenedPart = std::move(removedSegment);
        }
        else {
            const auto* arc = static_cast<const Part::GeomArcOfCircle*>(geometry);
            double startParameter;
            double endParameter;
            arc->getRange(startParameter, endParameter, true);
            if (endParameter - startParameter + Increment <= Precision::Confusion()) {
                setConstruction(BaseGeoId);
                return {
                    BaseGeoId,
                    ExtendFromStart ? Sketcher::PointPos::start : Sketcher::PointPos::end
                };
            }

            shortenedPart.reset(
                ExtendFromStart
                    ? arc->createArc(startParameter, startParameter - Increment)
                    : arc->createArc(endParameter + Increment, endParameter)
            );
            constructionSharedPoint
                = ExtendFromStart ? Sketcher::PointPos::end : Sketcher::PointPos::start;
        }

        if (!shortenedPart) {
            return {BaseGeoId, originalPoint};
        }

        Gui::cmdAppObjectArgs(
            sketchgui->getObject(),
            "extend(%d, %f, %d)\n",
            BaseGeoId,
            Increment,
            static_cast<int>(originalPoint)
        );

        const int constructionGeoId = addConstructionGeometry(std::move(shortenedPart));
        std::vector<std::unique_ptr<Sketcher::Constraint>> constraints;
        constraints.push_back(
            makeConstraint(
                Sketcher::Coincident,
                constructionGeoId,
                constructionSharedPoint,
                BaseGeoId,
                originalPoint
            )
        );
        if (isLineSegment) {
            constraints.push_back(
                makeConstraint(
                    Sketcher::Parallel,
                    constructionGeoId,
                    Sketcher::PointPos::none,
                    BaseGeoId
                )
            );
        }
        else {
            constraints.push_back(
                makeConstraint(
                    Sketcher::Coincident,
                    constructionGeoId,
                    Sketcher::PointPos::mid,
                    BaseGeoId,
                    Sketcher::PointPos::mid
                )
            );
            constraints.push_back(
                makeConstraint(
                    Sketcher::Equal,
                    constructionGeoId,
                    Sketcher::PointPos::none,
                    BaseGeoId
                )
            );
        }
        addFilteredConstraints(constraints);
        return {BaseGeoId, originalPoint};
    }

    ExtensionTarget extendWithConstruction()
    {
        if (Increment > Precision::Confusion()) {
            return createConstructionExtension();
        }
        if (Increment < -Precision::Confusion()) {
            return convertShortenedPartToConstruction();
        }
        return {
            BaseGeoId,
            ExtendFromStart ? Sketcher::PointPos::start : Sketcher::PointPos::end
        };
    }

    int crossProduct(Base::Vector2d& vec1, Base::Vector2d& vec2)
    {
        return vec1.x * vec2.y - vec1.y * vec2.x;
    }
};

}  // namespace SketcherGui
