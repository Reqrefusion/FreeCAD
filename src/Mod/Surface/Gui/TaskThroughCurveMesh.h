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

#include <QWidget>
#include <Gui/TaskView/TaskDialog.h>
#include <Mod/Part/Gui/ViewProviderSpline.h>
#include <Mod/Surface/App/FeatureThroughCurveMesh.h>

class QDoubleSpinBox;
class QLabel;
class QListWidget;
class QMenu;
class QSpinBox;

namespace SurfaceGui
{

class ViewProviderThroughCurveMesh: public PartGui::ViewProviderSpline
{
    PROPERTY_HEADER_WITH_OVERRIDE(SurfaceGui::ViewProviderThroughCurveMesh);

public:
    void setupContextMenu(QMenu*, QObject*, const char*) override;
    bool setEdit(int ModNum) override;
    void unsetEdit(int ModNum) override;
    QIcon getIcon() const override;
};

class ThroughCurveMeshPanel: public QWidget
{
public:
    ThroughCurveMeshPanel(ViewProviderThroughCurveMesh* vp, Surface::ThroughCurveMesh* obj);

    void open();
    bool accept();
    bool reject();

private:
    enum class Family
    {
        Primary,
        Cross
    };

    void setupUi();
    QListWidget* createCurveCollector(const QString& title, Family family);
    void populateLists();
    void loadOptionsFromObject();
    void applyListsToObject();
    void applyOptionsToObject();
    void addSelected(Family family);
    void removeSelected(Family family);
    void moveSelected(Family family, int direction);
    void clearFamily(Family family);
    void validateObject();
    void updateCounts();
    void updateStatusText(const QString& text, bool error = false);
    void openTransactionIfNeeded();
    QListWidget* listForFamily(Family family) const;
    QLabel* countLabelForFamily(Family family) const;

private:
    ViewProviderThroughCurveMesh* viewProvider;
    Surface::ThroughCurveMesh* editedObject;
    QListWidget* primaryList;
    QListWidget* crossList;
    QLabel* primaryCountLabel;
    QLabel* crossCountLabel;
    QLabel* statusLabel;
    QLabel* deviationLabel;
    QDoubleSpinBox* toleranceSpin;
    QSpinBox* samplesSpin;
    bool checkCommand {true};
};

class TaskThroughCurveMesh: public Gui::TaskView::TaskDialog
{
public:
    TaskThroughCurveMesh(ViewProviderThroughCurveMesh* vp, Surface::ThroughCurveMesh* obj);

    void open() override;
    bool accept() override;
    bool reject() override;

    QDialogButtonBox::StandardButtons getStandardButtons() const override
    {
        return QDialogButtonBox::Ok | QDialogButtonBox::Cancel;
    }

private:
    ThroughCurveMeshPanel* widget;
    Surface::ThroughCurveMesh* editedObj;
};

}  // namespace SurfaceGui
