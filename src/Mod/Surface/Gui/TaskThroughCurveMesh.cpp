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

#include <QAction>
#include <QAbstractItemView>
#include <QCheckBox>
#include <QComboBox>
#include <QDoubleSpinBox>
#include <QFont>
#include <QGridLayout>
#include <QGroupBox>
#include <QHBoxLayout>
#include <QLabel>
#include <QListWidget>
#include <QMenu>
#include <QMessageBox>
#include <QPushButton>
#include <QStringList>
#include <QSpinBox>
#include <QTimer>
#include <QToolButton>
#include <QVBoxLayout>

#include <algorithm>
#include <string>
#include <vector>

#include <App/Application.h>
#include <App/Document.h>
#include <Gui/Application.h>
#include <Gui/BitmapFactory.h>
#include <Gui/Command.h>
#include <Gui/Control.h>
#include <Gui/Selection/Selection.h>
#include <Gui/Selection/SelectionObject.h>
#include <Mod/Part/App/PartFeature.h>
#include <Mod/Part/App/TopoShape.h>

#include <TopAbs_ShapeEnum.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS_Shape.hxx>

#include "TaskThroughCurveMesh.h"


using namespace SurfaceGui;

PROPERTY_SOURCE(SurfaceGui::ViewProviderThroughCurveMesh, PartGui::ViewProviderSpline)

namespace
{

enum class RowKind
{
    Curve = 0,
    Family = 1,
    SupportFace = 2
};

struct RowMember
{
    App::DocumentObject* object {nullptr};
    std::string subName;
};

QString memberText(const RowMember& member)
{
    if (!member.object) {
        return QStringLiteral("<invalid>");
    }

    QString label = QString::fromUtf8(member.object->Label.getValue());
    if (label.isEmpty()) {
        label = QString::fromUtf8(member.object->getNameInDocument());
    }

    if (member.subName.empty()) {
        return label;
    }

    return QStringLiteral("%1.%2").arg(label, QString::fromStdString(member.subName));
}

QString membersText(const std::vector<RowMember>& members)
{
    QStringList names;
    for (const RowMember& member : members) {
        names << memberText(member);
    }
    return names.join(QStringLiteral(" + "));
}

QString subNamesText(const std::vector<RowMember>& members)
{
    QStringList names;
    for (const RowMember& member : members) {
        if (!member.subName.empty()) {
            names << QString::fromStdString(member.subName);
        }
    }
    return names.join(QStringLiteral(" + "));
}

QString itemTypeText(RowKind kind)
{
    if (kind == RowKind::Family) {
        return QObject::tr("Family");
    }
    if (kind == RowKind::SupportFace) {
        return QObject::tr("Support face");
    }
    return QObject::tr("Curve");
}

bool allMembersShareObject(const std::vector<RowMember>& members)
{
    if (members.empty() || !members.front().object) {
        return false;
    }
    App::DocumentObject* object = members.front().object;
    for (const RowMember& member : members) {
        if (member.object != object) {
            return false;
        }
    }
    return true;
}

QString itemText(const std::vector<RowMember>& members, RowKind kind)
{
    if (members.empty()) {
        return QStringLiteral("<invalid>");
    }

    const QString type = itemTypeText(kind);
    if (kind == RowKind::Family) {
        if (allMembersShareObject(members)) {
            QString label = QString::fromUtf8(members.front().object->Label.getValue());
            if (label.isEmpty()) {
                label = QString::fromUtf8(members.front().object->getNameInDocument());
            }

            const QString subText = subNamesText(members);
            if (subText.isEmpty()) {
                return QObject::tr("%1  %2  (whole object as one family)").arg(type, label);
            }

            return QObject::tr("%1  %2  (%3 member%4: %5)")
                .arg(type, label)
                .arg(members.size())
                .arg(members.size() == 1 ? QString() : QStringLiteral("s"))
                .arg(subText);
        }

        return QObject::tr("%1  (%2 member%3: %4)")
            .arg(type)
            .arg(members.size())
            .arg(members.size() == 1 ? QString() : QStringLiteral("s"))
            .arg(membersText(members));
    }

    return QObject::tr("%1  %2").arg(type, memberText(members.front()));
}

std::vector<std::string> subNamesFromVariant(const QVariant& variant)
{
    std::vector<std::string> subNames;

    if (variant.canConvert<QStringList>()) {
        const QStringList names = variant.toStringList();
        subNames.reserve(names.size());
        for (const QString& name : names) {
            subNames.emplace_back(name.toStdString());
        }
        return subNames;
    }

    // Backward compatibility with older task-panel item data that stored one
    // sub-element as a byte array.
    const QByteArray name = variant.toByteArray();
    if (!name.isEmpty()) {
        subNames.emplace_back(name.constData());
    }
    return subNames;
}

std::vector<RowMember> rowMembersFromData(const QList<QVariant>& data)
{
    std::vector<RowMember> members;
    if (data.size() < 3) {
        return members;
    }

    // Current format: [documentNames, objectNames, subNames, rowKind]. This
    // allows one visible row to represent a selection family containing curves
    // from several different objects.
    const QStringList docNames = data[0].toStringList();
    const QStringList objectNames = data[1].toStringList();
    const QStringList subNames = data[2].toStringList();
    if (!docNames.isEmpty() && docNames.size() == objectNames.size() && docNames.size() == subNames.size()) {
        members.reserve(docNames.size());
        for (int index = 0; index < docNames.size(); ++index) {
            App::Document* doc = App::GetApplication().getDocument(docNames[index].toUtf8().constData());
            App::DocumentObject* object = doc ? doc->getObject(objectNames[index].toUtf8().constData()) : nullptr;
            members.push_back({object, subNames[index].toStdString()});
        }
        return members;
    }

    // Older format: [documentName, objectName, subNameList, rowKind]. Convert
    // multiple sub-elements into multiple members of a single row.
    App::Document* doc = App::GetApplication().getDocument(data[0].toByteArray().constData());
    App::DocumentObject* object = doc ? doc->getObject(data[1].toByteArray().constData()) : nullptr;
    const std::vector<std::string> legacySubNames = subNamesFromVariant(data[2]);
    if (legacySubNames.empty()) {
        members.push_back({object, std::string()});
    }
    else {
        for (const std::string& subName : legacySubNames) {
            members.push_back({object, subName});
        }
    }
    return members;
}

RowKind rowKindFromData(const QList<QVariant>& data)
{
    if (data.size() >= 4) {
        if (data[3].toInt() == static_cast<int>(RowKind::Family)) {
            return RowKind::Family;
        }
        if (data[3].toInt() == static_cast<int>(RowKind::SupportFace)) {
            return RowKind::SupportFace;
        }
        return RowKind::Curve;
    }
    return rowMembersFromData(data).size() > 1 ? RowKind::Family : RowKind::Curve;
}

QString itemTextFromData(const QList<QVariant>& data)
{
    return itemText(rowMembersFromData(data), rowKindFromData(data));
}

QList<QVariant> itemData(const std::vector<RowMember>& members, RowKind kind)
{
    QList<QVariant> data;
    QStringList docNames;
    QStringList objectNames;
    QStringList subNames;

    for (const RowMember& member : members) {
        if (!member.object || !member.object->getDocument()) {
            continue;
        }
        docNames << QString::fromUtf8(member.object->getDocument()->getName());
        objectNames << QString::fromUtf8(member.object->getNameInDocument());
        subNames << QString::fromStdString(member.subName);
    }

    if (!docNames.isEmpty()) {
        data << docNames;
        data << objectNames;
        data << subNames;
        data << static_cast<int>(kind);
    }
    return data;
}

bool appendItem(QListWidget* list, const std::vector<RowMember>& members, RowKind kind)
{
    if (members.empty() || !list) {
        return false;
    }

    const QList<QVariant> data = itemData(members, kind);
    if (data.isEmpty()) {
        return false;
    }

    for (int row = 0; row < list->count(); ++row) {
        if (list->item(row)->data(Qt::UserRole).toList() == data) {
            return false;
        }
    }

    auto* item = new QListWidgetItem(list);
    item->setText(itemText(members, kind));
    item->setData(Qt::UserRole, data);
    item->setToolTip(kind == RowKind::Family
        ? QObject::tr("Selection family: this row is treated as one composite curve/chain.")
        : (kind == RowKind::SupportFace
              ? QObject::tr("Adjacent support face for G1/G2 boundary continuity.")
              : QObject::tr("Single curve row.")));
    return true;
}

void updateFamilyListDecorations(QListWidget* list)
{
    if (!list) {
        return;
    }

    for (int row = 0; row < list->count(); ++row) {
        QListWidgetItem* item = list->item(row);
        const QList<QVariant> data = item->data(Qt::UserRole).toList();
        const QString baseText = itemTextFromData(data);
        const RowKind kind = rowKindFromData(data);
        const bool isSelectionFamily = kind == RowKind::Family;
        const bool isSupportFace = kind == RowKind::SupportFace;
        item->setText(row == 0 && !isSupportFace ? QObject::tr("Origin / start  —  %1").arg(baseText) : baseText);

        QFont font = item->font();
        font.setBold(row == 0 && !isSupportFace);
        item->setFont(font);
        item->setToolTip(isSupportFace
            ? QObject::tr("Adjacent support face for G1/G2 boundary continuity. %1").arg(baseText)
            : (row == 0
                  ? QObject::tr("Origin/reference row for this curve family. %1").arg(baseText)
                  : (isSelectionFamily
                        ? QObject::tr("Family row: one connected chain/composite curve. %1").arg(baseText)
                        : QObject::tr("Curve row: one independent curve. %1").arg(baseText))));
    }
}

void appendPropertyRefs(
    QListWidget* list,
    const App::PropertyLinkSubList& property,
    const App::PropertyIntegerList& groupSizes
)
{
    if (!list) {
        return;
    }

    std::vector<App::DocumentObject*> objects = property.getValues();
    std::vector<std::string> subNames = property.getSubValues();
    std::vector<long> groups = groupSizes.getValues();
    if (groups.empty()) {
        groups.assign(objects.size(), 1);
    }

    std::size_t offset = 0;
    for (long group : groups) {
        if (group <= 0 || offset + static_cast<std::size_t>(group) > objects.size()) {
            break;
        }

        std::vector<RowMember> members;
        members.reserve(static_cast<std::size_t>(group));
        for (std::size_t i = offset; i < offset + static_cast<std::size_t>(group); ++i) {
            members.push_back({objects[i], subNames[i]});
        }

        appendItem(list, members, group > 1 ? RowKind::Family : RowKind::Curve);
        offset += static_cast<std::size_t>(group);
    }
}

void appendFaceRefs(
    QListWidget* list,
    const App::PropertyLinkSubList& property
)
{
    if (!list) {
        return;
    }

    std::vector<App::DocumentObject*> objects = property.getValues();
    std::vector<std::string> subNames = property.getSubValues();
    const std::size_t count = std::min(objects.size(), subNames.size());
    for (std::size_t index = 0; index < count; ++index) {
        appendItem(list, {{objects[index], subNames[index]}}, RowKind::SupportFace);
    }
}

void propertyFromList(
    QListWidget* list,
    App::PropertyLinkSubList& property,
    App::PropertyIntegerList& groupSizes
)
{
    std::vector<App::DocumentObject*> objects;
    std::vector<std::string> subNames;
    std::vector<long> groups;

    if (!list) {
        property.setValues(objects, subNames);
        groupSizes.setValues(groups);
        return;
    }

    for (int row = 0; row < list->count(); ++row) {
        const std::vector<RowMember> members = rowMembersFromData(list->item(row)->data(Qt::UserRole).toList());
        long rowSize = 0;
        for (const RowMember& member : members) {
            if (!member.object) {
                continue;
            }
            objects.push_back(member.object);
            subNames.push_back(member.subName);
            ++rowSize;
        }
        if (rowSize > 0) {
            groups.push_back(rowSize);
        }
    }

    property.setValues(objects, subNames);
    groupSizes.setValues(groups);
}

bool isCurveSubName(const std::string& subName)
{
    return subName.rfind("Edge", 0) == 0 || subName.rfind("Wire", 0) == 0;
}

bool isFaceSubName(const std::string& subName)
{
    return subName.rfind("Face", 0) == 0;
}

int addSelectedSubShapes(QListWidget* list, App::DocumentObject* owner)
{
    std::vector<RowMember> members;
    std::vector<Gui::SelectionObject> selection = Gui::Selection().getSelectionEx(
        nullptr,
        Part::Feature::getClassTypeId()
    );

    for (Gui::SelectionObject& selected : selection) {
        App::DocumentObject* object = selected.getObject();
        if (!object || object == owner || !object->isDerivedFrom<Part::Feature>()) {
            continue;
        }

        const std::vector<std::string>& selectedSubNames = selected.getSubNames();
        std::vector<std::string> curveSubNames;
        curveSubNames.reserve(selectedSubNames.size());
        for (const std::string& subName : selectedSubNames) {
            if (isCurveSubName(subName)) {
                curveSubNames.push_back(subName);
            }
        }

        if (curveSubNames.empty()) {
            if (selectedSubNames.empty()) {
                members.push_back({object, std::string()});
            }
            continue;
        }

        for (const std::string& subName : curveSubNames) {
            members.push_back({object, subName});
        }
    }

    // Mesh-style selection family behavior: one Add Selection action consumes the
    // whole current selection as one row. If the user selected several objects
    // or several sub-elements, they form one connected family/chain. To add
    // independent curves, select and add them one at a time.
    if (members.empty()) {
        return 0;
    }

    const RowKind kind = members.size() > 1 ? RowKind::Family : RowKind::Curve;
    return appendItem(list, members, kind) ? 1 : 0;
}


int addSelectedFaces(QListWidget* list, App::DocumentObject* owner)
{
    int added = 0;
    std::vector<Gui::SelectionObject> selection = Gui::Selection().getSelectionEx(
        nullptr,
        Part::Feature::getClassTypeId()
    );

    for (Gui::SelectionObject& selected : selection) {
        App::DocumentObject* object = selected.getObject();
        if (!object || object == owner || !object->isDerivedFrom<Part::Feature>()) {
            continue;
        }

        const std::vector<std::string>& selectedSubNames = selected.getSubNames();
        if (selectedSubNames.empty()) {
            added += appendItem(list, {{object, std::string()}}, RowKind::SupportFace) ? 1 : 0;
            continue;
        }

        for (const std::string& subName : selectedSubNames) {
            if (isFaceSubName(subName)) {
                added += appendItem(list, {{object, subName}}, RowKind::SupportFace) ? 1 : 0;
            }
        }
    }

    return added;
}

void propertyFacesFromList(QListWidget* list, App::PropertyLinkSubList& property)
{
    std::vector<App::DocumentObject*> objects;
    std::vector<std::string> subNames;

    if (list) {
        for (int row = 0; row < list->count(); ++row) {
            const std::vector<RowMember> members = rowMembersFromData(list->item(row)->data(Qt::UserRole).toList());
            for (const RowMember& member : members) {
                if (!member.object) {
                    continue;
                }
                objects.push_back(member.object);
                subNames.push_back(member.subName);
            }
        }
    }

    property.setValues(objects, subNames);
}

QToolButton* makeToolButton(QWidget* parent, const QString& text, const QString& tooltip)
{
    auto* button = new QToolButton(parent);
    button->setText(text);
    button->setToolTip(tooltip);
    button->setAutoRaise(true);
    button->setMinimumWidth(28);
    return button;
}

}  // namespace

void ViewProviderThroughCurveMesh::setupContextMenu(QMenu* menu, QObject* receiver, const char* member)
{
    QAction* act = menu->addAction(QObject::tr("Edit Through Curve Mesh"), receiver, member);
    act->setData(QVariant(static_cast<int>(ViewProvider::Default)));
    PartGui::ViewProviderSpline::setupContextMenu(menu, receiver, member);
}

bool ViewProviderThroughCurveMesh::setEdit(int ModNum)
{
    if (ModNum == ViewProvider::Default) {
        auto* obj = this->getObject<Surface::ThroughCurveMesh>();
        Gui::Control().showDialog(new TaskThroughCurveMesh(this, obj));
        return true;
    }

    return PartGui::ViewProviderSpline::setEdit(ModNum);
}

void ViewProviderThroughCurveMesh::unsetEdit(int ModNum)
{
    if (ModNum == ViewProvider::Default) {
        QTimer::singleShot(0, [] { Gui::Control().closeDialog(nullptr); });
    }
    else {
        PartGui::ViewProviderSpline::unsetEdit(ModNum);
    }
}

QIcon ViewProviderThroughCurveMesh::getIcon() const
{
    return Gui::BitmapFactory().pixmap("Surface_ThroughCurveMesh");
}

ThroughCurveMeshPanel::ThroughCurveMeshPanel(
    ViewProviderThroughCurveMesh* vp,
    Surface::ThroughCurveMesh* obj
)
    : viewProvider(vp)
    , editedObject(obj)
    , primaryList(nullptr)
    , crossList(nullptr)
    , firstPrimarySupportFaceList(nullptr)
    , lastPrimarySupportFaceList(nullptr)
    , firstCrossSupportFaceList(nullptr)
    , lastCrossSupportFaceList(nullptr)
    , primaryCountLabel(nullptr)
    , crossCountLabel(nullptr)
    , firstPrimarySupportFaceCountLabel(nullptr)
    , lastPrimarySupportFaceCountLabel(nullptr)
    , firstCrossSupportFaceCountLabel(nullptr)
    , lastCrossSupportFaceCountLabel(nullptr)
    , statusLabel(nullptr)
    , deviationLabel(nullptr)
    , gapLabel(nullptr)
    , intersectionToleranceSpin(nullptr)
    , fitToleranceSpin(nullptr)
    , samplesUSpin(nullptr)
    , samplesVSpin(nullptr)
    , autoSortCheck(nullptr)
    , emphasisCombo(nullptr)
    , parameterizationCombo(nullptr)
    , firstPrimaryContinuityCombo(nullptr)
    , lastPrimaryContinuityCombo(nullptr)
    , firstCrossContinuityCombo(nullptr)
    , lastCrossContinuityCombo(nullptr)
{
    setupUi();
    populateLists();
    loadOptionsFromObject();
    updateCounts();
}

void ThroughCurveMeshPanel::setupUi()
{
    auto* mainLayout = new QVBoxLayout(this);
    mainLayout->setContentsMargins(6, 6, 6, 6);
    mainLayout->setSpacing(6);

    primaryList = createCurveCollector(tr("Primary curves"), Family::Primary);
    crossList = createCurveCollector(tr("Cross curves"), Family::Cross);

    auto* previewGroup = new QGroupBox(tr("Preview"), this);
    auto* previewLayout = new QGridLayout(previewGroup);
    auto* previewButton = new QPushButton(tr("Preview"), previewGroup);
    previewButton->setToolTip(tr("Updates the temporary shape without adding selected curves. Use OK to keep the result or Cancel to discard it."));
    deviationLabel = new QLabel(tr("Deviation: not computed"), previewGroup);
    deviationLabel->setWordWrap(true);
    gapLabel = new QLabel(tr("Max gap: not computed"), previewGroup);
    gapLabel->setWordWrap(true);
    previewLayout->addWidget(previewButton, 0, 0, 2, 1);
    previewLayout->addWidget(deviationLabel, 0, 1);
    previewLayout->addWidget(gapLabel, 1, 1);
    connect(previewButton, &QPushButton::clicked, this, &ThroughCurveMeshPanel::validateObject);
    mainLayout->addWidget(previewGroup);

    auto* outputGroup = new QGroupBox(tr("Output surface options"), this);
    auto* outputLayout = new QGridLayout(outputGroup);

    emphasisCombo = new QComboBox(outputGroup);
    emphasisCombo->addItem(tr("Both"));
    emphasisCombo->addItem(tr("Primary"));
    emphasisCombo->addItem(tr("Cross"));
    emphasisCombo->setToolTip(tr("Adds extra fitting samples along the selected curve family without changing the through-curve Gordon formula."));

    outputLayout->addWidget(new QLabel(tr("Emphasis"), outputGroup), 0, 0);
    outputLayout->addWidget(emphasisCombo, 0, 1);
    mainLayout->addWidget(outputGroup);

    auto* continuityGroup = new QGroupBox(tr("Boundary continuity"), this);
    auto* continuityLayout = new QGridLayout(continuityGroup);
    continuityLayout->addWidget(new QLabel(tr("Boundary"), continuityGroup), 0, 0);
    continuityLayout->addWidget(new QLabel(tr("Continuity"), continuityGroup), 0, 1);
    continuityLayout->addWidget(new QLabel(tr("Support faces"), continuityGroup), 0, 2);

    auto makeContinuityCombo = [this, continuityGroup]() {
        auto* combo = new QComboBox(continuityGroup);
        combo->addItem(tr("G0 (Position)"));
        combo->addItem(tr("G1 tangent"));
        combo->addItem(tr("G2 curvature"));
        combo->setToolTip(tr("G0 does not require a support face. G1/G2 uses the selected adjacent support face for that boundary."));
        return combo;
    };

    auto addBoundaryRow = [this, continuityGroup, continuityLayout, makeContinuityCombo](
                              int row,
                              Boundary boundary,
                              const QString& label,
                              QComboBox*& combo,
                              QListWidget*& list,
                              QLabel*& countLabel) {
        combo = makeContinuityCombo();
        countLabel = new QLabel(continuityGroup);
        auto* addButton = makeToolButton(continuityGroup, tr("Add"), tr("Add selected adjacent faces for this boundary"));
        auto* removeButton = makeToolButton(continuityGroup, tr("Remove"), tr("Remove selected support faces for this boundary"));
        auto* clearButton = makeToolButton(continuityGroup, tr("Clear"), tr("Clear support faces for this boundary"));
        list = new QListWidget(continuityGroup);
        list->setSelectionMode(QAbstractItemView::ExtendedSelection);
        list->setAlternatingRowColors(true);
        list->setMaximumHeight(58);

        auto* buttons = new QHBoxLayout();
        buttons->setContentsMargins(0, 0, 0, 0);
        buttons->addWidget(addButton);
        buttons->addWidget(removeButton);
        buttons->addWidget(clearButton);
        buttons->addStretch(1);

        auto* faceCell = new QWidget(continuityGroup);
        auto* faceLayout = new QVBoxLayout(faceCell);
        faceLayout->setContentsMargins(0, 0, 0, 0);
        faceLayout->setSpacing(2);
        faceLayout->addWidget(countLabel);
        faceLayout->addWidget(list);
        faceLayout->addLayout(buttons);

        continuityLayout->addWidget(new QLabel(label, continuityGroup), row, 0);
        continuityLayout->addWidget(combo, row, 1);
        continuityLayout->addWidget(faceCell, row, 2);

        connect(addButton, &QToolButton::clicked, this, [this, boundary]() { addSelectedSupportFaces(boundary); });
        connect(removeButton, &QToolButton::clicked, this, [this, boundary]() { removeSelectedSupportFaces(boundary); });
        connect(clearButton, &QToolButton::clicked, this, [this, boundary]() { clearSupportFaces(boundary); });
    };

    addBoundaryRow(1, Boundary::FirstPrimary, tr("First primary"), firstPrimaryContinuityCombo, firstPrimarySupportFaceList, firstPrimarySupportFaceCountLabel);
    addBoundaryRow(2, Boundary::LastPrimary, tr("Last primary"), lastPrimaryContinuityCombo, lastPrimarySupportFaceList, lastPrimarySupportFaceCountLabel);
    addBoundaryRow(3, Boundary::FirstCross, tr("First cross"), firstCrossContinuityCombo, firstCrossSupportFaceList, firstCrossSupportFaceCountLabel);
    addBoundaryRow(4, Boundary::LastCross, tr("Last cross"), lastCrossContinuityCombo, lastCrossSupportFaceList, lastCrossSupportFaceCountLabel);

    mainLayout->addWidget(continuityGroup);

    auto* settingsGroup = new QGroupBox(tr("Settings"), this);
    auto* settingsLayout = new QGridLayout(settingsGroup);

    intersectionToleranceSpin = new QDoubleSpinBox(settingsGroup);
    intersectionToleranceSpin->setDecimals(4);
    intersectionToleranceSpin->setRange(0.0, 100.0);
    intersectionToleranceSpin->setSingleStep(0.01);
    intersectionToleranceSpin->setSuffix(tr(" mm"));
    intersectionToleranceSpin->setToolTip(tr("Maximum gap allowed when accepting a primary/cross curve crossing."));

    fitToleranceSpin = new QDoubleSpinBox(settingsGroup);
    fitToleranceSpin->setDecimals(4);
    fitToleranceSpin->setRange(0.0, 100.0);
    fitToleranceSpin->setSingleStep(0.01);
    fitToleranceSpin->setSuffix(tr(" mm"));
    fitToleranceSpin->setToolTip(tr("Tolerance used by the B-spline surface approximation after the curve mesh has been sampled."));

    samplesUSpin = new QSpinBox(settingsGroup);
    samplesUSpin->setRange(4, 200);
    samplesUSpin->setSingleStep(1);
    samplesUSpin->setToolTip(tr("Sampling density in the primary-family direction."));

    samplesVSpin = new QSpinBox(settingsGroup);
    samplesVSpin->setRange(4, 200);
    samplesVSpin->setSingleStep(1);
    samplesVSpin->setToolTip(tr("Sampling density in the cross-family direction."));

    parameterizationCombo = new QComboBox(settingsGroup);
    parameterizationCombo->addItem(tr("Chord length"));
    parameterizationCombo->addItem(tr("Centripetal"));
    parameterizationCombo->addItem(tr("Uniform"));
    parameterizationCombo->setToolTip(tr("Controls how curve-network intersections are mapped to surface parameters before fitting."));

    autoSortCheck = new QCheckBox(tr("Auto sort and orient curve families"), settingsGroup);
    autoSortCheck->setToolTip(tr("Sorts rows using the curve-network intersections before building the surface."));

    settingsLayout->addWidget(new QLabel(tr("Intersection tolerance"), settingsGroup), 0, 0);
    settingsLayout->addWidget(intersectionToleranceSpin, 0, 1);
    settingsLayout->addWidget(new QLabel(tr("G0 / fit tolerance"), settingsGroup), 1, 0);
    settingsLayout->addWidget(fitToleranceSpin, 1, 1);
    settingsLayout->addWidget(new QLabel(tr("Primary samples"), settingsGroup), 2, 0);
    settingsLayout->addWidget(samplesUSpin, 2, 1);
    settingsLayout->addWidget(new QLabel(tr("Cross samples"), settingsGroup), 3, 0);
    settingsLayout->addWidget(samplesVSpin, 3, 1);
    settingsLayout->addWidget(new QLabel(tr("Parameterization"), settingsGroup), 4, 0);
    settingsLayout->addWidget(parameterizationCombo, 4, 1);
    settingsLayout->addWidget(autoSortCheck, 5, 0, 1, 2);

    auto optionChanged = [this]() {
        openTransactionIfNeeded();
        applyOptionsToObject();
    };
    connect(intersectionToleranceSpin, qOverload<double>(&QDoubleSpinBox::valueChanged), this, optionChanged);
    connect(fitToleranceSpin, qOverload<double>(&QDoubleSpinBox::valueChanged), this, optionChanged);
    connect(samplesUSpin, qOverload<int>(&QSpinBox::valueChanged), this, optionChanged);
    connect(samplesVSpin, qOverload<int>(&QSpinBox::valueChanged), this, optionChanged);
    connect(autoSortCheck, &QCheckBox::toggled, this, optionChanged);
    connect(emphasisCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, optionChanged);
    connect(parameterizationCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, optionChanged);
    connect(firstPrimaryContinuityCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, optionChanged);
    connect(lastPrimaryContinuityCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, optionChanged);
    connect(firstCrossContinuityCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, optionChanged);
    connect(lastCrossContinuityCombo, qOverload<int>(&QComboBox::currentIndexChanged), this, optionChanged);

    mainLayout->addWidget(settingsGroup);

    statusLabel = new QLabel(this);
    statusLabel->setWordWrap(true);
    statusLabel->setText(tr("Add one or more primary rows and cross rows, then preview or press OK."));
    mainLayout->addWidget(statusLabel);
    mainLayout->addStretch(1);
}

QListWidget* ThroughCurveMeshPanel::createCurveCollector(const QString& title, Family family)
{
    auto* group = new QGroupBox(title, this);
    auto* layout = new QVBoxLayout(group);

    auto* selectionRow = new QHBoxLayout();
    auto* countLabel = new QLabel(group);
    auto* addButton = new QPushButton(tr("Add"), group);
    addButton->setToolTip(tr("Adds the current selection as one row. A single selected curve is added as Curve; multiple selected curves are added as one Family."));
    selectionRow->addWidget(countLabel, 1);
    selectionRow->addWidget(addButton);
    layout->addLayout(selectionRow);


    auto* listRow = new QHBoxLayout();
    auto* list = new QListWidget(group);
    list->setSelectionMode(QAbstractItemView::ExtendedSelection);
    list->setAlternatingRowColors(true);
    listRow->addWidget(list, 1);

    auto* buttonColumn = new QVBoxLayout();
    auto* removeButton = makeToolButton(group, tr("Remove"), tr("Remove selected rows"));
    auto* moveUpButton = makeToolButton(group, QStringLiteral("↑"), tr("Move selected rows up"));
    auto* moveDownButton = makeToolButton(group, QStringLiteral("↓"), tr("Move selected rows down"));
    auto* clearButton = makeToolButton(group, tr("Clear"), tr("Clear this list"));
    buttonColumn->addWidget(removeButton);
    buttonColumn->addWidget(moveUpButton);
    buttonColumn->addWidget(moveDownButton);
    buttonColumn->addWidget(clearButton);
    buttonColumn->addStretch(1);
    listRow->addLayout(buttonColumn);
    layout->addLayout(listRow);

    connect(addButton, &QPushButton::clicked, this, [this, family]() { addSelected(family); });
    connect(removeButton, &QToolButton::clicked, this, [this, family]() { removeSelected(family); });
    connect(moveUpButton, &QToolButton::clicked, this, [this, family]() { moveSelected(family, -1); });
    connect(moveDownButton, &QToolButton::clicked, this, [this, family]() { moveSelected(family, 1); });
    connect(clearButton, &QToolButton::clicked, this, [this, family]() { clearFamily(family); });

    if (family == Family::Primary) {
        primaryCountLabel = countLabel;
    }
    else {
        crossCountLabel = countLabel;
    }

    static_cast<QVBoxLayout*>(this->layout())->addWidget(group);
    return list;
}

void ThroughCurveMeshPanel::populateLists()
{
    primaryList->clear();
    crossList->clear();
    firstPrimarySupportFaceList->clear();
    lastPrimarySupportFaceList->clear();
    firstCrossSupportFaceList->clear();
    lastCrossSupportFaceList->clear();
    if (!editedObject) {
        return;
    }

    appendPropertyRefs(primaryList, editedObject->PrimaryCurves, editedObject->PrimaryCurveGroupSizes);
    appendPropertyRefs(crossList, editedObject->CrossCurves, editedObject->CrossCurveGroupSizes);
    appendFaceRefs(firstPrimarySupportFaceList, editedObject->FirstPrimarySupportFaces);
    appendFaceRefs(lastPrimarySupportFaceList, editedObject->LastPrimarySupportFaces);
    appendFaceRefs(firstCrossSupportFaceList, editedObject->FirstCrossSupportFaces);
    appendFaceRefs(lastCrossSupportFaceList, editedObject->LastCrossSupportFaces);
}

void ThroughCurveMeshPanel::loadOptionsFromObject()
{
    if (!editedObject) {
        return;
    }

    intersectionToleranceSpin->blockSignals(true);
    fitToleranceSpin->blockSignals(true);
    samplesUSpin->blockSignals(true);
    samplesVSpin->blockSignals(true);
    autoSortCheck->blockSignals(true);
    emphasisCombo->blockSignals(true);
    parameterizationCombo->blockSignals(true);
    firstPrimaryContinuityCombo->blockSignals(true);
    lastPrimaryContinuityCombo->blockSignals(true);
    firstCrossContinuityCombo->blockSignals(true);
    lastCrossContinuityCombo->blockSignals(true);

    intersectionToleranceSpin->setValue(editedObject->Tolerance.getValue());
    fitToleranceSpin->setValue(editedObject->PositionTolerance.getValue());
    samplesUSpin->setValue(editedObject->SamplesU.getValue());
    samplesVSpin->setValue(editedObject->SamplesV.getValue());
    autoSortCheck->setChecked(editedObject->AutoSort.getValue());
    emphasisCombo->setCurrentIndex(static_cast<int>(editedObject->Emphasis.getValue()));
    parameterizationCombo->setCurrentIndex(static_cast<int>(editedObject->Parameterization.getValue()));
    firstPrimaryContinuityCombo->setCurrentIndex(static_cast<int>(editedObject->FirstPrimaryContinuity.getValue()));
    lastPrimaryContinuityCombo->setCurrentIndex(static_cast<int>(editedObject->LastPrimaryContinuity.getValue()));
    firstCrossContinuityCombo->setCurrentIndex(static_cast<int>(editedObject->FirstCrossContinuity.getValue()));
    lastCrossContinuityCombo->setCurrentIndex(static_cast<int>(editedObject->LastCrossContinuity.getValue()));

    intersectionToleranceSpin->blockSignals(false);
    fitToleranceSpin->blockSignals(false);
    samplesUSpin->blockSignals(false);
    samplesVSpin->blockSignals(false);
    autoSortCheck->blockSignals(false);
    emphasisCombo->blockSignals(false);
    parameterizationCombo->blockSignals(false);
    firstPrimaryContinuityCombo->blockSignals(false);
    lastPrimaryContinuityCombo->blockSignals(false);
    firstCrossContinuityCombo->blockSignals(false);
    lastCrossContinuityCombo->blockSignals(false);
}

QListWidget* ThroughCurveMeshPanel::listForFamily(Family family) const
{
    return family == Family::Primary ? primaryList : crossList;
}

QLabel* ThroughCurveMeshPanel::countLabelForFamily(Family family) const
{
    return family == Family::Primary ? primaryCountLabel : crossCountLabel;
}

QListWidget* ThroughCurveMeshPanel::supportFaceListForBoundary(Boundary boundary) const
{
    switch (boundary) {
        case Boundary::LastPrimary:
            return lastPrimarySupportFaceList;
        case Boundary::FirstCross:
            return firstCrossSupportFaceList;
        case Boundary::LastCross:
            return lastCrossSupportFaceList;
        case Boundary::FirstPrimary:
        default:
            return firstPrimarySupportFaceList;
    }
}

QLabel* ThroughCurveMeshPanel::supportFaceCountLabelForBoundary(Boundary boundary) const
{
    switch (boundary) {
        case Boundary::LastPrimary:
            return lastPrimarySupportFaceCountLabel;
        case Boundary::FirstCross:
            return firstCrossSupportFaceCountLabel;
        case Boundary::LastCross:
            return lastCrossSupportFaceCountLabel;
        case Boundary::FirstPrimary:
        default:
            return firstPrimarySupportFaceCountLabel;
    }
}

QComboBox* ThroughCurveMeshPanel::continuityComboForBoundary(Boundary boundary) const
{
    switch (boundary) {
        case Boundary::LastPrimary:
            return lastPrimaryContinuityCombo;
        case Boundary::FirstCross:
            return firstCrossContinuityCombo;
        case Boundary::LastCross:
            return lastCrossContinuityCombo;
        case Boundary::FirstPrimary:
        default:
            return firstPrimaryContinuityCombo;
    }
}

void ThroughCurveMeshPanel::openTransactionIfNeeded()
{
    if (!checkCommand || !editedObject || !editedObject->getDocument()) {
        return;
    }

    if (!editedObject->getDocument()->hasPendingTransaction()) {
        std::string message("Edit ");
        message += editedObject->Label.getValue();
        editedObject->getDocument()->openTransaction(message.c_str());
    }
    checkCommand = false;
}

void ThroughCurveMeshPanel::open()
{
    openTransactionIfNeeded();
}

void ThroughCurveMeshPanel::addSelected(Family family)
{
    openTransactionIfNeeded();
    const int added = addSelectedSubShapes(listForFamily(family), editedObject);
    if (added == 0) {
        updateStatusText(
            tr("Select one or more edge/wire items, then press Add."),
            true
        );
    }
    else {
        updateStatusText(QString());
    }
    applyListsToObject();
    updateCounts();
}

void ThroughCurveMeshPanel::addSelectedSupportFaces(Boundary boundary)
{
    openTransactionIfNeeded();
    const int added = addSelectedFaces(supportFaceListForBoundary(boundary), editedObject);
    if (added == 0) {
        updateStatusText(tr("Select adjacent face items, then press Add."), true);
    }
    else {
        updateStatusText(QString());
    }
    applySupportFacesToObject();
    updateCounts();
}

void ThroughCurveMeshPanel::removeSelectedSupportFaces(Boundary boundary)
{
    openTransactionIfNeeded();
    QListWidget* list = supportFaceListForBoundary(boundary);
    const QList<QListWidgetItem*> selectedItems = list->selectedItems();
    for (QListWidgetItem* item : selectedItems) {
        delete list->takeItem(list->row(item));
    }
    applySupportFacesToObject();
    updateCounts();
}

void ThroughCurveMeshPanel::clearSupportFaces(Boundary boundary)
{
    openTransactionIfNeeded();
    supportFaceListForBoundary(boundary)->clear();
    applySupportFacesToObject();
    updateCounts();
    updateStatusText(QString());
}

void ThroughCurveMeshPanel::removeSelected(Family family)
{
    openTransactionIfNeeded();
    QListWidget* list = listForFamily(family);
    const QList<QListWidgetItem*> selectedItems = list->selectedItems();
    for (QListWidgetItem* item : selectedItems) {
        delete list->takeItem(list->row(item));
    }
    applyListsToObject();
    updateCounts();
}

void ThroughCurveMeshPanel::moveSelected(Family family, int direction)
{
    openTransactionIfNeeded();
    QListWidget* list = listForFamily(family);
    if (!list || direction == 0) {
        return;
    }

    if (direction < 0) {
        for (int row = 1; row < list->count(); ++row) {
            QListWidgetItem* item = list->item(row);
            if (item && item->isSelected()) {
                QListWidgetItem* moved = list->takeItem(row);
                list->insertItem(row - 1, moved);
                moved->setSelected(true);
            }
        }
    }
    else {
        for (int row = list->count() - 2; row >= 0; --row) {
            QListWidgetItem* item = list->item(row);
            if (item && item->isSelected()) {
                QListWidgetItem* moved = list->takeItem(row);
                list->insertItem(row + 1, moved);
                moved->setSelected(true);
            }
        }
    }

    applyListsToObject();
    updateCounts();
}

void ThroughCurveMeshPanel::clearFamily(Family family)
{
    openTransactionIfNeeded();
    listForFamily(family)->clear();
    applyListsToObject();
    updateCounts();
    updateStatusText(QString());
}

void ThroughCurveMeshPanel::applyListsToObject()
{
    if (!editedObject) {
        return;
    }

    propertyFromList(primaryList, editedObject->PrimaryCurves, editedObject->PrimaryCurveGroupSizes);
    propertyFromList(crossList, editedObject->CrossCurves, editedObject->CrossCurveGroupSizes);
    applySupportFacesToObject();
}

void ThroughCurveMeshPanel::applySupportFacesToObject()
{
    if (!editedObject) {
        return;
    }
    propertyFacesFromList(firstPrimarySupportFaceList, editedObject->FirstPrimarySupportFaces);
    propertyFacesFromList(lastPrimarySupportFaceList, editedObject->LastPrimarySupportFaces);
    propertyFacesFromList(firstCrossSupportFaceList, editedObject->FirstCrossSupportFaces);
    propertyFacesFromList(lastCrossSupportFaceList, editedObject->LastCrossSupportFaces);
}

void ThroughCurveMeshPanel::applyOptionsToObject()
{
    if (!editedObject) {
        return;
    }

    editedObject->Tolerance.setValue(intersectionToleranceSpin->value());
    editedObject->PositionTolerance.setValue(fitToleranceSpin->value());
    editedObject->SamplesU.setValue(samplesUSpin->value());
    editedObject->SamplesV.setValue(samplesVSpin->value());
    editedObject->Samples.setValue(std::max(samplesUSpin->value(), samplesVSpin->value()));
    editedObject->AutoSort.setValue(autoSortCheck->isChecked());
    editedObject->Emphasis.setValue(emphasisCombo->currentIndex());
    editedObject->Parameterization.setValue(parameterizationCombo->currentIndex());
    editedObject->FirstPrimaryContinuity.setValue(firstPrimaryContinuityCombo->currentIndex());
    editedObject->LastPrimaryContinuity.setValue(lastPrimaryContinuityCombo->currentIndex());
    editedObject->FirstCrossContinuity.setValue(firstCrossContinuityCombo->currentIndex());
    editedObject->LastCrossContinuity.setValue(lastCrossContinuityCombo->currentIndex());
}

void ThroughCurveMeshPanel::validateObject()
{
    applyListsToObject();
    applyOptionsToObject();
    updateCounts();
    if (!editedObject) {
        return;
    }

    try {
        editedObject->recomputeFeature();
    }
    catch (const std::exception& e) {
        deviationLabel->setText(tr("Deviation: not computed"));
        gapLabel->setText(tr("Max gap: not computed"));
        updateStatusText(QString::fromUtf8(e.what()), true);
        return;
    }
    catch (...) {
        deviationLabel->setText(tr("Deviation: not computed"));
        gapLabel->setText(tr("Max gap: not computed"));
        updateStatusText(tr("Preview failed. Check the selected curve network."), true);
        return;
    }

    if (editedObject->isValid()) {
        deviationLabel->setText(
            tr("Deviation: %1 mm").arg(editedObject->MaxDeviation.getValue(), 0, 'g', 6)
        );
        gapLabel->setText(
            tr("Max gap: %1 mm").arg(editedObject->MaxIntersectionGap.getValue(), 0, 'g', 6)
        );
        updateStatusText(tr("Preview updated. Press OK to keep the surface, or Cancel to discard it."));
    }
    else {
        deviationLabel->setText(tr("Deviation: not computed"));
        gapLabel->setText(tr("Max gap: not computed"));
        updateStatusText(QString::fromLatin1(editedObject->getStatusString()), true);
    }
    Gui::Command::updateActive();
}

void ThroughCurveMeshPanel::updateCounts()
{
    updateFamilyListDecorations(primaryList);
    updateFamilyListDecorations(crossList);
    if (primaryCountLabel) {
        primaryCountLabel->setText(tr("Primary rows (%1)").arg(primaryList ? primaryList->count() : 0));
    }
    if (crossCountLabel) {
        crossCountLabel->setText(tr("Cross rows (%1)").arg(crossList ? crossList->count() : 0));
    }
    auto updateSupportCount = [this](Boundary boundary, const QString& label) {
        QLabel* countLabel = supportFaceCountLabelForBoundary(boundary);
        QListWidget* list = supportFaceListForBoundary(boundary);
        if (countLabel) {
            countLabel->setText(tr("%1 support faces (%2)").arg(label).arg(list ? list->count() : 0));
        }
        updateFamilyListDecorations(list);
    };
    updateSupportCount(Boundary::FirstPrimary, tr("First primary"));
    updateSupportCount(Boundary::LastPrimary, tr("Last primary"));
    updateSupportCount(Boundary::FirstCross, tr("First cross"));
    updateSupportCount(Boundary::LastCross, tr("Last cross"));
}

void ThroughCurveMeshPanel::updateStatusText(const QString& text, bool error)
{
    if (!statusLabel) {
        return;
    }

    statusLabel->setText(text);
    statusLabel->setStyleSheet(error ? QStringLiteral("color: #b00000;") : QStringLiteral("color: #006000;"));
}

bool ThroughCurveMeshPanel::accept()
{
    applyListsToObject();
    applyOptionsToObject();
    updateCounts();

    try {
        if (editedObject) {
            editedObject->recomputeFeature();
        }
    }
    catch (const std::exception& e) {
        QMessageBox::warning(
            this,
            tr("Invalid through curve mesh"),
            QString::fromUtf8(e.what())
        );
        return false;
    }
    catch (...) {
        QMessageBox::warning(
            this,
            tr("Invalid through curve mesh"),
            tr("Failed to recompute the through curve mesh. Check the selected curve network.")
        );
        return false;
    }

    if (editedObject && !editedObject->isValid()) {
        QMessageBox::warning(
            this,
            tr("Invalid through curve mesh"),
            QString::fromLatin1(editedObject->getStatusString())
        );
        return false;
    }

    return true;
}

bool ThroughCurveMeshPanel::reject()
{
    return true;
}

TaskThroughCurveMesh::TaskThroughCurveMesh(
    ViewProviderThroughCurveMesh* vp,
    Surface::ThroughCurveMesh* obj
)
    : editedObj(obj)
{
    widget = new ThroughCurveMeshPanel(vp, obj);
    addTaskBox(Gui::BitmapFactory().pixmap("Surface_ThroughCurveMesh"), widget);
}

void TaskThroughCurveMesh::open()
{
    widget->open();
}

bool TaskThroughCurveMesh::accept()
{
    if (!widget->accept()) {
        return false;
    }

    if (editedObj && editedObj->getDocument() && editedObj->getDocument()->hasPendingTransaction()) {
        editedObj->getDocument()->commitTransaction();
    }
    Gui::Command::doCommand(Gui::Command::Gui, "Gui.ActiveDocument.resetEdit()");
    Gui::Command::updateActive();
    return true;
}

bool TaskThroughCurveMesh::reject()
{
    if (!widget->reject()) {
        return false;
    }

    if (editedObj && editedObj->getDocument() && editedObj->getDocument()->hasPendingTransaction()) {
        editedObj->getDocument()->abortTransaction();
    }
    Gui::Command::doCommand(Gui::Command::Gui, "Gui.ActiveDocument.resetEdit()");
    Gui::Command::updateActive();
    return true;
}
