// SPDX-License-Identifier: LGPL-2.1-or-later

#include "LazyExternalGeometryLayer.h"
#include "ViewProviderSketch.h"

#include <App/Application.h>
#include <App/Document.h>
#include <App/DocumentObject.h>
#include <Base/Tools.h>
#include <Gui/Application.h>
#include <Gui/Document.h>
#include <Gui/Selection/Selection.h>
#include <Gui/ViewProviderDocumentObject.h>
#include <Mod/Sketcher/App/SketchObject.h>

#include <Inventor/SoPickedPoint.h>
#include <Inventor/SoFullPath.h>

#include <QApplication>
#include <QTimer>

#include <algorithm>
#include <cmath>
#include <map>

namespace SketcherGui {

struct LazyExternalGeometryLayer::Context
{
    std::optional<Pick> hover;
    std::optional<Pick> press;
    SbVec2s lastCursor;
    bool hasLastCursor = false;
};

namespace {
std::map<ViewProviderSketch*, LazyExternalGeometryLayer::Context> contexts;

bool sameCursor(const LazyExternalGeometryLayer::Pick& pick, const SbVec2s& cursorPos)
{
    if (!pick.hasCursorPos) {
        return true;
    }

    short dx = 0;
    short dy = 0;
    (cursorPos - pick.cursorPos).getValue(dx, dy);
    const int tolerance = std::max(QApplication::startDragDistance(), 3);
    return std::abs(dx) <= tolerance && std::abs(dy) <= tolerance;
}

App::DocumentObject* objectFromNames(App::Document* fallbackDocument,
                                     const std::string& documentName,
                                     const std::string& objectName)
{
    if (objectName.empty()) {
        return nullptr;
    }

    App::Document* document = fallbackDocument;
    if (!documentName.empty()) {
        if (App::Document* namedDocument = App::GetApplication().getDocument(documentName.c_str())) {
            document = namedDocument;
        }
    }

    if (!document) {
        return nullptr;
    }

    if (App::DocumentObject* object = document->getObject(objectName.c_str())) {
        return object;
    }

    std::string normalizedObjectName(objectName);
    const std::string::size_type hashPos = normalizedObjectName.rfind('#');
    if (hashPos != std::string::npos && hashPos + 1 < normalizedObjectName.size()) {
        normalizedObjectName = normalizedObjectName.substr(hashPos + 1);
        return document->getObject(normalizedObjectName.c_str());
    }

    return nullptr;
}

} // namespace

LazyExternalGeometryLayer& LazyExternalGeometryLayer::instance()
{
    static LazyExternalGeometryLayer layer;
    return layer;
}

LazyExternalGeometryLayer::Context& LazyExternalGeometryLayer::context(ViewProviderSketch* sketchgui)
{
    return contexts[sketchgui];
}

const LazyExternalGeometryLayer::Context* LazyExternalGeometryLayer::contextIfPresent(
    ViewProviderSketch* sketchgui) const
{
    auto it = contexts.find(sketchgui);
    return it == contexts.end() ? nullptr : &it->second;
}

LazyExternalGeometryLayer::Kind LazyExternalGeometryLayer::classifySubName(const char* subName)
{
    if (Base::Tools::isNullOrEmpty(subName)) {
        return Kind::WholeObject;
    }

    const std::string element(subName);
    if (element.rfind("Edge", 0) == 0) {
        return Kind::Edge;
    }
    if (element.rfind("Vertex", 0) == 0) {
        return Kind::Vertex;
    }
    if (element.rfind("Face", 0) == 0) {
        return Kind::Face;
    }
    return Kind::Other;
}

bool LazyExternalGeometryLayer::isConstraintSubName(const char* subName)
{
    const Kind kind = classifySubName(subName);
    return kind == Kind::Edge || kind == Kind::Vertex;
}

void LazyExternalGeometryLayer::noteCursor(ViewProviderSketch* sketchgui, const SbVec2s& cursorPos)
{
    if (!sketchgui) {
        return;
    }
    Context& ctx = context(sketchgui);
    ctx.lastCursor = cursorPos;
    ctx.hasLastCursor = true;
}

bool LazyExternalGeometryLayer::pickFromSelectionChange(ViewProviderSketch* sketchgui,
                                                        const Gui::SelectionChanges& msg,
                                                        Pick& pick) const
{
    pick = Pick{};
    if (!sketchgui || !sketchgui->getSketchObject()
        || Base::Tools::isNullOrEmpty(msg.pObjectName)) {
        return false;
    }

    auto* sketchObject = sketchgui->getSketchObject();
    App::Document* sketchDocument = sketchObject->getDocument();
    if (!sketchDocument) {
        return false;
    }

    const std::string documentName = Base::Tools::isNullOrEmpty(msg.pDocName)
        ? std::string()
        : std::string(msg.pDocName);
    const std::string objectName(msg.pObjectName);

    App::DocumentObject* selectedObject = objectFromNames(sketchDocument, documentName, objectName);
    if (!selectedObject || selectedObject == sketchObject) {
        return false;
    }

    const Kind kind = classifySubName(msg.pSubName);
    const bool allowed = sketchObject->isExternalAllowed(selectedObject->getDocument(), selectedObject);
    if (!allowed && (kind == Kind::Edge || kind == Kind::Vertex)) {
        return false;
    }

    pick.kind = kind;
    pick.documentName = selectedObject->getDocument() ? selectedObject->getDocument()->getName() : documentName;
    pick.objectName = selectedObject->getNameInDocument();
    pick.subName = Base::Tools::isNullOrEmpty(msg.pSubName) ? std::string() : std::string(msg.pSubName);

    if (const Context* ctx = contextIfPresent(sketchgui); ctx && ctx->hasLastCursor) {
        pick.cursorPos = ctx->lastCursor;
        pick.hasCursorPos = true;
    }

    return pick.valid();
}

bool LazyExternalGeometryLayer::pickFromPickedPoint(ViewProviderSketch* sketchgui,
                                                    const SoPickedPoint* pickedPoint,
                                                    const SbVec2s& cursorPos,
                                                    Pick& pick) const
{
    pick = Pick{};
    if (!sketchgui || !pickedPoint || !sketchgui->getSketchObject()) {
        return false;
    }

    Gui::Document* guiDocument = sketchgui->getDocument();
    if (!guiDocument) {
        return false;
    }

    SoFullPath* path = static_cast<SoFullPath*>(pickedPoint->getPath());
    if (!path) {
        return false;
    }

    auto* viewProvider = guiDocument->getViewProviderByPathFromHead(path);
    auto* viewProviderObject = dynamic_cast<Gui::ViewProviderDocumentObject*>(viewProvider);
    if (!viewProviderObject || !viewProviderObject->getObject()) {
        return false;
    }

    auto* sketchObject = sketchgui->getSketchObject();
    App::DocumentObject* selectedObject = viewProviderObject->getObject();
    if (selectedObject == sketchObject) {
        return false;
    }

    std::string subName;
    if (!viewProviderObject->getElementPicked(pickedPoint, subName)) {
        subName.clear();
    }

    const Kind kind = classifySubName(subName.c_str());
    const bool allowed = sketchObject->isExternalAllowed(selectedObject->getDocument(), selectedObject);
    if (!allowed && (kind == Kind::Edge || kind == Kind::Vertex)) {
        return false;
    }

    pick.kind = kind;
    pick.documentName = selectedObject->getDocument() ? selectedObject->getDocument()->getName() : std::string();
    pick.objectName = selectedObject->getNameInDocument();
    pick.subName = subName;
    pick.cursorPos = cursorPos;
    pick.hasCursorPos = true;
    return pick.valid();
}

void LazyExternalGeometryLayer::recordHoverPick(ViewProviderSketch* sketchgui, const Pick& pick)
{
    if (!sketchgui || !pick.valid()) {
        return;
    }
    context(sketchgui).hover = pick;
}

std::optional<LazyExternalGeometryLayer::Pick> LazyExternalGeometryLayer::hoverPick(
    ViewProviderSketch* sketchgui) const
{
    const Context* ctx = contextIfPresent(sketchgui);
    return ctx ? ctx->hover : std::optional<Pick>{};
}

std::optional<LazyExternalGeometryLayer::Pick> LazyExternalGeometryLayer::consumeHoverPick(
    ViewProviderSketch* sketchgui)
{
    auto it = contexts.find(sketchgui);
    if (it == contexts.end() || !it->second.hover) {
        return std::nullopt;
    }
    std::optional<Pick> pick = it->second.hover;
    it->second.hover.reset();
    return pick;
}

std::optional<LazyExternalGeometryLayer::Pick> LazyExternalGeometryLayer::consumeHoverPickAt(
    ViewProviderSketch* sketchgui,
    const SbVec2s& cursorPos) const
{
    const Context* ctx = contextIfPresent(sketchgui);
    if (!ctx || !ctx->hover || !sameCursor(*ctx->hover, cursorPos)) {
        return std::nullopt;
    }
    return ctx->hover;
}

void LazyExternalGeometryLayer::clearHoverPick(ViewProviderSketch* sketchgui)
{
    auto it = contexts.find(sketchgui);
    if (it != contexts.end()) {
        it->second.hover.reset();
    }
}

void LazyExternalGeometryLayer::setPressPick(ViewProviderSketch* sketchgui, const Pick& pick)
{
    if (!sketchgui || !pick.valid()) {
        return;
    }
    context(sketchgui).press = pick;
}

std::optional<LazyExternalGeometryLayer::Pick> LazyExternalGeometryLayer::pressPick(
    ViewProviderSketch* sketchgui) const
{
    const Context* ctx = contextIfPresent(sketchgui);
    return ctx ? ctx->press : std::optional<Pick>{};
}

std::optional<LazyExternalGeometryLayer::Pick> LazyExternalGeometryLayer::consumePressPick(
    ViewProviderSketch* sketchgui)
{
    auto it = contexts.find(sketchgui);
    if (it == contexts.end() || !it->second.press) {
        return std::nullopt;
    }
    std::optional<Pick> pick = it->second.press;
    it->second.press.reset();
    return pick;
}

bool LazyExternalGeometryLayer::hasIgnoredSupportPress(ViewProviderSketch* sketchgui) const
{
    const Context* ctx = contextIfPresent(sketchgui);
    return ctx && ctx->press && ctx->press->ignoredSupport();
}

void LazyExternalGeometryLayer::clear(ViewProviderSketch* sketchgui)
{
    contexts.erase(sketchgui);
}

void LazyExternalGeometryLayer::scheduleRemoveSelection(const Pick& pick) const
{
    if (!pick.valid()) {
        return;
    }

    const std::string documentName = pick.documentName;
    const std::string objectName = pick.objectName;
    const std::string subName = pick.subName;

    QTimer::singleShot(0, [documentName, objectName, subName]() {
        if (documentName.empty() || objectName.empty()) {
            return;
        }
        Gui::Selection().rmvSelection(documentName.c_str(), objectName.c_str(), subName.c_str());
    });
}

} // namespace SketcherGui
