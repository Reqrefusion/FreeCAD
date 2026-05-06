// SPDX-License-Identifier: LGPL-2.1-or-later

#ifndef SKETCHERGUI_LAZYEXTERNALGEOMETRYLAYER_H
#define SKETCHERGUI_LAZYEXTERNALGEOMETRYLAYER_H

#include <Inventor/SbVec2s.h>

#include <optional>
#include <string>

class SoPickedPoint;

namespace App {
class Document;
class DocumentObject;
}

namespace Gui {
class SelectionChanges;
}

namespace SketcherGui {
class ViewProviderSketch;

/**
 * A sketch-local adapter for lazy external geometry picks.
 *
 * Important invariant:
 *   Source-object Edge/Vertex/Face hits are never kept as durable Gui::Selection
 *   state.  Gui::Selection is only an input notification channel.  This layer owns
 *   the transient pick that constraints may later materialize with addExternal().
 */
class LazyExternalGeometryLayer
{
public:
    enum class Kind
    {
        None,
        Edge,
        Vertex,
        Face,
        WholeObject,
        Other
    };

    struct Pick
    {
        Kind kind = Kind::None;
        std::string documentName;
        std::string objectName;
        std::string subName;
        SbVec2s cursorPos;
        bool hasCursorPos = false;

        bool valid() const { return kind != Kind::None && !objectName.empty(); }
        bool usableForConstraint() const { return kind == Kind::Edge || kind == Kind::Vertex; }
        bool ignoredSupport() const
        {
            return kind == Kind::Face || kind == Kind::WholeObject || kind == Kind::Other;
        }
    };

    static LazyExternalGeometryLayer& instance();

    static Kind classifySubName(const char* subName);
    static bool isConstraintSubName(const char* subName);

    void noteCursor(ViewProviderSketch* sketchgui, const SbVec2s& cursorPos);

    bool pickFromSelectionChange(ViewProviderSketch* sketchgui,
                                 const Gui::SelectionChanges& msg,
                                 Pick& pick) const;
    bool pickFromPickedPoint(ViewProviderSketch* sketchgui,
                             const SoPickedPoint* pickedPoint,
                             const SbVec2s& cursorPos,
                             Pick& pick) const;

    void recordHoverPick(ViewProviderSketch* sketchgui, const Pick& pick);
    std::optional<Pick> hoverPick(ViewProviderSketch* sketchgui) const;
    std::optional<Pick> consumeHoverPick(ViewProviderSketch* sketchgui);
    std::optional<Pick> consumeHoverPickAt(ViewProviderSketch* sketchgui,
                                           const SbVec2s& cursorPos) const;
    void clearHoverPick(ViewProviderSketch* sketchgui);

    void setPressPick(ViewProviderSketch* sketchgui, const Pick& pick);
    std::optional<Pick> pressPick(ViewProviderSketch* sketchgui) const;
    std::optional<Pick> consumePressPick(ViewProviderSketch* sketchgui);
    bool hasIgnoredSupportPress(ViewProviderSketch* sketchgui) const;

    void clear(ViewProviderSketch* sketchgui);

    void scheduleRemoveSelection(const Pick& pick) const;

    struct Context;

private:
    Context& context(ViewProviderSketch* sketchgui);
    const Context* contextIfPresent(ViewProviderSketch* sketchgui) const;
};

} // namespace SketcherGui

#endif // SKETCHERGUI_LAZYEXTERNALGEOMETRYLAYER_H
