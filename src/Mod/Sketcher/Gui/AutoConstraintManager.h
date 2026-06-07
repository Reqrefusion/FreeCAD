// SPDX-License-Identifier: LGPL-2.1-or-later

#pragma once

#include <Mod/Sketcher/App/Constraint.h>

namespace SketcherGui
{

class AutoConstraintManager
{
public:
    enum class Mode
    {
        Coincident,
        PointOnObject,
        Horizontal,
        Vertical,
        Parallel,
        Perpendicular,
        Tangent,
        Symmetric,
    };

    static bool isModeActive(Mode mode);
    static void setModeActive(Mode mode, bool active);
    static void toggleMode(Mode mode);

    static bool isConstraintActive(Sketcher::ConstraintType type);

    static const char* modeName(Mode mode);
    static const char* commandIconName(Mode mode);
    static Sketcher::ConstraintType constraintType(Mode mode);
};

}  // namespace SketcherGui
