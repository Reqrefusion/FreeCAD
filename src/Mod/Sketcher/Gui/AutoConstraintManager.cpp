// SPDX-License-Identifier: LGPL-2.1-or-later

#include <array>
#include <string>

#include <App/Application.h>

#include "AutoConstraintManager.h"

using namespace SketcherGui;

namespace
{
constexpr std::array<AutoConstraintManager::Mode, 8> Modes = {
    AutoConstraintManager::Mode::Coincident,
    AutoConstraintManager::Mode::PointOnObject,
    AutoConstraintManager::Mode::Horizontal,
    AutoConstraintManager::Mode::Vertical,
    AutoConstraintManager::Mode::Parallel,
    AutoConstraintManager::Mode::Perpendicular,
    AutoConstraintManager::Mode::Tangent,
    AutoConstraintManager::Mode::Symmetric,
};

constexpr const char* ParameterName = "autoConstraintModes";
constexpr const char* DefaultModes = "11111111";

ParameterGrp::handle getParameterPath()
{
    return App::GetApplication().GetParameterGroupByPath(
        "User parameter:BaseApp/Preferences/Mod/Sketcher/AutoConstraints"
    );
}

int modeIndex(AutoConstraintManager::Mode mode)
{
    for (std::size_t i = 0; i < Modes.size(); ++i) {
        if (Modes[i] == mode) {
            return static_cast<int>(i);
        }
    }

    return -1;
}

std::string normalizedModeString()
{
    std::string modes = getParameterPath()->GetASCII(ParameterName, DefaultModes);

    if (modes.size() < Modes.size()) {
        modes.resize(Modes.size(), '1');
    }
    else if (modes.size() > Modes.size()) {
        modes.resize(Modes.size());
    }

    for (char& c : modes) {
        if (c != '0' && c != '1') {
            c = '1';
        }
    }

    return modes;
}

void saveModeString(const std::string& modes)
{
    getParameterPath()->SetASCII(ParameterName, modes.c_str());
}

bool modeForConstraintType(Sketcher::ConstraintType type, AutoConstraintManager::Mode& mode)
{
    switch (type) {
        case Sketcher::Coincident:
            mode = AutoConstraintManager::Mode::Coincident;
            return true;
        case Sketcher::PointOnObject:
            mode = AutoConstraintManager::Mode::PointOnObject;
            return true;
        case Sketcher::Horizontal:
            mode = AutoConstraintManager::Mode::Horizontal;
            return true;
        case Sketcher::Vertical:
            mode = AutoConstraintManager::Mode::Vertical;
            return true;
        case Sketcher::Parallel:
            mode = AutoConstraintManager::Mode::Parallel;
            return true;
        case Sketcher::Perpendicular:
            mode = AutoConstraintManager::Mode::Perpendicular;
            return true;
        case Sketcher::Tangent:
            mode = AutoConstraintManager::Mode::Tangent;
            return true;
        case Sketcher::Symmetric:
            mode = AutoConstraintManager::Mode::Symmetric;
            return true;
        default:
            return false;
    }
}
}  // namespace

bool AutoConstraintManager::isModeActive(Mode mode)
{
    const int index = modeIndex(mode);
    if (index < 0) {
        return true;
    }

    return normalizedModeString()[static_cast<std::size_t>(index)] == '1';
}

void AutoConstraintManager::setModeActive(Mode mode, bool active)
{
    const int index = modeIndex(mode);
    if (index < 0) {
        return;
    }

    std::string modes = normalizedModeString();
    modes[static_cast<std::size_t>(index)] = active ? '1' : '0';
    saveModeString(modes);
}

void AutoConstraintManager::toggleMode(Mode mode)
{
    setModeActive(mode, !isModeActive(mode));
}

bool AutoConstraintManager::isConstraintActive(Sketcher::ConstraintType type)
{
    Mode mode;
    if (!modeForConstraintType(type, mode)) {
        return true;
    }

    return isModeActive(mode);
}

const char* AutoConstraintManager::modeName(Mode mode)
{
    switch (mode) {
        case Mode::Coincident:
            return "Coincident";
        case Mode::PointOnObject:
            return "PointOnObject";
        case Mode::Horizontal:
            return "Horizontal";
        case Mode::Vertical:
            return "Vertical";
        case Mode::Parallel:
            return "Parallel";
        case Mode::Perpendicular:
            return "Perpendicular";
        case Mode::Tangent:
            return "Tangent";
        case Mode::Symmetric:
            return "Symmetric";
    }

    return "";
}

const char* AutoConstraintManager::commandIconName(Mode mode)
{
    switch (mode) {
        case Mode::Coincident:
            return "Constraint_PointOnPoint";
        case Mode::PointOnObject:
            return "Constraint_PointOnObject";
        case Mode::Horizontal:
            return "Constraint_Horizontal";
        case Mode::Vertical:
            return "Constraint_Vertical";
        case Mode::Parallel:
            return "Constraint_Parallel";
        case Mode::Perpendicular:
            return "Constraint_Perpendicular";
        case Mode::Tangent:
            return "Constraint_Tangent";
        case Mode::Symmetric:
            return "Constraint_Symmetric";
    }

    return "";
}

Sketcher::ConstraintType AutoConstraintManager::constraintType(Mode mode)
{
    switch (mode) {
        case Mode::Coincident:
            return Sketcher::Coincident;
        case Mode::PointOnObject:
            return Sketcher::PointOnObject;
        case Mode::Horizontal:
            return Sketcher::Horizontal;
        case Mode::Vertical:
            return Sketcher::Vertical;
        case Mode::Parallel:
            return Sketcher::Parallel;
        case Mode::Perpendicular:
            return Sketcher::Perpendicular;
        case Mode::Tangent:
            return Sketcher::Tangent;
        case Mode::Symmetric:
            return Sketcher::Symmetric;
    }

    return Sketcher::None;
}
