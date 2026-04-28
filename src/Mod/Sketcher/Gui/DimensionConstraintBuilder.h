// SPDX-License-Identifier: LGPL-2.1-or-later
#pragma once

#include "DimensionCandidate.h"


#include <functional>
#include <memory>
#include <optional>

namespace Sketcher {
class SketchObject;
class Constraint;
}


namespace SketcherGui {
class ViewProviderSketch;

/// Explicit driving/reference policy used instead of bare bool flags.
enum class DimensionDrivingPolicy
{
    Driving,
    Reference
};

/// Explicit selection clearing policy for post-create datum dialogs.
enum class DimensionDialogSelectionPolicy
{
    KeepSelection,
    ClearBeforeDialog
};

struct DimensionCommitOptions
{
    DimensionDialogSelectionPolicy selectionPolicy {DimensionDialogSelectionPolicy::KeepSelection};
    std::optional<float> forcedLabelPosition;
};

void finalizeCreatedDimension(Sketcher::SketchObject& sketch,
                              int constraintIndex,
                              DimensionDrivingPolicy drivingPolicy,
                              const std::function<void(int)>& moveConstraintFn);

DimensionCommitOptions buildDimensionCommitOptions(const Sketcher::SketchObject& sketch,
                                                   const DimensionCandidate& candidate,
                                                   DimensionDialogSelectionPolicy selectionPolicy =
                                                       DimensionDialogSelectionPolicy::KeepSelection);

int appendDimensionFromCandidate(Sketcher::SketchObject& sketch,
                                 const DimensionCandidate& candidate,
                                 DimensionDrivingPolicy drivingPolicy,
                                 const std::function<void(int)>& moveConstraintFn);

bool commitDimensionCandidate(ViewProviderSketch& viewProvider,
                              Sketcher::SketchObject& sketch,
                              const DimensionCandidate& candidate);

/** Build the shared Sketcher constraint shape from a dimension candidate.
 *  Preview, dedupe and both dimension tools use this helper before diverging into
 *  tool-specific UI flow.
 */
std::unique_ptr<Sketcher::Constraint> buildDimensionConstraint(
    const Sketcher::SketchObject& sketch,
    const DimensionCandidate& candidate);

} // namespace SketcherGui
