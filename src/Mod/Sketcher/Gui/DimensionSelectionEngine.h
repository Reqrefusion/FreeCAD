// SPDX-License-Identifier: LGPL-2.1-or-later
#pragma once

#include "DimensionCandidate.h"

#include <vector>

namespace Sketcher {
class SketchObject;
}

namespace SketcherGui {

/** Selection-to-candidate adapter entry point.
 *
 *  The implementation stays in the .cpp file; the header only exposes the single operation that
 *  the rest of Sketcher needs: build relax-dimension candidates from the current selection.
 *  Preview stays intentionally strict: only supported one- and two-selection groups can produce
 *  candidates; three or more active selections suppress preview completely.
 */
[[nodiscard]] std::vector<DimensionCandidate> buildDimensionCandidates(
    Sketcher::SketchObject* sketch,
    const std::vector<DimensionReference>& selectionRefs);

} // namespace SketcherGui
