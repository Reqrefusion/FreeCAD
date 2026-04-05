// SPDX-License-Identifier: LGPL-2.1-or-later
#pragma once

#include "DimensionSelectionPattern.h"
#include "DimensionCandidate.h"

#include <Mod/Sketcher/App/Constraint.h>

#include <array>
#include <optional>
#include <vector>

namespace Sketcher {
class SketchObject;
}

namespace SketcherGui {

struct TwoLineDimensionResolution
{
    DimensionSemantic semantic {DimensionSemantic::Angle};
    int firstGeoId {Sketcher::GeoEnum::GeoUndef};
    int secondGeoId {Sketcher::GeoEnum::GeoUndef};
    Sketcher::PointPos firstPos {Sketcher::PointPos::none};
    Sketcher::PointPos secondPos {Sketcher::PointPos::none};
    double angleValue {0.0};
};

std::optional<TwoLineDimensionResolution> resolveTwoLineDimension(
    Sketcher::SketchObject* sketch,
    int firstGeoId,
    int secondGeoId);

std::vector<DimensionSemantic> buildRoundDimensionSemantics(
    Sketcher::SketchObject* sketch,
    int geoId);

std::vector<DimensionSemantic> buildLinearDimensionSemantics(BasicDimensionSelectionPattern pattern);

[[nodiscard]] bool hasLinearBlockOption(BasicDimensionSelectionPattern pattern);

using DimensionRefPair = std::array<DimensionReference, 2>;

std::optional<DimensionRefPair> resolvePrimaryDistanceRefs(
    Sketcher::SketchObject* sketch,
    BasicDimensionSelectionPattern pattern,
    const DimensionReference& first,
    const DimensionReference* second = nullptr);

} // namespace SketcherGui
