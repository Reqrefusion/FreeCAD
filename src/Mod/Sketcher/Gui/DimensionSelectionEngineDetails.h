// SPDX-License-Identifier: LGPL-2.1-or-later
#pragma once

#include "DimensionSelectionEngine.h"

#include "DimensionSelectionPattern.h"
#include "DimensionResolver.h"

#include <Base/Tools2D.h>

#include <cstddef>

#include <optional>
#include <vector>

namespace Part {
class GeomArcOfCircle;
}

namespace Sketcher {
class SketchObject;
class Constraint;
}

namespace SketcherGui::DimensionSelectionEngineDetail {

struct AngleRayChoice
{
    bool valid {false};
    int firstGeoId {Sketcher::GeoEnum::GeoUndef};
    int secondGeoId {Sketcher::GeoEnum::GeoUndef};
    Sketcher::PointPos firstPos {Sketcher::PointPos::none};
    Sketcher::PointPos secondPos {Sketcher::PointPos::none};
    Base::Vector2d vertex;
    Base::Vector2d dir1;
    Base::Vector2d dir2;
    double firstLength {0.0};
    double secondLength {0.0};
    double angleValue {0.0};
};

enum class PreviewSelectionCountKind {
    Empty,
    Single,
    Pair,
    Unsupported
};

[[nodiscard]] PreviewSelectionCountKind classifyPreviewSelectionCount(std::size_t selectionCount);

void filterPreviewCandidates(const Sketcher::SketchObject& sketch,
                             std::vector<DimensionCandidate>& candidates);

[[nodiscard]] Base::Vector2d makeStableLinearLabelPos(const Base::Vector2d& a,
                                                      const Base::Vector2d& b,
                                                      DimensionSemantic semantic);
[[nodiscard]] bool shouldCreateProjectedX(const Base::Vector2d& a, const Base::Vector2d& b);
[[nodiscard]] bool shouldCreateProjectedY(const Base::Vector2d& a, const Base::Vector2d& b);
[[nodiscard]] double computeLinearPreviewValue(const Base::Vector2d& a,
                                               const Base::Vector2d& b,
                                               DimensionSemantic semantic);
[[nodiscard]] double computeRoundPreviewValue(double radius, DimensionSemantic semantic);
[[nodiscard]] bool isDegenerateAngle(double angleValue);
[[nodiscard]] bool isLineLikeSelection(const Sketcher::SketchObject& sketch,
                                       const DimensionReference& ref);
[[nodiscard]] Base::Vector2d makeStableAngularLabelPos(const Base::Vector2d& vertex,
                                                       const Base::Vector2d& dir1,
                                                       const Base::Vector2d& dir2,
                                                       double firstLength,
                                                       double secondLength);
[[nodiscard]] bool buildAngleRayChoice(const Sketcher::SketchObject& sketch,
                                       int firstGeoId,
                                       Sketcher::PointPos firstPos,
                                       int secondGeoId,
                                       Sketcher::PointPos secondPos,
                                       AngleRayChoice& choice);
void appendResolvedAngleCandidates(std::vector<DimensionCandidate>& result,
                                   const Sketcher::SketchObject& sketch,
                                   const TwoLineDimensionResolution& resolved);
[[nodiscard]] DimensionCandidate makeLinearCandidate(DimensionSemantic semantic,
                                                     std::vector<DimensionReference> refs,
                                                     const Base::Vector2d& a,
                                                     const Base::Vector2d& b,
                                                     double previewValue);
[[nodiscard]] std::optional<DimensionCandidate> buildDirectDistanceCandidate(
    const Sketcher::SketchObject& sketch,
    std::vector<DimensionReference> refs);
[[nodiscard]] std::optional<DimensionCandidate> buildPrimaryDistanceCandidate(
    Sketcher::SketchObject* sketch,
    BasicDimensionSelectionPattern pattern,
    const DimensionReference& first,
    const DimensionReference* second = nullptr);
void appendLinearCandidates(std::vector<DimensionCandidate>& result,
                            const std::vector<DimensionSemantic>& order,
                            const Base::Vector2d& a,
                            const Base::Vector2d& b,
                            const std::vector<DimensionReference>& refs,
                            std::optional<DimensionCandidate> directCandidate = std::nullopt);
[[nodiscard]] Base::Vector2d makeStableArcLengthLabelPos(const Part::GeomArcOfCircle& arc);

} // namespace SketcherGui::DimensionSelectionEngineDetail
