// SPDX-License-Identifier: LGPL-2.1-or-later
#pragma once

#include "Utils.h"

#include <Mod/Sketcher/App/GeoEnum.h>

#include <vector>

namespace SketcherGui {

/// Shared semantic meaning used by both native dimension resolution and smart previews.
enum class DimensionSemantic
{
    Unknown,
    DirectLength,
    DirectDistance,
    ProjectedX,
    ProjectedY,
    Radius,
    Diameter,
    Angle,
    ArcLength,
};

[[nodiscard]] inline bool isLinearDimensionSemantic(DimensionSemantic semantic)
{
    switch (semantic) {
        case DimensionSemantic::ProjectedX:
        case DimensionSemantic::ProjectedY:
        case DimensionSemantic::DirectLength:
        case DimensionSemantic::DirectDistance:
            return true;
        default:
            return false;
    }
}

/// Shared geometry/point reference used by dimension resolvers and preview builders.
struct DimensionReference
{
    int geoId {-1};
    Sketcher::PointPos posId {Sketcher::PointPos::none};
};

[[nodiscard]] inline bool isPointReference(const DimensionReference& ref)
{
    return ref.posId != Sketcher::PointPos::none;
}

/// Shared resolved dimension candidate. Smart previews render all options; the native
/// dimension tool selects one of them based on the current mode slot.
struct DimensionCandidate
{
    DimensionSemantic semantic {DimensionSemantic::Unknown};
    double previewValue {0.0};

    std::vector<DimensionReference> refs;

    /// Preferred preview label position in sketch coordinates.
    Base::Vector2d labelPos;
};

/// Returns whether the semantic/ref-count contract is satisfied for this candidate.
[[nodiscard]] bool hasExpectedDimensionReferenceCount(const DimensionCandidate& candidate);

/// Lightweight public invariant check used before preview dedupe/build/commit.
[[nodiscard]] bool isValidDimensionCandidate(const DimensionCandidate& candidate);

} // namespace SketcherGui
