// SPDX-License-Identifier: LGPL-2.1-or-later
#include "DimensionCandidate.h"

#include <algorithm>
#include <cmath>

namespace SketcherGui {

namespace DimensionCandidateDetail {

[[nodiscard]] bool hasFinitePreviewValue(const DimensionCandidate& candidate)
{
    return std::isfinite(candidate.previewValue);
}

[[nodiscard]] bool hasDefinedReferences(const DimensionCandidate& candidate)
{
    return std::all_of(candidate.refs.begin(), candidate.refs.end(), [](const DimensionReference& ref) {
        return ref.geoId != Sketcher::GeoEnum::GeoUndef;
    });
}

} // namespace DimensionCandidateDetail

bool hasExpectedDimensionReferenceCount(const DimensionCandidate& candidate)
{
    switch (candidate.semantic) {
        case DimensionSemantic::ProjectedX:
        case DimensionSemantic::ProjectedY:
        case DimensionSemantic::DirectLength:
        case DimensionSemantic::DirectDistance:
            return !candidate.refs.empty() && candidate.refs.size() <= 2;
        case DimensionSemantic::Radius:
        case DimensionSemantic::Diameter:
        case DimensionSemantic::ArcLength:
            return candidate.refs.size() == 1;
        case DimensionSemantic::Angle:
            return !candidate.refs.empty() && candidate.refs.size() <= 3;
        case DimensionSemantic::Unknown:
        default:
            return false;
    }
}

bool isValidDimensionCandidate(const DimensionCandidate& candidate)
{
    using namespace DimensionCandidateDetail;
    if (candidate.semantic == DimensionSemantic::Unknown) {
        return false;
    }

    if (!hasFinitePreviewValue(candidate) || !hasDefinedReferences(candidate)) {
        return false;
    }

    return hasExpectedDimensionReferenceCount(candidate);
}

} // namespace SketcherGui
