// SPDX-License-Identifier: LGPL-2.1-or-later
#include "DimensionSelectionEngineDetails.h"

#include "DimensionConstraintBuilder.h"
#include "DimensionGeometry.h"

#include <Mod/Sketcher/App/Constraint.h>
#include <Mod/Sketcher/App/SketchObject.h>

#include <algorithm>
#include <cmath>
#include <utility>

namespace SketcherGui::DimensionSelectionEngineFilterDetail {

bool isSameRef(const DimensionReference& a, const DimensionReference& b)
{
    return a.geoId == b.geoId && a.posId == b.posId;
}

bool isSameRefPairUnordered(const DimensionReference& firstA,
                            const DimensionReference& secondA,
                            const DimensionReference& firstB,
                            const DimensionReference& secondB)
{
    return (isSameRef(firstA, firstB) && isSameRef(secondA, secondB))
        || (isSameRef(firstA, secondB) && isSameRef(secondA, firstB));
}

DimensionReference firstRef(const Sketcher::Constraint& constraint)
{
    return DimensionReference {constraint.First, constraint.FirstPos};
}

DimensionReference secondRef(const Sketcher::Constraint& constraint)
{
    return DimensionReference {constraint.Second, constraint.SecondPos};
}

DimensionReference thirdRef(const Sketcher::Constraint& constraint)
{
    return DimensionReference {constraint.Third, constraint.ThirdPos};
}

bool sameConstraintIdentity(DimensionSemantic semantic,
                            const Sketcher::Constraint& lhs,
                            const Sketcher::Constraint& rhs)
{
    if (lhs.Type != rhs.Type) {
        return false;
    }

    switch (semantic) {
        case DimensionSemantic::Radius:
        case DimensionSemantic::Diameter:
        case DimensionSemantic::ArcLength:
            return isSameRef(firstRef(lhs), firstRef(rhs));
        case DimensionSemantic::ProjectedX:
        case DimensionSemantic::ProjectedY:
        case DimensionSemantic::DirectLength:
        case DimensionSemantic::DirectDistance:
            return isSameRefPairUnordered(firstRef(lhs), secondRef(lhs), firstRef(rhs), secondRef(rhs));
        case DimensionSemantic::Angle:
            if (lhs.Third != Sketcher::GeoEnum::GeoUndef || rhs.Third != Sketcher::GeoEnum::GeoUndef) {
                return isSameRefPairUnordered(firstRef(lhs), secondRef(lhs), firstRef(rhs), secondRef(rhs))
                    && isSameRef(thirdRef(lhs), thirdRef(rhs));
            }
            return isSameRefPairUnordered(firstRef(lhs), secondRef(lhs), firstRef(rhs), secondRef(rhs));
        case DimensionSemantic::Unknown:
        default:
            return false;
    }
}

bool isEquivalentExistingConstraint(const Sketcher::SketchObject& sketch,
                                    const DimensionCandidate& candidate)
{
    auto prepared = buildDimensionConstraint(sketch, candidate);
    if (!prepared) {
        return false;
    }

    const auto& constraints = sketch.Constraints.getValues();
    for (const auto* existing : constraints) {
        if (!existing || !existing->isDriving) {
            continue;
        }

        if (std::abs(existing->getValue() - prepared->getValue()) > 1e-7) {
            continue;
        }

        if (!sameConstraintIdentity(candidate.semantic, *existing, *prepared)) {
            continue;
        }

        return true;
    }

    return false;
}

void filterEquivalentExistingCandidates(const Sketcher::SketchObject& sketch,
                                        std::vector<DimensionCandidate>& candidates)
{
    std::erase_if(candidates, [&](const DimensionCandidate& candidate) {
        return isEquivalentExistingConstraint(sketch, candidate);
    });
}

bool areEquivalentPreviewCandidates(const Sketcher::SketchObject& sketch,
                                    const DimensionCandidate& lhs,
                                    const DimensionCandidate& rhs)
{
    if (lhs.semantic != rhs.semantic) {
        return false;
    }

    auto preparedLhs = buildDimensionConstraint(sketch, lhs);
    auto preparedRhs = buildDimensionConstraint(sketch, rhs);
    if (!preparedLhs || !preparedRhs) {
        return false;
    }

    if (std::abs(preparedLhs->getValue() - preparedRhs->getValue()) > 1e-7) {
        return false;
    }

    return sameConstraintIdentity(lhs.semantic, *preparedLhs, *preparedRhs);
}

void filterDuplicatePreviewCandidates(const Sketcher::SketchObject& sketch,
                                      std::vector<DimensionCandidate>& candidates)
{
    std::vector<DimensionCandidate> unique;
    unique.reserve(candidates.size());

    for (auto& candidate : candidates) {
        const bool duplicate = std::any_of(unique.begin(), unique.end(), [&](const auto& existing) {
            return areEquivalentPreviewCandidates(sketch, existing, candidate);
        });
        if (!duplicate) {
            unique.push_back(std::move(candidate));
        }
    }

    candidates = std::move(unique);
}

} // namespace SketcherGui::DimensionSelectionEngineFilterDetail

namespace SketcherGui::DimensionSelectionEngineDetail {

using namespace SketcherGui::DimensionSelectionEngineFilterDetail;

PreviewSelectionCountKind classifyPreviewSelectionCount(std::size_t selectionCount)
{
    switch (selectionCount) {
        case 0u:
            return PreviewSelectionCountKind::Empty;
        case 1u:
            return PreviewSelectionCountKind::Single;
        case 2u:
            return PreviewSelectionCountKind::Pair;
        default:
            return PreviewSelectionCountKind::Unsupported;
    }
}

void filterPreviewCandidates(const Sketcher::SketchObject& sketch,
                             std::vector<DimensionCandidate>& candidates)
{
    std::erase_if(candidates, [](const DimensionCandidate& candidate) {
        return !isValidDimensionCandidate(candidate);
    });

    if (candidates.empty()) {
        return;
    }

    filterDuplicatePreviewCandidates(sketch, candidates);
    filterEquivalentExistingCandidates(sketch, candidates);
}

} // namespace SketcherGui::DimensionSelectionEngineDetail
