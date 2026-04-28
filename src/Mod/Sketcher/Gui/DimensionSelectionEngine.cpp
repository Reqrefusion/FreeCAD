// SPDX-License-Identifier: LGPL-2.1-or-later
#include "DimensionSelectionEngine.h"

#include "DimensionSelectionEngineDetails.h"

#include "DimensionGeometry.h"

#include <Mod/Part/App/Geometry.h>
#include <Mod/Sketcher/App/SketchObject.h>

#include <cmath>
#include <utility>

namespace SketcherGui::DimensionSelectionEngineDetail {

using namespace DimensionGeometry;

std::vector<DimensionCandidate> buildLineCandidates(Sketcher::SketchObject* sketch, int geoId)
{
    std::vector<DimensionCandidate> result;

    const DimensionReference item {geoId, Sketcher::PointPos::none};
    const Part::Geometry* geometry = sketch->getGeometry(geoId);
    const auto* line = (geometry && geometry->is<Part::GeomLineSegment>())
        ? static_cast<const Part::GeomLineSegment*>(geometry)
        : nullptr;
    if (!line) {
        return result;
    }

    const Base::Vector2d a = toVector2d(line->getStartPoint());
    const Base::Vector2d b = toVector2d(line->getEndPoint());
    const std::vector<DimensionReference> projectedRefs {{geoId, Sketcher::PointPos::start},
                                                         {geoId, Sketcher::PointPos::end}};
    appendLinearCandidates(result,
                           buildLinearDimensionSemantics(BasicDimensionSelectionPattern::OneLine),
                           a,
                           b,
                           projectedRefs,
                           buildPrimaryDistanceCandidate(sketch,
                                                         BasicDimensionSelectionPattern::OneLine,
                                                         item));
    return result;
}

std::vector<DimensionCandidate> buildRoundCandidates(Sketcher::SketchObject* sketch, int geoId)
{
    std::vector<DimensionCandidate> result;

    const DimensionReference item {geoId, Sketcher::PointPos::none};
    const Part::Geometry* geometry = sketch->getGeometry(geoId);
    Base::Vector2d center;
    double radius = 0.0;
    if (!geometry || !roundRadiusCenter(geometry, center, radius) || radius <= kEpsilon) {
        return result;
    }

    const auto semantics = buildRoundDimensionSemantics(sketch, geoId);
    for (const auto semantic : semantics) {
        DimensionCandidate candidate;
        candidate.refs = {item};

        switch (semantic) {
            case DimensionSemantic::Radius:
            case DimensionSemantic::Diameter:
                candidate.semantic = semantic;
                candidate.labelPos = makeStableRoundLabelPos(geometry, candidate.semantic);
                candidate.previewValue = computeRoundPreviewValue(radius, candidate.semantic);
                result.push_back(candidate);
                break;
            case DimensionSemantic::ArcLength:
                if (const auto* arc = geometry->is<Part::GeomArcOfCircle>()
                        ? static_cast<const Part::GeomArcOfCircle*>(geometry)
                        : nullptr) {
                    candidate.semantic = DimensionSemantic::ArcLength;
                    candidate.labelPos = makeStableArcLengthLabelPos(*arc);
                    candidate.previewValue = arc->getAngle(false) * arc->getRadius();
                    result.push_back(candidate);
                }
                break;
            case DimensionSemantic::Angle:
                if (const auto* arc = geometry->is<Part::GeomArcOfCircle>()
                        ? static_cast<const Part::GeomArcOfCircle*>(geometry)
                        : nullptr) {
                    candidate.semantic = DimensionSemantic::Angle;
                    candidate.labelPos = makeStableRoundLabelPos(geometry, candidate.semantic);
                    candidate.previewValue = arc->getAngle(/*EmulateCCWXY=*/true);
                    result.push_back(candidate);
                }
                break;
            case DimensionSemantic::Unknown:
            case DimensionSemantic::DirectLength:
            case DimensionSemantic::DirectDistance:
            case DimensionSemantic::ProjectedX:
            case DimensionSemantic::ProjectedY:
                break;
        }
    }

    return result;
}

std::vector<DimensionCandidate> buildTwoLineCandidates(Sketcher::SketchObject* sketch,
                                                       int firstGeoId,
                                                       int secondGeoId)
{
    std::vector<DimensionCandidate> result;

    const auto resolved = resolveTwoLineDimension(sketch, firstGeoId, secondGeoId);
    if (!resolved) {
        return result;
    }

    if (resolved->semantic == DimensionSemantic::DirectDistance) {
        const DimensionReference first {firstGeoId, Sketcher::PointPos::none};
        const DimensionReference second {secondGeoId, Sketcher::PointPos::none};
        if (auto candidate = buildPrimaryDistanceCandidate(
                sketch,
                BasicDimensionSelectionPattern::TwoLines,
                first,
                &second)) {
            result.push_back(std::move(*candidate));
        }
        return result;
    }

    appendResolvedAngleCandidates(result, *sketch, *resolved);
    return result;
}

std::vector<DimensionCandidate> buildTwoPointCandidates(Sketcher::SketchObject* sketch,
                                                        const DimensionReference& firstRef,
                                                        const DimensionReference& secondRef)
{
    std::vector<DimensionCandidate> result;

    const Base::Vector2d a = sketchPoint(*sketch, firstRef);
    const Base::Vector2d b = sketchPoint(*sketch, secondRef);
    if (length(Base::Vector2d(b.x - a.x, b.y - a.y)) < kEpsilon) {
        return result;
    }

    const DimensionReference firstItem = firstRef;
    const DimensionReference secondItem = secondRef;
    appendLinearCandidates(result,
                           buildLinearDimensionSemantics(BasicDimensionSelectionPattern::TwoPoints),
                           a,
                           b,
                           {firstRef, secondRef},
                           buildPrimaryDistanceCandidate(sketch,
                                                         BasicDimensionSelectionPattern::TwoPoints,
                                                         firstItem,
                                                         &secondItem));

    return result;
}

std::vector<DimensionCandidate> buildSinglePointCandidates(
    Sketcher::SketchObject* sketch,
    const DimensionReference& pointRef)
{
    std::vector<DimensionCandidate> result;

    const DimensionReference origin {Sketcher::GeoEnum::RtPnt, Sketcher::PointPos::start};
    const Base::Vector2d a = sketchPoint(*sketch, origin);
    const Base::Vector2d b = sketchPoint(*sketch, pointRef);

    appendLinearCandidates(result,
                           buildLinearDimensionSemantics(BasicDimensionSelectionPattern::OnePoint),
                           a,
                           b,
                           {origin, pointRef});

    return result;
}

std::vector<DimensionCandidate> buildPointAxisCandidates(
    Sketcher::SketchObject* sketch,
    const DimensionReference& pointRef,
    int axisGeoId)
{
    std::vector<DimensionCandidate> result;
    const DimensionReference axisRef {axisGeoId, Sketcher::PointPos::start};
    const Base::Vector2d axisPoint = sketchPoint(*sketch, axisRef);
    const Base::Vector2d point = sketchPoint(*sketch, pointRef);

    if (axisGeoId == Sketcher::GeoEnum::HAxis && std::abs(point.y - axisPoint.y) > kEpsilon) {
        result.push_back(makeLinearCandidate(DimensionSemantic::ProjectedY,
                                             {axisRef, pointRef},
                                             axisPoint,
                                             point,
                                             std::abs(point.y - axisPoint.y)));
    }
    else if (axisGeoId == Sketcher::GeoEnum::VAxis && std::abs(point.x - axisPoint.x) > kEpsilon) {
        result.push_back(makeLinearCandidate(DimensionSemantic::ProjectedX,
                                             {axisRef, pointRef},
                                             axisPoint,
                                             point,
                                             std::abs(point.x - axisPoint.x)));
    }

    return result;
}

std::vector<DimensionCandidate> buildSingleSelectionCandidates(
    Sketcher::SketchObject* sketch,
    const DimensionReference& item)
{
    if (isPointReference(item)) {
        return buildSinglePointCandidates(sketch, item);
    }

    if (isAxisGeoId(item.geoId)) {
        return {};
    }

    const Part::Geometry* geometry = sketch->getGeometry(item.geoId);
    if (!geometry) {
        return {};
    }
    if (isLineGeometry(geometry)) {
        return buildLineCandidates(sketch, item.geoId);
    }
    if (isRoundGeometry(geometry)) {
        return buildRoundCandidates(sketch, item.geoId);
    }
    return {};
}

std::vector<DimensionCandidate> buildPointGeometryCandidates(
    Sketcher::SketchObject* sketch,
    const DimensionReference& pointRef,
    const DimensionReference& geometryRef)
{
    if (isAxisGeoId(geometryRef.geoId)) {
        return buildPointAxisCandidates(sketch, pointRef, geometryRef.geoId);
    }

    const Part::Geometry* geometry = sketch->getGeometry(geometryRef.geoId);
    const auto pattern = geometry && isRoundGeometry(geometry)
        ? BasicDimensionSelectionPattern::OnePointOneCircle
        : BasicDimensionSelectionPattern::OnePointOneLine;
    if (auto candidate = buildPrimaryDistanceCandidate(sketch, pattern, pointRef, &geometryRef)) {
        return {std::move(*candidate)};
    }

    return {};
}

std::vector<DimensionCandidate> buildNonPointPairCandidates(
    Sketcher::SketchObject* sketch,
    const DimensionReference& first,
    const DimensionReference& second)
{
    if (isLineLikeSelection(*sketch, first) && isLineLikeSelection(*sketch, second)) {
        return buildTwoLineCandidates(sketch, first.geoId, second.geoId);
    }

    const Part::Geometry* firstGeometry = isAxisGeoId(first.geoId) ? nullptr : sketch->getGeometry(first.geoId);
    const Part::Geometry* secondGeometry = isAxisGeoId(second.geoId) ? nullptr : sketch->getGeometry(second.geoId);
    if ((!isAxisGeoId(first.geoId) && !firstGeometry) || (!isAxisGeoId(second.geoId) && !secondGeometry)) {
        return {};
    }

    const auto pattern = (isRoundGeometry(firstGeometry) && isRoundGeometry(secondGeometry))
        ? BasicDimensionSelectionPattern::TwoCircles
        : ((isLineGeometry(firstGeometry) && isRoundGeometry(secondGeometry))
            || (isRoundGeometry(firstGeometry) && isLineGeometry(secondGeometry))
            ? BasicDimensionSelectionPattern::OneLineOneCircle
            : BasicDimensionSelectionPattern::Unsupported);
    if (auto candidate = buildPrimaryDistanceCandidate(sketch, pattern, first, &second)) {
        return {std::move(*candidate)};
    }

    return {};
}

std::vector<DimensionCandidate> buildPairSelectionCandidates(
    Sketcher::SketchObject* sketch,
    const DimensionReference& first,
    const DimensionReference& second)
{
    if (isPointReference(first) && isPointReference(second)) {
        return buildTwoPointCandidates(sketch, first, second);
    }

    if (isPointReference(first)) {
        return buildPointGeometryCandidates(sketch, first, second);
    }

    if (isPointReference(second)) {
        return buildPointGeometryCandidates(sketch, second, first);
    }

    return buildNonPointPairCandidates(sketch, first, second);
}

} // namespace SketcherGui::DimensionSelectionEngineDetail

namespace SketcherGui {

std::vector<DimensionCandidate> buildDimensionCandidates(
    Sketcher::SketchObject* sketch,
    const std::vector<DimensionReference>& selectionRefs)
{
    using namespace DimensionSelectionEngineDetail;

    if (!sketch) {
        return {};
    }

    switch (classifyPreviewSelectionCount(selectionRefs.size())) {
        case PreviewSelectionCountKind::Empty:
        case PreviewSelectionCountKind::Unsupported:
            return {};
        case PreviewSelectionCountKind::Single: {
            auto candidates = buildSingleSelectionCandidates(sketch, selectionRefs.front());
            filterPreviewCandidates(*sketch, candidates);
            return candidates;
        }
        case PreviewSelectionCountKind::Pair: {
            auto candidates = buildPairSelectionCandidates(sketch, selectionRefs[0], selectionRefs[1]);
            filterPreviewCandidates(*sketch, candidates);
            return candidates;
        }
    }

    return {};
}

} // namespace SketcherGui
