// SPDX-License-Identifier: LGPL-2.1-or-later
#include "DimensionResolver.h"

#include "CommandConstraints.h"
#include "DimensionGeometry.h"

#include <Mod/Part/App/Geometry.h>
#include <Mod/Sketcher/App/SketchObject.h>


namespace SketcherGui {

namespace DimensionResolverDetail {

using namespace DimensionGeometry;

bool isLineLikeReference(Sketcher::SketchObject* sketch, const DimensionReference& ref)
{
    if (isPointReference(ref)) {
        return false;
    }
    if (isAxisGeoId(ref.geoId)) {
        return true;
    }

    const Part::Geometry* geometry = sketch ? sketch->getGeometry(ref.geoId) : nullptr;
    return geometry && isLineGeometry(geometry);
}

bool isRoundReference(Sketcher::SketchObject* sketch, const DimensionReference& ref)
{
    if (isPointReference(ref) || !sketch || isAxisGeoId(ref.geoId)) {
        return false;
    }

    const Part::Geometry* geometry = sketch->getGeometry(ref.geoId);
    return geometry && isRoundGeometry(geometry);
}

std::vector<DimensionSemantic> linearSemanticsForPattern(BasicDimensionSelectionPattern pattern)
{
    switch (pattern) {
        case BasicDimensionSelectionPattern::OnePoint:
            return {DimensionSemantic::ProjectedX,
                    DimensionSemantic::ProjectedY};
        case BasicDimensionSelectionPattern::TwoPoints:
        case BasicDimensionSelectionPattern::OneLine:
            return {DimensionSemantic::DirectLength,
                    DimensionSemantic::ProjectedX,
                    DimensionSemantic::ProjectedY};
        case BasicDimensionSelectionPattern::OnePointOneLine:
        case BasicDimensionSelectionPattern::OnePointOneCircle:
        case BasicDimensionSelectionPattern::TwoLines:
        case BasicDimensionSelectionPattern::OneLineOneCircle:
        case BasicDimensionSelectionPattern::OneCircle:
        case BasicDimensionSelectionPattern::TwoCircles:
        case BasicDimensionSelectionPattern::Unsupported:
            return {};
    }

    return {};
}

} // namespace DimensionResolverDetail


std::vector<DimensionSemantic> buildLinearDimensionSemantics(BasicDimensionSelectionPattern pattern)
{
    using namespace DimensionResolverDetail;
    return linearSemanticsForPattern(pattern);
}

bool hasLinearBlockOption(BasicDimensionSelectionPattern pattern)
{
    return pattern == BasicDimensionSelectionPattern::OneLine;
}

std::vector<DimensionSemantic> buildRoundDimensionSemantics(
    Sketcher::SketchObject* sketch,
    int geoId)
{
    using namespace DimensionResolverDetail;
    if (!sketch) {
        return {};
    }

    const Part::Geometry* geometry = sketch->getGeometry(geoId);
    if (!geometry || !isRoundGeometry(geometry)) {
        return {};
    }

    if (geometry->is<Part::GeomArcOfCircle>()) {
        return {DimensionSemantic::Radius,
                DimensionSemantic::Diameter,
                DimensionSemantic::Angle,
                DimensionSemantic::ArcLength};
    }

    return {DimensionSemantic::Radius,
            DimensionSemantic::Diameter};
}

std::optional<TwoLineDimensionResolution> resolveTwoLineDimension(
    Sketcher::SketchObject* sketch,
    int firstGeoId,
    int secondGeoId)
{
    if (!sketch) {
        return std::nullopt;
    }

    TwoLineDimensionResolution result;
    result.firstGeoId = firstGeoId;
    result.secondGeoId = secondGeoId;

    if (!calculateAngle(sketch,
                        result.firstGeoId,
                        result.secondGeoId,
                        result.firstPos,
                        result.secondPos,
                        result.angleValue)) {
        return std::nullopt;
    }

    result.semantic = result.angleValue == 0.0
        ? DimensionSemantic::DirectDistance
        : DimensionSemantic::Angle;
    return result;
}




std::optional<DimensionRefPair> resolvePrimaryDistanceRefs(
    Sketcher::SketchObject* sketch,
    BasicDimensionSelectionPattern pattern,
    const DimensionReference& first,
    const DimensionReference* second)
{
    using namespace DimensionResolverDetail;
    auto makePair = [](const DimensionReference& lhs, const DimensionReference& rhs)
        -> std::optional<DimensionRefPair> {
        return DimensionRefPair {lhs, rhs};
    };

    switch (pattern) {
        case BasicDimensionSelectionPattern::OnePoint:
            if (!isPointReference(first)
                || first.geoId == Sketcher::GeoEnum::RtPnt) {
                return std::nullopt;
            }
            return makePair({Sketcher::GeoEnum::RtPnt, Sketcher::PointPos::start}, first);

        case BasicDimensionSelectionPattern::TwoPoints:
            if (!second
                || !isPointReference(first)
                || !isPointReference(*second)) {
                return std::nullopt;
            }
            return makePair(first, *second);

        case BasicDimensionSelectionPattern::OnePointOneLine:
            if (!second) {
                return std::nullopt;
            }
            if (isPointReference(first)
                && isLineLikeReference(sketch, *second)) {
                return makePair(first, *second);
            }
            if (isPointReference(*second)
                && isLineLikeReference(sketch, first)) {
                return makePair(*second, first);
            }
            return std::nullopt;

        case BasicDimensionSelectionPattern::OnePointOneCircle:
            if (!second) {
                return std::nullopt;
            }
            if (isPointReference(first)
                && isRoundReference(sketch, *second)) {
                return makePair(first, *second);
            }
            if (isPointReference(*second)
                && isRoundReference(sketch, first)) {
                return makePair(*second, first);
            }
            return std::nullopt;

        case BasicDimensionSelectionPattern::OneLine:
            if (!isLineLikeReference(sketch, first) || isAxisGeoId(first.geoId)) {
                return std::nullopt;
            }
            return makePair({first.geoId, Sketcher::PointPos::start},
                            {first.geoId, Sketcher::PointPos::end});

        case BasicDimensionSelectionPattern::TwoLines:
            if (!second || !isLineLikeReference(sketch, first) || !isLineLikeReference(sketch, *second)) {
                return std::nullopt;
            }
            if (const auto resolved = resolveTwoLineDimension(sketch, first.geoId, second->geoId);
                resolved && resolved->semantic == DimensionSemantic::DirectDistance) {
                return makePair({resolved->secondGeoId, Sketcher::PointPos::start},
                                {resolved->firstGeoId, Sketcher::PointPos::none});
            }
            return std::nullopt;

        case BasicDimensionSelectionPattern::OneLineOneCircle:
            if (!second) {
                return std::nullopt;
            }
            if (isRoundReference(sketch, first) && isLineLikeReference(sketch, *second)) {
                return makePair(first, *second);
            }
            if (isRoundReference(sketch, *second) && isLineLikeReference(sketch, first)) {
                return makePair(*second, first);
            }
            return std::nullopt;

        case BasicDimensionSelectionPattern::OneCircle:
            return std::nullopt;

        case BasicDimensionSelectionPattern::TwoCircles:
            if (!second || !isRoundReference(sketch, first) || !isRoundReference(sketch, *second)) {
                return std::nullopt;
            }
            return makePair(first, *second);

        case BasicDimensionSelectionPattern::Unsupported:
            return std::nullopt;
    }

    return std::nullopt;
}

} // namespace SketcherGui
