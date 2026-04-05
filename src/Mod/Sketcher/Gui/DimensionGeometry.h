// SPDX-License-Identifier: LGPL-2.1-or-later
#pragma once

#include "DimensionCandidate.h"

#include <Mod/Part/App/Geometry.h>
#include <Mod/Sketcher/App/GeoEnum.h>
#include <Mod/Sketcher/App/Constraint.h>
#include <Mod/Sketcher/App/SketchObject.h>

#include <algorithm>
#include <cmath>
#include <utility>

namespace SketcherGui::DimensionGeometry {

constexpr double kEpsilon = 1e-9;

struct DistanceSegment
{
    bool valid {false};
    Base::Vector2d a;
    Base::Vector2d b;
    double value {0.0};
};

inline Base::Vector2d toVector2d(const Base::Vector3d& p)
{
    return Base::Vector2d(p.x, p.y);
}


inline Base::Vector2d midpoint(const Base::Vector2d& a, const Base::Vector2d& b);
inline Base::Vector2d tangentFromLine(const Base::Vector2d& a, const Base::Vector2d& b);
inline Base::Vector2d stableNormalFromLine(const Base::Vector2d& a, const Base::Vector2d& b);
inline DistanceSegment directDistanceSegment(const Sketcher::SketchObject& sketch,
                                             const DimensionReference& first,
                                             const DimensionReference& second);

inline bool shouldSwapDirectLinearEndpoints(const Base::Vector2d& a,
                                           const Base::Vector2d& b)
{
    // Keep direct linear measurements in a single canonical direction everywhere:
    // left -> right, and for vertical lines bottom -> top.
    //
    // Why this and not lower -> upper?
    // - DistanceX/DistanceY already canonicalize against their measurement axes.
    // - SoDatumLabel::DISTANCE still depends on pnt1 -> pnt2 for which side is drawn.
    // - When we used lower -> upper, descending lines ended up canonicalized right -> left,
    //   which is exactly the mirrored case the user keeps seeing for DirectLength.
    //
    // Using the same geometric direction for preview, hover, duplicate filtering, and
    // commit keeps DirectLength symmetric across positive/negative slopes.
    if (b.x < a.x - kEpsilon) {
        return true;
    }
    if (std::abs(b.x - a.x) <= kEpsilon && b.y < a.y - kEpsilon) {
        return true;
    }
    return false;
}

inline bool shouldSwapProjectedEndpoints(const Base::Vector2d& a,
                                        const Base::Vector2d& b,
                                        DimensionSemantic semantic)
{
    switch (semantic) {
        case DimensionSemantic::ProjectedX:
            return b.x < a.x - kEpsilon;
        case DimensionSemantic::ProjectedY:
            return b.y < a.y - kEpsilon;
        default:
            return false;
    }
}

struct LinearPreviewFrame
{
    bool valid {false};
    Base::Vector2d a;
    Base::Vector2d b;
    Base::Vector2d dir;
    Base::Vector2d normal;
    Base::Vector2d datumMid;
    Base::Vector2d lower;
    Base::Vector2d upper;
};

inline LinearPreviewFrame makeLinearPreviewFrame(Base::Vector2d a,
                                                 Base::Vector2d b,
                                                 DimensionSemantic semantic)
{
    if (!isLinearDimensionSemantic(semantic)) {
        return {};
    }

    if (shouldSwapProjectedEndpoints(a, b, semantic)) {
        std::swap(a, b);
    }

    const bool aIsLower = (a.y < b.y)
        || (std::abs(a.y - b.y) <= kEpsilon && a.x <= b.x);
    const Base::Vector2d lower = aIsLower ? a : b;
    const Base::Vector2d upper = aIsLower ? b : a;

    // Canonicalize direct linear measurements to the same direction everywhere:
    // left -> right, and for vertical lines bottom -> top.
    // This is the method that keeps DirectLength symmetric with the axis-projected
    // dimensions and avoids the descending-line mirror bug.
    if ((semantic == DimensionSemantic::DirectLength
         || semantic == DimensionSemantic::DirectDistance)
        && shouldSwapDirectLinearEndpoints(a, b)) {
        std::swap(a, b);
    }

    LinearPreviewFrame frame;
    frame.valid = true;
    frame.a = a;
    frame.b = b;
    frame.lower = lower;
    frame.upper = upper;
    frame.datumMid = midpoint(a, b);

    switch (semantic) {
        case DimensionSemantic::ProjectedX:
            frame.dir = Base::Vector2d((b.x - a.x >= kEpsilon) ? 1.0 : -1.0, 0.0);
            frame.normal = Base::Vector2d(-frame.dir.y, frame.dir.x);
            break;
        case DimensionSemantic::ProjectedY:
            frame.dir = Base::Vector2d(0.0, (b.y - a.y >= kEpsilon) ? 1.0 : -1.0);
            frame.normal = Base::Vector2d(-frame.dir.y, frame.dir.x);
            break;
        case DimensionSemantic::DirectLength:
        case DimensionSemantic::DirectDistance:
        default:
            frame.dir = tangentFromLine(a, b);
            frame.normal = stableNormalFromLine(a, b);
            break;
    }

    // Keep the canonical frame aligned with SoDatumLabel's real placement math.
    // For DISTANCEX/DISTANCEY the widget does not anchor from the geometric
    // midpoint of the measured span; it anchors from the "second" endpoint after
    // the constraint refs are canonicalized. If we use bbox-based mids here, the
    // preview can look correct in one slope direction and mirrored in the other,
    // and commit-time placement will jump to a different side.
    if (semantic == DimensionSemantic::ProjectedX) {
        frame.datumMid = Base::Vector2d((a.x + b.x) * 0.5, b.y);
    }
    else if (semantic == DimensionSemantic::ProjectedY) {
        frame.datumMid = Base::Vector2d(b.x, (a.y + b.y) * 0.5);
    }

    return frame;
}

inline LinearPreviewFrame makeLinearPreviewFrame(const Sketcher::SketchObject& sketch,
                                                 const DimensionReference& first,
                                                 const DimensionReference& second,
                                                 DimensionSemantic semantic)
{
    const auto segment = directDistanceSegment(sketch, first, second);
    if (!segment.valid) {
        return {};
    }

    return makeLinearPreviewFrame(segment.a, segment.b, semantic);
}

inline LinearPreviewFrame makeLinearPreviewFrame(const Sketcher::SketchObject& sketch,
                                                 const Sketcher::Constraint& constraint,
                                                 DimensionSemantic semantic)
{
    const DimensionReference first {constraint.First, constraint.FirstPos};
    const DimensionReference second {constraint.Second, constraint.SecondPos};
    return makeLinearPreviewFrame(sketch, first, second, semantic);
}

inline void decomposeLinearLabelPosition(const LinearPreviewFrame& frame,
                                         const Base::Vector2d& labelPos,
                                         double& signedNormal,
                                         double& along)
{
    const Base::Vector2d rel(labelPos.x - frame.datumMid.x, labelPos.y - frame.datumMid.y);
    signedNormal = rel.x * frame.normal.x + rel.y * frame.normal.y;
    along = rel.x * frame.dir.x + rel.y * frame.dir.y;
}

inline Base::Vector2d composeLinearLabelPosition(const LinearPreviewFrame& frame,
                                                 double signedNormal,
                                                 double along)
{
    return Base::Vector2d(frame.datumMid.x + frame.normal.x * signedNormal + frame.dir.x * along,
                          frame.datumMid.y + frame.normal.y * signedNormal + frame.dir.y * along);
}

inline Base::Vector2d preferredExteriorLinearLabelPosition(const LinearPreviewFrame& frame,
                                                           DimensionSemantic semantic,
                                                           double offset,
                                                           double along = 0.0)
{
    switch (semantic) {
        case DimensionSemantic::ProjectedX: {
            const double minY = std::min(frame.a.y, frame.b.y);
            return Base::Vector2d((frame.a.x + frame.b.x) * 0.5 + along,
                                  minY - offset);
        }
        case DimensionSemantic::ProjectedY: {
            const double minX = std::min(frame.a.x, frame.b.x);
            const double maxX = std::max(frame.a.x, frame.b.x);
            const double preferredX = frame.b.x >= frame.a.x ? (maxX + offset)
                                                              : (minX - offset);
            return Base::Vector2d(preferredX,
                                  (frame.a.y + frame.b.y) * 0.5 + along);
        }
        case DimensionSemantic::DirectLength:
        case DimensionSemantic::DirectDistance:
        default:
            return composeLinearLabelPosition(frame, offset, along);
    }
}

inline Base::Vector2d sketchPoint(const Sketcher::SketchObject& sketch,
                                  const DimensionReference& ref)
{
    return toVector2d(sketch.getPoint(ref.geoId, ref.posId));
}

inline Base::Vector2d midpoint(const Base::Vector2d& a, const Base::Vector2d& b)
{
    return Base::Vector2d((a.x + b.x) * 0.5, (a.y + b.y) * 0.5);
}

inline double length(const Base::Vector2d& v)
{
    return std::sqrt(v.x * v.x + v.y * v.y);
}

inline Base::Vector2d normalized(const Base::Vector2d& v, const Base::Vector2d& fallback)
{
    const double len = length(v);
    if (len < kEpsilon) {
        return fallback;
    }

    return Base::Vector2d(v.x / len, v.y / len);
}

inline Base::Vector2d tangentFromLine(const Base::Vector2d& a, const Base::Vector2d& b)
{
    return normalized(Base::Vector2d(b.x - a.x, b.y - a.y), Base::Vector2d(1.0, 0.0));
}

inline Base::Vector2d stableNormalFromLine(const Base::Vector2d& a, const Base::Vector2d& b)
{
    Base::Vector2d normal(-tangentFromLine(a, b).y, tangentFromLine(a, b).x);
    if (normal.y < -kEpsilon || (std::abs(normal.y) < kEpsilon && normal.x < 0.0)) {
        normal.x = -normal.x;
        normal.y = -normal.y;
    }

    return normal;
}

inline bool roundRadiusCenter(const Part::Geometry* geometry,
                              Base::Vector2d& center,
                              double& radius)
{
    if (!geometry) {
        return false;
    }

    if (geometry->is<Part::GeomCircle>()) {
        const auto* circle = static_cast<const Part::GeomCircle*>(geometry);
        const auto c = circle->getCenter();
        center = Base::Vector2d(c.x, c.y);
        radius = circle->getRadius();
        return true;
    }

    if (geometry->is<Part::GeomArcOfCircle>()) {
        const auto* arc = static_cast<const Part::GeomArcOfCircle*>(geometry);
        const auto c = arc->getCenter();
        center = Base::Vector2d(c.x, c.y);
        radius = arc->getRadius();
        return true;
    }

    return false;
}

inline bool roundRadiusCenter(const Sketcher::SketchObject& sketch,
                              int geoId,
                              Base::Vector2d& center,
                              double& radius)
{
    return roundRadiusCenter(sketch.getGeometry(geoId), center, radius);
}


struct RoundPlacementData
{
    bool valid {false};
    bool isArc {false};
    Base::Vector2d center;
    double radius {0.0};
    double preferredAngle {0.0};
    double arcStartAngle {0.0};
    double arcEndAngle {0.0};
};

inline double normalizeAnglePositive(double angle)
{
    constexpr double kTwoPi = 2.0 * 3.14159265358979323846;
    angle = std::fmod(angle, kTwoPi);
    if (angle < 0.0) {
        angle += kTwoPi;
    }
    return angle;
}

inline double wrapAngleNear(double angle, double reference)
{
    constexpr double kTwoPi = 2.0 * 3.14159265358979323846;
    angle = normalizeAnglePositive(angle);
    reference = normalizeAnglePositive(reference);

    while (angle - reference > 3.14159265358979323846) {
        angle -= kTwoPi;
    }
    while (reference - angle > 3.14159265358979323846) {
        angle += kTwoPi;
    }
    return angle;
}

inline double defaultRoundPreferredAngle(DimensionSemantic semantic)
{
    switch (semantic) {
        case DimensionSemantic::Diameter:
            return 2.356194490192345; // 135°
        case DimensionSemantic::Radius:
        default:
            return 0.7853981633974483; // 45°
    }
}

inline RoundPlacementData describeRoundPlacement(const Part::Geometry* geometry,
                                                 DimensionSemantic semantic)
{
    RoundPlacementData data;
    if (!roundRadiusCenter(geometry, data.center, data.radius) || data.radius <= kEpsilon) {
        return data;
    }

    data.valid = true;
    data.preferredAngle = defaultRoundPreferredAngle(semantic);

    const auto* arc = (geometry && geometry->is<Part::GeomArcOfCircle>())
        ? static_cast<const Part::GeomArcOfCircle*>(geometry)
        : nullptr;
    if (!arc) {
        return data;
    }

    data.isArc = true;
    double startAngle = 0.0;
    double endAngle = 0.0;
    arc->getRange(startAngle, endAngle, /*emulateCCW=*/true);
    data.arcStartAngle = startAngle;
    data.arcEndAngle = endAngle;

    constexpr double kMargin = 0.18;
    const double minAngle = startAngle + kMargin;
    const double maxAngle = endAngle - kMargin;
    const double midAngle = startAngle + (endAngle - startAngle) * 0.5;
    if (minAngle > maxAngle) {
        data.preferredAngle = midAngle;
        return data;
    }

    // Keep arc-based round previews on deterministic semantic-specific slots.
    // Radius and diameter used to share almost the same midpoint-driven angle,
    // then later preview/layout/commit paths tried to separate them independently.
    // That made labels and preview leaders stack on top of each other even though
    // they were describing different semantics. Assign the semantic slot here once
    // and let every downstream path reuse the same angle.
    const double usableRange = maxAngle - minAngle;
    if (usableRange < 0.30) {
        data.preferredAngle = minAngle + usableRange * 0.5;
        return data;
    }

    double fraction = 0.5;
    switch (semantic) {
        case DimensionSemantic::Diameter:
            // Keep diameter previews near the early part of the usable arc span so the
            // full through-center preview line stays away from radius previews.
            fraction = 0.18;
            break;
        case DimensionSemantic::Radius:
            // Keep radius previews on a later semantic-specific slot with a visibly
            // different leader direction from diameter.
            fraction = 0.72;
            break;
        default:
            fraction = 0.50;
            break;
    }

    data.preferredAngle = minAngle + usableRange * fraction;
    data.preferredAngle = std::clamp(data.preferredAngle, minAngle, maxAngle);
    return data;
}

inline double clampRoundLabelAngle(const RoundPlacementData& data, double angle)
{
    if (!data.valid) {
        return angle;
    }

    if (!data.isArc) {
        return angle;
    }

    constexpr double kMargin = 0.18;
    const double minAngle = data.arcStartAngle + kMargin;
    const double maxAngle = data.arcEndAngle - kMargin;
    if (minAngle > maxAngle) {
        return data.preferredAngle;
    }

    angle = wrapAngleNear(angle, data.preferredAngle);
    return std::clamp(angle, minAngle, maxAngle);
}

inline double roundConstraintAngle(const RoundPlacementData& data,
                                   const Base::Vector2d& labelPos)
{
    if (!data.valid) {
        return 0.0;
    }

    const Base::Vector2d rel(labelPos.x - data.center.x, labelPos.y - data.center.y);
    const double angle = (std::abs(rel.x) <= kEpsilon && std::abs(rel.y) <= kEpsilon)
        ? data.preferredAngle
        : std::atan2(rel.y, rel.x);
    return clampRoundLabelAngle(data, angle);
}

inline double roundConstraintLabelDistance(const RoundPlacementData& data,
                                           const Base::Vector2d& labelPos)
{
    if (!data.valid) {
        return 0.0;
    }

    const double angle = roundConstraintAngle(data, labelPos);
    const Base::Vector2d dir(std::cos(angle), std::sin(angle));
    const Base::Vector2d rim(data.center.x + dir.x * data.radius,
                             data.center.y + dir.y * data.radius);
    const Base::Vector2d rel(labelPos.x - rim.x, labelPos.y - rim.y);
    return std::max(1.0, rel.x * dir.x + rel.y * dir.y);
}

inline Base::Vector2d roundLabelPosition(const RoundPlacementData& data,
                                         double angle,
                                         double labelDistance)
{
    if (!data.valid) {
        return {};
    }

    angle = clampRoundLabelAngle(data, angle);
    const Base::Vector2d dir(std::cos(angle), std::sin(angle));
    const double radius = std::max(1.0, data.radius + labelDistance);
    return Base::Vector2d(data.center.x + dir.x * radius,
                          data.center.y + dir.y * radius);
}

inline Base::Vector2d makeStableRoundLabelPos(const Part::Geometry* geometry,
                                              DimensionSemantic semantic,
                                              double minOffset = 5.0,
                                              double maxOffset = 12.0)
{
    const auto data = describeRoundPlacement(geometry, semantic);
    if (!data.valid) {
        return {};
    }

    double offset = std::clamp(data.radius * 0.25, minOffset, maxOffset);
    // Keep radius/diameter candidates on visibly different rings as well as different angles.
    // The later preview optimizer is intentionally local per-candidate, so a small distance
    // separation here helps preserve the semantic distinction without reintroducing global
    // cross-candidate layout jumps.
    switch (semantic) {
        case DimensionSemantic::Diameter:
            // Push diameter previews onto a clearly outer ring so the long preview line
            // and label do not collapse back toward the radius candidate on large arcs.
            offset = std::max(offset + 10.0, offset * 1.70);
            break;
        case DimensionSemantic::Radius:
            offset = std::max(offset + 1.0, offset * 1.10);
            break;
        default:
            break;
    }
    return roundLabelPosition(data, data.preferredAngle, offset);
}

struct ArcLengthPlacementData
{
    bool valid {false};
    Base::Vector2d center;
    Base::Vector2d visualDir;
    double radius {0.0};
    double startAngle {0.0};
    double range {0.0};
    double midAngle {0.0};
};

inline ArcLengthPlacementData describeArcLengthPlacement(const Part::GeomArcOfCircle& arc)
{
    ArcLengthPlacementData data;

    const auto center3d = arc.getCenter();
    const Base::Vector2d center(center3d.x, center3d.y);
    const double radius = arc.getRadius();
    if (radius <= kEpsilon) {
        return data;
    }

    double startAngle = 0.0;
    double endAngle = 0.0;
    // Follow the same trimmed-arc interpretation that the regular Sketcher command path uses
    // when a newly created arc-length constraint is placed through ViewProviderSketch::moveConstraint.
    // Smart-dimension preview/final drawing must use that exact same branch selection, otherwise
    // >180° arcs can preview one branch and commit the complementary one.
    arc.getRange(startAngle, endAngle, /*emulateCCW=*/true);
    const double range = endAngle - startAngle;

    if (std::abs(range) <= kEpsilon) {
        return data;
    }

    const double midAngle = startAngle + range * 0.5;
    const Base::Vector2d visualDir = normalized(Base::Vector2d(std::cos(midAngle), std::sin(midAngle)),
                                                Base::Vector2d(1.0, 0.0));

    data.valid = true;
    data.center = center;
    data.visualDir = visualDir;
    data.radius = radius;
    data.startAngle = startAngle;
    data.range = range;
    data.midAngle = midAngle;
    return data;
}

inline bool describeArcLengthPlacement(const Part::Geometry* geometry, ArcLengthPlacementData& data)
{
    const auto* arc = (geometry && geometry->is<Part::GeomArcOfCircle>())
        ? static_cast<const Part::GeomArcOfCircle*>(geometry)
        : nullptr;
    if (!arc) {
        return false;
    }

    data = describeArcLengthPlacement(*arc);
    return data.valid;
}

inline bool resolveArcLengthDatumSweep(const Part::GeomArcOfCircle& arc,
                                     double signedLabelDistance,
                                     double& startAngle,
                                     double& range)
{
    (void)signedLabelDistance;

    const auto placement = describeArcLengthPlacement(arc);
    if (!placement.valid) {
        return false;
    }

    // Arc-length must always follow the actual selected arc. The sign of LabelDistance only
    // controls how far the label sits from the arc; using it to swap to the complementary sweep
    // makes >180° arcs jump to the wrong side.
    startAngle = placement.startAngle;
    range = placement.range;
    return std::abs(range) > kEpsilon;
}

inline bool resolveArcLengthDatumSweep(const Part::Geometry* geometry,
                                     double signedLabelDistance,
                                     double& startAngle,
                                     double& range)
{
    const auto* arc = (geometry && geometry->is<Part::GeomArcOfCircle>())
        ? static_cast<const Part::GeomArcOfCircle*>(geometry)
        : nullptr;
    if (!arc) {
        return false;
    }

    return resolveArcLengthDatumSweep(*arc, signedLabelDistance, startAngle, range);
}

inline Base::Vector2d projectArcLengthLabelPosition(const ArcLengthPlacementData& data,
                                                    const Base::Vector2d& desired)
{
    if (!data.valid) {
        return desired;
    }

    const Base::Vector2d rel(desired.x - data.center.x, desired.y - data.center.y);
    const double scalar = std::max(1.0, rel.x * data.visualDir.x + rel.y * data.visualDir.y);
    return Base::Vector2d(data.center.x + data.visualDir.x * scalar,
                          data.center.y + data.visualDir.y * scalar);
}

inline double arcLengthLabelRadius(const ArcLengthPlacementData& data,
                                   const Base::Vector2d& labelPos)
{
    if (!data.valid) {
        return 0.0;
    }

    const Base::Vector2d rel(labelPos.x - data.center.x, labelPos.y - data.center.y);
    return rel.x * data.visualDir.x + rel.y * data.visualDir.y;
}

inline double arcLengthConstraintLabelDistance(const ArcLengthPlacementData& data,
                                               const Base::Vector2d& labelPos)
{
    if (!data.valid) {
        return 0.0;
    }

    return std::max(1.0, arcLengthLabelRadius(data, labelPos));
}


inline bool isAxisGeoId(int geoId)
{
    return geoId == Sketcher::GeoEnum::HAxis || geoId == Sketcher::GeoEnum::VAxis;
}

inline bool isLineGeometry(const Part::Geometry* geometry)
{
    return geometry && geometry->is<Part::GeomLineSegment>();
}

inline bool isRoundGeometry(const Part::Geometry* geometry)
{
    return geometry && (geometry->is<Part::GeomCircle>() || geometry->is<Part::GeomArcOfCircle>());
}

inline const Part::GeomLineSegment* asLineSegment(const Sketcher::SketchObject& sketch, int geoId)
{
    const Part::Geometry* geometry = sketch.getGeometry(geoId);
    return isLineGeometry(geometry) ? static_cast<const Part::GeomLineSegment*>(geometry) : nullptr;
}

inline Base::Vector2d pointPosVertex(const Part::GeomLineSegment& line, Sketcher::PointPos pos)
{
    return pos == Sketcher::PointPos::start
        ? toVector2d(line.getStartPoint())
        : toVector2d(line.getEndPoint());
}

inline Base::Vector2d directedTangent(const Part::GeomLineSegment& line, Sketcher::PointPos pos)
{
    const Base::Vector2d a = toVector2d(line.getStartPoint());
    const Base::Vector2d b = toVector2d(line.getEndPoint());
    const Base::Vector2d dir = tangentFromLine(a, b);
    return pos == Sketcher::PointPos::start ? dir : Base::Vector2d(-dir.x, -dir.y);
}

inline DistanceSegment makeDistanceSegment(const Base::Vector2d& a,
                                           const Base::Vector2d& b,
                                           double value)
{
    return DistanceSegment {true, a, b, value};
}

inline DistanceSegment pointLineDistance(const Sketcher::SketchObject& sketch,
                                         const DimensionReference& pointRef,
                                         int lineGeoId)
{
    const auto* line = asLineSegment(sketch, lineGeoId);
    if (!line) {
        return {};
    }

    const Base::Vector2d point = sketchPoint(sketch, pointRef);
    const Base::Vector2d a = toVector2d(line->getStartPoint());
    const Base::Vector2d b = toVector2d(line->getEndPoint());
    const Base::Vector2d d(b.x - a.x, b.y - a.y);
    const double ll = d.x * d.x + d.y * d.y;
    if (ll < kEpsilon) {
        return {};
    }

    const double t = ((point.x - a.x) * d.x + (point.y - a.y) * d.y) / ll;
    const Base::Vector2d projected(a.x + d.x * t, a.y + d.y * t);
    const double value = std::abs(-point.x * d.y + point.y * d.x + a.x * b.y - b.x * a.y)
        / std::sqrt(ll);
    return makeDistanceSegment(point, projected, value);
}

inline DistanceSegment pointRoundDistance(const Sketcher::SketchObject& sketch,
                                          const DimensionReference& pointRef,
                                          int roundGeoId)
{
    Base::Vector2d center;
    double radius = 0.0;
    if (!roundRadiusCenter(sketch, roundGeoId, center, radius)) {
        return {};
    }

    const Base::Vector2d point = sketchPoint(sketch, pointRef);
    Base::Vector2d dir(point.x - center.x, point.y - center.y);
    dir = normalized(dir, Base::Vector2d(1.0, 0.0));
    const Base::Vector2d curvePoint(center.x + dir.x * radius, center.y + dir.y * radius);
    return makeDistanceSegment(
        point,
        curvePoint,
        std::abs(length(Base::Vector2d(point.x - center.x, point.y - center.y)) - radius));
}

inline DistanceSegment roundLineDistance(const Sketcher::SketchObject& sketch,
                                         int roundGeoId,
                                         int lineGeoId)
{
    Base::Vector2d center;
    double radius = 0.0;
    const auto* line = asLineSegment(sketch, lineGeoId);
    if (!roundRadiusCenter(sketch, roundGeoId, center, radius) || !line) {
        return {};
    }

    const Base::Vector2d a = toVector2d(line->getStartPoint());
    const Base::Vector2d b = toVector2d(line->getEndPoint());
    const Base::Vector2d d(b.x - a.x, b.y - a.y);
    const double ll = d.x * d.x + d.y * d.y;
    if (ll < kEpsilon) {
        return {};
    }

    const double t = ((center.x - a.x) * d.x + (center.y - a.y) * d.y) / ll;
    const Base::Vector2d projected(a.x + d.x * t, a.y + d.y * t);
    Base::Vector2d dir(projected.x - center.x, projected.y - center.y);
    dir = normalized(dir, stableNormalFromLine(a, b));
    const Base::Vector2d curvePoint(center.x + dir.x * radius, center.y + dir.y * radius);
    return makeDistanceSegment(
        projected,
        curvePoint,
        std::abs(length(Base::Vector2d(projected.x - center.x, projected.y - center.y)) - radius));
}

inline DistanceSegment roundRoundDistance(const Sketcher::SketchObject& sketch,
                                          int firstGeoId,
                                          int secondGeoId)
{
    Base::Vector2d c1;
    Base::Vector2d c2;
    double r1 = 0.0;
    double r2 = 0.0;
    if (!roundRadiusCenter(sketch, firstGeoId, c1, r1)
        || !roundRadiusCenter(sketch, secondGeoId, c2, r2)) {
        return {};
    }

    Base::Vector2d dir(c2.x - c1.x, c2.y - c1.y);
    dir = normalized(dir, Base::Vector2d(1.0, 0.0));
    const double centerDistance = length(Base::Vector2d(c2.x - c1.x, c2.y - c1.y));

    if (centerDistance >= r1 && centerDistance >= r2) {
        return makeDistanceSegment(Base::Vector2d(c1.x + dir.x * r1, c1.y + dir.y * r1),
                                   Base::Vector2d(c2.x - dir.x * r2, c2.y - dir.y * r2),
                                   centerDistance - r1 - r2);
    }

    if (r1 >= r2) {
        return makeDistanceSegment(Base::Vector2d(c1.x + dir.x * r1, c1.y + dir.y * r1),
                                   Base::Vector2d(c2.x + dir.x * r2, c2.y + dir.y * r2),
                                   r1 - r2 - centerDistance);
    }

    return makeDistanceSegment(Base::Vector2d(c1.x - dir.x * r1, c1.y - dir.y * r1),
                               Base::Vector2d(c2.x - dir.x * r2, c2.y - dir.y * r2),
                               r2 - r1 - centerDistance);
}

inline DistanceSegment directDistanceSegment(const Sketcher::SketchObject& sketch,
                                             const DimensionReference& first,
                                             const DimensionReference& second)
{
    const bool firstIsPoint = first.posId != Sketcher::PointPos::none;
    const bool secondIsPoint = second.posId != Sketcher::PointPos::none;

    if (firstIsPoint && secondIsPoint) {
        const Base::Vector2d a = sketchPoint(sketch, first);
        const Base::Vector2d b = sketchPoint(sketch, second);
        return makeDistanceSegment(a, b, length(Base::Vector2d(b.x - a.x, b.y - a.y)));
    }

    if (firstIsPoint) {
        const Part::Geometry* geometry = second.geoId == Sketcher::GeoEnum::GeoUndef
            ? nullptr
            : sketch.getGeometry(second.geoId);
        if (isLineGeometry(geometry)) {
            return pointLineDistance(sketch, first, second.geoId);
        }
        if (isRoundGeometry(geometry)) {
            return pointRoundDistance(sketch, first, second.geoId);
        }
        return {};
    }

    if (secondIsPoint) {
        return directDistanceSegment(sketch, second, first);
    }

    const Part::Geometry* firstGeometry = first.geoId == Sketcher::GeoEnum::GeoUndef
        ? nullptr
        : sketch.getGeometry(first.geoId);
    const Part::Geometry* secondGeometry = second.geoId == Sketcher::GeoEnum::GeoUndef
        ? nullptr
        : sketch.getGeometry(second.geoId);
    if (!firstGeometry || !secondGeometry) {
        return {};
    }

    if (isRoundGeometry(firstGeometry) && isLineGeometry(secondGeometry)) {
        return roundLineDistance(sketch, first.geoId, second.geoId);
    }
    if (isLineGeometry(firstGeometry) && isRoundGeometry(secondGeometry)) {
        const auto segment = roundLineDistance(sketch, second.geoId, first.geoId);
        if (!segment.valid) {
            return {};
        }
        return makeDistanceSegment(segment.b, segment.a, segment.value);
    }
    if (isRoundGeometry(firstGeometry) && isRoundGeometry(secondGeometry)) {
        return roundRoundDistance(sketch, first.geoId, second.geoId);
    }

    return {};
}

} // namespace SketcherGui::DimensionGeometry
