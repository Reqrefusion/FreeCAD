// SPDX-License-Identifier: LGPL-2.1-or-later
#include "DimensionConstraintBuilder.h"

#include "DimensionGeometry.h"

#include <Gui/CommandT.h>
#include <Mod/Sketcher/App/Constraint.h>
#include <Mod/Sketcher/App/SketchObject.h>
#include <Mod/Part/App/Geometry.h>

#include <algorithm>
#include <cassert>
#include <cmath>
#include <limits>
#include <optional>
#include <utility>

namespace SketcherGui {

namespace DimensionConstraintBuilderDetail {

using namespace DimensionGeometry;

[[nodiscard]] bool isDrivingPolicy(DimensionDrivingPolicy drivingPolicy)
{
    return drivingPolicy == DimensionDrivingPolicy::Driving;
}


std::optional<Sketcher::ConstraintType> constraintTypeForSemantic(DimensionSemantic semantic)
{
    switch (semantic) {
        case DimensionSemantic::DirectLength:
        case DimensionSemantic::DirectDistance:
        case DimensionSemantic::ArcLength:
            return Sketcher::ConstraintType::Distance;
        case DimensionSemantic::ProjectedX:
            return Sketcher::ConstraintType::DistanceX;
        case DimensionSemantic::ProjectedY:
            return Sketcher::ConstraintType::DistanceY;
        case DimensionSemantic::Radius:
            return Sketcher::ConstraintType::Radius;
        case DimensionSemantic::Diameter:
            return Sketcher::ConstraintType::Diameter;
        case DimensionSemantic::Angle:
            return Sketcher::ConstraintType::Angle;
        case DimensionSemantic::Unknown:
        default:
            return std::nullopt;
    }
}

void canonicalizeLinearConstraint(const Sketcher::SketchObject& sketch,
                                  const DimensionCandidate& candidate,
                                  Sketcher::Constraint& constraint)
{
    if (candidate.refs.size() < 2
        || !isPointReference(candidate.refs[0])
        || !isPointReference(candidate.refs[1])) {
        // Keep mixed-topology constraints in the native solver order. Sketcher stores
        // point-line, point-circle and similar linear distances asymmetrically even
        // when the preview geometry looks symmetric on screen.
        return;
    }

    const auto segment = directDistanceSegment(sketch, candidate.refs[0], candidate.refs[1]);
    if (!segment.valid) {
        return;
    }

    bool swap = false;
    if (candidate.semantic == DimensionSemantic::ProjectedX
        || candidate.semantic == DimensionSemantic::ProjectedY) {
        swap = shouldSwapProjectedEndpoints(segment.a, segment.b, candidate.semantic);
    }
    else if (candidate.semantic == DimensionSemantic::DirectLength
             || candidate.semantic == DimensionSemantic::DirectDistance) {
        swap = shouldSwapDirectLinearEndpoints(segment.a, segment.b);
    }

    if (!swap) {
        return;
    }

    std::swap(constraint.First, constraint.Second);
    std::swap(constraint.FirstPos, constraint.SecondPos);
}

bool computeAngleIntersection(const Sketcher::SketchObject& sketch,
                              const Sketcher::Constraint& constraint,
                              Base::Vector2d& intersection)
{
    const Part::Geometry* geo1 = sketch.getGeometry(constraint.First);
    const Part::Geometry* geo2 = sketch.getGeometry(constraint.Second);
    if (!geo1 || !geo2 || !geo1->is<Part::GeomLineSegment>() || !geo2->is<Part::GeomLineSegment>()) {
        return false;
    }

    const auto* line1 = static_cast<const Part::GeomLineSegment*>(geo1);
    const auto* line2 = static_cast<const Part::GeomLineSegment*>(geo2);
    const bool flip1 = (constraint.FirstPos == Sketcher::PointPos::end);
    const bool flip2 = (constraint.SecondPos == Sketcher::PointPos::end);

    Base::Vector3d dir1 = (flip1 ? -1.0 : 1.0)
        * (line1->getEndPoint() - line1->getStartPoint()).Normalize();
    Base::Vector3d dir2 = (flip2 ? -1.0 : 1.0)
        * (line2->getEndPoint() - line2->getStartPoint()).Normalize();
    Base::Vector3d pnt1 = flip1 ? line1->getEndPoint() : line1->getStartPoint();
    Base::Vector3d pnt2 = flip2 ? line2->getEndPoint() : line2->getStartPoint();

    Base::Vector3d hit;
    const double det = dir1.x * dir2.y - dir1.y * dir2.x;
    if (std::abs(det) < 1e-10) {
        Base::Vector3d p1[2] = {line1->getStartPoint(), line1->getEndPoint()};
        Base::Vector3d p2[2] = {line2->getStartPoint(), line2->getEndPoint()};
        double shortest = std::numeric_limits<double>::max();
        for (int i = 0; i <= 1; ++i) {
            for (int j = 0; j <= 1; ++j) {
                const double candidateDistance = (p2[j] - p1[i]).Length();
                if (candidateDistance < shortest) {
                    shortest = candidateDistance;
                    hit = Base::Vector3d((p2[j].x + p1[i].x) * 0.5,
                                         (p2[j].y + p1[i].y) * 0.5,
                                         0.0);
                }
            }
        }
    }
    else {
        const double c1 = dir1.y * pnt1.x - dir1.x * pnt1.y;
        const double c2 = dir2.y * pnt2.x - dir2.x * pnt2.y;
        hit = Base::Vector3d((dir1.x * c2 - dir2.x * c1) / det,
                             (dir1.y * c2 - dir2.y * c1) / det,
                             0.0);
    }

    intersection = Base::Vector2d(hit.x, hit.y);
    return true;
}


} // namespace DimensionConstraintBuilderDetail

void applyPreparedPlacement(const Sketcher::SketchObject& sketch,
                           const DimensionCandidate& candidate,
                           Sketcher::Constraint& constraint)
{
    switch (candidate.semantic) {
        case DimensionSemantic::ProjectedX:
        case DimensionSemantic::ProjectedY:
        case DimensionSemantic::DirectLength:
        case DimensionSemantic::DirectDistance: {
            const auto frame = DimensionGeometry::makeLinearPreviewFrame(sketch, constraint, candidate.semantic);
            if (!frame.valid) {
                break;
            }

            double signedNormal = 0.0;
            double along = 0.0;
            DimensionGeometry::decomposeLinearLabelPosition(frame, candidate.labelPos, signedNormal, along);
            constraint.LabelDistance = static_cast<float>(signedNormal);
            constraint.LabelPosition = static_cast<float>(along);
            break;
        }
        case DimensionSemantic::ArcLength: {
            if (candidate.refs.empty()) {
                break;
            }

            const auto* geometry = sketch.getGeometry(candidate.refs.front().geoId);
            DimensionGeometry::ArcLengthPlacementData placement;
            if (!DimensionGeometry::describeArcLengthPlacement(geometry, placement)) {
                break;
            }

            // The label distance controls only the label offset. The selected arc branch is
            // resolved from the actual arc geometry and must stay stable even when the label is
            // dragged to the other side of the center.
            const double labelDistance = DimensionGeometry::arcLengthConstraintLabelDistance(placement,
                                                                          candidate.labelPos);
            constraint.LabelDistance = static_cast<float>(labelDistance);
            break;
        }
        case DimensionSemantic::Radius:
        case DimensionSemantic::Diameter: {
            if (candidate.refs.empty()) {
                break;
            }

            const auto placement = DimensionGeometry::describeRoundPlacement(sketch.getGeometry(candidate.refs.front().geoId),
                                                          candidate.semantic);
            if (!placement.valid) {
                break;
            }

            constraint.LabelPosition = static_cast<float>(DimensionGeometry::roundConstraintAngle(placement,
                                                                               candidate.labelPos));
            constraint.LabelDistance = static_cast<float>(DimensionGeometry::roundConstraintLabelDistance(placement,
                                                                                       candidate.labelPos));
            break;
        }
        case DimensionSemantic::Angle: {
            Base::Vector2d intersection(0.0, 0.0);
            if (!DimensionConstraintBuilderDetail::computeAngleIntersection(sketch, constraint, intersection)) {
                break;
            }

            const Base::Vector2d rel(candidate.labelPos.x - intersection.x,
                                     candidate.labelPos.y - intersection.y);
            const double radius = std::sqrt(rel.x * rel.x + rel.y * rel.y);
            constraint.LabelDistance = static_cast<float>(radius * 0.5);
            break;
        }
        default:
            break;
    }
}

const char* dimensionConstraintName(Sketcher::ConstraintType type)
{
    switch (type) {
        case Sketcher::Distance:
            return "Distance";
        case Sketcher::DistanceX:
            return "DistanceX";
        case Sketcher::DistanceY:
            return "DistanceY";
        case Sketcher::Radius:
            return "Radius";
        case Sketcher::Diameter:
            return "Diameter";
        case Sketcher::Angle:
            return "Angle";
        default:
            return nullptr;
    }
}

bool addBuiltDimensionConstraint(Sketcher::SketchObject& sketch,
                                 const Sketcher::Constraint& constraint)
{
    const char* name = dimensionConstraintName(constraint.Type);
    if (!name) {
        return false;
    }

    const double value = constraint.getValue();
    if (constraint.Type == Sketcher::Radius || constraint.Type == Sketcher::Diameter) {
        Gui::cmdAppObjectArgs(&sketch,
                              "addConstraint(Sketcher.Constraint('%s',%d,%.17g))",
                              name,
                              constraint.First,
                              value);
        return true;
    }

    if (constraint.Type == Sketcher::Angle) {
        if (constraint.Third != Sketcher::GeoEnum::GeoUndef) {
            Gui::cmdAppObjectArgs(&sketch,
                                  "addConstraint(Sketcher.Constraint('AngleViaPoint',%d,%d,%d,%d,%.17g))",
                                  constraint.First,
                                  constraint.Second,
                                  constraint.Third,
                                  static_cast<int>(constraint.ThirdPos),
                                  value);
            return true;
        }

        if (constraint.Second != Sketcher::GeoEnum::GeoUndef
            && constraint.Third == Sketcher::GeoEnum::GeoUndef) {
            Gui::cmdAppObjectArgs(&sketch,
                                  "addConstraint(Sketcher.Constraint('%s',%d,%d,%d,%d,%.17g))",
                                  name,
                                  constraint.First,
                                  static_cast<int>(constraint.FirstPos),
                                  constraint.Second,
                                  static_cast<int>(constraint.SecondPos),
                                  value);
            return true;
        }

        if (constraint.Second == Sketcher::GeoEnum::GeoUndef) {
            Gui::cmdAppObjectArgs(&sketch,
                                  "addConstraint(Sketcher.Constraint('%s',%d,%.17g))",
                                  name,
                                  constraint.First,
                                  value);
            return true;
        }

        return false;
    }

    if (constraint.Second == Sketcher::GeoEnum::GeoUndef) {
        if (constraint.FirstPos != Sketcher::PointPos::none) {
            Gui::cmdAppObjectArgs(&sketch,
                                  "addConstraint(Sketcher.Constraint('%s',%d,%d,%.17g))",
                                  name,
                                  constraint.First,
                                  static_cast<int>(constraint.FirstPos),
                                  value);
        }
        else {
            Gui::cmdAppObjectArgs(&sketch,
                                  "addConstraint(Sketcher.Constraint('%s',%d,%.17g))",
                                  name,
                                  constraint.First,
                                  value);
        }
        return true;
    }

    if (constraint.SecondPos != Sketcher::PointPos::none) {
        Gui::cmdAppObjectArgs(&sketch,
                              "addConstraint(Sketcher.Constraint('%s',%d,%d,%d,%d,%.17g))",
                              name,
                              constraint.First,
                              static_cast<int>(constraint.FirstPos),
                              constraint.Second,
                              static_cast<int>(constraint.SecondPos),
                              value);
        return true;
    }

    if (constraint.FirstPos != Sketcher::PointPos::none) {
        Gui::cmdAppObjectArgs(&sketch,
                              "addConstraint(Sketcher.Constraint('%s',%d,%d,%d,%.17g))",
                              name,
                              constraint.First,
                              static_cast<int>(constraint.FirstPos),
                              constraint.Second,
                              value);
        return true;
    }

    Gui::cmdAppObjectArgs(&sketch,
                          "addConstraint(Sketcher.Constraint('%s',%d,%d,%.17g))",
                          name,
                          constraint.First,
                          constraint.Second,
                          value);
    return true;
}


void finalizeCreatedDimension(Sketcher::SketchObject& sketch,
                              int constraintIndex,
                              DimensionDrivingPolicy drivingPolicy,
                              const std::function<void(int)>& moveConstraintFn);

int appendBuiltDimension(Sketcher::SketchObject& sketch,
                         std::unique_ptr<Sketcher::Constraint> constraint,
                         DimensionDrivingPolicy drivingPolicy,
                         const std::function<void(int)>& moveConstraintFn)
{
    if (!constraint) {
        return -1;
    }

    constraint->isDriving = DimensionConstraintBuilderDetail::isDrivingPolicy(drivingPolicy);
    if (!addBuiltDimensionConstraint(sketch, *constraint)) {
        return -1;
    }

    const int createdIndex = static_cast<int>(sketch.Constraints.getValues().size()) - 1;
    finalizeCreatedDimension(sketch, createdIndex, drivingPolicy, moveConstraintFn);
    return createdIndex;
}

DimensionCommitOptions buildDimensionCommitOptions(const Sketcher::SketchObject& sketch,
                                                   const DimensionCandidate& candidate,
                                                   DimensionDialogSelectionPolicy selectionPolicy)
{
    using namespace DimensionConstraintBuilderDetail;
    DimensionCommitOptions options;
    options.selectionPolicy = selectionPolicy;

    if (!candidate.refs.empty()
        && (candidate.semantic == DimensionSemantic::Radius
            || candidate.semantic == DimensionSemantic::Diameter)) {
        Base::Vector2d center;
        double radius = 0.0;
        if (roundRadiusCenter(sketch, candidate.refs.front().geoId, center, radius)) {
            const Base::Vector2d v(candidate.labelPos.x - center.x,
                                   candidate.labelPos.y - center.y);
            options.forcedLabelPosition = static_cast<float>(std::atan2(v.y, v.x));
        }
    }

    return options;
}

void finalizeCreatedDimension(Sketcher::SketchObject& sketch,
                              int constraintIndex,
                              DimensionDrivingPolicy drivingPolicy,
                              const std::function<void(int)>& moveConstraintFn)
{
    if (constraintIndex < 0) {
        return;
    }

    if (!DimensionConstraintBuilderDetail::isDrivingPolicy(drivingPolicy)) {
        Gui::cmdAppObjectArgs(&sketch, "setDriving(%d,%s)", constraintIndex, "False");
    }

    moveConstraintFn(constraintIndex);
}

int appendDimensionFromCandidate(Sketcher::SketchObject& sketch,
                                 const DimensionCandidate& candidate,
                                 DimensionDrivingPolicy drivingPolicy,
                                 const std::function<void(int)>& moveConstraintFn)
{
    using namespace DimensionConstraintBuilderDetail;
    return appendBuiltDimension(sketch,
                                buildDimensionConstraint(sketch, candidate),
                                drivingPolicy,
                                moveConstraintFn);
}

std::unique_ptr<Sketcher::Constraint> buildDimensionConstraint(
    const Sketcher::SketchObject& sketch,
    const DimensionCandidate& candidate)
{
    using namespace DimensionConstraintBuilderDetail;
    const auto type = constraintTypeForSemantic(candidate.semantic);
    if (!type || !isValidDimensionCandidate(candidate)) {
        assert(false && "invalid dimension candidate");
        return {};
    }

    auto constraint = std::make_unique<Sketcher::Constraint>();
    constraint->Type = *type;
    constraint->setValue(candidate.previewValue);
    constraint->isDriving = true;
    if (!candidate.refs.empty()) {
        constraint->First = candidate.refs[0].geoId;
        constraint->FirstPos = candidate.refs[0].posId;
    }
    if (candidate.refs.size() > 1) {
        constraint->Second = candidate.refs[1].geoId;
        constraint->SecondPos = candidate.refs[1].posId;
    }
    if (candidate.refs.size() > 2) {
        constraint->Third = candidate.refs[2].geoId;
        constraint->ThirdPos = candidate.refs[2].posId;
    }

    canonicalizeLinearConstraint(sketch, candidate, *constraint);
    applyPreparedPlacement(sketch, candidate, *constraint);
    return constraint;
}


} // namespace SketcherGui
