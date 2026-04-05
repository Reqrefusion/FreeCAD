// SPDX-License-Identifier: LGPL-2.1-or-later
#pragma once

#include <cstddef>

namespace SketcherGui {

class GeomSelectionSizes
{
public:
    GeomSelectionSizes(std::size_t s_pts,
                       std::size_t s_lns,
                       std::size_t s_cir,
                       std::size_t s_ell,
                       std::size_t s_spl)
        : s_pts(s_pts)
        , s_lns(s_lns)
        , s_cir(s_cir)
        , s_ell(s_ell)
        , s_spl(s_spl)
    {}

    bool hasPoints() const { return s_pts > 0; }
    bool hasLines() const { return s_lns > 0; }
    bool hasCirclesOrArcs() const { return s_cir > 0; }
    bool hasEllipseAndCo() const { return s_ell > 0; }
    bool hasSplineAndCo() const { return s_spl > 0; }

    bool has1Point() const { return s_pts == 1 && s_lns == 0 && s_cir == 0 && s_ell == 0 && s_spl == 0; }
    bool has2Points() const { return s_pts == 2 && s_lns == 0 && s_cir == 0 && s_ell == 0 && s_spl == 0; }
    bool has1Point1Line() const { return s_pts == 1 && s_lns == 1 && s_cir == 0 && s_ell == 0 && s_spl == 0; }
    bool has3Points() const { return s_pts == 3 && s_lns == 0 && s_cir == 0 && s_ell == 0 && s_spl == 0; }
    bool has4MorePoints() const { return s_pts >= 4 && s_lns == 0 && s_cir == 0 && s_ell == 0 && s_spl == 0; }
    bool has2Points1Line() const { return s_pts == 2 && s_lns == 1 && s_cir == 0 && s_ell == 0 && s_spl == 0; }
    bool has3MorePoints1Line() const { return s_pts >= 3 && s_lns == 1 && s_cir == 0 && s_ell == 0 && s_spl == 0; }
    bool has1Point1Circle() const { return s_pts == 1 && s_lns == 0 && s_cir == 1 && s_ell == 0 && s_spl == 0; }
    bool has1MorePoint1Ellipse() const { return s_pts >= 1 && s_lns == 0 && s_cir == 0 && s_ell == 1 && s_spl == 0; }

    bool has1Line() const { return s_pts == 0 && s_lns == 1 && s_cir == 0 && s_ell == 0 && s_spl == 0; }
    bool has2Lines() const { return s_pts == 0 && s_lns == 2 && s_cir == 0 && s_ell == 0 && s_spl == 0; }
    bool has3MoreLines() const { return s_pts == 0 && s_lns >= 3 && s_cir == 0 && s_ell == 0 && s_spl == 0; }
    bool has1Line1Circle() const { return s_pts == 0 && s_lns == 1 && s_cir == 1 && s_ell == 0 && s_spl == 0; }
    bool has1Line2Circles() const { return s_pts == 0 && s_lns == 1 && s_cir == 2 && s_ell == 0 && s_spl == 0; }
    bool has1Line1Ellipse() const { return s_pts == 0 && s_lns == 1 && s_cir == 0 && s_ell == 1 && s_spl == 0; }

    bool has1Circle() const { return s_pts == 0 && s_lns == 0 && s_cir == 1 && s_ell == 0 && s_spl == 0; }
    bool has2Circles() const { return s_pts == 0 && s_lns == 0 && s_cir == 2 && s_ell == 0 && s_spl == 0; }
    bool has3MoreCircles() const { return s_pts == 0 && s_lns == 0 && s_cir >= 3 && s_ell == 0 && s_spl == 0; }
    bool has1Circle1Ellipse() const { return s_pts == 0 && s_lns == 0 && s_cir == 1 && s_ell == 1 && s_spl == 0; }

    bool has1Ellipse() const { return s_pts == 0 && s_lns == 0 && s_cir == 0 && s_ell == 1 && s_spl == 0; }
    bool has2MoreEllipses() const { return s_pts == 0 && s_lns == 0 && s_cir == 0 && s_ell >= 2 && s_spl == 0; }
    bool has1Point1Spline1MoreEdge() const { return s_pts == 1 && s_spl >= 1 && (s_lns + s_cir + s_ell + s_spl) == 2; }

    std::size_t s_pts, s_lns, s_cir, s_ell, s_spl;
};

enum class BasicDimensionSelectionPattern
{
    Unsupported,
    OnePoint,
    TwoPoints,
    OnePointOneLine,
    OnePointOneCircle,
    OneLine,
    TwoLines,
    OneLineOneCircle,
    OneCircle,
    TwoCircles,
};

inline BasicDimensionSelectionPattern classifyBasicDimensionSelection(const GeomSelectionSizes& selection)
{
    if (selection.has1Point()) {
        return BasicDimensionSelectionPattern::OnePoint;
    }
    if (selection.has2Points()) {
        return BasicDimensionSelectionPattern::TwoPoints;
    }
    if (selection.has1Point1Line()) {
        return BasicDimensionSelectionPattern::OnePointOneLine;
    }
    if (selection.has1Point1Circle()) {
        return BasicDimensionSelectionPattern::OnePointOneCircle;
    }
    if (selection.has1Line()) {
        return BasicDimensionSelectionPattern::OneLine;
    }
    if (selection.has2Lines()) {
        return BasicDimensionSelectionPattern::TwoLines;
    }
    if (selection.has1Line1Circle()) {
        return BasicDimensionSelectionPattern::OneLineOneCircle;
    }
    if (selection.has1Circle()) {
        return BasicDimensionSelectionPattern::OneCircle;
    }
    if (selection.has2Circles()) {
        return BasicDimensionSelectionPattern::TwoCircles;
    }

    return BasicDimensionSelectionPattern::Unsupported;
}

} // namespace SketcherGui
