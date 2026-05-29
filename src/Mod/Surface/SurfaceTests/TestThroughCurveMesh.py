# SPDX-License-Identifier: LGPL-2.1-or-later

import math
import unittest

import FreeCAD
import Part
import Surface


class TestThroughCurveMesh(unittest.TestCase):
    def setUp(self):
        self.doc = FreeCAD.newDocument("SurfaceThroughCurveMeshTest")

    def tearDown(self):
        FreeCAD.closeDocument(self.doc.Name)

    def _edge_object(self, name, points):
        curve = Part.BSplineCurve()
        curve.interpolate(points)
        obj = self.doc.addObject("Part::Feature", name)
        obj.Shape = curve.toShape()
        return obj

    def _wire_object(self, name, points):
        obj = self.doc.addObject("Part::Feature", name)
        obj.Shape = Part.makePolygon(points)
        return obj

    def _z(self, x, y):
        return 0.15 * math.sin(x * 0.4) + 0.10 * math.cos(y * 0.35)

    def _build_mesh(self, primary, cross, tolerance=0.08, position_tolerance=0.25, samples=14, auto_sort=False, parameterization="ChordLength", construction="Approximation"):
        mesh = self.doc.addObject("Surface::ThroughCurveMesh", "ThroughCurveMesh")
        mesh.PrimaryCurves = [(obj, ["Edge1"]) for obj in primary]
        mesh.CrossCurves = [(obj, ["Edge1"]) for obj in cross]
        mesh.Tolerance = tolerance
        mesh.PositionTolerance = position_tolerance
        mesh.Samples = samples
        mesh.SamplesU = samples
        mesh.SamplesV = samples
        mesh.AutoSort = auto_sort
        mesh.Parameterization = parameterization
        mesh.Construction = construction
        mesh.SurfaceContinuity = "C2"
        mesh.BoundaryContinuity = "G0"
        mesh.MinDegree = 3
        mesh.MaxDegree = 5
        self.doc.recompute()
        return mesh

    def test_rectangular_curve_mesh_from_edges(self):
        xs = [0.0, 4.0, 8.0]
        ys = [0.0, 3.0, 6.0]

        primary = []
        for index, x in enumerate(xs):
            points = [FreeCAD.Vector(x, y, self._z(x, y)) for y in ys]
            primary.append(self._edge_object("Primary%d" % index, points))

        cross = []
        for index, y in enumerate(ys):
            points = [FreeCAD.Vector(x, y, self._z(x, y)) for x in xs]
            cross.append(self._edge_object("Cross%d" % index, points))

        mesh = self._build_mesh(primary, cross)

        self.assertFalse(mesh.Shape.isNull())
        self.assertGreater(len(mesh.Shape.Faces), 0)
        self.assertLess(mesh.MaxDeviation, 0.2)

    def test_shuffled_and_reversed_curve_families(self):
        xs = [0.0, 4.0, 8.0]
        ys = [0.0, 3.0, 6.0]

        primary = []
        for index, x in enumerate([8.0, 0.0, 4.0]):
            points = [FreeCAD.Vector(x, y, self._z(x, y)) for y in ys]
            if index == 0:
                points.reverse()
            primary.append(self._edge_object("PrimaryShuffled%d" % index, points))

        cross = []
        for index, y in enumerate([6.0, 0.0, 3.0]):
            points = [FreeCAD.Vector(x, y, self._z(x, y)) for x in xs]
            if index == 1:
                points.reverse()
            cross.append(self._edge_object("CrossShuffled%d" % index, points))

        mesh = self._build_mesh(primary, cross, auto_sort=True)

        self.assertFalse(mesh.Shape.isNull())
        self.assertGreater(len(mesh.Shape.Faces), 0)
        self.assertLess(mesh.MaxDeviation, 0.25)


    def test_manual_order_rejects_non_monotonic_curve_families(self):
        xs = [0.0, 4.0, 8.0]
        ys = [0.0, 3.0, 6.0]

        primary = []
        for index, x in enumerate([8.0, 0.0, 4.0]):
            points = [FreeCAD.Vector(x, y, self._z(x, y)) for y in ys]
            primary.append(self._edge_object("PrimaryManualOrder%d" % index, points))

        cross = []
        for index, y in enumerate(ys):
            points = [FreeCAD.Vector(x, y, self._z(x, y)) for x in xs]
            cross.append(self._edge_object("CrossManualOrder%d" % index, points))

        mesh = self._build_mesh(primary, cross, auto_sort=False)
        self.assertTrue(mesh.Shape.isNull())


    def test_emphasis_option_is_applied(self):
        xs = [0.0, 4.0, 8.0]
        ys = [0.0, 3.0, 6.0]

        primary = []
        for index, x in enumerate(xs):
            points = [FreeCAD.Vector(x, y, self._z(x, y)) for y in ys]
            primary.append(self._edge_object("EmphasisPrimary%d" % index, points))

        cross = []
        for index, y in enumerate(ys):
            points = [FreeCAD.Vector(x, y, self._z(x, y)) for x in xs]
            cross.append(self._edge_object("EmphasisCross%d" % index, points))

        mesh = self._build_mesh(primary, cross, position_tolerance=0.5)
        mesh.Emphasis = "Primary"
        self.doc.recompute()

        self.assertFalse(mesh.Shape.isNull())
        self.assertGreater(len(mesh.Shape.Faces), 0)

    def test_requires_two_curve_families(self):
        primary = self._edge_object(
            "PrimaryOnly",
            [FreeCAD.Vector(0, 0, 0), FreeCAD.Vector(0, 1, 0), FreeCAD.Vector(0, 2, 0)],
        )

        mesh = self.doc.addObject("Surface::ThroughCurveMesh", "InvalidThroughCurveMesh")
        mesh.PrimaryCurves = [(primary, ["Edge1"])]
        self.doc.recompute()

        self.assertTrue(mesh.Shape.isNull())

    def test_rejects_non_intersecting_network(self):
        primary = [
            self._edge_object("P0", [FreeCAD.Vector(0, 0, 0), FreeCAD.Vector(0, 5, 0)]),
            self._edge_object("P1", [FreeCAD.Vector(5, 0, 0), FreeCAD.Vector(5, 5, 0)]),
        ]
        cross = [
            self._edge_object("C0", [FreeCAD.Vector(10, 0, 0), FreeCAD.Vector(15, 0, 0)]),
            self._edge_object("C1", [FreeCAD.Vector(10, 5, 0), FreeCAD.Vector(15, 5, 0)]),
        ]

        mesh = self._build_mesh(primary, cross, tolerance=0.01)
        self.assertTrue(mesh.Shape.isNull())

    def test_rejects_duplicate_curve_in_both_families(self):
        shared = self._edge_object(
            "SharedCurve",
            [FreeCAD.Vector(0, 0, 0), FreeCAD.Vector(0, 2, 0)],
        )
        other_primary = self._edge_object(
            "OtherPrimary",
            [FreeCAD.Vector(2, 0, 0), FreeCAD.Vector(2, 2, 0)],
        )
        other_cross = self._edge_object(
            "OtherCross",
            [FreeCAD.Vector(0, 1, 0), FreeCAD.Vector(2, 1, 0)],
        )

        mesh = self.doc.addObject("Surface::ThroughCurveMesh", "DuplicateCurveMesh")
        mesh.PrimaryCurves = [(shared, ["Edge1"]), (other_primary, ["Edge1"])]
        mesh.CrossCurves = [(shared, ["Edge1"]), (other_cross, ["Edge1"])]
        self.doc.recompute()

        self.assertTrue(mesh.Shape.isNull())


    def test_accepts_connected_wire_selection_families(self):
        primary0 = self._wire_object(
            "FamilyPrimary0",
            [FreeCAD.Vector(0, 0, 0), FreeCAD.Vector(0, 1, 0.1), FreeCAD.Vector(0, 2, 0)],
        )
        primary1 = self._wire_object(
            "FamilyPrimary1",
            [FreeCAD.Vector(2, 0, 0), FreeCAD.Vector(2, 1, -0.1), FreeCAD.Vector(2, 2, 0)],
        )
        cross0 = self._edge_object(
            "FamilyCross0",
            [FreeCAD.Vector(0, 0.5, 0.05), FreeCAD.Vector(2, 0.5, -0.05)],
        )
        cross1 = self._edge_object(
            "FamilyCross1",
            [FreeCAD.Vector(0, 1.5, 0.05), FreeCAD.Vector(2, 1.5, -0.05)],
        )

        mesh = self.doc.addObject("Surface::ThroughCurveMesh", "SelectionFamilyMesh")
        mesh.PrimaryCurves = [(primary0, []), (primary1, [])]
        mesh.CrossCurves = [(cross0, ["Edge1"]), (cross1, ["Edge1"])]
        mesh.Tolerance = 0.05
        mesh.PositionTolerance = 0.3
        mesh.Samples = 14
        mesh.SamplesU = 14
        mesh.SamplesV = 14
        self.doc.recompute()

        self.assertFalse(mesh.Shape.isNull())
        self.assertGreater(len(mesh.Shape.Faces), 0)


    def test_accepts_multi_object_selection_family_rows(self):
        p0a = self._edge_object("FamilyObjP0a", [FreeCAD.Vector(0, 0, 0), FreeCAD.Vector(0, 1, 0.05)])
        p0b = self._edge_object("FamilyObjP0b", [FreeCAD.Vector(0, 1, 0.05), FreeCAD.Vector(0, 2, 0)])
        p1a = self._edge_object("FamilyObjP1a", [FreeCAD.Vector(2, 0, 0), FreeCAD.Vector(2, 1, -0.05)])
        p1b = self._edge_object("FamilyObjP1b", [FreeCAD.Vector(2, 1, -0.05), FreeCAD.Vector(2, 2, 0)])
        cross0 = self._edge_object("FamilyObjC0", [FreeCAD.Vector(0, 0.5, 0.03), FreeCAD.Vector(2, 0.5, -0.03)])
        cross1 = self._edge_object("FamilyObjC1", [FreeCAD.Vector(0, 1.5, 0.03), FreeCAD.Vector(2, 1.5, -0.03)])

        mesh = self.doc.addObject("Surface::ThroughCurveMesh", "MultiObjectFamilyMesh")
        mesh.PrimaryCurves = [
            (p0a, ["Edge1"]),
            (p0b, ["Edge1"]),
            (p1a, ["Edge1"]),
            (p1b, ["Edge1"]),
        ]
        mesh.PrimaryCurveGroupSizes = [2, 2]
        mesh.CrossCurves = [(cross0, ["Edge1"]), (cross1, ["Edge1"])]
        mesh.Tolerance = 0.05
        mesh.PositionTolerance = 0.3
        mesh.Samples = 14
        mesh.SamplesU = 14
        mesh.SamplesV = 14
        self.doc.recompute()

        self.assertFalse(mesh.Shape.isNull())
        self.assertGreater(len(mesh.Shape.Faces), 0)

    def test_rejects_self_reference(self):
        primary0 = self._edge_object("SelfP0", [FreeCAD.Vector(0, 0, 0), FreeCAD.Vector(0, 2, 0)])
        cross0 = self._edge_object("SelfC0", [FreeCAD.Vector(0, 1, 0), FreeCAD.Vector(2, 1, 0)])
        cross1 = self._edge_object("SelfC1", [FreeCAD.Vector(0, 2, 0), FreeCAD.Vector(2, 2, 0)])

        mesh = self.doc.addObject("Surface::ThroughCurveMesh", "SelfReferenceMesh")
        mesh.PrimaryCurves = [(primary0, ["Edge1"]), (mesh, ["Edge1"])]
        mesh.CrossCurves = [(cross0, ["Edge1"]), (cross1, ["Edge1"])]
        self.doc.recompute()

        self.assertTrue(mesh.Shape.isNull())

    def test_rejects_mixed_whole_object_and_subelement_reference(self):
        primary = self._edge_object(
            "MixedWholeAndSub",
            [FreeCAD.Vector(0, 0, 0), FreeCAD.Vector(0, 2, 0)],
        )
        other_primary = self._edge_object(
            "MixedOtherPrimary",
            [FreeCAD.Vector(2, 0, 0), FreeCAD.Vector(2, 2, 0)],
        )
        cross0 = self._edge_object("MixedCross0", [FreeCAD.Vector(0, 1, 0), FreeCAD.Vector(2, 1, 0)])
        cross1 = self._edge_object("MixedCross1", [FreeCAD.Vector(0, 1.5, 0), FreeCAD.Vector(2, 1.5, 0)])

        mesh = self.doc.addObject("Surface::ThroughCurveMesh", "MixedReferenceMesh")
        mesh.PrimaryCurves = [(primary, []), (primary, ["Edge1"]), (other_primary, ["Edge1"])]
        mesh.CrossCurves = [(cross0, ["Edge1"]), (cross1, ["Edge1"])]
        self.doc.recompute()

        self.assertTrue(mesh.Shape.isNull())

    def test_rejects_face_object_input(self):
        face = Part.makePlane(2, 2)
        face_obj = self.doc.addObject("Part::Feature", "FaceInput")
        face_obj.Shape = face

        cross0 = self._edge_object("FaceCross0", [FreeCAD.Vector(0, 0, 0), FreeCAD.Vector(2, 0, 0)])
        cross1 = self._edge_object("FaceCross1", [FreeCAD.Vector(0, 1, 0), FreeCAD.Vector(2, 1, 0)])

        mesh = self.doc.addObject("Surface::ThroughCurveMesh", "FaceRejectedMesh")
        mesh.PrimaryCurves = [(face_obj, [])]
        mesh.CrossCurves = [(cross0, ["Edge1"]), (cross1, ["Edge1"])]
        self.doc.recompute()

        self.assertTrue(mesh.Shape.isNull())

    def test_rejects_disconnected_whole_object_selection_family(self):
        edge0 = Part.makeLine(FreeCAD.Vector(0, 0, 0), FreeCAD.Vector(0, 2, 0))
        edge1 = Part.makeLine(FreeCAD.Vector(2, 0, 0), FreeCAD.Vector(2, 2, 0))
        multi = self.doc.addObject("Part::Feature", "MultiCurveWholeObject")
        multi.Shape = Part.makeCompound([edge0, edge1])

        cross0 = self._edge_object("WholeMultiCross0", [FreeCAD.Vector(0, 0.5, 0), FreeCAD.Vector(2, 0.5, 0)])
        cross1 = self._edge_object("WholeMultiCross1", [FreeCAD.Vector(0, 1.5, 0), FreeCAD.Vector(2, 1.5, 0)])

        mesh = self.doc.addObject("Surface::ThroughCurveMesh", "WholeObjectMultiCurveRejectedMesh")
        mesh.PrimaryCurves = [(multi, [])]
        mesh.CrossCurves = [(cross0, ["Edge1"]), (cross1, ["Edge1"])]
        self.doc.recompute()

        self.assertTrue(mesh.Shape.isNull())


    def test_intersection_and_fit_tolerances_are_independent(self):
        primary = [
            self._edge_object("TolP0", [FreeCAD.Vector(0, 0, 0), FreeCAD.Vector(0, 2, 0)]),
            self._edge_object("TolP1", [FreeCAD.Vector(2, 0, 0), FreeCAD.Vector(2, 2, 0)]),
        ]
        cross = [
            self._edge_object("TolC0", [FreeCAD.Vector(0.02, 0.5, 0), FreeCAD.Vector(2.02, 0.5, 0)]),
            self._edge_object("TolC1", [FreeCAD.Vector(0.02, 1.5, 0), FreeCAD.Vector(2.02, 1.5, 0)]),
        ]

        mesh = self._build_mesh(primary, cross, tolerance=0.05, position_tolerance=0.005, samples=10)

        self.assertFalse(mesh.Shape.isNull())
        self.assertLess(mesh.PositionTolerance, mesh.Tolerance)

    def test_directional_sample_counts_are_applied(self):
        xs = [0.0, 4.0, 8.0]
        ys = [0.0, 3.0, 6.0]

        primary = []
        for index, x in enumerate(xs):
            points = [FreeCAD.Vector(x, y, self._z(x, y)) for y in ys]
            primary.append(self._edge_object("SamplesPrimary%d" % index, points))

        cross = []
        for index, y in enumerate(ys):
            points = [FreeCAD.Vector(x, y, self._z(x, y)) for x in xs]
            cross.append(self._edge_object("SamplesCross%d" % index, points))

        mesh = self._build_mesh(primary, cross, samples=12)
        mesh.SamplesU = 9
        mesh.SamplesV = 17
        self.doc.recompute()

        self.assertFalse(mesh.Shape.isNull())
        self.assertEqual(mesh.SamplesU, 9)
        self.assertEqual(mesh.SamplesV, 17)


    def test_parameterization_modes_and_gap_diagnostics(self):
        xs = [0.0, 1.0, 4.0, 8.0]
        ys = [0.0, 0.8, 3.0, 6.0]

        primary = []
        for index, x in enumerate(xs):
            points = [FreeCAD.Vector(x, y, self._z(x, y)) for y in ys]
            primary.append(self._edge_object("ParamPrimary%d" % index, points))

        cross = []
        for index, y in enumerate(ys):
            points = [FreeCAD.Vector(x, y, self._z(x, y)) for x in xs]
            cross.append(self._edge_object("ParamCross%d" % index, points))

        for mode in ("ChordLength", "Centripetal", "Uniform"):
            mesh = self._build_mesh(primary, cross, samples=12, parameterization=mode)
            self.assertFalse(mesh.Shape.isNull())
            self.assertGreaterEqual(mesh.MaxIntersectionGap, 0.0)
            self.assertLessEqual(mesh.MaxIntersectionGap, mesh.Tolerance)
            self.assertEqual(mesh.Parameterization, mode)


    def test_occt_construction_modes_and_degree_controls(self):
        xs = [0.0, 4.0, 8.0]
        ys = [0.0, 3.0, 6.0]

        primary = []
        for index, x in enumerate(xs):
            points = [FreeCAD.Vector(x, y, self._z(x, y)) for y in ys]
            primary.append(self._edge_object("ConstructionPrimary%d" % index, points))

        cross = []
        for index, y in enumerate(ys):
            points = [FreeCAD.Vector(x, y, self._z(x, y)) for x in xs]
            cross.append(self._edge_object("ConstructionCross%d" % index, points))

        for mode in ("Approximation", "Interpolation", "Variational"):
            mesh = self._build_mesh(primary, cross, samples=12, construction=mode)
            mesh.MinDegree = 2
            mesh.MaxDegree = 4
            mesh.SurfaceContinuity = "C1"
            if mode == "Variational":
                mesh.SmoothLengthWeight = 0.01
                mesh.SmoothCurvatureWeight = 0.01
                mesh.SmoothTorsionWeight = 0.0
            self.doc.recompute()

            self.assertFalse(mesh.Shape.isNull())
            self.assertEqual(mesh.Construction, mode)
            self.assertEqual(mesh.MinDegree, 2)
            self.assertEqual(mesh.MaxDegree, 4)
            self.assertEqual(mesh.SurfaceContinuity, "C1")


    def test_endpoint_parameters_snap_to_curve_ends(self):
        primary = [
            self._edge_object("SnapP0", [FreeCAD.Vector(0, 0, 0), FreeCAD.Vector(0, 2, 0)]),
            self._edge_object("SnapP1", [FreeCAD.Vector(2, 0, 0), FreeCAD.Vector(2, 2, 0)]),
        ]
        cross = [
            self._edge_object("SnapC0", [FreeCAD.Vector(0, 1e-7, 0), FreeCAD.Vector(2, 1e-7, 0)]),
            self._edge_object("SnapC1", [FreeCAD.Vector(0, 1.9999999, 0), FreeCAD.Vector(2, 1.9999999, 0)]),
        ]

        mesh = self._build_mesh(primary, cross, tolerance=0.01, position_tolerance=0.05, samples=10)

        self.assertFalse(mesh.Shape.isNull())
        self.assertGreaterEqual(mesh.MaxIntersectionGap, 0.0)


    def test_g1_g2_boundary_continuity_requires_support_faces(self):
        primary = [
            self._edge_object("ContinuityP0", [FreeCAD.Vector(0, 0, 0), FreeCAD.Vector(0, 2, 0)]),
            self._edge_object("ContinuityP1", [FreeCAD.Vector(2, 0, 0), FreeCAD.Vector(2, 2, 0)]),
        ]
        cross = [
            self._edge_object("ContinuityC0", [FreeCAD.Vector(0, 0, 0), FreeCAD.Vector(2, 0, 0)]),
            self._edge_object("ContinuityC1", [FreeCAD.Vector(0, 2, 0), FreeCAD.Vector(2, 2, 0)]),
        ]

        mesh = self._build_mesh(primary, cross, tolerance=0.01, position_tolerance=0.05, samples=10)
        self.assertFalse(mesh.Shape.isNull())

        mesh.BoundaryContinuity = "G1"
        self.doc.recompute()

        self.assertTrue(mesh.Shape.isNull())
        self.assertEqual(mesh.BoundaryContinuity, "G1")

        mesh.BoundaryContinuity = "G0"
        self.doc.recompute()

        self.assertFalse(mesh.Shape.isNull())
        self.assertEqual(mesh.BoundaryContinuity, "G0")


    def test_rejects_multiple_intersections(self):
        primary = [
            self._edge_object("MultiP0", [FreeCAD.Vector(0, 0, 0), FreeCAD.Vector(0, 2, 0)]),
            self._edge_object("MultiP1", [FreeCAD.Vector(2, 0, 0), FreeCAD.Vector(2, 2, 0)]),
        ]

        # Cross0 deliberately crosses primary0 twice. A through-curve mesh cell
        # needs one unambiguous intersection for every primary/cross pair.
        cross0 = self._wire_object(
            "MultiC0",
            [
                FreeCAD.Vector(-0.5, 0.5, 0),
                FreeCAD.Vector(0.5, 0.5, 0),
                FreeCAD.Vector(-0.5, 1.5, 0),
                FreeCAD.Vector(2.5, 1.5, 0),
            ],
        )
        cross1 = self._edge_object(
            "MultiC1",
            [FreeCAD.Vector(-0.5, 1.8, 0), FreeCAD.Vector(2.5, 1.8, 0)],
        )

        mesh = self.doc.addObject("Surface::ThroughCurveMesh", "MultipleIntersectionMesh")
        mesh.PrimaryCurves = [(obj, ["Edge1"]) for obj in primary]
        mesh.CrossCurves = [(cross0, []), (cross1, ["Edge1"])]
        mesh.Tolerance = 0.02
        mesh.PositionTolerance = 0.25
        mesh.Samples = 14
        mesh.SamplesU = 14
        mesh.SamplesV = 14
        self.doc.recompute()

        self.assertTrue(mesh.Shape.isNull())
