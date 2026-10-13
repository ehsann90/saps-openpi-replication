"""Offline checks for the read-only HG1 Fusion face exporter."""

import importlib.util
import csv
import json
import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch


SCRIPT = Path(__file__).resolve().parents[2] / "tools/hg1_fusion_face_export.py"
SPEC = importlib.util.spec_from_file_location("hg1_fusion_face_export", SCRIPT)
EXPORT = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(EXPORT)


class Point:
    def __init__(self, x, y, z):
        self.x, self.y, self.z = x, y, z


class Matrix:
    def __init__(self, rows):
        self.rows = rows

    def getCell(self, row, column):
        return self.rows[row][column]


class Face:
    @staticmethod
    def cast(face):
        return face if isinstance(face, Face) else None

    def __init__(self, native=None, occurrence=None, normal=(0, 0, 1),
                 point=(1, 2, 3)):
        self.nativeObject = native
        self.assemblyContext = occurrence
        self.geometry = SimpleNamespace(objectType="adsk::core::Plane")
        self.body = SimpleNamespace(name="pad", isSolid=True)
        self.pointOnFace = Point(*point)
        self.evaluator = SimpleNamespace(
            getNormalAtPoint=lambda point: (True, Point(*normal)))
        self.entityToken = "face-token"
        self.tempId = 7
        self.area = 2.5
        self.objectType = "adsk::fusion::BRepFace"

    def isPointOnFace(self, point):
        return (abs(point.z - self.pointOnFace.z) < 1e-8
                and 0 <= point.x <= 10 and 0 <= point.y <= 10)


class TestHg1FusionFaceExport(unittest.TestCase):
    def setUp(self):
        self.rows = [[0, -1, 0, 4], [1, 0, 0, 5],
                     [0, 0, 1, 6], [0, 0, 0, 1]]

    def face_pair(self, rows=None, proxy_normal=(0, 0, 1)):
        native = Face()
        occurrence = SimpleNamespace(
            transform2=Matrix(rows or self.rows), fullPathName="root+pad:1",
            component=SimpleNamespace(name="pad_component"))
        proxy = Face(native=native, occurrence=occurrence,
                     normal=proxy_normal, point=(3, 7, 9))
        adsk = SimpleNamespace(
            fusion=SimpleNamespace(BRepFace=Face),
            core=SimpleNamespace(
                Plane=SimpleNamespace(cast=lambda geometry: geometry),
                Point3D=SimpleNamespace(create=Point)))
        return proxy, adsk

    def test_rigid_point_normal_and_unit_conversion(self):
        EXPORT.validate_rigid_matrix(self.rows)
        self.assertEqual(EXPORT.transform_point(self.rows, [1, 2, 3]), [2, 6, 9])
        self.assertEqual(EXPORT.transform_normal(self.rows, [1, 0, 0]), [0, 1, 0])
        self.assertEqual(EXPORT.inverse_point(self.rows, [2, 6, 9]), [1, 2, 3])
        self.assertEqual(EXPORT.mm_matrix(self.rows)[0][3], 40)

    def test_rejects_nonrigid_and_nonunit_normal(self):
        scale = [row[:] for row in self.rows]
        scale[0][0] = 2
        with self.assertRaisesRegex(ValueError, "orthonormal"):
            EXPORT.validate_rigid_matrix(scale)
        with self.assertRaisesRegex(ValueError, "unit normal"):
            EXPORT.unit_normal([0, 0, 2])

    def test_distinct_native_and_proxy_samples_on_same_face(self):
        proxy, adsk = self.face_pair()
        row = EXPORT.record_face("left_pad_contact", proxy, adsk)
        self.assertEqual(row["point_component_mm"], [10, 20, 30])
        self.assertEqual(row["point_root_mm"], [20, 60, 90])
        self.assertEqual(row["point_proxy_assembly_mm"], [30, 70, 90])
        self.assertEqual(row["outward_normal_root_unit"], [0, 0, 1])
        self.assertIn("not a centroid", row["point_kind"])
        self.assertEqual(row["T_root_from_component"]["matrix_mm"][0][3], 40)
        self.assertEqual(row["validation"]["status"], "qualified")
        self.assertEqual(row["validation"]["signed_root_plane_residual_cm"], 0)

    def test_face_rejects_wrong_proxy_normal(self):
        proxy, adsk = self.face_pair(proxy_normal=(0, 0, -1))
        with self.assertRaisesRegex(ValueError, "normal_max_component_error"):
            EXPORT.record_face("left_pad_contact", proxy, adsk)

    def test_incorrect_transform_reports_identity_points_and_residual(self):
        wrong = [row[:] for row in self.rows]
        wrong[2][3] = 7
        proxy, adsk = self.face_pair(rows=wrong)
        with self.assertRaisesRegex(ValueError, "signed_root_plane_residual_cm") as error:
            EXPORT.record_face("fr3_flange_mating", proxy, adsk)
        message = str(error.exception)
        self.assertIn("root+pad:1", message)
        self.assertIn("face-token", message)
        self.assertIn("native_sample_component_cm", message)
        self.assertIn("transformed_native_sample_assembly_cm", message)

    def test_containment_discrepancy_is_exported_unqualified(self):
        proxy, adsk = self.face_pair()
        proxy.isPointOnFace = lambda point: (point.x, point.y, point.z) == (3, 7, 9)
        row = EXPORT.record_face("fr3_flange_mating", proxy, adsk)
        validation = row["validation"]
        self.assertTrue(validation["plane_and_normal_qualified"])
        self.assertFalse(validation["transformed_native_on_proxy_trim"])
        self.assertFalse(validation["trim_containment_qualified"])
        self.assertEqual(validation["status"],
                         "plane_qualified_trim_containment_unresolved")

    def test_in_plane_wrong_transform_remains_unqualified(self):
        wrong = [row[:] for row in self.rows]
        wrong[0][3] = 20
        proxy, adsk = self.face_pair(rows=wrong)
        validation = EXPORT.record_face("fr3_flange_mating", proxy, adsk)["validation"]
        self.assertEqual(validation["signed_root_plane_residual_cm"], 0)
        self.assertFalse(validation["transformed_native_on_proxy_trim"])
        self.assertFalse(validation["inverse_proxy_on_native_trim"])
        self.assertEqual(validation["status"],
                         "plane_qualified_trim_containment_unresolved")

    def test_json_csv_written_only_to_new_stem(self):
        proxy, adsk = self.face_pair()
        row = EXPORT.record_face("left_pad_contact", proxy, adsk)
        with tempfile.TemporaryDirectory() as temp:
            with patch.object(EXPORT, "OUTPUT_DIR", Path(temp)):
                json_path, csv_path = EXPORT.write_export("FR3_2F85_ZED", [row])
            self.assertTrue(json_path.exists())
            self.assertTrue(csv_path.exists())
            payload = json.loads(json_path.read_text())
            self.assertEqual(payload["schema"], "fusion_cad_planar_face_audit_v2")
            self.assertEqual(payload["faces"][0]["point_proxy_assembly_cm"],
                             [3, 7, 9])
            with csv_path.open(newline="") as stream:
                written = next(csv.DictReader(stream))
            self.assertEqual(written["validation_status"], "qualified")
            self.assertEqual(json.loads(written["point_proxy_assembly_cm"]),
                             [3, 7, 9])


if __name__ == "__main__":
    unittest.main()
