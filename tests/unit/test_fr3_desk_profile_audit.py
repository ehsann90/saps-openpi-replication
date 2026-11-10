"""Independent rigid-body examples for the offline physical tool calculation."""
import importlib.util
from pathlib import Path
import unittest

import numpy as np

PATH = Path(__file__).resolve().parents[2] / 'tools/analysis/audit_fr3_desk_profile.py'
SPEC = importlib.util.spec_from_file_location('desk_audit', PATH)
audit = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(audit)


def body(mass, center, inertia=None, transform=None):
    return dict(mass=mass, com=center,
                inertia=np.eye(3) if inertia is None else inertia,
                transform=np.eye(4) if transform is None else transform)


class ToolCalculationTests(unittest.TestCase):
    def test_mass_weighted_center_and_parallel_axis_shift(self):
        result = audit.combine([body(1, [-1, 0, 0]), body(3, [1, 0, 0])])
        self.assertEqual(result['mass_kg'], 4)
        np.testing.assert_allclose(result['flange_com_m'], [.5, 0, 0])
        np.testing.assert_allclose(result['com_inertia_flange_axes_kg_m2'], np.diag([2, 5, 5]))

    def test_rotation_and_translation_of_single_body(self):
        t = np.array([[0, -1, 0, 1], [1, 0, 0, 2],
                      [0, 0, 1, 3], [0, 0, 0, 1]])
        result = audit.combine([body(2, [1, 0, 0], np.diag([1, 2, 2.5]), t)])
        np.testing.assert_allclose(result['flange_com_m'], [1, 3, 3])
        np.testing.assert_allclose(result['com_inertia_flange_axes_kg_m2'], np.diag([2, 1, 2.5]))

    def test_missing_physical_properties_are_not_zero_filled(self):
        for key in ('mass', 'com', 'inertia', 'transform'):
            sample = body(1, [0, 0, 0])
            sample[key] = None
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, 'Incomplete'):
                audit.combine([sample])

    def test_nonphysical_inertias_are_rejected(self):
        for tensor in (np.diag([-1, 1, 1]), np.diag([1, 1, 3]),
                       [[1, .1, 0], [0, 1, 0], [0, 0, 1]],
                       np.eye(3) * np.nan):
            with self.subTest(tensor=tensor), self.assertRaises(ValueError):
                audit.inertia_check(tensor)

    def test_reflected_or_nonrigid_frames_are_rejected(self):
        for t in (np.diag([-1, 1, 1, 1]), np.diag([2, 1, 1, 1])):
            with self.assertRaises(ValueError):
                audit.rigid_transform(t)


if __name__ == '__main__':
    unittest.main()
