"""Tests for the hand-derived FR3 forward kinematics and Jacobian."""

from __future__ import annotations

import unittest

import numpy as np

from saps.physical.fr3_forward_kinematics import fr3_flange_fk
from saps.physical.fr3_forward_kinematics import fr3_joint_transforms
from saps.physical.fr3_forward_kinematics import fr3_tcp_fk
from saps.physical.fr3_forward_kinematics import fr3_tcp_jacobian


class Fr3ForwardKinematicsTest(unittest.TestCase):
    def test_fk_returns_proper_homogeneous_transforms(self) -> None:
        q = np.linspace(-0.3, 0.3, 7, dtype=np.float64)

        transforms = (
            *fr3_joint_transforms(q),
            fr3_flange_fk(q),
            fr3_tcp_fk(q),
        )

        for transform in transforms:
            with self.subTest(transform=transform):
                self.assertEqual(transform.shape, (4, 4))
                np.testing.assert_allclose(
                    transform[3],
                    [0.0, 0.0, 0.0, 1.0],
                )
                rotation = transform[:3, :3]
                np.testing.assert_allclose(
                    rotation.T @ rotation,
                    np.eye(3),
                    atol=1e-12,
                )
                self.assertAlmostEqual(float(np.linalg.det(rotation)), 1.0)

    def test_tcp_is_1034_mm_from_flange_origin(self) -> None:
        q = np.zeros(7, dtype=np.float64)
        flange = fr3_flange_fk(q)
        tcp = fr3_tcp_fk(q)

        self.assertAlmostEqual(
            float(np.linalg.norm(tcp[:3, 3] - flange[:3, 3])),
            0.1034,
        )

    def test_analytical_jacobian_matches_centered_finite_difference(self) -> None:
        q = np.asarray(
            [-0.2, 0.1, 0.3, -1.7, 0.2, 1.2, -0.4],
            dtype=np.float64,
        )
        epsilon = 1e-6
        analytical = fr3_tcp_jacobian(q)
        initial_rotation = fr3_tcp_fk(q)[:3, :3]
        numeric = np.zeros((6, 7), dtype=np.float64)
        for index in range(7):
            q_plus = q.copy()
            q_minus = q.copy()
            q_plus[index] += epsilon
            q_minus[index] -= epsilon
            plus = fr3_tcp_fk(q_plus)
            minus = fr3_tcp_fk(q_minus)
            numeric[:3, index] = (
                plus[:3, 3] - minus[:3, 3]
            ) / (2.0 * epsilon)
            rotation_derivative = (
                plus[:3, :3] - minus[:3, :3]
            ) / (2.0 * epsilon)
            skew = rotation_derivative @ initial_rotation.T
            skew = 0.5 * (skew - skew.T)
            numeric[3:, index] = [skew[2, 1], skew[0, 2], skew[1, 0]]

        np.testing.assert_allclose(analytical, numeric, atol=1e-8)

    def test_invalid_joint_vectors_are_rejected(self) -> None:
        with self.assertRaisesRegex(TypeError, "floating dtype"):
            fr3_tcp_fk(np.zeros(7, dtype=np.int64))
        with self.assertRaisesRegex(ValueError, "shape"):
            fr3_tcp_fk(np.zeros(6, dtype=np.float64))
        q = np.zeros(7, dtype=np.float64)
        q[3] = np.nan
        with self.assertRaisesRegex(ValueError, "finite"):
            fr3_tcp_jacobian(q)


if __name__ == "__main__":
    unittest.main()
