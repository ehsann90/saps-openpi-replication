"""Tests for pure DROID-to-FR3 embodiment mathematics."""

from __future__ import annotations

import unittest

import numpy as np

from saps.physical.embodiment import CartesianNormalization
from saps.physical.embodiment import diagnose_jacobian
from saps.physical.embodiment import DroidJointActionSemantics
from saps.physical.embodiment import DroidToFr3TaskSpaceAdapter
from saps.physical.embodiment import FR3_JOINT_NAMES
from saps.physical.embodiment import validate_gripper_closure


class SyntheticJacobianProvider:
    joint_names = FR3_JOINT_NAMES
    base_frame = "fr3_link0"
    end_effector_frame = "fr3_hand_tcp"

    def __init__(self, jacobian: np.ndarray) -> None:
        self.matrix = jacobian
        self.queries: list[np.ndarray] = []

    def jacobian(self, joint_position: np.ndarray) -> np.ndarray:
        self.queries.append(np.array(joint_position, copy=True))
        return self.matrix


def full_rank_jacobian() -> np.ndarray:
    matrix = np.zeros((6, 7), dtype=np.float64)
    matrix[:, :6] = np.eye(6, dtype=np.float64)
    return matrix


def valid_policy_action() -> np.ndarray:
    return np.asarray(
        [0.1, -0.2, 0.3, -0.4, 0.5, -0.6, 0.7, 0.25],
        dtype=np.float64,
    )


class DroidJointActionSemanticsTest(unittest.TestCase):
    def setUp(self) -> None:
        self.semantics = DroidJointActionSemantics()

    def test_relative_vector_below_limit_is_unchanged(self) -> None:
        native = np.asarray(
            [0.1, -0.4, 0.25, 0.0, 0.7, -0.9, 1.0],
            dtype=np.float64,
        )

        transformed = self.semantics.transform(native)

        self.assertEqual(transformed.clipping_scale, 1.0)
        np.testing.assert_array_equal(transformed.clipped, native)

    def test_relative_vector_above_limit_uses_one_common_scale(self) -> None:
        native = np.asarray(
            [0.5, -2.0, 1.0, 0.0, -0.25, 1.5, -0.5],
            dtype=np.float64,
        )

        transformed = self.semantics.transform(native)

        self.assertEqual(transformed.clipping_scale, 2.0)
        np.testing.assert_allclose(transformed.clipped, native / 2.0)

    def test_vector_clipping_preserves_direction(self) -> None:
        native = np.asarray(
            [2.0, -1.0, 0.5, 0.25, -0.5, 1.0, -1.5],
            dtype=np.float64,
        )

        clipped = self.semantics.transform(native).clipped

        np.testing.assert_allclose(
            clipped / np.linalg.norm(clipped),
            native / np.linalg.norm(native),
        )

    def test_delta_and_nominal_qdot_follow_droid_scales(self) -> None:
        native = np.full(7, 0.5, dtype=np.float64)

        transformed = self.semantics.transform(native)

        np.testing.assert_allclose(transformed.delta_q_rad, 0.1)
        np.testing.assert_allclose(
            transformed.nominal_qdot_rad_s,
            1.5,
        )

    def test_invalid_or_nonfinite_native_action_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "shape"):
            self.semantics.transform(np.zeros(6, dtype=np.float64))
        for invalid in (np.nan, np.inf, -np.inf):
            with self.subTest(invalid=invalid):
                native = np.zeros(7, dtype=np.float64)
                native[2] = invalid
                with self.assertRaisesRegex(ValueError, "finite"):
                    self.semantics.transform(native)


class DroidToFr3TaskSpaceAdapterTest(unittest.TestCase):
    def test_correct_six_by_seven_jacobian_multiplication(self) -> None:
        jacobian = np.arange(42, dtype=np.float64).reshape(6, 7) / 10
        provider = SyntheticJacobianProvider(jacobian)
        adapter = DroidToFr3TaskSpaceAdapter(provider)
        action = valid_policy_action()

        projected = adapter.project(action, np.zeros(7, dtype=np.float64))

        expected_qdot = 3.0 * action[:7]
        np.testing.assert_allclose(
            projected.twist_si,
            jacobian @ expected_qdot,
        )
        np.testing.assert_allclose(
            projected.delta_x_linearized,
            jacobian @ (0.2 * action[:7]),
        )

    def test_invalid_jacobian_shape_is_rejected(self) -> None:
        adapter = DroidToFr3TaskSpaceAdapter(
            SyntheticJacobianProvider(np.zeros((7, 6), dtype=np.float64))
        )

        with self.assertRaisesRegex(ValueError, r"shape \(6, 7\)"):
            adapter.project(
                valid_policy_action(),
                np.zeros(7, dtype=np.float64),
            )

    def test_invalid_or_nonfinite_q_is_rejected(self) -> None:
        adapter = DroidToFr3TaskSpaceAdapter(
            SyntheticJacobianProvider(full_rank_jacobian())
        )
        with self.assertRaisesRegex(ValueError, "shape"):
            adapter.project(
                valid_policy_action(),
                np.zeros(6, dtype=np.float64),
            )
        invalid_q = np.zeros(7, dtype=np.float64)
        invalid_q[0] = np.nan
        with self.assertRaisesRegex(ValueError, "finite"):
            adapter.project(valid_policy_action(), invalid_q)

    def test_joint_order_is_validated(self) -> None:
        provider = SyntheticJacobianProvider(full_rank_jacobian())
        provider.joint_names = tuple(reversed(FR3_JOINT_NAMES))

        with self.assertRaisesRegex(ValueError, "ordered"):
            DroidToFr3TaskSpaceAdapter(provider)

    def test_motion_and_gripper_remain_separate(self) -> None:
        adapter = DroidToFr3TaskSpaceAdapter(
            SyntheticJacobianProvider(full_rank_jacobian())
        )
        first = valid_policy_action()
        second = first.copy()
        second[-1] = 0.9

        projected_first = adapter.project(
            first,
            np.zeros(7, dtype=np.float64),
        )
        projected_second = adapter.project(
            second,
            np.zeros(7, dtype=np.float64),
        )

        np.testing.assert_array_equal(
            projected_first.twist_si,
            projected_second.twist_si,
        )
        self.assertEqual(projected_first.gripper_closure, 0.25)
        self.assertEqual(projected_second.gripper_closure, 0.9)

    def test_projection_queries_jacobian_for_each_current_state(self) -> None:
        provider = SyntheticJacobianProvider(full_rank_jacobian())
        adapter = DroidToFr3TaskSpaceAdapter(provider)
        first_q = np.zeros(7, dtype=np.float64)
        second_q = np.ones(7, dtype=np.float64)

        adapter.project(valid_policy_action(), first_q)
        adapter.project(valid_policy_action(), second_q)

        self.assertEqual(len(provider.queries), 2)
        np.testing.assert_array_equal(provider.queries[0], first_q)
        np.testing.assert_array_equal(provider.queries[1], second_q)

    def test_projection_is_deterministic(self) -> None:
        adapter = DroidToFr3TaskSpaceAdapter(
            SyntheticJacobianProvider(full_rank_jacobian()),
            normalization=CartesianNormalization(),
        )
        q = np.linspace(-0.3, 0.3, 7, dtype=np.float64)

        first = adapter.project(valid_policy_action(), q)
        second = adapter.project(valid_policy_action(), q)

        np.testing.assert_array_equal(first.jacobian, second.jacobian)
        np.testing.assert_array_equal(first.twist_si, second.twist_si)
        np.testing.assert_array_equal(
            first.normalized_motion,
            second.normalized_motion,
        )
        np.testing.assert_array_equal(
            first.null_space_diagnostic.null_qdot_rad_s,
            second.null_space_diagnostic.null_qdot_rad_s,
        )


class CartesianRepresentationTest(unittest.TestCase):
    def test_canonical_gripper_boundaries_and_range(self) -> None:
        self.assertEqual(validate_gripper_closure(0.0), 0.0)
        self.assertEqual(validate_gripper_closure(1.0), 1.0)
        for invalid in (-0.01, 1.01, np.nan):
            with self.subTest(invalid=invalid):
                with self.assertRaises(ValueError):
                    validate_gripper_closure(invalid)

    def test_normalization_round_trip(self) -> None:
        normalization = CartesianNormalization()
        physical = np.asarray(
            [0.075, -0.0375, 0.0, 0.15, -0.3, 0.075],
            dtype=np.float64,
        )

        normalized = normalization.normalize_step(physical)
        recovered = normalization.denormalize_step(normalized)

        np.testing.assert_allclose(recovered, physical)

    def test_normalization_does_not_clip_above_unit_range(self) -> None:
        normalization = CartesianNormalization()
        physical = np.asarray(
            [0.15, 0.0, 0.0, 0.0, -0.45, 0.0],
            dtype=np.float64,
        )

        normalized = normalization.normalize_step(physical)

        self.assertEqual(normalized[0], 2.0)
        self.assertEqual(normalized[4], -3.0)

    def test_singular_and_near_singular_diagnostics_are_finite_safe(
        self,
    ) -> None:
        singular = full_rank_jacobian()
        singular[5, 5] = 1e-14

        diagnostic = diagnose_jacobian(singular)

        self.assertTrue(diagnostic.near_singular)
        self.assertEqual(diagnostic.rank, 5)
        self.assertTrue(np.isinf(diagnostic.condition_number))


if __name__ == "__main__":
    unittest.main()
