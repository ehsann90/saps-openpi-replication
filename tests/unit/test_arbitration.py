"""Tests for autonomous and takeover arbitration."""

from __future__ import annotations

import unittest

import numpy as np

from saps.arbitration import ActionArbitrator
from saps.arbitration import ActivityState
from saps.arbitration import ArbitrationMode


class ActionArbitratorTest(unittest.TestCase):
    def setUp(self) -> None:
        self.autonomous_action = np.asarray(
            [
                0.10,
                -0.20,
                0.30,
                -0.40,
                0.50,
                -0.60,
                -1.0,
            ],
            dtype=np.float32,
        )

    def test_autonomous_executes_policy_action_exactly(
        self,
    ) -> None:
        human_action = np.asarray(
            [
                -0.25,
                0.10,
                0.05,
                0.20,
                -0.10,
                0.15,
                1.0,
            ],
            dtype=np.float32,
        )

        result = ActionArbitrator(
            ArbitrationMode.AUTONOMOUS
        ).arbitrate(
            autonomous_action=self.autonomous_action,
            human_action=human_action,
        )

        np.testing.assert_array_equal(
            result.executed_action,
            self.autonomous_action,
        )
        self.assertEqual(
            result.arbitration_mode,
            ArbitrationMode.AUTONOMOUS,
        )
        self.assertEqual(
            result.activity_state,
            ActivityState.ACTIVE,
        )
        self.assertEqual(result.autonomy_weight, 1.0)

    def test_takeover_idle_uses_autonomous_motion(
        self,
    ) -> None:
        human_action = np.asarray(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0],
            dtype=np.float32,
        )

        result = ActionArbitrator(
            "takeover"
        ).arbitrate(
            autonomous_action=self.autonomous_action,
            human_action=human_action,
        )

        np.testing.assert_array_equal(
            result.executed_action,
            self.autonomous_action,
        )
        self.assertFalse(result.human_active)
        self.assertEqual(result.autonomy_weight, 1.0)

    def test_takeover_active_uses_human_motion(
        self,
    ) -> None:
        human_action = np.asarray(
            [
                -0.25,
                0.10,
                0.05,
                0.20,
                -0.10,
                0.15,
                -1.0,
            ],
            dtype=np.float32,
        )
        autonomous_action = self.autonomous_action.copy()
        autonomous_action[6] = 1.0

        result = ActionArbitrator(
            ArbitrationMode.TAKEOVER
        ).arbitrate(
            autonomous_action=autonomous_action,
            human_action=human_action,
        )

        np.testing.assert_array_equal(
            result.executed_action[:6],
            human_action[:6],
        )
        self.assertEqual(float(result.executed_action[6]), 1.0)
        self.assertTrue(result.human_active)
        self.assertEqual(result.autonomy_weight, 0.0)

    def test_takeover_gripper_biases_toward_closing(
        self,
    ) -> None:
        human_action = np.asarray(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            dtype=np.float32,
        )

        result = ActionArbitrator(
            ArbitrationMode.TAKEOVER
        ).arbitrate(
            autonomous_action=self.autonomous_action,
            human_action=human_action,
        )

        np.testing.assert_array_equal(
            result.executed_action[:6],
            self.autonomous_action[:6],
        )
        self.assertEqual(float(result.executed_action[6]), 1.0)
        self.assertFalse(result.human_active)

    def test_activity_uses_only_first_six_dimensions(
        self,
    ) -> None:
        human_action = np.asarray(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0],
            dtype=np.float32,
        )

        result = ActionArbitrator(
            ArbitrationMode.TAKEOVER
        ).arbitrate(
            autonomous_action=self.autonomous_action,
            human_action=human_action,
        )

        self.assertEqual(
            result.activity_state,
            ActivityState.IDLE,
        )
        self.assertEqual(result.human_motion_norm, 0.0)

    def test_result_is_json_compatible(
        self,
    ) -> None:
        human_action = np.asarray(
            [0.01, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0],
            dtype=np.float32,
        )

        result = ActionArbitrator(
            ArbitrationMode.TAKEOVER
        ).arbitrate(
            autonomous_action=self.autonomous_action,
            human_action=human_action,
        )
        record = result.as_dict()

        self.assertEqual(
            record["arbitration_mode"],
            "takeover",
        )
        self.assertEqual(record["activity_state"], "active")
        self.assertTrue(record["human_active"])
        self.assertIsNone(
            record["configured_autonomy_weight"]
        )
        self.assertEqual(
            record["effective_autonomy_weight"],
            0.0,
        )
        self.assertEqual(record["autonomy_weight"], 0.0)
        self.assertEqual(len(record["human_action"]), 7)
        self.assertEqual(len(record["autonomous_action"]), 7)
        self.assertEqual(len(record["executed_action"]), 7)

    def test_invalid_actions_are_rejected(self) -> None:
        arbitrator = ActionArbitrator(
            ArbitrationMode.TAKEOVER
        )

        with self.assertRaisesRegex(
            ValueError,
            "shape",
        ):
            arbitrator.arbitrate(
                autonomous_action=np.zeros(
                    6,
                    dtype=np.float32,
                ),
                human_action=np.zeros(
                    7,
                    dtype=np.float32,
                ),
            )

        invalid_human = np.zeros(
            7,
            dtype=np.float32,
        )
        invalid_human[2] = np.nan

        with self.assertRaisesRegex(
            ValueError,
            "finite",
        ):
            arbitrator.arbitrate(
                autonomous_action=self.autonomous_action,
                human_action=invalid_human,
            )

    def test_fixed_blend_idle_uses_full_autonomy(
        self,
    ) -> None:
        human_action = np.asarray(
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, -1.0],
            dtype=np.float32,
        )
        result = ActionArbitrator(
            ArbitrationMode.FIXED_BLEND,
            fixed_autonomy_weight=0.25,
        ).arbitrate(
            autonomous_action=self.autonomous_action,
            human_action=human_action,
        )
        np.testing.assert_array_equal(
            result.executed_action[:6],
            self.autonomous_action[:6],
        )
        self.assertFalse(result.human_active)
        self.assertEqual(result.configured_autonomy_weight, 0.25)
        self.assertEqual(result.autonomy_weight, 1.0)

    def test_fixed_blend_active_uses_motion_midpoint(
        self,
    ) -> None:
        human_action = np.asarray(
            [-0.30, 0.40, -0.50, 0.60, -0.70, 0.80, 1.0],
            dtype=np.float32,
        )
        result = ActionArbitrator(
            "fixed_blend",
            fixed_autonomy_weight=0.5,
        ).arbitrate(
            autonomous_action=self.autonomous_action,
            human_action=human_action,
        )
        expected_motion = (
            0.5 * self.autonomous_action[:6]
            + 0.5 * human_action[:6]
        )
        np.testing.assert_allclose(
            result.executed_action[:6],
            expected_motion,
        )
        self.assertEqual(float(result.executed_action[6]), 1.0)
        self.assertTrue(result.human_active)
        self.assertEqual(result.configured_autonomy_weight, 0.5)
        self.assertEqual(result.autonomy_weight, 0.5)

    def test_fixed_blend_boundary_weights(
        self,
    ) -> None:
        human_action = np.asarray(
            [-0.25, 0.10, 0.05, 0.20, -0.10, 0.15, -1.0],
            dtype=np.float32,
        )
        autonomous_result = ActionArbitrator(
            "fixed_blend",
            fixed_autonomy_weight=1.0,
        ).arbitrate(
            autonomous_action=self.autonomous_action,
            human_action=human_action,
        )
        human_result = ActionArbitrator(
            "fixed_blend",
            fixed_autonomy_weight=0.0,
        ).arbitrate(
            autonomous_action=self.autonomous_action,
            human_action=human_action,
        )
        np.testing.assert_array_equal(
            autonomous_result.executed_action[:6],
            self.autonomous_action[:6],
        )
        np.testing.assert_array_equal(
            human_result.executed_action[:6],
            human_action[:6],
        )

    def test_invalid_fixed_weights_are_rejected(
        self,
    ) -> None:
        for weight in (-0.01, 1.01, float("nan")):
            with self.subTest(weight=weight):
                with self.assertRaisesRegex(
                    ValueError,
                    "fixed_autonomy_weight",
                ):
                    ActionArbitrator(
                        "fixed_blend",
                        fixed_autonomy_weight=weight,
                    )



if __name__ == "__main__":
    unittest.main()
