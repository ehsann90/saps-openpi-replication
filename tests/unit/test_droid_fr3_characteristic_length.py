"""Tests for offline DROID-to-FR3 characteristic-length analysis."""

from __future__ import annotations

from pathlib import Path
import tempfile
import unittest
from unittest import mock

import numpy as np

from tools.analysis.analyze_droid_fr3_characteristic_length import analyze_run
from tools.analysis.analyze_droid_fr3_characteristic_length import build_report
from tools.analysis.analyze_droid_fr3_characteristic_length import load_m3_run
from tools.analysis.analyze_droid_fr3_characteristic_length import M3RunData
from tools.analysis.analyze_droid_fr3_characteristic_length import norm_statistics
from tools.analysis.analyze_droid_fr3_characteristic_length import (
    summarize_motion_samples,
)


def _sample(
    translation: float,
    rotation: float,
    *,
    run_id: str = "run",
    observation_index: int = 0,
    action_index: int = 0,
) -> dict[str, object]:
    return {
        "run_id": run_id,
        "observation_index": observation_index,
        "action_index": action_index,
        "translation_norm_m": translation,
        "rotation_norm_rad": rotation,
        "next_state_joint_limit_violation": False,
        "violated_joint_indices": [],
    }


def _write_npz_run(
    run_dir: Path,
    *,
    joint_positions: np.ndarray,
    actions: np.ndarray,
) -> None:
    run_dir.mkdir()
    count = joint_positions.shape[0]
    np.savez_compressed(
        run_dir / "observation_bundle.npz",
        exterior_images=np.zeros((count, 2, 3, 3), dtype=np.uint8),
        wrist_images=np.zeros((count, 2, 3, 3), dtype=np.uint8),
        joint_positions=joint_positions,
        gripper_positions=np.zeros((count, 1), dtype=np.float32),
        prompts=np.asarray(["test prompt"] * count),
        source_ros_seconds=np.ones((count, 4), dtype=np.float64),
    )
    np.savez_compressed(run_dir / "policy_actions.npz", actions=actions)


class CharacteristicLengthFormulaTest(unittest.TestCase):
    def test_required_norm_statistics(self) -> None:
        statistics = norm_statistics(np.asarray([1.0, 2.0, 3.0, 4.0]))

        self.assertEqual(statistics["count"], 4)
        self.assertEqual(statistics["median"], 2.5)
        self.assertEqual(statistics["p75"], 3.25)
        self.assertEqual(statistics["p90"], 3.7)
        self.assertEqual(statistics["p95"], 3.8499999999999996)
        self.assertEqual(statistics["maximum"], 4.0)

    def test_characteristic_length_and_energy_formulas(self) -> None:
        samples = [
            _sample(translation, 2.0, action_index=index)
            for index, translation in enumerate((1.0, 2.0, 3.0, 4.0))
        ]

        statistics = summarize_motion_samples(
            samples,
            ell_0_m_per_rad=0.5,
            rotation_exclusion_threshold_rad=1e-9,
        )
        anchors = statistics["characteristic_length_m_per_rad"]

        self.assertEqual(anchors["ell_median"], 1.25)
        self.assertAlmostEqual(anchors["ell_p95"], 1.925)
        self.assertAlmostEqual(anchors["ell_rms"], np.sqrt(30.0 / 16.0))
        self.assertEqual(anchors["ell_max"], 2.0)
        energy = statistics["squared_energy_balance_under_ell_0"]
        self.assertEqual(
            energy["translation_to_weighted_rotation_ratio"],
            7.5,
        )

    def test_near_zero_rotations_are_excluded_without_epsilon(self) -> None:
        samples = [
            _sample(0.2, 0.0, action_index=0),
            _sample(0.3, 1e-10, action_index=1),
            _sample(0.4, 0.2, action_index=2),
        ]

        statistics = summarize_motion_samples(
            samples,
            ell_0_m_per_rad=0.3,
            rotation_exclusion_threshold_rad=1e-9,
        )
        ratios = statistics["per_action_ratio_m_per_rad"]

        self.assertEqual(ratios["valid_count"], 1)
        self.assertEqual(ratios["excluded_count"], 2)
        self.assertEqual(ratios["distribution"]["median"], 2.0)


class CharacteristicLengthAggregationTest(unittest.TestCase):
    def test_multiple_runs_are_kept_separate_and_pooled(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            first = root / "first"
            second = root / "second"
            q = np.zeros((1, 7), dtype=np.float64)
            actions = np.zeros((1, 15, 8), dtype=np.float64)
            _write_npz_run(first, joint_positions=q, actions=actions)
            _write_npz_run(second, joint_positions=q, actions=actions)

            report = build_report(
                [first, second],
                position_lower_rad=np.full(7, -10.0),
                position_upper_rad=np.full(7, 10.0),
            )

        self.assertEqual([run["run_id"] for run in report["runs"]], [
            "first",
            "second",
        ])
        self.assertEqual(report["runs"][0]["first_eight_action_count"], 8)
        self.assertEqual(report["runs"][1]["first_eight_action_count"], 8)
        self.assertEqual(report["pooled"]["run_count"], 2)
        self.assertEqual(report["pooled"]["observation_count"], 2)
        self.assertEqual(report["pooled"]["first_eight_action_count"], 16)
        pooled_translation = report["pooled"]["statistics"][
            "translation_norm_m"
        ]
        self.assertEqual(pooled_translation["count"], 16)
        self.assertEqual(pooled_translation["median"], 0.0)

    def test_rollout_uses_only_first_eight_actions_sequentially(self) -> None:
        actions = np.zeros((1, 15, 8), dtype=np.float64)
        actions[0, :, 0] = 0.5
        run = M3RunData(
            run_id="sequential",
            run_dir=Path("sequential"),
            joint_positions=np.zeros((1, 7), dtype=np.float64),
            actions=actions,
            prompts=("test",),
            validation={},
        )
        seen_q = []

        def finite_displacement(q: np.ndarray, delta_q: np.ndarray) -> np.ndarray:
            seen_q.append(q.copy())
            return np.zeros(6, dtype=np.float64)

        with mock.patch(
            "tools.analysis.analyze_droid_fr3_characteristic_length."
            "fr3_tcp_finite_displacement",
            side_effect=finite_displacement,
        ):
            report = analyze_run(
                run,
                position_lower_rad=np.full(7, -10.0),
                position_upper_rad=np.full(7, 10.0),
                ell_0_m_per_rad=0.3,
                rotation_exclusion_threshold_rad=1e-9,
            )

        self.assertEqual(len(seen_q), 8)
        np.testing.assert_allclose(
            [q[0] for q in seen_q],
            np.arange(8, dtype=np.float64) * 0.1,
        )
        self.assertEqual(
            [sample["action_index"] for sample in report["samples"]],
            list(range(8)),
        )

    def test_malformed_joint_and_action_shapes_are_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            bad_joint_run = root / "bad_joints"
            _write_npz_run(
                bad_joint_run,
                joint_positions=np.zeros((1, 6), dtype=np.float64),
                actions=np.zeros((1, 15, 8), dtype=np.float64),
            )
            with self.assertRaisesRegex(ValueError, "joint positions"):
                load_m3_run(bad_joint_run)

            bad_action_run = root / "bad_actions"
            _write_npz_run(
                bad_action_run,
                joint_positions=np.zeros((1, 7), dtype=np.float64),
                actions=np.zeros((1, 8, 8), dtype=np.float64),
            )
            with self.assertRaisesRegex(ValueError, "policy actions"):
                load_m3_run(bad_action_run)


if __name__ == "__main__":
    unittest.main()
