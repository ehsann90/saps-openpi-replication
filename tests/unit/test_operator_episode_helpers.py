"""Tests for reusable operator-episode helpers."""

from __future__ import annotations

import json
from pathlib import Path
import tempfile
import unittest
from typing import Any

import numpy as np

from saps.evaluation.operator_episode import (
    load_config,
)
from saps.evaluation.operator_episode import (
    operator_view_rgb,
)
from saps.evaluation.operator_episode import (
    select_condition,
)
from saps.evaluation.operator_episode import (
    write_json_atomic,
)


class OperatorEpisodeHelpersTest(unittest.TestCase):
    def test_load_and_select_condition(self) -> None:
        payload: dict[str, Any] = {
            "task_suite_name": "libero_object",
            "task_id": 1,
            "joint_name": "object_joint",
            "body_name": "object_body",
            "offsets": [
                {
                    "id": "nominal",
                    "dx": 0.0,
                    "dy": 0.0,
                },
                {
                    "id": "p01",
                    "dx": 0.1,
                    "dy": 0.0,
                },
            ],
        }

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "config.json"
            path.write_text(
                json.dumps(payload),
                encoding="utf-8",
            )

            config = load_config(path)
            condition = select_condition(
                config,
                "p01",
            )

        self.assertEqual(condition["id"], "p01")
        self.assertEqual(condition["dx"], 0.1)

    def test_unknown_condition_is_rejected(self) -> None:
        config = {
            "offsets": [
                {
                    "id": "nominal",
                    "dx": 0.0,
                    "dy": 0.0,
                }
            ]
        }

        with self.assertRaisesRegex(
            ValueError,
            "Unknown condition",
        ):
            select_condition(config, "missing")

    def test_operator_view_combines_both_cameras(
        self,
    ) -> None:
        agent = np.zeros(
            (40, 40, 3),
            dtype=np.uint8,
        )
        wrist = np.zeros(
            (40, 40, 3),
            dtype=np.uint8,
        )

        agent[0, 0] = [1, 2, 3]
        wrist[0, 0] = [4, 5, 6]

        combined = operator_view_rgb(
            {
                "agentview_image": agent,
                "robot0_eye_in_hand_image": wrist,
            }
        )

        self.assertEqual(
            combined.shape,
            (40, 96, 3),
        )

        np.testing.assert_array_equal(
            combined[-1, 39],
            [1, 2, 3],
        )
        np.testing.assert_array_equal(
            combined[-1, -1],
            [4, 5, 6],
        )
        np.testing.assert_array_equal(
            combined[-1, 40:56],
            np.full(
                (16, 3),
                220,
                dtype=np.uint8,
            ),
        )

    def test_atomic_json_writer(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            path = (
                Path(directory)
                / "nested"
                / "result.json"
            )

            write_json_atomic(
                path,
                {
                    "mode": "takeover",
                    "success": True,
                },
            )

            self.assertTrue(path.is_file())
            self.assertFalse(
                path.with_name(
                    ".result.json.tmp"
                ).exists()
            )

            payload = json.loads(
                path.read_text(encoding="utf-8")
            )

        self.assertEqual(
            payload,
            {
                "mode": "takeover",
                "success": True,
            },
        )


if __name__ == "__main__":
    unittest.main()
