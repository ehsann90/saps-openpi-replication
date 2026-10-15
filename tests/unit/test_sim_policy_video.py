"""Simulation-time ordering and recoverable policy-video recording tests."""

from __future__ import annotations

import json
import hashlib
from pathlib import Path
import sys
import tempfile
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "sim/src"))
sys.path.insert(0, str(ROOT / "tools/analysis"))

from PIL import Image
from isaac_fr3.policy_video import (
    PolicyVideoRecorder, export_video, validate_timeline,
)
from export_isaac_policy_video import archived_frames


class PolicyVideoTest(unittest.TestCase):
    def test_frame_order_and_simulation_spacing(self) -> None:
        frames = [{"index": i, "simulation_time_seconds": 5 + i / 15}
                  for i in range(3)]
        validate_timeline(frames, 1 / 15)
        with self.assertRaisesRegex(ValueError, "indices"):
            validate_timeline([frames[1], frames[0]], 1 / 15)
        frames[2]["simulation_time_seconds"] += 1 / 15
        with self.assertRaisesRegex(ValueError, "spacing"):
            validate_timeline(frames, 1 / 15)

    def test_finalization_excludes_wall_inference_pause(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "video"
            recorder = PolicyVideoRecorder(output, source_rollout_id="test")
            pixels = np.zeros((18, 32, 3), dtype=np.uint8)
            recorder.append(replan_index=0, action_index=-1,
                            simulation_time_seconds=5,
                            simulation_step_index=300,
                            exterior=pixels, wrist=pixels)
            # A pause between chunks does not become a video frame or duration.
            with patch("isaac_fr3.policy_video._encode_stream") as encode, \
                    patch("isaac_fr3.policy_video._inspect_stream",
                          return_value={"nb_frames": "2"}):
                recorder.append(replan_index=0, action_index=0,
                                simulation_time_seconds=5 + 1 / 15,
                                simulation_step_index=304,
                                exterior=pixels, wrist=pixels)
                result = recorder.finalize()
            self.assertEqual(encode.call_count, 1)
            self.assertEqual(result["status"], "complete")
            self.assertEqual(result["frame_count"], 2)
            self.assertAlmostEqual(result["simulated_span_seconds"], 1 / 15)
            self.assertEqual(len(list((output / "raw" / "wrist").glob("*.png"))), 2)
            self.assertEqual(json.loads((output / "simulation_motion_recording.json").read_text())[
                "status"], "complete")
            with self.assertRaisesRegex(RuntimeError, "finalized"):
                recorder.append(replan_index=1, action_index=1,
                                simulation_time_seconds=5 + 2 / 15,
                                simulation_step_index=308,
                                exterior=pixels, wrist=pixels)

    def test_interrupted_recording_retains_saved_frames(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            output = Path(temporary) / "video"
            recorder = PolicyVideoRecorder(output, source_rollout_id="test")
            pixels = np.zeros((18, 32, 3), dtype=np.uint8)
            recorder.append(replan_index=0, action_index=-1,
                            simulation_time_seconds=0,
                            simulation_step_index=0,
                            exterior=pixels, wrist=pixels)
            with patch("isaac_fr3.policy_video._encode_stream"), \
                    patch("isaac_fr3.policy_video._inspect_stream",
                          return_value={"nb_frames": "1"}):
                result = recorder.finalize(termination_reason="manual_interrupt")
            self.assertEqual(result["frame_count"], 1)
            self.assertEqual(result["status"], "partial")
            self.assertEqual(result["rollout_termination_reason"],
                             "manual_interrupt")
            self.assertTrue((output / "raw" / "exterior" / "000000.png").exists())

    def test_archive_uses_numeric_replan_order_and_actual_timestamps(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            run = Path(temporary)
            (run / "replans").mkdir()
            (run / "episode.json").write_text(json.dumps({
                "completed_replans": 2, "simulation_control_seconds": 16 / 15,
            }))
            for index, stamp, intent in ((0, 5.3, "CLOSED"),
                                         (1, 5.3 + 8 / 15, "OPEN")):
                directory = run / "replans" / f"{index:04d}"
                directory.mkdir()
                (directory / "observation.json").write_text(json.dumps({
                    "simulation_time_seconds": stamp,
                    "simulation_step_index": 318 + 32 * index,
                    "array_evidence": {},
                }))
                (directory / "execution.json").write_text(json.dumps({
                    "status": "complete", "actions_executed": 8,
                    "gripper_intent_after": intent,
                }))
            frames, period, missing = archived_frames(run)
            self.assertEqual([f["replan_index"] for f in frames], [0, 1])
            self.assertEqual(missing, [])
            self.assertAlmostEqual(frames[1]["simulation_time_seconds"]
                                   - frames[0]["simulation_time_seconds"], period)

    def test_observation_export_rejects_image_changed_from_request(self) -> None:
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            pixels = np.zeros((18, 32, 3), dtype=np.uint8)
            pixels[:, :, 0] = 15
            source = root / "image.png"
            Image.fromarray(pixels).save(source)
            evidence = {
                "shape": list(pixels.shape), "dtype": str(pixels.dtype),
                "sha256": hashlib.sha256(pixels.tobytes()).hexdigest(),
            }
            frame = {
                "index": 0, "replan_index": 0,
                "simulation_time_seconds": 1.0,
                "exterior": source, "wrist": source,
                "array_evidence": {
                    "observation/exterior_image_1_left": evidence,
                    "observation/wrist_image_left": evidence,
                },
            }
            corrupted = pixels.copy()
            corrupted[0, 0, 0] = 99
            Image.fromarray(corrupted).save(source)
            with self.assertRaisesRegex(ValueError, "differs from request"):
                export_video([frame], root / "videos", period_s=8 / 15,
                             label="test", video_name="policy_observations",
                             source_rollout_id="test")


if __name__ == "__main__":
    unittest.main()
