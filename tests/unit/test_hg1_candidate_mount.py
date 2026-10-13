"""Static checks for separately selected Fusion-v13 mount geometry."""

import json
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
BASE = ROOT / "sim/configs/fr3_droid_robotiq_scene.json"
CANDIDATE = (ROOT / "sim/configs/"
             "fr3_droid_robotiq_fusion_v13_candidate_scene.json")
HISTORICAL = (ROOT / "sim/configs/"
              "fr3_droid_robotiq_historical_hg1_scene.json")
RECORD = (ROOT / "sim/assets/FR3_2F85_ZED_transforms/"
          "fusion_v13_candidate.json")


class CandidateMountTests(unittest.TestCase):
    def test_scene_selection_changes_only_asset(self) -> None:
        baseline = json.loads(BASE.read_text())
        candidate = json.loads(CANDIDATE.read_text())
        self.assertEqual(baseline["robot"]["asset"],
                         "assets/fr3_robotiq_droid.usda")
        historical = json.loads(HISTORICAL.read_text())
        self.assertEqual(historical["robot"]["asset"],
                         "assets/fr3_robotiq_droid_historical_hg1.usda")
        historical["robot"]["asset"] = baseline["robot"]["asset"]
        self.assertEqual(historical, baseline)
        candidate["robot"]["asset"] = baseline["robot"]["asset"]
        self.assertEqual(candidate, baseline)

    def test_derived_frames_are_consistent(self) -> None:
        record = json.loads(RECORD.read_text())
        self.assertEqual(record["source_fusion_design_version"], 13)
        matrices = record["matrices_m"]
        base = matrices["T_flange_from_usd_base"]
        grip = matrices["T_flange_from_usd_grip"]
        camera = matrices["T_flange_from_usd_camera_nominal"]
        self.assertAlmostEqual(base[2][3], 0.010758, places=9)
        self.assertAlmostEqual(grip[2][3],
                               0.010758 + 0.13371706760793903, places=9)
        for row_a, row_b in zip(base, grip):
            self.assertEqual(row_a[:3], row_b[:3])
        camera_from_grip = matrices["T_usd_camera_from_usd_grip"]
        for i in range(3):
            for j in range(3):
                expected_rotation = sum(
                    camera[k][i] * grip[k][j] for k in range(3)
                )
                self.assertAlmostEqual(camera_from_grip[i][j],
                                       expected_rotation, places=8)
            expected_translation = sum(
                camera[k][i] * (grip[k][3] - camera[k][3])
                for k in range(3)
            )
            self.assertAlmostEqual(camera_from_grip[i][3],
                                   expected_translation, places=8)
        historical = json.loads((ROOT / "sim/assets/"
                                 "FR3_2F85_ZED_transforms/"
                                 "fr3_2f85_zed_transforms.json").read_text())
        for row in range(3):
            self.assertAlmostEqual(
                camera[row][3],
                historical["matrices"]
                ["T_flange_from_usd_camera_nominal"][row][3],
                places=9,
            )


if __name__ == "__main__":
    unittest.main()
