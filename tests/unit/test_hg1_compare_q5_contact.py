"""Focused pair acceptance and position-delta tests for HG1-B2."""

import importlib.util
import unittest
from pathlib import Path


SCRIPT = (Path(__file__).resolve().parents[2] /
          "tools/hg1_compare_q5_contact.py")
SPEC = importlib.util.spec_from_file_location("hg1_compare_q5_contact", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC and SPEC.loader
SPEC.loader.exec_module(MODULE)


def pair() -> tuple[dict, dict]:
    target = [0.0] * 7
    common = {"assembly": "robotiq", "initial_clearance_m": 0.006,
              "pre_contact_normal_speed_m_s": 0.02, "aborted": None,
              "retreat_completed": True}
    event = {"step": 1, "collider0": "/fingerpad", "collider1": "/World/Table",
             "minimum_separation_m": -0.0001, "impulse_norm_sum_ns": 0.1,
             "loaded_fingertip_table_contact": True}
    def run(table: str) -> dict:
        enabled = table == "enabled"
        return {
            "summary": {**common, "table": table,
                        "contact_api_runtime_qualified": enabled,
                        "loaded_fingertip_table_contact_count": int(enabled),
                        "first_loaded_fingertip_contact": event if enabled else None},
            "plan": {"plan": {"commands": [{"target_q_rad": target.copy()}]}},
            "resolved_config": {"scene": {"robot": "same"},
                                "source_config": "scene.json",
                                "resolved_robot_asset": "robot.usda"},
            "samples": [
                {"step": 0, "target_q_rad": target,
                 "measured_q_rad": target, "clearance_m": 0.006},
                {"step": 1, "target_q_rad": target,
                 "measured_q_rad": ([0.0] * 4 + [0.1 if enabled else 0.0]
                                    + [0.0] * 2), "clearance_m": -0.0001},
            ],
            "contacts": [event] if enabled else [],
        }
    return run("enabled"), run("disabled")


class CompareContactTests(unittest.TestCase):
    def test_loaded_pair_uses_measured_position(self) -> None:
        on, off = pair()
        result = MODULE.describe_pair(on, off)
        self.assertEqual(result["joint_position_on_minus_off_peaks"][4]
                         ["signed_rad"], 0.1)
        self.assertEqual(result["first_loaded_step"], 1)
        self.assertEqual(result["loaded_contact_pairs"],
                         [("/fingerpad", "/World/Table")])

    def test_rejects_mismatched_plan_and_unloaded_contact(self) -> None:
        on, off = pair()
        off["plan"]["plan"]["commands"][0]["target_q_rad"][4] = 0.2
        with self.assertRaisesRegex(ValueError, "command plans differ"):
            MODULE.describe_pair(on, off)
        on, off = pair()
        on["contacts"][0]["impulse_norm_sum_ns"] = 0
        with self.assertRaisesRegex(ValueError, "No qualified loaded impulse"):
            MODULE.describe_pair(on, off)


if __name__ == "__main__":
    unittest.main()
