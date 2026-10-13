"""Compare one historical and one candidate Robotiq table ON/OFF pair."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def load_run(path: Path) -> dict:
    return {name: json.loads((path / f"{name}.json").read_text())
            for name in ("resolved_config", "plan", "samples", "contacts",
                         "summary")}


def describe_pair(on: dict, off: dict) -> dict:
    """Require the established loaded-contact gates before computing q deltas."""
    a, b = on["summary"], off["summary"]
    if a["assembly"] != b["assembly"] or a["assembly"] != "robotiq":
        raise ValueError("Both runs must use Robotiq")
    if a["table"] != "enabled" or b["table"] != "disabled":
        raise ValueError("Expected table ON/OFF pair")
    if on["plan"]["plan"] != off["plan"]["plan"]:
        raise ValueError("ON/OFF command plans differ")
    if on["resolved_config"]["scene"] != off["resolved_config"]["scene"]:
        raise ValueError("ON/OFF scenes differ")
    for key in ("source_config", "resolved_robot_asset"):
        if on["resolved_config"][key] != off["resolved_config"][key]:
            raise ValueError(f"ON/OFF {key} differs")
    if any(s["aborted"] or not s["retreat_completed"] for s in (a, b)):
        raise ValueError("Abort or incomplete retreat")
    clearance = a["initial_clearance_m"]
    speed = a["pre_contact_normal_speed_m_s"]
    if not (0.004 <= clearance <= 0.008 and 0.005 <= speed <= 0.04):
        raise ValueError("Initial clearance or approach speed outside gates")
    if (not a["contact_api_runtime_qualified"] or
            a["loaded_fingertip_table_contact_count"] <= 0 or
            b["loaded_fingertip_table_contact_count"] != 0):
        raise ValueError("Loaded pad/table contact not qualified")
    first = a["first_loaded_fingertip_contact"]
    if first is None or first["impulse_norm_sum_ns"] <= 1e-8:
        raise ValueError("No qualified loaded impulse")
    on_samples, off_samples = on["samples"], off["samples"]
    commands = on["plan"]["plan"]["commands"]
    if len(on_samples) != len(off_samples) or len(on_samples) != len(commands) + 1:
        raise ValueError("ON/OFF sample count mismatch")
    for index, (left, right) in enumerate(zip(on_samples, off_samples)):
        if left["step"] != right["step"] or left["target_q_rad"] != right["target_q_rad"]:
            raise ValueError(f"ON/OFF target or step mismatch at {index}")
        if index and left["target_q_rad"] != commands[index - 1]["target_q_rad"]:
            raise ValueError(f"Plan target mismatch at {index}")
        if len(left["measured_q_rad"]) != 7 or len(right["measured_q_rad"]) != 7:
            raise ValueError("Expected seven measured arm joints")
    if any(abs(aq - bq) > 1e-5 for aq, bq in zip(
            on_samples[0]["measured_q_rad"], off_samples[0]["measured_q_rad"])):
        raise ValueError("Initial measured arm states differ")
    loaded = [c for c in on["contacts"]
              if c["loaded_fingertip_table_contact"] and
              c["impulse_norm_sum_ns"] > 1e-8]
    if not loaded:
        raise ValueError("No loaded pad/table event")
    if any(c["loaded_fingertip_table_contact"] and
           c["impulse_norm_sum_ns"] > 1e-8 for c in off["contacts"]):
        raise ValueError("Loaded pad/table event in no-table control")
    deltas = [[left["measured_q_rad"][joint] -
               right["measured_q_rad"][joint]
               for joint in range(7)]
              for left, right in zip(on_samples, off_samples)]
    peaks = []
    for joint in range(7):
        step = max(range(len(deltas)), key=lambda k: abs(deltas[k][joint]))
        peaks.append({"joint": joint + 1, "step": step,
                      "signed_rad": deltas[step][joint],
                      "absolute_rad": abs(deltas[step][joint]),
                      "final_rad": deltas[-1][joint]})
    return {
        "scene_config": on["resolved_config"]["source_config"],
        "robot_asset": on["resolved_config"]["resolved_robot_asset"],
        "initial_clearance_m": clearance,
        "pre_contact_normal_speed_m_s": speed,
        "first_loaded_step": min(c["step"] for c in loaded),
        "last_loaded_step": max(c["step"] for c in loaded),
        "loaded_contact_steps": len({c["step"] for c in loaded}),
        "loaded_contact_pairs": sorted({(c["collider0"], c["collider1"])
                                        for c in loaded}),
        "minimum_recorded_contact_separation_m": min(
            c["minimum_separation_m"] for c in loaded),
        "minimum_measured_clearance_m": min(
            s["clearance_m"] for s in on_samples
            if s["clearance_m"] is not None),
        "joint_position_on_minus_off_peaks": peaks,
    }


def main() -> None:
    parser = argparse.ArgumentParser()
    for name in ("historical_on", "historical_off", "candidate_on",
                 "candidate_off"):
        parser.add_argument("--" + name.replace("_", "-"), type=Path,
                            required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    historical = describe_pair(load_run(args.historical_on),
                               load_run(args.historical_off))
    candidate = describe_pair(load_run(args.candidate_on),
                              load_run(args.candidate_off))
    if historical["robot_asset"] == candidate["robot_asset"]:
        raise ValueError("Historical and candidate robot assets must differ")
    result = {"historical": historical, "candidate": candidate,
              "q5_peak_absolute_change_rad":
              candidate["joint_position_on_minus_off_peaks"][4]["absolute_rad"] -
              historical["joint_position_on_minus_off_peaks"][4]["absolute_rad"]}
    with args.output.open("x", encoding="utf-8") as file:
        json.dump(result, file, indent=2)
        file.write("\n")
    print(args.output, flush=True)


if __name__ == "__main__":
    main()
