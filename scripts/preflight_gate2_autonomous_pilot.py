#!/usr/bin/env python3
"""Validate and print the frozen Gate-2 v2 autonomous design."""

from __future__ import annotations

import dataclasses
from pathlib import Path

import tyro

from saps.evaluation.experiment_session import json_file_identity
from saps.evaluation.experiment_session import load_manifest
from saps.evaluation.gate2_v2_protocol import (
    build_gate2_v2_autonomous_schedule,
)
from saps.evaluation.gate2_v2_protocol import build_gate2_v2_shared_schedule
from saps.evaluation.gate2_v2_protocol import GATE2_V2_CONFIG_SHA256
from saps.evaluation.gate2_v2_protocol import GATE2_V2_MANIFEST_PATH
from saps.evaluation.gate2_v2_protocol import GATE2_V2_SHARED_OUTPUT_ROOT
from saps.evaluation.gate2_v2_protocol import (
    GATE2_V2_AUTONOMOUS_PROTOCOL_PATH,
)
from saps.evaluation.gate2_v2_protocol import (
    load_gate2_v2_autonomous_protocol,
)
from saps.evaluation.gate2_v2_protocol import validate_gate2_v2_matched_design


@dataclasses.dataclass
class Args:
    protocol_path: str = GATE2_V2_AUTONOMOUS_PROTOCOL_PATH


def main(args: Args) -> None:
    """Print all 20 rows without reading or creating collection outputs."""

    protocol = load_gate2_v2_autonomous_protocol(Path(args.protocol_path))
    config = json_file_identity(Path(protocol["config_path"]))
    if config["sha256"] != GATE2_V2_CONFIG_SHA256:
        raise ValueError("Gate-2 v2 perturbation configuration drifted.")
    contents = config["contents"]
    if (
        contents.get("task_suite_name") != protocol["task_suite_name"]
        or int(contents.get("task_id", -1)) != protocol["task_id"]
    ):
        raise ValueError("Autonomous task does not match the perturbation config.")
    schedule = build_gate2_v2_autonomous_schedule(protocol)
    shared_manifest = load_manifest(Path(GATE2_V2_MANIFEST_PATH))
    shared_schedule = build_gate2_v2_shared_schedule(
        manifest=shared_manifest,
        task_id=protocol["task_id"],
        output_root=Path(GATE2_V2_SHARED_OUTPUT_ROOT),
    )
    matched = validate_gate2_v2_matched_design(
        shared_schedule=shared_schedule,
        autonomous_schedule=schedule,
    )
    shared_protocol_values = {
        "initial_state_index": shared_manifest.initial_state_index,
        "environment_seed": shared_manifest.environment_seed,
        "policy_base_seed": shared_manifest.policy_base_seed,
        "max_steps": shared_manifest.operator_max_steps,
        "control_frequency_hz": shared_manifest.control_frequency_hz,
    }
    if any(
        protocol[name] != value
        for name, value in shared_protocol_values.items()
    ):
        raise ValueError(
            "Autonomous timing or reproducibility settings do not match "
            "the frozen shared manifest."
        )

    print("Gate-2 v2 autonomous pilot preflight: PASSED")
    print(f"Experiment ID: {protocol['experiment_id']}")
    print(f"Output root: {protocol['output_root']}")
    print(f"Conditions: {', '.join(protocol['conditions'])}")
    print(f"Trials: {protocol['trials']}")
    print(f"Episodes: {len(schedule['episodes'])}")
    print(f"Matched triplets with shared design: {matched['matched_triplets']}")
    print(f"Environment seed: {protocol['environment_seed']}")
    print(f"Policy base seed: {protocol['policy_base_seed']}")
    print(f"Policy seed protocol: {schedule['policy_seed_protocol']}")
    print(
        "Task / initial state: "
        f"{protocol['task_id']} / {protocol['initial_state_index']}"
    )
    print(f"Replan steps: {protocol['replan_steps']}")
    print(f"Settle steps: {protocol['settle_steps']}")
    print(f"Environment-step horizon: {protocol['max_steps']}")
    print(f"LIBERO control frequency: {protocol['control_frequency_hz']} Hz")
    print(
        f"Perturbation config: {config['path']} "
        f"sha256={config['sha256']}"
    )
    print()
    print("index  condition  trial  initial_state  policy_seed")
    for row in schedule["episodes"]:
        print(
            f"{row['order_index']:>5}  {row['condition_id']:<10} "
            f"{row['trial_index']:>5}  {row['initial_state_index']:>13}  "
            f"{row['policy_episode_seed']:>11}"
        )
    print()
    print("Preflight does not read outputs or launch an episode.")


if __name__ == "__main__":
    main(tyro.cli(Args))
